import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros

class VSLAMNode(Node):
    def __init__(self):
        super().__init__('vslam_node')
        
        self.bridge = CvBridge()
        
        # Camera intrinsics
        self.K = None  
        self.fx = 600.0
        self.fy = 600.0
        self.cx = 320.0
        self.cy = 240.0
        
        # Feature detector and matcher
        self.orb = cv2.ORB_create(nfeatures=500)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        # Pose state
        self.pose = np.eye(4)  # Cumulative camera pose in world frame
        self.frame_count = 0
        
        # Keyframe state 
        self.keyframe = None
        self.keyframe_kp = None
        self.keyframe_des = None
        self.keyframe_pose = np.eye(4)
        self.frames_since_keyframe = 0
        
        # Keyframe thresholds
        self.min_translation_m = 0.05      # Min movement to create new keyframe
        self.min_rotation_deg = 2.0        # Min rotation to create new keyframe  
        self.min_pixel_displacement = 15.0 # Skip pose update if features move less
        self.max_frames_since_keyframe = 30
        
        # QoS
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self._image_callback, qos)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth_map', self._depth_callback, qos)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self._camera_info_callback, 10)
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/vslam/pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/vslam/odom', 10)
        self.map_pub = self.create_publisher(PointCloud2, '/vslam/map_points', 10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Depth buffer
        self.current_depth = None
        
        self.get_logger().info('VSLAM Node initialized')
    
    def _camera_info_callback(self, msg: CameraInfo):
        self.K = np.array(msg.k).reshape(3, 3)
        self.fx = self.K[0, 0]
        self.fy = self.K[1, 1]
        self.cx = self.K[0, 2]
        self.cy = self.K[1, 2]
    
    def _depth_callback(self, msg: Image):
        """Cache latest depth map."""
        try:
            self.current_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion error: {e}')
    
    def _image_callback(self, msg: Image):
        """Main VO pipeline"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return
        
        self.frame_count += 1
        self.frames_since_keyframe += 1
        
        # Detect features
        kp, des = self.orb.detectAndCompute(frame, None)
        
        # Initialize keyframe on first frame
        if self.keyframe is None or self.keyframe_des is None or des is None:
            self.keyframe = frame
            self.keyframe_kp = kp
            self.keyframe_des = des
            self.keyframe_pose = self.pose.copy()
            self.frames_since_keyframe = 0
            return
        
        # Match against keyframe
        matches = self.bf.match(self.keyframe_des, des)
        matches = sorted(matches, key=lambda x: x.distance)[:100]
        
        if len(matches) < 8:
            self.get_logger().warn('Not enough matches for pose estimation')
            return
        
        # Extract matched points
        pts1 = np.float32([self.keyframe_kp[m.queryIdx].pt for m in matches])
        pts2 = np.float32([kp[m.trainIdx].pt for m in matches])
        
        # Check if there's enough movement to warrant a pose update
        avg_displacement = np.mean(np.linalg.norm(pts2 - pts1, axis=1))
        if avg_displacement < self.min_pixel_displacement and self.frames_since_keyframe < self.max_frames_since_keyframe:
            # Not enough movement, skip pose update 
            return
        
        # Estimate essential matrix
        K = np.array([[self.fx, 0, self.cx],
                      [0, self.fy, self.cy],
                      [0, 0, 1]])
        
        E, mask = cv2.findEssentialMat(pts1, pts2, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
        
        if E is None:
            return
        
        # Recover rotation and translation from essential matrix
        _, R, t, mask = cv2.recoverPose(E, pts1, pts2, K)
        
        # Scale recovery from depth (if available)
        scale = 1.0
        if self.current_depth is not None:
            scale = self._estimate_scale(pts1, pts2, mask)
        
        # Build transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = (t * scale).flatten()
        
        # Update cumulative pose (relative to keyframe)
        self.pose = self.keyframe_pose @ np.linalg.inv(T)
        
        # Publish pose
        self._publish_pose(msg.header.stamp)
        
        # Publish map points
        if self.current_depth is not None:
            self._publish_map_points(pts2, self.current_depth, msg.header.stamp)
        
        # Decide if this frame should become the new keyframe
        translation_norm = np.linalg.norm(t * scale)
        rotation_angle = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)) * 180 / np.pi
        
        should_update_keyframe = (
            translation_norm > self.min_translation_m or
            rotation_angle > self.min_rotation_deg or
            self.frames_since_keyframe >= self.max_frames_since_keyframe
        )
        
        if should_update_keyframe:
            self.keyframe = frame
            self.keyframe_kp = kp
            self.keyframe_des = des
            self.keyframe_pose = self.pose.copy()
            self.frames_since_keyframe = 0
    
    def _estimate_scale(self, pts1, pts2, mask) -> float:
        if self.current_depth is None:
            return 1.0
        
        scales = []
        h, w = self.current_depth.shape
        
        for i, (p1, p2) in enumerate(zip(pts1, pts2)):
            if mask[i] == 0:
                continue
            
            x1, y1 = int(p1[0]), int(p1[1])
            if 0 <= x1 < w and 0 <= y1 < h:
                d = self.current_depth[y1, x1]
                if 0.2 < d < 20.0:  
                    # Compute 3D point in camera frame
                    Z1 = d
                    X1 = (p1[0] - self.cx) * Z1 / self.fx
                    Y1 = (p1[1] - self.cy) * Z1 / self.fy
                    
                    # For the matched point in frame 2, estimate its depth
                    x2, y2 = int(p2[0]), int(p2[1])
                    if 0 <= x2 < w and 0 <= y2 < h:
                        d2 = self.current_depth[y2, x2]
                        if 0.2 < d2 < 20.0:
                            Z2 = d2
                            X2 = (p2[0] - self.cx) * Z2 / self.fx
                            Y2 = (p2[1] - self.cy) * Z2 / self.fy
                            
                            # Euclidean distance between 3D points
                            dist_3d = np.sqrt((X2-X1)**2 + (Y2-Y1)**2 + (Z2-Z1)**2)
                            if dist_3d > 0.01:  # At least 1cm movement
                                scales.append(dist_3d)
        
        if len(scales) < 5:
            return 1.0
        
        # MAD-based outlier rejection 
        scales = np.array(scales)
        median = np.median(scales)
        mad = np.median(np.abs(scales - median))  # Median Absolute Deviation
        
        if mad < 1e-6:
            return float(median)
        
        # Keep only inliers within 2.5 MAD of median (~ 2 sigma for normal dist)
        threshold = 2.5 * mad * 1.4826  # 1.4826 makes MAD consistent with std dev
        inliers = scales[np.abs(scales - median) < threshold]
        
        if len(inliers) > 3:
            return float(np.mean(inliers))  # Mean of inliers is more accurate
        return float(median)
    
    def _publish_map_points(self, pts, depth_img, stamp):
        if depth_img is None:
            return

        points_3d = []
        h, w = depth_img.shape
        
        # Convert 2D keypoints to 3D points in Camera Frame
        for pt in pts:
            u, v = int(pt[0]), int(pt[1])
            if 0 <= u < w and 0 <= v < h:
                d = depth_img[v, u]
                if 0.2 < d < 10.0: # Valid depth range
                    # Back-project to 3D
                    z = float(d)
                    x = (u - self.cx) * z / self.fx
                    y = (v - self.cy) * z / self.fy
                    points_3d.append([x, y, z])

        if not points_3d:
            return

        # Create PointCloud2 message
        header = Header()
        header.stamp = stamp
        header.frame_id = "vslam_link" # Points are in camera frame, TF handles the rest
        
        pc_msg = point_cloud2.create_cloud_xyz32(header, points_3d)
        self.map_pub.publish(pc_msg)

    def _publish_pose(self, stamp):
        """Publish pose as PoseStamped and Odometry"""
        # Extract position and orientation from pose matrix
        position = self.pose[:3, 3]
        R = self.pose[:3, :3]
        
        # Convert rotation matrix to quaternion
        quat = self._rotation_matrix_to_quaternion(R)
        
        # PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose.position.x = float(position[0])
        pose_msg.pose.position.y = float(position[1])
        pose_msg.pose.position.z = float(position[2])
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        self.pose_pub.publish(pose_msg)
        
        # Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose = pose_msg.pose
        self.odom_pub.publish(odom_msg)
        
        # TF
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'vslam_link'
        t.transform.translation.x = float(position[0])
        t.transform.translation.y = float(position[1])
        t.transform.translation.z = float(position[2])
        t.transform.rotation = pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)
        
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'VSLAM pose: [{position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}]',
                throttle_duration_sec=2.0)
    
    def _rotation_matrix_to_quaternion(self, R):
        """Convert 3x3 rotation matrix to quaternion [x, y, z, w]"""
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return np.array([x, y, z, w])


def main(args=None):
    rclpy.init(args=args)
    node = VSLAMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

