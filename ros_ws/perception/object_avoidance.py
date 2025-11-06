import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import numpy as np
import message_filters
from geometry_msgs.msg import TwistStamped

class ObjectAvoidanceNode(Node):
    def __init__(self):
        super().__init__('object_avoidance')
        self.bridge = CvBridge()

        # Avoidance configurations
        self.stop_dist_m = 1.0
        self.caution_dist_m = 2.5
        self.max_side_speed = 0.6
        self.max_forward_speed = 0.8
        self.min_roi_pixels = 150 # Ignore tiny ROIs
        self.smoothing = 0.6

        # QoS profiles
        qos_input = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        qos_output = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers (synchronized)
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth_map', qos_input)
        self.det_sub = message_filters.Subscriber(self, Detection2DArray, '/detected_objects', qos_input)
        ts = message_filters.ApproximateTimeSynchronizer([self.depth_sub, self.det_sub], 10, 0.1)
        ts.registerCallback(self.synced_callback)

        # Publishers - velocity command + forward clearance
        self.cmd_pub = self.create_publisher(TwistStamped, '/avoidance/cmd_vel', qos_output)
        self.clearance_pub = self.create_publisher(Float32, '/avoidance/forward_clearance', qos_output)

        # Last command for smoothing (EMA) 
        self._last_vy = 0.0
        self._last_vz = 0.0

        self.get_logger().info('Object avoidance node started')
    
    def _median_ignore_nan(self, a):
        if a.size == 0:
            return np.nan
        return np.nanmedian(a)
    
    def _sector_scores(self, depth_img, y1, y2, x1, x2):
        roi = depth_img[y1:y2, x1:x2]
        if roi.size < self.min_roi_pixels:
            return np.nan, np.nan, np.nan, np.nan
        
        H, W = roi.shape
        mid_y = H // 2
        mid_x = W // 2

        left = roi[:, :mid_x]
        right = roi[:, mid_x:]
        top = roi[:mid_y, :]
        bottom = roi[mid_y:, :]

        median_left = self._median_ignore_nan(left)
        median_right = self._median_ignore_nan(right)
        median_top = self._median_ignore_nan(top)
        median_bottom = self._median_ignore_nan(bottom)

        return median_left, median_right, median_top, median_bottom

    def _compose_cmd(self, forward_clear_m, median_left, median_right, median_top, median_bottom):
        """
        Decide body-frame lateral/vertical velocities (vy, vz) given sector distances.
        Note: vx is controlled by node_interface based on clearance, not by avoidance
        - vy toward the side with more space (left positive, right negative)
        - vz up is NEGATIVE in FLU convention; down is POSITIVE
        """
        # Lateral choice
        vy = 0.0
        if not np.isnan(median_left) or not np.isnan(median_right):
            L = -np.inf if np.isnan(median_left) else median_left
            R = -np.inf if np.isnan(median_right) else median_right
            if max(L, R) < self.caution_dist_m:
                # Both sides are tight -> choose the one with more space
                if L > R:
                    vy = +0.3 * self.max_side_speed
                elif R > L:
                    vy = -0.3 * self.max_side_speed
            else:
                # Pick side with more room, full lateral speed
                if L > R:
                    vy = +self.max_side_speed
                elif R > L:
                    vy = -self.max_side_speed
                else:
                    vy = 0.0

        # Vertical choice
        vz = 0.0
        if not np.isnan(median_top) or not np.isnan(median_bottom):
            T = -np.inf if np.isnan(median_top) else median_top
            B = -np.inf if np.isnan(median_bottom) else median_bottom
            if max(T, B) < self.caution_dist_m:
                # Both sides are tight -> choose the one with more space
                if T > B:
                    vz = -self.max_side_speed # Up
                elif B > T:
                    vz = +self.max_side_speed # Down
     
        # If forward is blocked hard, prioritize lateral movement
        if not np.isnan(forward_clear_m) and forward_clear_m <= self.stop_dist_m:
            if (not np.isnan(median_left)) and (not np.isnan(median_right)):
                    vy = +self.max_side_speed if median_left >= median_right else -self.max_side_speed
            elif not np.isnan(median_left):
                vy = +self.max_side_speed
            elif not np.isnan(median_right):
                vy = -self.max_side_speed

        # Smooth (EMA)
        alpha = self.smoothing
        vy = alpha * self._last_vy + (1 - alpha) * vy
        vz = alpha * self._last_vz + (1 - alpha) * vz
        self._last_vy, self._last_vz = vy, vz

        return vy, vz

    def synced_callback(self, depth_msg, det_msg):
        # Convert depth image
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        H, W = depth_img.shape[:2]

        # Defaults
        closest_forward = np.nan
        best_cmd = (0.0, 0.0)  # (vy, vz)
        best_min = np.inf  # Track the smallest roi_min seen

        # Process detections
        for det in det_msg.detections:
            x = int(round(det.bbox.center.x)) if hasattr(det.bbox.center, 'x') else int(round(det.bbox.center.position.x))
            y = int(round(det.bbox.center.y)) if hasattr(det.bbox.center, 'y') else int(round(det.bbox.center.position.y))
            w = int(det.bbox.size_x)
            h = int(det.bbox.size_y)

            # Region of interest
            y1 = max(0, y-h//2)
            y2 = min(H, y+h//2)
            x1 = max(0, x-w//2)
            x2 = min(W, x+w//2)

            roi = depth_img[y1:y2, x1:x2]
            if roi.size == 0:
                continue

            # Get closest forward
            roi_min = np.nanmin(roi) if np.isfinite(roi).any() else np.nan
            if np.isnan(closest_forward) or (np.isfinite(roi_min) and roi_min < closest_forward):
                closest_forward = roi_min

            # Sector distances
            mL, mR, mT, mB = self._sector_scores(depth_img, y1, y2, x1, x2)
            vy, vz = self._compose_cmd(closest_forward, mL, mR, mT, mB)

            # Choose the command associated with the tightest obstacle
            if np.isfinite(roi_min) and roi_min < best_min:
                best_min = roi_min
                best_cmd = (vy, vz)

        # Publish velocity command (only lateral/vertical, vx=0)
        vy, vz = best_cmd
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.0  # Forward controlled by node_interface
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.z = 0.0  # No yaw rotation
        self.cmd_pub.publish(msg)

        # Publish forward clearance on separate topic
        clearance_msg = Float32()
        clearance_msg.data = float(closest_forward) if np.isfinite(closest_forward) else float('inf')
        self.clearance_pub.publish(clearance_msg)

        if np.isnan(closest_forward):
            self.get_logger().info('Avoidance: unknown forward clearance, creeping')
        else:
            self.get_logger().info(f'Avoidance: forward_clear={closest_forward:.2f} m | cmd vy={vy:.2f}, vz={vz:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()