import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import TwistStamped
import cv2

class ObjectAvoidanceNode(Node):
    def __init__(self):
        super().__init__('object_avoidance')
        self.bridge = CvBridge()

        # MiDaS depth conversion parameters
        self.midas_scale = 200.0
        self.midas_max_dist = 50.0
        self.midas_shift = 0.0

        # MiDaS inverse depth calibration
        midas_calib_path = self.declare_parameter('midas_calib_npz', '').value
        if midas_calib_path:
            try:
                midas_calib = np.load(midas_calib_path)
                self.midas_scale = midas_calib['scale']
                self.midas_shift = midas_calib['shift']
                self.get_logger().info('Loaded MiDaS calibration file')
            except Exception as e:
                self.get_logger().error(f'Failed to load MiDaS calibration file: {e}')

        # Avoidance configurations
        self.stop_dist_m = 0.5
        self.caution_dist_m = 0.8
        self.max_side_speed = 0.6
        self.min_roi_pixels = 150
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

        # Subscribe to depth only - no object detector needed!
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth_map',
            self.depth_callback,
            qos_profile=qos_input
        )

        # Publishers
        self.cmd_pub = self.create_publisher(TwistStamped, '/avoidance/cmd_vel', qos_output)
        self.clearance_pub = self.create_publisher(Float32, '/avoidance/forward_clearance', qos_output)

        # Last command for smoothing (EMA) 
        self._last_vy = 0.0
        self._last_vz = 0.0

        self.get_logger().info('Object avoidance node started')
    
    def _convert_midas_depth(self, depth_img):
        """Convert MiDaS inverse depth to metric depth in meters"""
        # Mask out invalid depths
        ignore_mask = depth_img < 0.1

        # Prevent division by zero
        depth_img = np.clip(depth_img, 0.1, None)

        # Depth = scale / inv_depth
        metric_depth = self.midas_scale / depth_img

        if self.midas_shift != 0.0: 
            metric_depth += self.midas_shift

        # Mask ignored pixels as infinite depth
        metric_depth[ignore_mask] = np.inf
        metric_depth = np.where(np.isinf(metric_depth), np.inf, np.clip(metric_depth, 0, self.midas_max_dist))
        return metric_depth.astype(np.float32)
    
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

    def _compose_cmd(self, forward_clear_m, median_left, median_right, median_top, median_bottom, left_clearance=None, right_clearance=None):
        """Generate lateral/vertical avoidance velocities based on clearances"""
        vy = 0.0
        vz = 0.0
        
        if np.isnan(forward_clear_m) or forward_clear_m > self.caution_dist_m:
            self._last_vy = 0.0
            self._last_vz = 0.0
            return 0.0, 0.0
        
        # Lateral choice
        if left_clearance is not None and right_clearance is not None:
            L = left_clearance if np.isfinite(left_clearance) else -np.inf
            R = right_clearance if np.isfinite(right_clearance) else -np.inf
        elif not np.isnan(median_left) or not np.isnan(median_right):
            L = -np.inf if np.isnan(median_left) else median_left
            R = -np.inf if np.isnan(median_right) else median_right
        else:
            L = R = -np.inf
        
        if L > -np.inf or R > -np.inf:
            
            if forward_clear_m <= self.stop_dist_m:
                if L > R + 0.5:
                    vy = +self.max_side_speed
                elif R > L + 0.2:
                    vy = -self.max_side_speed
                else:
                    vy = -self.max_side_speed * 0.85
            else:
                lateral_intensity = 1.0 - ((forward_clear_m - self.stop_dist_m) / (self.caution_dist_m - self.stop_dist_m))
                lateral_intensity = max(0.6, min(1.0, lateral_intensity))
                
                if L > R + 0.5:
                    vy = +lateral_intensity * self.max_side_speed
                elif R > L + 0.2:
                    vy = -lateral_intensity * self.max_side_speed
                else:
                    vy = -lateral_intensity * self.max_side_speed * 0.85

        # Vertical choice
        if not np.isnan(median_top) or not np.isnan(median_bottom):
            T = -np.inf if np.isnan(median_top) else median_top
            B = -np.inf if np.isnan(median_bottom) else median_bottom
            
            if forward_clear_m <= self.stop_dist_m:
                if max(T, B) < self.caution_dist_m:
                    if T > B:
                        vz = -0.3 * self.max_side_speed
                    elif B > T:
                        vz = +0.3 * self.max_side_speed

        # Smooth (EMA)
        alpha = self.smoothing
        vy = alpha * self._last_vy + (1 - alpha) * vy
        vz = alpha * self._last_vz + (1 - alpha) * vz
        self._last_vy, self._last_vz = vy, vz

        return vy, vz

    def depth_callback(self, depth_msg):
        """Process depth image and generate avoidance commands"""
        depth_img_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        H, W = depth_img_raw.shape[:2]
        
        # Define ROI dimensions
        center_h = int(H * 0.40)
        center_w = int(W * 0.60)
        y1_center = (H - center_h) // 2
        y2_center = y1_center + center_h
        x1_center = (W - center_w) // 2
        x2_center = x1_center + center_w
        
        # Apply masks to raw image BEFORE conversion
        depth_img_raw_masked = depth_img_raw.copy()
        
        # Mask out top 30%
        propeller_mask_height = int(H * 0.30)
        depth_img_raw_masked[:propeller_mask_height, :] = 0.0
        
        # Mask out bottom 30%
        ground_mask_height = int(H * 0.30)
        depth_img_raw_masked[H - ground_mask_height:, :] = 0.0
        
        # Mask out left/right 25%
        edge_mask_width = int(W * 0.25)
        depth_img_raw_masked[:, :edge_mask_width] = 0.0
        depth_img_raw_masked[:, W - edge_mask_width:] = 0.0
        
        # Convert to metric depth
        depth_img = self._convert_midas_depth(depth_img_raw_masked)
        
        # Extract center ROI
        center_roi = depth_img[y1_center:y2_center, x1_center:x2_center]
        
        # Split into thirds
        roi_width = center_roi.shape[1]
        third_width = roi_width // 3
        
        left_roi = center_roi[:, :third_width]
        center_third_roi = center_roi[:, third_width:2*third_width]
        right_roi = center_roi[:, 2*third_width:]
        
        # Get clearance for each region
        left_roi_valid = left_roi[np.isfinite(left_roi)]
        center_third_roi_valid = center_third_roi[np.isfinite(center_third_roi)]
        right_roi_valid = right_roi[np.isfinite(right_roi)]
        
        left_clearance = np.percentile(left_roi_valid, 20) if left_roi_valid.size > 0 else np.inf
        center_clearance = np.percentile(center_third_roi_valid, 20) if center_third_roi_valid.size > 0 else np.inf
        right_clearance = np.percentile(right_roi_valid, 20) if right_roi_valid.size > 0 else np.inf
        
        closest_forward = center_clearance
        
        # Check if we can navigate around obstacle
        if center_clearance < self.caution_dist_m:
            if left_clearance > center_clearance * 1.5 and left_clearance > self.caution_dist_m:
                closest_forward = min(left_clearance, self.caution_dist_m * 2.0)
            elif right_clearance > center_clearance * 1.5 and right_clearance > self.caution_dist_m:
                closest_forward = min(right_clearance, self.caution_dist_m * 2.0)
        
        # Get sector scores
        mL, mR, mT, mB = self._sector_scores(depth_img, y1_center, y2_center, x1_center, x2_center)
        
        # Generate avoidance command
        vy, vz = self._compose_cmd(closest_forward, mL, mR, mT, mB, left_clearance, right_clearance)

        # Publish velocity command
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.z = 0.0
        self.cmd_pub.publish(msg)

        # Publish forward clearance
        clearance_msg = Float32()
        clearance_msg.data = float(closest_forward) if np.isfinite(closest_forward) else float('inf')
        self.clearance_pub.publish(clearance_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()