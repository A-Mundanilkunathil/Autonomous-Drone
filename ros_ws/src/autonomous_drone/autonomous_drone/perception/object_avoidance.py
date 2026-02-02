import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import TwistStamped
import cv2
import time

class ObjectAvoidanceNode(Node):
    def __init__(self):
        super().__init__('object_avoidance')
        self.bridge = CvBridge()
        self.max_dist_m = 50.0  # meters

        # Avoidance configurations
        self.stop_dist_m = 0.5
        self.caution_dist_m = 0.8
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

        # Subscribe to depth only
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
        self._last_vx = 0.0
        self._last_wz = 0.0
        self._last_vz = 0.0
        
        # Speed limits
        self.max_forward_speed = 0.5
        self.max_yaw_rate = 0.5  # rad/s

        # Timer-based control
        self._latest_depth_msg = None
        self._last_depth_received_time = 0.0
        self._depth_timeout = 0.5
        self._rate_hz = 20.0
        self._timer = self.create_timer(1.0 / self._rate_hz, self._on_timer)

        self.get_logger().info('Object avoidance node started')
    
    def _validate_depth(self, depth_img):
        """Clamp depth to valid range and mask out invalid values."""
        # Mask out invalid depths (too close or invalid)
        ignore_mask = depth_img < 0.1
        
        # Clamp to valid range
        metric_depth = np.clip(depth_img, 0.1, self.max_dist_m)
        
        # Mask ignored pixels as infinite depth
        metric_depth[ignore_mask] = np.inf
        
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
        """Generate forward/vertical/yaw avoidance velocities based on clearances"""
        vx = 0.0
        vz = 0.0
        wz = 0.0
        
        if np.isnan(forward_clear_m) or forward_clear_m > self.caution_dist_m: 
            self._last_vx = 0.0
            self._last_vz = 0.0
            self._last_wz = 0.0
            return 0.0, 0.0, 0.0
                
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
                    wz = +self.max_yaw_rate * 0.85  # Turn Left 
                elif R > L + 0.2:
                    wz = -self.max_yaw_rate * 0.85  # Turn Right 
                else:
                    wz = 0.0
            else:
                yaw_intensity = 1.0 - ((forward_clear_m - self.stop_dist_m) / (self.caution_dist_m - self.stop_dist_m))
                yaw_intensity = max(0.0, min(1.0, yaw_intensity))

                if L > R + 0.5:
                    wz = +self.max_yaw_rate * yaw_intensity  # Turn Left
                elif R > L + 0.2:
                    wz = -self.max_yaw_rate * yaw_intensity  # Turn Right
                else:
                    wz = 0.0

        # Vertical choice
        if not np.isnan(median_top) or not np.isnan(median_bottom):
            T = -np.inf if np.isnan(median_top) else median_top
            B = -np.inf if np.isnan(median_bottom) else median_bottom
            
            if forward_clear_m <= self.stop_dist_m:
                if max(T, B) < self.caution_dist_m:
                    if T >= B:
                        vz = +0.3  # Up
                    elif B > T:
                        vz = -0.3  # Down

        # Smooth (EMA)
        alpha = self.smoothing
        vx = alpha * self._last_vx + (1 - alpha) * vx
        vz = alpha * self._last_vz + (1 - alpha) * vz
        wz = alpha * self._last_wz + (1 - alpha) * wz
        
        self._last_vx = vx
        self._last_vz = vz
        self._last_wz = wz

        return vx, vz, wz

    def depth_callback(self, depth_msg):
        """Store latest depth map."""
        self._latest_depth_msg = depth_msg
        self._last_depth_received_time = time.time()

    def _on_timer(self):
        """Process depth and publish commands at fixed rate."""
        if self._latest_depth_msg is None:
            return

        # Check for timeout
        if time.time() - self._last_depth_received_time > self._depth_timeout:
            # Publish stop command
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            self.cmd_pub.publish(msg)
            
            # Publish danger clearance to force stop
            clearance_msg = Float32()
            clearance_msg.data = float('inf')
            self.clearance_pub.publish(clearance_msg)
            return

        try:
            depth_img_raw = self.bridge.imgmsg_to_cv2(self._latest_depth_msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().warn(f'CV Bridge error: {e}')
            return

        H, W = depth_img_raw.shape[:2]
        
        # Define ROI dimensions
        center_h = int(H * 0.25) # 25% of image height
        center_w = int(W * 0.60) # 60% of image width
        y1_center = int(H * 0.20) # Start at 20% from top
        y2_center = y1_center + center_h
        x1_center = (W - center_w) // 2
        x2_center = x1_center + center_w
        
        # Apply masks to raw image before conversion
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
        
        depth_img = self._validate_depth(depth_img_raw_masked)
        
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
        vx, vz, wz = self._compose_cmd(closest_forward, mL, mR, mT, mB, left_clearance, right_clearance)

        # Publish velocity command
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = float(vz)
        msg.twist.angular.z = float(wz)
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