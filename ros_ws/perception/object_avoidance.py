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
        # MiDaS outputs inverse depth (larger=closer, smaller=farther)
        # Raw MiDaS values for warehouse: ~10-50 (far), ~100-300 (close poles)
        # Scale chosen so: MiDaS=100 → 200/100 = 2m, MiDaS=20 → 200/20 = 10m
        self.midas_scale = 200.0  # Tuned for raw MiDaS warehouse values
        self.midas_max_dist = 50.0  # Maximum distance to consider (meters)
        
        # Avoidance configurations
        self.stop_dist_m = 0.8  # Stop when close (increased for safety with drone width)
        self.caution_dist_m = 1.5  # Start lateral avoidance earlier (increased for safety)
        self.max_side_speed = 0.6
        self.max_forward_speed = 0.8
        self.min_roi_pixels = 150 # Ignore tiny ROIs
        self.smoothing = 0.6
        
        # Drone physical dimensions for collision avoidance
        # Iris quadcopter: ~0.45m diagonal (motor to motor)
        # Use conservative estimate for safety margin
        self.drone_width_m = 0.6  # Half-width from center to edge (with safety margin)
        self.safety_margin_m = 0.3  # Extra clearance for safe navigation

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

        # Publishers - velocity command + forward clearance
        self.cmd_pub = self.create_publisher(TwistStamped, '/avoidance/cmd_vel', qos_output)
        self.clearance_pub = self.create_publisher(Float32, '/avoidance/forward_clearance', qos_output)
        self.debug_img_pub = self.create_publisher(Image, '/avoidance/debug_img', qos_output)

        # Last command for smoothing (EMA) 
        self._last_vy = 0.0
        self._last_vz = 0.0

        self.get_logger().info('Object avoidance node started')
    
    def _convert_midas_depth(self, depth_img):
        """
        Convert MiDaS inverse/relative depth to approximate metric depth in meters
        
        MiDaS outputs inverse depth where:
        - Larger values = closer objects
        - Smaller values = farther objects
        - Raw values are relative but consistent within scene
        
        We invert and scale to get approximate metric depth
        """
        # Create mask for pixels that should be ignored (zeros from masking)
        ignore_mask = depth_img < 0.1
        
        # Avoid division by zero - clip to small positive value
        depth_img = np.clip(depth_img, 0.1, None)
        
        # Invert: smaller MiDaS value (far) → larger depth (meters)
        # Scale=200 tuned for raw MiDaS warehouse values:
        # - MiDaS=10 (far) → depth=200/10=20m
        # - MiDaS=20 → depth=200/20=10m
        # - MiDaS=50 → depth=200/50=4m
        # - MiDaS=100 (mid) → depth=200/100=2m
        # - MiDaS=200 (close) → depth=200/200=1m
        # - MiDaS=400 (very close) → depth=200/400=0.5m
        metric_depth = self.midas_scale / depth_img
        
        # Set ignored pixels to infinity
        metric_depth[ignore_mask] = np.inf
        
        # Clip to maximum distance (but keep inf values)
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
        """
        Decide body-frame lateral/vertical velocities (vy, vz) given sector distances.
        Note: vx is controlled by node_interface based on clearance, not by avoidance
        - vy toward the side with more space (left positive, right negative)
        - vz up is NEGATIVE in FLU convention; down is POSITIVE
        
        KEY LOGIC: Only avoid if forward path is actually blocked!
        """
        vy = 0.0
        vz = 0.0
        
        # CRITICAL: Only generate avoidance commands if forward clearance is actually tight
        # If forward is clear (>caution distance), keep flying straight (vy=0, vz=0)
        if np.isnan(forward_clear_m) or forward_clear_m > self.caution_dist_m:
            # Forward path is clear - no avoidance needed
            self._last_vy = 0.0
            self._last_vz = 0.0
            return 0.0, 0.0
        
        # Forward is getting tight - now check lateral/vertical options
        # Lateral choice (only if forward clearance < caution distance)
        # Use left/right clearance if provided, otherwise use median sectors
        if left_clearance is not None and right_clearance is not None:
            # Use direct left/right clearance measurements
            L = left_clearance if np.isfinite(left_clearance) else -np.inf
            R = right_clearance if np.isfinite(right_clearance) else -np.inf
        elif not np.isnan(median_left) or not np.isnan(median_right):
            L = -np.inf if np.isnan(median_left) else median_left
            R = -np.inf if np.isnan(median_right) else median_right
        else:
            L = R = -np.inf
        
        if L > -np.inf or R > -np.inf:
            
            if forward_clear_m <= self.stop_dist_m:
                # Forward is VERY blocked (≤0.8m) - aggressive lateral movement toward more open side
                if L > R + 0.5:  # Left is significantly clearer (with 0.5m bias threshold - increased from 0.3m)
                    vy = +self.max_side_speed
                elif R > L + 0.2:  # Right needs less advantage (0.2m) to prefer it - AGGRESSIVE right bias
                    vy = -self.max_side_speed
                else:
                    # Similar clearance - strongly prefer RIGHT to avoid left-wall hugging
                    vy = -self.max_side_speed * 0.85  # Move right at 85% speed (increased from 70%)
            else:
                # Forward is in caution zone (0.8m < fc < 1.5m) - scale lateral with distance
                # Closer = more lateral, farther = less lateral
                lateral_intensity = 1.0 - ((forward_clear_m - self.stop_dist_m) / (self.caution_dist_m - self.stop_dist_m))
                lateral_intensity = max(0.6, min(1.0, lateral_intensity))  # Increased from 0.4 to 0.6 for more aggressive lateral
                
                if L > R + 0.5:  # Left needs more advantage (0.5m) to prefer it
                    vy = +lateral_intensity * self.max_side_speed
                elif R > L + 0.2:  # Right needs less advantage (0.2m) - AGGRESSIVE right bias
                    vy = -lateral_intensity * self.max_side_speed
                else:
                    # Similar clearance - strongly prefer RIGHT to avoid left-wall hugging
                    vy = -lateral_intensity * self.max_side_speed * 0.85  # Move right at 85% speed (increased from 70%)

        # Vertical choice (only if forward clearance < caution distance)
        if not np.isnan(median_top) or not np.isnan(median_bottom):
            T = -np.inf if np.isnan(median_top) else median_top
            B = -np.inf if np.isnan(median_bottom) else median_bottom
            
            if forward_clear_m <= self.stop_dist_m:
                # Forward is VERY blocked - consider vertical movement
                if max(T, B) < self.caution_dist_m:
                    # Both vertical directions tight - pick best one
                    if T > B:
                        vz = -0.3 * self.max_side_speed  # Up
                    elif B > T:
                        vz = +0.3 * self.max_side_speed  # Down

        # Smooth (EMA)
        alpha = self.smoothing
        vy = alpha * self._last_vy + (1 - alpha) * vy
        vz = alpha * self._last_vz + (1 - alpha) * vz
        self._last_vy, self._last_vz = vy, vz

        return vy, vz

    def depth_callback(self, depth_msg):
        """Process depth image and generate avoidance commands - no object detector needed!"""
        # Convert depth image from MiDaS inverse depth to metric depth
        depth_img_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        H, W = depth_img_raw.shape[:2]
        
        # Define ROI dimensions accounting for drone physical width
        # The ROI should be wider than just center to check for obstacles
        # that would hit the drone's edges during forward/lateral movement
        center_h = int(H * 0.40)  # Vertical: 40% (middle section, avoiding ground/propellers)
        center_w = int(W * 0.60)  # Horizontal: 60% (wider to account for drone width, but not too wide)
        y1_center = (H - center_h) // 2
        y2_center = y1_center + center_h
        x1_center = (W - center_w) // 2
        x2_center = x1_center + center_w
        
        # DIAGNOSTIC: Check raw MiDaS values in center BEFORE any masking
        center_roi_raw_unmasked = depth_img_raw[y1_center:y2_center, x1_center:x2_center]
        center_roi_raw_valid = center_roi_raw_unmasked[center_roi_raw_unmasked > 1e-6]
        
        if center_roi_raw_valid.size > 0:
            raw_p10 = np.percentile(center_roi_raw_valid, 10)
            raw_p50 = np.percentile(center_roi_raw_valid, 50)
            raw_min = np.min(center_roi_raw_valid)
            raw_max = np.max(center_roi_raw_valid)
            self.get_logger().info(
                f'RAW MiDaS center ROI: min={raw_min:.1f}, p10={raw_p10:.1f}, p50={raw_p50:.1f}, max={raw_max:.1f} | '
                f'Converted: p10={200.0/raw_p10:.2f}m, p50={200.0/raw_p50:.2f}m',
                throttle_duration_sec=2.0
            )
        
        # Apply masks to raw image BEFORE conversion (set to 0)
        depth_img_raw_masked = depth_img_raw.copy()
        
        # Mask out top 30% where propellers appear
        propeller_mask_height = int(H * 0.30)
        depth_img_raw_masked[:propeller_mask_height, :] = 0.0
        
        # Mask out bottom 30% where ground appears
        ground_mask_height = int(H * 0.30)
        depth_img_raw_masked[H - ground_mask_height:, :] = 0.0
        
        # Mask out left/right 25% edges
        edge_mask_width = int(W * 0.25)
        depth_img_raw_masked[:, :edge_mask_width] = 0.0
        depth_img_raw_masked[:, W - edge_mask_width:] = 0.0
        
        # Convert masked raw image to metric depth (zeros become inf)
        depth_img = self._convert_midas_depth(depth_img_raw_masked)
        
        # Extract center ROI from converted depth
        center_roi = depth_img[y1_center:y2_center, x1_center:x2_center]
        
        # Split center ROI into left/center/right thirds to check lateral clearance
        roi_width = center_roi.shape[1]
        third_width = roi_width // 3
        
        left_roi = center_roi[:, :third_width]  # Left third
        center_third_roi = center_roi[:, third_width:2*third_width]  # Center third  
        right_roi = center_roi[:, 2*third_width:]  # Right third
        
        # Get clearance for each region
        left_roi_valid = left_roi[np.isfinite(left_roi)]
        center_third_roi_valid = center_third_roi[np.isfinite(center_third_roi)]
        right_roi_valid = right_roi[np.isfinite(right_roi)]
        
        # Use 20th percentile for each region
        left_clearance = np.percentile(left_roi_valid, 20) if left_roi_valid.size > 0 else np.inf
        center_clearance = np.percentile(center_third_roi_valid, 20) if center_third_roi_valid.size > 0 else np.inf
        right_clearance = np.percentile(right_roi_valid, 20) if right_roi_valid.size > 0 else np.inf
        
        # KEY FIX: Use CENTER clearance for forward movement
        # This allows forward progress even when sides show obstacles
        closest_forward = center_clearance
        
        # Check if we can navigate around obstacle
        # If center is blocked but one side is clear, allow forward movement toward clear side
        if center_clearance < self.caution_dist_m:
            # Center has obstacle - check if we can go around
            if left_clearance > center_clearance * 1.5 and left_clearance > self.caution_dist_m:
                # Left side is clear - use it for forward clearance
                closest_forward = min(left_clearance, self.caution_dist_m * 2.0)
                self.get_logger().info(
                    f'Navigate LEFT: L={left_clearance:.2f}m C={center_clearance:.2f}m R={right_clearance:.2f}m | using={closest_forward:.2f}m',
                    throttle_duration_sec=2.0
                )
            elif right_clearance > center_clearance * 1.5 and right_clearance > self.caution_dist_m:
                # Right side is clear - use it for forward clearance
                closest_forward = min(right_clearance, self.caution_dist_m * 2.0)
                self.get_logger().info(
                    f'Navigate RIGHT: L={left_clearance:.2f}m C={center_clearance:.2f}m R={right_clearance:.2f}m | using={closest_forward:.2f}m',
                    throttle_duration_sec=2.0
                )
        
        # Get sector scores for center region (for avoidance direction)
        mL, mR, mT, mB = self._sector_scores(depth_img, y1_center, y2_center, x1_center, x2_center)
        
        # Generate avoidance command using depth-based analysis only
        vy, vz = self._compose_cmd(closest_forward, mL, mR, mT, mB, left_clearance, right_clearance)

        # Publish velocity command (only lateral/vertical, vx=0)
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

        # Publish debug visualization
        debug_img = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        debug_img = cv2.applyColorMap(debug_img, cv2.COLORMAP_JET)
        # Draw ROI rectangle
        cv2.rectangle(debug_img, (x1_center, y1_center), (x2_center, y2_center), (0, 255, 0), 2)
        # Draw text with clearance
        text = f"Clear: {closest_forward:.2f}m" if np.isfinite(closest_forward) else "Clear: INF"
        cv2.putText(debug_img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
        self.debug_img_pub.publish(debug_msg)

        # Log depth statistics for debugging
        depth_min = np.nanmin(depth_img) if np.isfinite(depth_img).any() else np.nan
        depth_max = np.nanmax(depth_img) if np.isfinite(depth_img).any() else np.nan
        depth_mean = np.nanmean(depth_img) if np.isfinite(depth_img).any() else np.nan
        
        # Log center ROI statistics including raw depth before filtering
        depth_img_unfiltered = self._convert_midas_depth(depth_img_raw)
        center_roi_raw = depth_img_unfiltered[y1_center:y2_center, x1_center:x2_center]
        raw_min = np.nanmin(center_roi_raw) if np.isfinite(center_roi_raw).any() else np.nan
        
        center_roi_valid = center_roi[np.isfinite(center_roi)]
        if center_roi_valid.size > 0:
            roi_min = np.min(center_roi_valid)
            roi_mean = np.mean(center_roi_valid)
            roi_max = np.max(center_roi_valid)
        else:
            roi_min = roi_mean = roi_max = np.nan
        
        # Enhanced logging with more detail
        valid_pixels = center_roi_valid.size if 'center_roi_valid' in locals() else 0
        if np.isinf(closest_forward):
            self.get_logger().info(
                f'Depth-only | L={left_clearance:.2f}m C={center_clearance:.2f}m R={right_clearance:.2f}m | '
                f'ROI: min={roi_min:.2f}, p20={roi_min:.2f}, mean={roi_mean:.2f} | '
                f'Forward clear: INF | cmd vy={vy:.2f}, vz={vz:.2f}',
                throttle_duration_sec=2.0
            )
        else:
            self.get_logger().info(
                f'Depth-only | L={left_clearance:.2f}m C={center_clearance:.2f}m R={right_clearance:.2f}m | '
                f'ROI: min={roi_min:.2f}, p20={closest_forward:.2f}, mean={roi_mean:.2f} | '
                f'Forward clear: {closest_forward:.2f}m | cmd vy={vy:.2f}, vz={vz:.2f}',
                throttle_duration_sec=1.0
            )

def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()