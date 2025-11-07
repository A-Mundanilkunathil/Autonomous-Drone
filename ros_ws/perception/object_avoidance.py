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
import cv2

class ObjectAvoidanceNode(Node):
    def __init__(self):
        super().__init__('object_avoidance')
        self.bridge = CvBridge()

        # MiDaS depth conversion parameters
        # MiDaS outputs inverse depth (larger=closer, smaller=farther)
        # These parameters convert to approximate metric depth
        self.midas_scale = 50.0  # Scaling factor to convert to meters (increased for warehouse scale)
        self.midas_max_dist = 50.0  # Maximum distance to consider (meters)
        
        # Avoidance configurations
        self.stop_dist_m = 0.6  # Stop when VERY close (was 1.0m - too conservative)
        self.caution_dist_m = 2.0  # Start lateral avoidance (reduced from 2.5m)
        self.min_valid_depth = 0.15  # Filter only extreme noise (was 0.3m - too aggressive)
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
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth_map', qos_profile=qos_input)
        self.det_sub = message_filters.Subscriber(self, Detection2DArray, '/detected_objects', qos_profile=qos_input)
        ts = message_filters.ApproximateTimeSynchronizer([self.depth_sub, self.det_sub], 10, 0.1)
        ts.registerCallback(self.synced_callback)

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
        - Values are not in any specific unit
        
        We invert and scale to get approximate metric depth
        """
        # Avoid division by zero
        depth_img = np.clip(depth_img, 1e-6, None)
        
        # Invert: smaller MiDaS value (far) → larger depth (meters)
        # Scale to approximate metric range
        metric_depth = self.midas_scale / depth_img
        
        # Clip to maximum distance
        metric_depth = np.clip(metric_depth, 0, self.midas_max_dist)
        
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

    def _compose_cmd(self, forward_clear_m, median_left, median_right, median_top, median_bottom):
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
        if not np.isnan(median_left) or not np.isnan(median_right):
            L = -np.inf if np.isnan(median_left) else median_left
            R = -np.inf if np.isnan(median_right) else median_right
            
            if forward_clear_m <= self.stop_dist_m:
                # Forward is VERY blocked (≤0.6m) - aggressive lateral movement toward more open side
                if L > R:
                    vy = +self.max_side_speed
                elif R > L:
                    vy = -self.max_side_speed
                else:
                    vy = +self.max_side_speed  # Default to left if equal
            else:
                # Forward is in caution zone (0.6m < fc < 2.0m) - gentle lateral toward open side
                # Scale lateral speed with distance: closer = more lateral movement
                lateral_intensity = 1.0 - ((forward_clear_m - self.stop_dist_m) / (self.caution_dist_m - self.stop_dist_m))
                lateral_intensity = max(0.3, min(1.0, lateral_intensity))  # Clamp between 0.3 and 1.0
                
                if L > R:
                    vy = +lateral_intensity * self.max_side_speed
                elif R > L:
                    vy = -lateral_intensity * self.max_side_speed
                else:
                    vy = +lateral_intensity * self.max_side_speed  # Default to left if equal

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

    def synced_callback(self, depth_msg, det_msg):
        # Convert depth image from MiDaS inverse depth to metric depth
        depth_img_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        depth_img = self._convert_midas_depth(depth_img_raw)
        H, W = depth_img.shape[:2]
        
        # Filter out noise: set anything < min_valid_depth to inf (likely artifacts)
        depth_img[depth_img < self.min_valid_depth] = np.inf
        
        # Mask out top 30% of image where propellers appear
        propeller_mask_height = int(H * 0.30)
        depth_img[:propeller_mask_height, :] = np.inf  # Set propeller area to infinite distance
        
        # Mask out bottom 30% where ground appears (more aggressive - was 25%)
        ground_mask_height = int(H * 0.30)
        depth_img[H - ground_mask_height:, :] = np.inf  # Set ground area to infinite distance
        
        # Mask out left/right 25% edges (more aggressive - was 20%)
        edge_mask_width = int(W * 0.25)
        depth_img[:, :edge_mask_width] = np.inf  # Left edge
        depth_img[:, W - edge_mask_width:] = np.inf  # Right edge

        # ALWAYS analyze forward center region for obstacles (even without YOLO detections)
        # Focus TIGHTLY on the actual flight path: center 50% horizontally, middle 40% vertically
        center_h = int(H * 0.40)  # Middle 40% vertically (smaller window)
        center_w = int(W * 0.50)  # Center 50% horizontally (tighter focus)
        y1_center = (H - center_h) // 2
        y2_center = y1_center + center_h
        x1_center = (W - center_w) // 2
        x2_center = x1_center + center_w
        
        center_roi = depth_img[y1_center:y2_center, x1_center:x2_center]
        closest_forward = np.nanmin(center_roi) if np.isfinite(center_roi).any() else np.inf
        
        # Get sector scores for center region (for avoidance direction)
        mL, mR, mT, mB = self._sector_scores(depth_img, y1_center, y2_center, x1_center, x2_center)
        base_cmd = self._compose_cmd(closest_forward, mL, mR, mT, mB)

        # Defaults for detection refinement
        best_cmd = base_cmd  # Start with depth-only command
        best_min = closest_forward if np.isfinite(closest_forward) else np.inf

        # Refine with YOLO detections if available
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

            # Get closest point in this detection
            roi_min = np.nanmin(roi) if np.isfinite(roi).any() else np.nan
            
            # Update global closest if this detection is closer
            if np.isfinite(roi_min) and roi_min < closest_forward:
                closest_forward = roi_min

            # Sector distances for this detection
            mL, mR, mT, mB = self._sector_scores(depth_img, y1, y2, x1, x2)
            vy, vz = self._compose_cmd(roi_min, mL, mR, mT, mB)

            # Use command from tightest obstacle (closest detection)
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
        
        num_detections = len(det_msg.detections)
        if np.isinf(closest_forward):
            self.get_logger().info(
                f'Depth | Dets: {num_detections} | ROI raw_min={raw_min:.2f} | ROI(center 50x40%): min={roi_min:.2f}, mean={roi_mean:.2f}, max={roi_max:.2f} | '
                f'Forward clear: INF (all clear) | cmd vy={vy:.2f}, vz={vz:.2f}',
                throttle_duration_sec=2.0
            )
        else:
            self.get_logger().info(
                f'Depth | Dets: {num_detections} | ROI raw_min={raw_min:.2f} | ROI(center 50x40%): min={roi_min:.2f}, mean={roi_mean:.2f}, max={roi_max:.2f} | '
                f'Forward clear: {closest_forward:.2f}m | cmd vy={vy:.2f}, vz={vz:.2f}',
                throttle_duration_sec=2.0
            )

def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()