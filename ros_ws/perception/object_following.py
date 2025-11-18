import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import math
import numpy as np

class ObjectFollowingNode(Node):
    def __init__(self):
        super().__init__('object_following')
        self.bridge = CvBridge()

        # Following parameters
        self.follow_target_label = 'car'
        self.follow_desired_area = 0.06  # ~3-4m with area-based fallback
        self.follow_desired_distance = 3.0  # 3m target distance for depth-based control
        self.follow_area_band = 0.02
        self.k_vx = 1.2  # Proportional gain for distance
        self.k_vx_d = 0.3  # Derivative gain for distance (velocity prediction)
        self.k_vy = 0.8
        self.k_vz = 0.8
        self.k_yaw = 35.0
        self.follow_vx_cap = 1.2
        self.follow_vy_cap = 0.7
        self.follow_vz_cap = 0.6
        self.follow_yaw_cap = 80.0
        self.follow_lost_timeout = 10.0  # Increased for slow YOLOE inference (~7s per frame)
        self.follow_min_vx = 0.05
        
        self.img_width = 640
        self.img_height = 480
        
        # MiDaS depth calibration (convert inverse depth → metric depth)
        self.midas_scale = 200.0  # Default if no calibration file (matches avoidance)
        self.midas_shift = 0.0
        self.midas_max_dist = 50.0  # Max valid distance in meters
        
        # Load MiDaS calibration if provided
        midas_calib_path = self.declare_parameter('midas_calib_npz', '').value
        if midas_calib_path:
            try:
                midas_calib = np.load(midas_calib_path)
                self.midas_scale = float(midas_calib['scale'])
                self.midas_shift = float(midas_calib['shift'])
                self.get_logger().info(f'Loaded MiDaS calibration: scale={self.midas_scale:.3f}, shift={self.midas_shift:.3f}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load MiDaS calibration: {e}. Using defaults.')

        # Detection state
        self._last_det_time = None  # Use None to indicate no detection yet
        self._last_det_bbox = None
        self._last_det_score = 0.0
        
        # Depth-based distance tracking for derivative control
        self._latest_depth_map = None
        self._depth_timestamp = 0.0
        self._prev_distance = None
        self._prev_distance_time = None
        
        # Dynamic setpoint (adjusted based on clearance)
        self._forward_clearance = float('inf')
        self._clearance_timestamp = 0.0

        # Fixed rate timer for publishing
        self._rate_hz = 20.0
        self._timer = self.create_timer(1.0 / self._rate_hz, self._on_timer)

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

        # Subscribe to detections
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detected_objects',
            self._detection_callback,
            qos_profile=qos_input
        )
        
        # Subscribe to depth map for distance estimation
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth_map',
            self._depth_callback,
            qos_profile=qos_input
        )
        
        # Subscribe to forward clearance for dynamic setpoint
        self.clearance_sub = self.create_subscription(
            Float32,
            '/avoidance/forward_clearance',
            self._clearance_callback,
            qos_profile=qos_input
        )

        # Publish following commands
        self.cmd_pub = self.create_publisher(
            TwistStamped, 
            '/following/cmd_vel', 
            qos_output
        )
        self.target_found_pub = self.create_publisher(
            Bool, 
            '/following/target_found', 
            qos_output
        )

        self.get_logger().info(f'Object following node started. Target: {self.follow_target_label}')

    def _depth_callback(self, msg: Image):
        """Store latest depth map for distance estimation"""
        try:
            self._latest_depth_map = msg
            self._depth_timestamp = time.monotonic()
        except Exception as e:
            self.get_logger().warn(f'Depth callback error: {e}')
    
    def _clearance_callback(self, msg: Float32):
        """Store forward clearance for dynamic setpoint adjustment"""
        try:
            fc = float(msg.data)
            if not (math.isnan(fc) or math.isinf(fc)):
                self._forward_clearance = fc
                self._clearance_timestamp = time.monotonic()
        except Exception as e:
            self.get_logger().warn(f'Clearance callback error: {e}')

    def _detection_callback(self, msg: Detection2DArray):
        # Find best matching target
        best = None
        best_score = -1.0
        num_detections = len(msg.detections)
        matching_targets = []

        for det in msg.detections:
            if not det.results:
                continue
            
            hyp = det.results[0].hypothesis
            cls = getattr(hyp, "class_id", "")
            score = float(getattr(hyp, "score", 0.0))
            
            # Log all detections for debugging
            if cls:
                matching_targets.append(f"{cls}({score:.2f})")
            
            if cls == self.follow_target_label and score > best_score:
                cx = det.bbox.center.position.x
                cy = det.bbox.center.position.y
                bw = det.bbox.size_x
                bh = det.bbox.size_y

                best = (cx, cy, bw, bh)
                best_score = score

        # Update tracking state
        if best is not None:
            self._last_det_bbox = best
            self._last_det_score = best_score
            self._last_det_time = time.monotonic()
            
            # Log successful detection occasionally
            if not hasattr(self, '_last_det_log_time') or (time.monotonic() - self._last_det_log_time) > 2.0:
                self.get_logger().info(f'Target "{self.follow_target_label}" found! Score: {best_score:.2f}')
                self._last_det_log_time = time.monotonic()
        else:
            # Log when target is not found (but not too frequently)
            if not hasattr(self, '_last_miss_log_time') or (time.monotonic() - self._last_miss_log_time) > 2.0:
                detected_str = ', '.join(matching_targets) if matching_targets else 'none'
                self.get_logger().warn(
                    f'Target "{self.follow_target_label}" not found in {num_detections} detections. '
                    f'Detected: [{detected_str}]'
                )
                self._last_miss_log_time = time.monotonic()

    def _has_fresh_detection(self) -> bool:
        if self._last_det_bbox is None or self._last_det_time is None:
            return False
        return (time.monotonic() - self._last_det_time) < self.follow_lost_timeout

    def _get_follow_measurements(self):
        if self._last_det_bbox is None:
            return None
        
        cx, cy, bw, bh = self._last_det_bbox

        # Normalize to [-1, 1]
        ex = (cx - self.img_width * 0.5) / max(1e-3, (self.img_width * 0.5))  
        ey = (cy - self.img_height * 0.5) / max(1e-3, (self.img_height * 0.5))   
        area_frac = (bw * bh) / float(self.img_width * self.img_height + 1e-6)
        
        return float(ex), float(ey), float(area_frac)

    def _saturate(self, x, lo, hi):
        return max(lo, min(x, hi))
    
    def _get_depth_at_bbox(self, cx, cy, bw, bh) -> float:
        """Extract depth value at the center of the bounding box"""
        if self._latest_depth_map is None:
            return None
        if (time.monotonic() - self._depth_timestamp) > 0.5:
            return None  # Depth too old
            
        try:
            # Decode depth image using CvBridge (matches avoidance)
            inverse_depth_array = self.bridge.imgmsg_to_cv2(self._latest_depth_map, desired_encoding='32FC1')
            height, width = inverse_depth_array.shape[:2]
            
            # Get bbox center pixel coordinates
            center_x = int((cx / self.img_width) * width)
            center_y = int((cy / self.img_height) * height)
            
            # Clamp to valid range
            center_x = max(0, min(width - 1, center_x))
            center_y = max(0, min(height - 1, center_y))
            
            # Extract inverse depth at center (average 5x5 region for robustness)
            y_start = max(0, center_y - 2)
            y_end = min(height, center_y + 3)
            x_start = max(0, center_x - 2)
            x_end = min(width, center_x + 3)
            
            inv_depth_region = inverse_depth_array[y_start:y_end, x_start:x_end]
            
            # Filter out invalid inverse depths
            valid_inv_depths = inv_depth_region[(inv_depth_region > 0.1) & 
                                                ~np.isnan(inv_depth_region) & 
                                                ~np.isinf(inv_depth_region)]
            
            if len(valid_inv_depths) > 0:
                # Use median inverse depth for robustness
                median_inv_depth = float(np.median(valid_inv_depths))
                
                # Convert inverse depth to metric depth: depth = scale / inv_depth
                metric_depth = self.midas_scale / median_inv_depth
                
                # Apply shift if configured (matches avoidance logic)
                if self.midas_shift != 0.0:
                    metric_depth += self.midas_shift
                
                # Sanity check: reject unrealistic distances
                if 0.3 < metric_depth < self.midas_max_dist:
                    return metric_depth
                else:
                    return None
            else:
                return None
        except Exception as e:
            self.get_logger().warn(f'Depth extraction error: {e}')
            return None
    
    def _get_dynamic_setpoint(self) -> float:
        """Calculate dynamic distance setpoint based on forward clearance"""
        # If clearance is fresh (< 0.5s old), use it to adjust setpoint
        if (time.monotonic() - self._clearance_timestamp) < 0.5:
            clearance = self._forward_clearance
            
            # In tight spaces (< 2m clearance), maintain closer distance
            if clearance < 2.0:
                return max(1.5, clearance * 0.6)  # 60% of clearance, min 1.5m
            # In open spaces (> 5m clearance), can maintain farther distance
            elif clearance > 5.0:
                return 4.0  # Max 4m
            # Normal range (2-5m clearance): scale linearly
            else:
                return 2.0 + (clearance - 2.0) * (2.0 / 3.0)  # 2m to 4m
        
        # No fresh clearance data, use default
        return self.follow_desired_distance

    def _compute_follow_cmd(self):
        if not self._has_fresh_detection():
            # Log target lost occasionally
            if not hasattr(self, '_last_lost_log_time') or (time.monotonic() - self._last_lost_log_time) > 2.0:
                if self._last_det_time is not None:
                    time_since_last = time.monotonic() - self._last_det_time
                    self.get_logger().warn(
                        f'Target lost! Last seen {time_since_last:.2f}s ago (timeout: {self.follow_lost_timeout}s)'
                    )
                else:
                    self.get_logger().warn('Target lost! No detections received yet.')
                self._last_lost_log_time = time.monotonic()
            
            # Reset derivative tracking when target is lost
            self._prev_distance = None
            self._prev_distance_time = None
            return False, 0.0, 0.0, 0.0, 0.0
        
        meas = self._get_follow_measurements()
        if meas is None:
            return False, 0.0, 0.0, 0.0, 0.0

        ex, ey, area = meas
        
        # Try to get depth-based distance for more accurate control
        cx, cy, bw, bh = self._last_det_bbox
        depth_distance = self._get_depth_at_bbox(cx, cy, bw, bh)
        
        # Get dynamic setpoint based on environment
        desired_distance = self._get_dynamic_setpoint()

        # Forward control: Use depth fusion + velocity prediction (PD control)
        current_time = time.monotonic()
        
        if depth_distance is not None and depth_distance > 0.1:
            # Depth-based control (more accurate)
            distance_error = depth_distance - desired_distance
            
            # Calculate derivative term (velocity prediction)
            derivative_term = 0.0
            if self._prev_distance is not None and self._prev_distance_time is not None:
                dt = current_time - self._prev_distance_time
                if dt > 0.01:  # Avoid division by very small dt
                    distance_rate = (depth_distance - self._prev_distance) / dt
                    derivative_term = self.k_vx_d * distance_rate
            
            # PD controller: vx = -k_p * error - k_d * derivative
            # Negative because: positive error = too far, need positive vx (forward)
            # But distance_rate is already signed (positive = moving away)
            vx = -self.k_vx * distance_error - derivative_term
            
            # Update previous measurements
            self._prev_distance = depth_distance
            self._prev_distance_time = current_time
            
            # Log occasionally for debugging
            if not hasattr(self, '_last_log_time') or (current_time - self._last_log_time) > 2.0:
                self.get_logger().info(
                    f'Following: depth={depth_distance:.2f}m, target={desired_distance:.1f}m, '
                    f'error={distance_error:.2f}m, vx={vx:.2f}m/s (PD)'
                )
                self._last_log_time = current_time
        else:
            # Fall back to area-based control when depth unavailable
            # Scale desired area based on dynamic setpoint
            # If desired_distance increases, desired_area should decrease (farther = smaller)
            area_scale = (self.follow_desired_distance / desired_distance) ** 2
            scaled_desired_area = self.follow_desired_area * area_scale
            
            area_err = scaled_desired_area - area
            vx = self.k_vx * area_err
            
            # Reset derivative tracking
            self._prev_distance = None
            self._prev_distance_time = None
        
        # Prevent stalling
        if vx > 0.0:
            vx = max(vx, self.follow_min_vx)

        # Lateral: center horizontally
        vy = -self.k_vy * ex
        # Vertical: maintain altitude alignment
        vz = +self.k_vz * ey
        # Yaw: face target
        yaw_rate = self.k_yaw * ex

        # Deadband to avoid jitter (only for area-based control)
        if depth_distance is None:
            area_err = (self.follow_desired_area * (self.follow_desired_distance / desired_distance) ** 2) - area
            if abs(area_err) < self.follow_area_band:
                vx = 0.0
        
        # Apply velocity caps
        vx = self._saturate(vx, -self.follow_vx_cap, self.follow_vx_cap)
        vy = self._saturate(vy, -self.follow_vy_cap, self.follow_vy_cap)
        vz = self._saturate(vz, -self.follow_vz_cap, self.follow_vz_cap)
        yaw_rate = self._saturate(yaw_rate, -self.follow_yaw_cap, self.follow_yaw_cap)

        return True, vx, vy, vz, yaw_rate

    def _on_timer(self):
        """Timer callback to publish at fixed rate"""
        self._publish_follow_cmd()

    def _publish_follow_cmd(self):
        valid, vx, vy, vz, yaw_rate = self._compute_follow_cmd()

        target_msg = Bool()
        target_msg.data = valid
        self.target_found_pub.publish(target_msg)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'body'
        cmd.twist.linear.x = vx
        cmd.twist.linear.y = vy
        cmd.twist.linear.z = vz
        cmd.twist.angular.z = yaw_rate
        
        self.cmd_pub.publish(cmd)
        
        # Log detection status occasionally
        if not hasattr(self, '_last_status_log_time') or (time.monotonic() - self._last_status_log_time) > 3.0:
            if self._last_det_time is not None:
                time_since_det = time.monotonic() - self._last_det_time
                status = "TRACKING" if valid else f"LOST ({time_since_det:.1f}s ago)"
                self.get_logger().info(
                    f'Status: {status} | Cmd: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, yaw={yaw_rate:.1f}'
                )
            else:
                self.get_logger().info('Status: NO DETECTIONS YET | Waiting for first detection...')
            self._last_status_log_time = time.monotonic()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectFollowingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()