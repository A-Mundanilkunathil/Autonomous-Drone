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
        self.follow_desired_area = 0.06  # Target area for 3-4m distance (fallback)
        self.follow_desired_distance = 3.0  # meters
        self.follow_area_band = 0.02  # Deadband to prevent jitter
        self.k_vx = 1.2  # P gain for distance control
        self.k_vx_d = 0.3  # D gain for velocity damping
        self.k_vy = 0.8  # Lateral centering gain
        self.k_vz = 0.8  # Vertical centering gain
        self.k_yaw = 35.0  # Yaw tracking gain (deg/s)
        self.follow_vx_cap = 1.2  # m/s
        self.follow_vy_cap = 0.7  # m/s
        self.follow_vz_cap = 0.6  # m/s
        self.follow_yaw_cap = 80.0  # deg/s
        self.follow_lost_timeout = 10.0  # Accommodate slow inference
        self.follow_min_vx = 0.05  # Prevent stalling
        
        self.img_width = 640
        self.img_height = 480
        
        # MiDaS depth calibration
        self.midas_scale = 200.0
        self.midas_shift = 0.0
        self.midas_max_dist = 50.0
        
        # Load calibration file if provided
        midas_calib_path = self.declare_parameter('midas_calib_npz', '').value
        if midas_calib_path:
            try:
                midas_calib = np.load(midas_calib_path)
                self.midas_scale = float(midas_calib['scale'])
                self.midas_shift = float(midas_calib['shift'])
                self.get_logger().info(f'Loaded MiDaS calibration: scale={self.midas_scale:.3f}, shift={self.midas_shift:.3f}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load MiDaS calibration: {e}. Using defaults.')

        # Detection tracking
        self._last_det_time = None
        self._last_det_bbox = None
        self._last_det_score = 0.0
        
        # Distance tracking for PD control
        self._latest_depth_map = None
        self._depth_timestamp = 0.0
        self._prev_distance = None
        self._prev_distance_time = None
        
        # Dynamic setpoint adjustment
        self._forward_clearance = float('inf')
        self._clearance_timestamp = 0.0

        # Publish commands at fixed rate
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
        """Process detections and track target object"""
        best = None
        best_score = -1.0

        for det in msg.detections:
            if not det.results:
                continue
            
            hyp = det.results[0].hypothesis
            cls = getattr(hyp, "class_id", "")
            score = float(getattr(hyp, "score", 0.0))
            
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
        """Extract metric depth at bounding box center"""
        if self._latest_depth_map is None:
            return None
        if (time.monotonic() - self._depth_timestamp) > 0.5:
            return None
            
        try:
            inverse_depth_array = self.bridge.imgmsg_to_cv2(self._latest_depth_map, desired_encoding='32FC1')
            height, width = inverse_depth_array.shape[:2]
            
            # Convert bbox center to pixel coordinates
            center_x = int((cx / self.img_width) * width)
            center_y = int((cy / self.img_height) * height)
            center_x = max(0, min(width - 1, center_x))
            center_y = max(0, min(height - 1, center_y))
            
            # Sample 5x5 region for robustness
            y_start = max(0, center_y - 2)
            y_end = min(height, center_y + 3)
            x_start = max(0, center_x - 2)
            x_end = min(width, center_x + 3)
            
            inv_depth_region = inverse_depth_array[y_start:y_end, x_start:x_end]
            
            # Filter invalid values
            valid_inv_depths = inv_depth_region[(inv_depth_region > 0.1) & 
                                                ~np.isnan(inv_depth_region) & 
                                                ~np.isinf(inv_depth_region)]
            
            if len(valid_inv_depths) > 0:
                median_inv_depth = float(np.median(valid_inv_depths))
                metric_depth = self.midas_scale / median_inv_depth
                
                if self.midas_shift != 0.0:
                    metric_depth += self.midas_shift
                
                # Sanity check
                if 0.3 < metric_depth < self.midas_max_dist:
                    return metric_depth
            return None
        except Exception as e:
            self.get_logger().warn(f'Depth extraction error: {e}')
            return None
    
    def _get_dynamic_setpoint(self) -> float:
        """Adjust target distance based on available forward clearance"""
        if (time.monotonic() - self._clearance_timestamp) < 0.5:
            clearance = self._forward_clearance
            
            # Tight space: maintain closer
            if clearance < 2.0:
                return max(1.5, clearance * 0.6)
            # Open space: can be farther
            elif clearance > 5.0:
                return 4.0
            # Normal: scale linearly
            else:
                return 2.0 + (clearance - 2.0) * (2.0 / 3.0)
        
        return self.follow_desired_distance

    def _compute_follow_cmd(self):
        """Compute velocity commands to follow target"""
        if not self._has_fresh_detection():
            self._prev_distance = None
            self._prev_distance_time = None
            return False, 0.0, 0.0, 0.0, 0.0
        
        meas = self._get_follow_measurements()
        if meas is None:
            return False, 0.0, 0.0, 0.0, 0.0

        ex, ey, area = meas
        cx, cy, bw, bh = self._last_det_bbox
        depth_distance = self._get_depth_at_bbox(cx, cy, bw, bh)
        desired_distance = self._get_dynamic_setpoint()

        current_time = time.monotonic()
        
        if depth_distance is not None and depth_distance > 0.1:
            # PD control with depth
            distance_error = depth_distance - desired_distance
            
            derivative_term = 0.0
            if self._prev_distance is not None and self._prev_distance_time is not None:
                dt = current_time - self._prev_distance_time
                if dt > 0.01:
                    distance_rate = (depth_distance - self._prev_distance) / dt
                    derivative_term = self.k_vx_d * distance_rate
            
            vx = -self.k_vx * distance_error - derivative_term
            self._prev_distance = depth_distance
            self._prev_distance_time = current_time
        else:
            # Area-based fallback
            area_scale = (self.follow_desired_distance / desired_distance) ** 2
            scaled_desired_area = self.follow_desired_area * area_scale
            area_err = scaled_desired_area - area
            vx = self.k_vx * area_err
            self._prev_distance = None
            self._prev_distance_time = None
        
        # Prevent stalling
        if vx > 0.0:
            vx = max(vx, self.follow_min_vx)

        # Lateral, vertical, yaw control
        vy = -self.k_vy * ex
        vz = +self.k_vz * ey
        yaw_rate = self.k_yaw * ex

        # Deadband for area-based only
        if depth_distance is None:
            area_err = (self.follow_desired_area * (self.follow_desired_distance / desired_distance) ** 2) - area
            if abs(area_err) < self.follow_area_band:
                vx = 0.0
        
        # Apply limits
        vx = self._saturate(vx, -self.follow_vx_cap, self.follow_vx_cap)
        vy = self._saturate(vy, -self.follow_vy_cap, self.follow_vy_cap)
        vz = self._saturate(vz, -self.follow_vz_cap, self.follow_vz_cap)
        yaw_rate = self._saturate(yaw_rate, -self.follow_yaw_cap, self.follow_yaw_cap)

        return True, vx, vy, vz, yaw_rate

    def _on_timer(self):
        """Timer callback to publish at fixed rate"""
        self._publish_follow_cmd()

    def _publish_follow_cmd(self):
        """Publish velocity commands and target status"""
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

def main(args=None):
    rclpy.init(args=args)
    node = ObjectFollowingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()