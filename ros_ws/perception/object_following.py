import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
import time
import math

class ObjectFollowingNode(Node):
    def __init__(self):
        super().__init__('object_following')

        # Following parameters
        self.follow_target_label = 'person'
        self.follow_desired_area = 0.06
        self.follow_area_band = 0.02
        self.k_vx = 1.2
        self.k_vy = 0.8
        self.k_vz = 0.8
        self.k_yaw = 35.0
        self.follow_vx_cap = 1.2
        self.follow_vy_cap = 0.7
        self.follow_vz_cap = 0.6
        self.follow_yaw_cap = 80.0
        self.follow_lost_timeout = 1.0
        self.follow_min_vx = 0.05
        
        self.img_width = 640
        self.img_height = 480

        # Detection state
        self._last_det_time = 0.0
        self._last_det_bbox = None
        self._last_det_score = 0.0

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

    def _detection_callback(self, msg: Detection2DArray):
        # Find best matching target
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
        if self._last_det_bbox is None:
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

    def _compute_follow_cmd(self):
        if not self._has_fresh_detection():
            return False, 0.0, 0.0, 0.0, 0.0
        
        meas = self._get_follow_measurements()
        if meas is None:
            return False, 0.0, 0.0, 0.0, 0.0

        ex, ey, area = meas

        # Forward: based on target size
        area_err = self.follow_desired_area - area
        vx = self.k_vx * area_err
        
        # Prevent stalling
        if vx > 0.0:
            vx = max(vx, self.follow_min_vx)

        # Lateral: center horizontally
        vy = -self.k_vy * ex
        # Vertical: maintain altitude alignment
        vz = +self.k_vz * ey
        # Yaw: face target
        yaw_rate = self.k_yaw * ex

        # Deadband to avoid jitter
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

def main(args=None):
    rclpy.init(args=args)
    node = ObjectFollowingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()