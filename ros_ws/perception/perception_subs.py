"""
Subscribers receive data FROM perception/vision nodes
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32
import time
import math

class PerceptionSubscribers:
    """
    Subscribes to perception and vision data:
    - Avoidance commands from object avoidance node
    - Forward clearance from depth processing
    - Future: Object detections, SLAM poses, etc.
    """
    def __init__(self, node: Node):
        self.node = node

        # Store latest data from perception nodes
        self.avoidance_cmd = None
        self.avoidance_timestamp = 0.0
        self.forward_clearance = float('inf')

        # Subscribe to avoidance commands from perception nodes
        self.avoidance_sub = node.create_subscription(
            TwistStamped,
            '/avoidance/cmd_vel',
            self._avoidance_callback,
            qos_profile=1
        )

        # Subscribe to forward clearance from perception nodes
        self.clearance_sub = node.create_subscription(
            Float32,
            '/avoidance/forward_clearance',
            self._clearance_callback,
            qos_profile=1
        )

        node.get_logger().info('Perception Subscribers initialized.')

    def _avoidance_callback(self, msg: TwistStamped):
        """Receives avoidance velocity commands from perception nodes"""
        self.avoidance_cmd = msg
        self.avoidance_timestamp = time.monotonic()

    def _clearance_callback(self, msg: Float32):
        """Receives forward clearance distance from perception nodes"""
        try:
            fc = float(msg.data)
            if math.isnan(fc) or math.isinf(fc):
                self.forward_clearance = float('inf')
            else:
                self.forward_clearance = fc
        except Exception as e:
            self.forward_clearance = float('inf')
            self.node.get_logger().warn(f'Clearance callback error: {e}')

    # Getter methods for perception data
    def get_avoidance_cmd(self) -> TwistStamped:
        """
        Get latest avoidance velocity command from perception nodes.
        Returns: TwistStamped message with avoidance velocities, or None if not available
        """
        return self.avoidance_cmd
    
    def get_forward_clearance(self) -> float:
        """
        Get forward clearance distance from perception nodes.
        Returns: Distance in meters (inf if no obstacle detected)
        """
        return self.forward_clearance
    
    def is_avoidance_fresh(self, max_age_s: float = 1.0) -> bool:
        """
        Check if avoidance data is recent.
        Args:
            max_age_s: Maximum age in seconds to consider data fresh
        Returns: True if avoidance data is recent, False otherwise
        """
        if self.avoidance_cmd is None:
            return False
        return (time.monotonic() - self.avoidance_timestamp) < max_age_s
    
    def is_avoidance_requesting(self, eps: float = 0.05) -> bool:
        """
        Check if avoidance is requesting lateral/vertical movement.
        Args:
            eps: Epsilon threshold for considering movement significant
        Returns: True if avoidance wants to move sideways or up/down
        """
        if self.avoidance_cmd is None:
            return False
        av = self.avoidance_cmd.twist.linear
        return (abs(av.y) > eps) or (abs(av.z) > eps)
