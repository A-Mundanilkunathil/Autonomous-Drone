"""
Subscribers receive data FROM perception/vision nodes
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, Bool
from typing import Optional
import numpy as np
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

        # Avoidance data
        self.avoidance_cmd = None
        self.avoidance_timestamp = 0.0
        self.forward_clearance = 0.0
        self.clearance_timestamp = None

        # Following data
        self.following_cmd = None
        self.following_timestamp = 0.0
        self.target_found = False

        # VSLAM map data
        self.map_points: Optional[np.ndarray] = None  # (N, 3) float32 world-frame XYZ

        # Subscribe to avoidance commands from perception nodes
        self.avoidance_sub = node.create_subscription(
            TwistStamped,
            'avoidance/cmd_vel',
            self._avoidance_callback,
            qos_profile=1
        )

        # Subscribe to forward clearance from perception nodes
        self.clearance_sub = node.create_subscription(
            Float32,
            'avoidance/forward_clearance',
            self._clearance_callback,
            qos_profile=1
        )

        # Subscribe to following commands from object following node
        self.following_sub = node.create_subscription(
            TwistStamped,
            'following/cmd_vel',
            self._following_callback,
            qos_profile=1
        )

        # Subscribe to target found status
        self.target_found_sub = node.create_subscription(
            Bool,
            'following/target_found',
            self._target_found_callback,
            qos_profile=1
        )

        # Subscribe to VSLAM sparse 3-D map
        self.map_sub = node.create_subscription(
            PointCloud2,
            'vslam/map',
            self._map_callback,
            qos_profile=1
        )

        node.get_logger().info('Perception Subscribers initialized.')

    def _avoidance_callback(self, msg: TwistStamped):
        """Receives avoidance velocity commands from perception nodes"""
        self.avoidance_cmd = msg
        self.avoidance_timestamp = self._now_seconds()

    def _clearance_callback(self, msg: Float32):
        """Receives forward clearance distance from perception nodes"""
        try:
            fc = float(msg.data)
            if math.isnan(fc) or math.isinf(fc) or fc < 0.0:
                self.forward_clearance = 0.0
            else:
                self.forward_clearance = fc
            self.clearance_timestamp = self._now_seconds()
        except Exception as e:
            self.forward_clearance = 0.0
            self.clearance_timestamp = self._now_seconds()
            self.node.get_logger().warn(f'Clearance callback error: {e}')

    def _following_callback(self, msg: TwistStamped):
        """Receives following velocity commands from object following node"""
        self.following_cmd = msg
        self.following_timestamp = self._now_seconds()

    def _target_found_callback(self, msg: Bool):
        """Receives target found status from object following node"""
        self.target_found = msg.data

    def _map_callback(self, msg: PointCloud2):
        """
        Parse the VSLAM PointCloud2 into a (N, 3) numpy array.
        The map is XYZ float32 with point_step=12 bytes.
        """
        n = msg.width * msg.height
        if n == 0:
            return
        self.map_points = np.frombuffer(msg.data, dtype=np.float32).reshape(n, 3)

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

    def _now_seconds(self) -> float:
        return self.node.get_clock().now().nanoseconds / 1_000_000_000.0

    def is_clearance_fresh(self, max_age_s: float = 1.0) -> bool:
        if self.clearance_timestamp is None:
            return False
        return (self._now_seconds() - self.clearance_timestamp) < max_age_s
    
    def is_avoidance_fresh(self, max_age_s: float = 1.0) -> bool:
        """
        Check if avoidance data is recent.
        Args:
            max_age_s: Maximum age in seconds to consider data fresh
        Returns: True if avoidance data is recent, False otherwise
        """
        if self.avoidance_cmd is None:
            return False
        return (self._now_seconds() - self.avoidance_timestamp) < max_age_s
    
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
    
    # Following getters
    def get_following_cmd(self) -> Optional[TwistStamped]:
        """
        Get latest following velocity command from object following node.
        Returns: TwistStamped message with following velocities, or None if not available
        """
        return self.following_cmd
    
    def is_following_fresh(self, max_age_s: float = 1.0) -> bool:
        """
        Check if following data is recent.
        Args:
            max_age_s: Maximum age in seconds to consider data fresh
        Returns: True if following data is recent, False otherwise
        """
        if self.following_cmd is None:
            return False
        return (self._now_seconds() - self.following_timestamp) < max_age_s
    
    def has_target(self) -> bool:
        """
        Check if following node has detected target.
        Returns: True if target is detected, False otherwise
        """
        return self.target_found

    # ── VSLAM map queries ────────────────────────────────────────────────────

    def get_map_point_count(self) -> int:
        """Return how many 3-D landmarks are currently in the map."""
        if self.map_points is None:
            return 0
        return len(self.map_points)
