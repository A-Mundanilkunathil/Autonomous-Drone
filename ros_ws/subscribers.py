"""
Subscribers receive data FROM the drone via MAVROS
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State

class MavrosSubscribers:
    def __init__(self, node: Node):
        self.node = node

        # Store latest data
        self.current_state = None
        self.local_position = None
        self.velocity = None
        self.gps_fix = None

        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to drone state (armed, mode, connected)
        self.state_sub = node.create_subscription(
            State,
            '/mavros/state',
            self._state_callback,
            qos_profile  
        )

        # Subsribe to position
        self.local_pos_sub = node.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._local_pos_callback,
            qos_profile  
        )
        
        # Subscribe to velocity
        self.velocity_sub = node.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self._velocity_callback,
            qos_profile  
        )

        # Subscribe to GPS
        self.gps_sub = node.create_subscription(
            NavSatFix,
            'mavros/global_position/global',
            self._gps_callback,
            qos_profile
        )

        node.get_logger().info('MAVROS Subscribers initialized.')

    def _state_callback(self, msg: State):
        """
        Called every time MAVROS publishes state update
        
        State includes:
        - connected: Is MAVROS connected to ArduPilot?
        - armed: Is drone armed?
        - mode: Current flight mode (GUIDED, STABILIZE, etc.)
        """
        self.current_state = msg

        if not hasattr(self, '_last_armed') or msg.armed != self._last_armed:
            self.node.get_logger().info(f"Armed: {msg.armed}")
            self._last_armed = msg.armed

        if not hasattr(self, '_last_mode') or msg.mode != self._last_mode:
            self.node.get_logger().info(f"Mode: {msg.mode}")
            self._last_mode = msg.mode

    def _local_pos_callback(self, msg: PoseStamped):
        """Position updates (NED frame)"""
        self.local_position = msg

    def _velocity_callback(self, msg: TwistStamped):
        """Velocity updates (NED frame)"""
        self.velocity = msg

    def _gps_callback(self, msg: NavSatFix):
        """Receives GPS fix"""
        self.gps_fix = msg

    # Helper methods to get latest data
    def is_armed(self) -> bool:
        return self.current_state.armed if self.current_state else False

    def is_connected(self) -> bool:
        return self.current_state.connected if self.current_state else False
    
    def get_mode(self) -> str:
        return self.current_state.mode if self.current_state else "UNKNOWN"
    
    def get_position(self) -> tuple:
        if self.local_position:
            pos = self.local_position.pose.position
            return (pos.x, pos.y, pos.z)
        return (0.0, 0.0, 0.0)

    def get_velocity(self) -> tuple:
        if self.velocity:
            vel = self.velocity.twist.linear
            return (vel.x, vel.y, vel.z)
        return (0.0, 0.0, 0.0)
    
    def get_global_position(self) -> tuple:
        if self.gps_fix:
            return (
                self.gps_fix.latitude,
                self.gps_fix.longitude,
                self.gps_fix.altitude
            )
        return (0.0, 0.0, 0.0)
