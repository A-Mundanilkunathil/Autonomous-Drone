"""
Subscribers receive data FROM the drone via MAVROS
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, HomePosition
from mavros_msgs.msg import Altitude

class MavrosSubscribers:
    def __init__(self, node: Node):
        self.node = node

        # Store latest data from MAVROS
        self.current_state = None
        self.local_position = None
        self.global_position_local = None
        self.velocity = None
        self.gps_fix = None
        self.home = None
        self.altitude = None

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
        
        # Subscribe to global position local 
        self.global_pos_local_sub = node.create_subscription(
            Odometry,
            '/mavros/global_position/local',
            self._global_pos_local_callback,
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
            '/mavros/global_position/global',
            self._gps_callback,
            qos_profile
        )

        # Subscribe to altitude
        self.altitude_sub = node.create_subscription(
            Altitude,
            '/mavros/altitude',
            self._altitude_callback,
            qos_profile
        )

        # Subscribe to home position
        self.home_sub = node.create_subscription(
            HomePosition,
            '/mavros/home_position/home',
            self._home_callback,
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

    def _global_pos_local_callback(self, msg: Odometry):
        """Global position local updates (more accurate altitude)"""
        self.global_position_local = msg

    def _velocity_callback(self, msg: TwistStamped):
        """Velocity updates (NED frame)"""
        self.velocity = msg

    def _gps_callback(self, msg: NavSatFix):
        """Receives GPS fix"""
        self.gps_fix = msg

    def _altitude_callback(self, msg: Altitude):
        self.altitude = msg

    def _home_callback(self, msg: HomePosition):
        """Receives home position"""
        self.home = msg

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
        """
        Get current GPS position.
        Returns: (latitude, longitude, altitude_AMSL)
        Note: altitude is AMSL (Above Mean Sea Level), not relative to home!
        """
        if self.gps_fix:
            return (
                self.gps_fix.latitude,
                self.gps_fix.longitude,
                self.gps_fix.altitude
            )
        return (0.0, 0.0, 0.0)
    
    def get_relative_altitude(self) -> float:
        """
        Get current altitude relative to home position.
        Returns: altitude in meters relative to home
        """
        if self.altitude:
            return self.altitude.relative
        # Fallback to global position local z 
        if self.global_position_local:
            return abs(self.global_position_local.pose.pose.position.z)
        # Fallback to local position z if global position not available
        if self.local_position:
            return abs(self.local_position.pose.position.z)
        return 0.0
    
    def get_home_position(self) -> tuple:
        if self.home:
            return (
                self.home.geo.latitude,
                self.home.geo.longitude,
                self.home.geo.altitude
            )
        return (0.0, 0.0, 0.0)
