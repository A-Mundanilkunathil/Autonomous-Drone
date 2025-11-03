"""
Publishers send commands TO the drone via MAVROS
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import GlobalPositionTarget
import math

class MavrosPublishers:
    def __init__(self, node: Node):
        self.node = node

        # Position control - send where the drone should go
        self.local_pos_pub = node.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local', # MAVROS topic for local position setpoints
            10 # QoS history depth
        )

        # Velocity control - send how fast the drone should move
        self.velocity_pub = node.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel', # MAVROS topic for velocity setpoints
            10 # QoS history depth
        )

        # Raw position/velocity targets - advanced control
        self.position_target_pub = node.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local', # MAVROS topic for raw local setpoints
            10 # QoS history depth
        )

        # Raw position/velocity targets - advanced control
        self.position_target_global_pub = node.create_publisher(
            GlobalPositionTarget,
            '/mavros/setpoint_raw/global', # MAVROS topic for raw global setpoints
            10 # QoS history depth
        )

        node.get_logger().info('MAVROS Publishers initialized.')

    def publish_position(self, x: float, y: float, z: float):
        """
        Send position setpoint in ENU frame (East-North-Up)
        
        Why: Tell drone "go to this position"
        When: Use for waypoint navigation
        """
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x # East (meters)
        msg.pose.position.y = y # North (meters)
        msg.pose.position.z = z # Up (meters)
        self.local_pos_pub.publish(msg)
    
    def publish_velocity_body(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0):
        """
        Send velocity in BODY frame (FLU: Forward-Left-Up convention)
        
        Why: Move relative to where drone is facing
        When: Use for "move forward/right/up" commands
        
        vx: forward/backward (positive = forward)
        vy: left/right (positive = LEFT, negative = RIGHT)
        vz: up/down (positive = DOWN, negative = UP)
        yaw_rate: rotation speed (deg/s, positive = LEFT, negative = RIGHT)
        """
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        msg.twist.angular.z = math.radians(yaw_rate)
        self.velocity_pub.publish(msg)

    def publish_velocity_ned(self, vx: float, vy: float, vz: float):
        """
        Send velocity in WORLD frame (North-East-Down)
        
        Why: Move in absolute directions (North/East)
        When: Use for GPS-style navigation
        """
        msg = PositionTarget()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # Type mask to ignore variables we are not using
        msg.type_mask = (
            PositionTarget.IGNORE_PX |      # Ignore X position
            PositionTarget.IGNORE_PY |      # Ignore Y position
            PositionTarget.IGNORE_PZ |      # Ignore Z position
            PositionTarget.IGNORE_AFX |     # Ignore X acceleration
            PositionTarget.IGNORE_AFY |     # Ignore Y acceleration
            PositionTarget.IGNORE_AFZ |     # Ignore Z acceleration
            PositionTarget.IGNORE_YAW |     # Ignore yaw angle
            PositionTarget.IGNORE_YAW_RATE  # Ignore yaw rate
        )
        msg.velocity.x = vx
        msg.velocity.y = vy
        msg.velocity.z = vz
        self.position_target_pub.publish(msg)

    def publish_global_position(
        self,
        latitude: float,
        longitude: float,
        altitude_m: float,
        *,
        yaw_deg: float | None = None,
        yaw_rate_deg_s: float | None = None,
        relative_alt: bool = True
    ):
        """
        Command a GPS (global) position target.

        Args:
            latitude:  degrees (WGS84)
            longitude: degrees (WGS84)
            altitude_m: meters (AMSL if relative_alt=False, else REL_ALT from home)
            yaw_deg: optional yaw command in degrees (0..360, NED, positive CCW)
            yaw_rate_deg_s: optional yaw rate in deg/s (ignored if yaw given)
            relative_alt: True -> MAV_FRAME_GLOBAL_REL_ALT, False -> MAV_FRAME_GLOBAL_INT (AMSL)
        """
        msg = GlobalPositionTarget()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        
        # Coordinate frame
        msg.coordinate_frame = (
            GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
            if relative_alt else
            GlobalPositionTarget.FRAME_GLOBAL_INT
        )

        # Use position only
        type_mask = (
            GlobalPositionTarget.IGNORE_VX |
            GlobalPositionTarget.IGNORE_VY |
            GlobalPositionTarget.IGNORE_VZ |
            GlobalPositionTarget.IGNORE_AFX |
            GlobalPositionTarget.IGNORE_AFY |
            GlobalPositionTarget.IGNORE_AFZ
        )

        if yaw_deg is None and yaw_rate_deg_s is None:
            # Ignore both yaw and yaw rate
            type_mask |= (
                GlobalPositionTarget.IGNORE_YAW |
                GlobalPositionTarget.IGNORE_YAW_RATE
            )
        elif yaw_deg is not None:
            # Set yaw, ignore yaw rate
            msg.yaw = math.radians(yaw_deg)
            type_mask |= GlobalPositionTarget.IGNORE_YAW_RATE
        else:
            # Set yaw rate, ignore yaw
            msg.yaw_rate = math.radians(yaw_rate_deg_s)
            type_mask |= GlobalPositionTarget.IGNORE_YAW

        msg.type_mask = type_mask

        # Target position
        msg.latitude = float(latitude)
        msg.longitude = float(longitude)
        msg.altitude = float(altitude_m)

        self.position_target_global_pub.publish(msg)