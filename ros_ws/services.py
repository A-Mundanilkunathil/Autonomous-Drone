from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.srv import CommandHome
import rclpy

class MavrosServices:
    def __init__(self, node: Node):
        self.node = node

        # Arm/disarm service
        self.arm_client = node.create_client(CommandBool, "/mavros/cmd/arming")

        # Mode change service
        self.mode_client = node.create_client(SetMode, "/mavros/set_mode")

        # Takeoff service
        self.takeoff_client = node.create_client(CommandTOL, "/mavros/cmd/takeoff")

        # Land service
        self.land_client = node.create_client(CommandTOL, "/mavros/cmd/land")

        # Set home service
        self.set_home_client = node.create_client(CommandHome, "/mavros/cmd/set_home")

        node.get_logger().info('MAVROS Services initialized.')

    def arm(self, value: bool = True, timeout_sec=15.0) -> bool:
        """
        Arm or disarm the drone
        
        Why: Drone must be armed before takeoff
        Returns: True if successful
        """
        if not self.arm_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().error('Arm service not available.')
            return False
        
        req = CommandBool.Request()
        req.value = value

        # Call and wait for response
        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)

        if future.result() is not None:
            result = future.result()
            if result.success:
                self.node.get_logger().info(f'Drone {"armed" if value else "disarmed"} successfully.')
                return True
            else:
                self.node.get_logger().error('Failed to change arm state.')
                return False
        return False

    def set_mode(self, mode: str, timeout_sec=5.0) -> bool:
        """
        Set flight mode (GUIDED, STABILIZE, LOITER, RTL, LAND)
        
        Why: Different modes for different operations
        - GUIDED: For autonomous control
        - STABILIZE: Manual control
        - LOITER: Hold position
        - RTL: Return to launch
        - LAND: Auto-land
        """
        if not self.mode_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().error('Set mode service not available.')
            return False
        
        req = SetMode.Request()
        req.custom_mode = mode

        # Call and wait for response
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)

        if future.result() is not None:
            result = future.result()
            if result.mode_sent:
                self.node.get_logger().info(f'Mode set to {mode} successfully.')
                return True
            else:
                self.node.get_logger().warning(f'Mode {mode} sent but not confirmed.')
                return True
        
        self.node.get_logger().error(f'Failed to send mode {mode} command.')
        return False
    
    def takeoff(self, altitude: float, timeout_sec=15.0) -> bool:
        """
        Command the drone to take off to a specified altitude
        
        Why: Initiate takeoff sequence
        altitude: Target altitude in meters
        """
        if not self.takeoff_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().error('Takeoff service not available.')
            return False
        
        req = CommandTOL.Request()
        req.altitude = altitude

        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)

        if future.result() is not None:
            if future.result().success:
                self.node.get_logger().info(f'Takeoff to {altitude}m successful.')
                return True
        return False
    
    def land(self, timeout_sec=5.0) -> bool:
        """
        Command the drone to land at current position
        
        Why: Initiate landing sequence
        """
        if not self.land_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().error('Land service not available.')
            return False
        
        req = CommandTOL.Request()
        req.altitude = 0.0  # Land at ground level

        future = self.land_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)

        if future.result() is not None:
            if future.result().success:
                self.node.get_logger().info('Landing initiated successfully.')
                return True
        return False
    
    def set_home(
            self, 
            use_current_gps: bool = True,
            latitude: float = 0.0,
            longitude: float = 0.0,
            altitude: float = 0.0,
            timeout_sec=5.0
    ) -> bool:
        """
        Set home position for the drone
        
        Why: Define home position for RTL and other functions
        use_current_gps: If True, use current GPS as home
        latitude, longitude, altitude: If False, set specified coordinates as home
        """
        if not self.set_home_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().error('Set home service not available.')
            return False
        
        req = CommandHome.Request()
        req.current_gps = use_current_gps  # Changed from req.value
        if not use_current_gps:
            req.latitude = latitude
            req.longitude = longitude
            req.altitude = altitude

        future = self.set_home_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)

        if future.result() is not None:
            if future.result().success:
                self.node.get_logger().info('Home position set successfully.')
                return True
        return False