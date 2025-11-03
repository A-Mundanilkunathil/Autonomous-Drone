import rclpy
from rclpy.node import Node
from publishers import MavrosPublishers
from subscribers import MavrosSubscribers
from services import MavrosServices

class AutonomousDroneNode(Node):
    def __init__(self):
        super().__init__('autonomous_drone_node')

        self.mavros_pubs = MavrosPublishers(self)
        self.mavros_subs = MavrosSubscribers(self)
        self.mavros_srvs = MavrosServices(self)

        self.get_logger().info('Autonomous Drone Node initialized.')

    def control_loop(self):
        # Check connection
        if not self.mavros_subs.is_connected():
            return
        
        # TODO: Add control logic here
        pass

    def arm_and_takeoff(self, altitude: float, timeout: float = 30.0) -> bool:
        """
        Arm and takeoff to target altitude, waiting until reached
        
        Args:
            altitude: Target altitude in meters
            timeout: Maximum time to wait for altitude (seconds)
        
        Returns:
            True if successful and altitude reached
        """
        self.get_logger().info(f'Starting takeoff to {altitude}m...')

        # Set mode to GUIDED
        if not self.mavros_srvs.set_mode('GUIDED'):
            return False
        
        import time
        time.sleep(1.0)  # Wait a moment for mode to set

        # Arm the drone
        if not self.mavros_srvs.arm(True):
            return False
        time.sleep(1.0)  # Wait a moment for arming

        # Send takeoff command
        if not self.mavros_srvs.takeoff(altitude):
            return False
        
        self.get_logger().info(f'Takeoff command sent. Waiting for altitude {altitude}m...')
        
        # Wait for drone to reach altitude (with tolerance)
        target_reached = False
        altitude_tolerance = 0.5 
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            # Get current position 
            pos = self.mavros_subs.get_position()
            current_alt = abs(pos[2])  
            
            self.get_logger().info(
                f'Current altitude: {current_alt:.2f}m / Target: {altitude}m',
                throttle_duration_sec=1.0  # Log once per second
            )
            
            # Check if we've reached target altitude
            if abs(current_alt - altitude) < altitude_tolerance:
                target_reached = True
                break
            
            # Spin to process callbacks and sleep
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.5)
        
        if target_reached:
            final_alt = abs(self.mavros_subs.get_position()[2])
            self.get_logger().info(f'Takeoff successful! Reached {final_alt:.2f}m')
            return True
        else:
            final_alt = abs(self.mavros_subs.get_position()[2])
            self.get_logger().warn(
                f'Takeoff timeout! Only reached {final_alt:.2f}m in {timeout}s'
            )
            return False

    def move_body_velocity(self, vx: float, vy: float, vz: float, duration: float = 2.0, yaw_rate: float = 0.0):
        """
        Move with velocity for specified duration
        
        Uses BODY frame (FLU: Forward-Left-Up convention)
        
        Args:
            vx: Forward/backward velocity (m/s, positive = forward)
            vy: Left/right velocity (m/s, positive = LEFT, negative = RIGHT)
            vz: Up/down velocity (m/s, positive = DOWN, negative = UP)
            duration: How long to move (seconds)
            yaw_rate: Rotation rate (deg/s, positive = LEFT, negative = RIGHT)
        
        Usage: node.move_body_velocity(1.0, 0, 0, 5.0)  # Move forward 1m/s for 5s
        """
        import time
        rate_hz = 10
        dt = 1.0 / rate_hz
        end_time = time.time() + duration

        while time.time() < end_time:
            self.mavros_pubs.publish_velocity_body(vx, vy, vz, yaw_rate)
            time.sleep(dt)
        
    def goto_position(self, x: float, y: float, z: float, duration: float = 5.0):
        """
        Go to position and hold
        
        Uses ENU frame (East-North-Up)
        
        Args:
            x: East position (meters)
            y: North position (meters) 
            z: Altitude (meters, positive = up)
        
        Usage: node.goto_position(10, 5, 5.0, 10.0)  # Go 10m east, 5m north, 5m altitude
        """
        import time
        rate_hz = 10
        dt = 1.0 / rate_hz
        end_time = time.time() + duration

        while time.time() < end_time:
            self.mavros_pubs.publish_position(x, y, z)
            time.sleep(dt)
    
    def _haversine_distance(self, lat1, lon1, lat2, lon2):
        import math

        R = 6371000 # Earth radius in meters
        d_lat = math.radians(lat2 - lat1)
        d_lon = math.radians(lon2 - lon1)
        a = (
            math.sin(d_lat / 2)**2 + 
            math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * 
            math.sin(d_lon / 2)**2
        )
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c # Distance in meters
        return distance

    def goto_gps(self, target_lat, target_lon, target_alt, timeout_s=90.0):
        import time
        rate_hz = 10
        dt = 1.0 / rate_hz
        start = time.time()

        while (time.time() - start) < timeout_s:
            # Publish GPS target
            self.mavros_pubs.publish_global_position(
                latitude=target_lat,
                longitude=target_lon,
                altitude_m=target_alt
            )

            # Read current position
            curr_lat, curr_lon, curr_alt = self.mavros_subs.get_position()

            # Compute distance to target
            distance = self._haversine_distance(
                curr_lat, curr_lon, target_lat, target_lon
            )

            self.get_logger().info(f"Distance to target: {distance:.2f} m | Alt diff: {abs(target_alt - curr_alt):.2f} m")

            if distance < 1.5 and abs(target_alt - curr_alt) < 0.8:
                self.get_logger().info("Target reached!")
                break

            time.sleep(dt)
            
    def land(self, timeout: float = 30.0) -> bool:
        """
        Command the drone to land and wait until on ground
        
        Args:
            timeout: Maximum time to wait for landing (seconds)
        
        Returns:
            True if successfully landed
        """
        self.get_logger().info('Starting landing sequence...')
        
        # Send land command
        if not self.mavros_srvs.land():
            self.get_logger().error('Land command failed')
            return False
        
        import time
        
        # Wait for drone to land (altitude near 0)
        ground_threshold = 0.4  # Consider landed if below 40cm
        start_time = time.time()
        landed = False
        
        while (time.time() - start_time) < timeout:
            pos = self.mavros_subs.get_position()
            current_alt = abs(pos[2]) 
            
            self.get_logger().info(
                f'Landing... altitude: {current_alt:.2f}m',
                throttle_duration_sec=1.0
            )
            
            # Check if on ground
            if current_alt < ground_threshold:
                landed = True
                break
            
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.5)
        
        if landed:
            self.get_logger().info('Landed successfully')
            
            # Wait a moment, then disarm
            time.sleep(4.0)
            self.mavros_srvs.arm(False)
            return True
        else:
            self.get_logger().warn(f'Landing timeout after {timeout}s')
            return False
        
    def hover(self, duration: float = 5.0):
        """
        Hover in place for specified duration
        """
        import time

        self.get_logger().info(f'Hovering for {duration}s...')
        end_time = time.time() + duration
        rate_hz = 10
        dt = 1.0 / rate_hz

        while time.time() < end_time:
            # Send zero velocity to hold position
            self.mavros_pubs.publish_velocity_body(0.0, 0.0, 0.0, 0.0)
            time.sleep(dt)

    # ------------------------- High-level missions -------------------------
    def move_circle_global(self, radius: float = 5.0, speed: float = 1.0, duration: float = 60.0):
        """
        Move in a circular pattern in the GLOBAL frame (ENU)
        
        Args:
            radius: Circle radius (meters)
            speed: Tangential speed (m/s)
            duration: Total duration to circle (seconds)
            update_interval: Time between position updates (seconds)
        
        Usage: node.move_circle_global(radius=5.0, speed=1.0, duration=60.0)
        """
        import math 

        center_pos = self.mavros_subs.get_position()  # (x, y, z)
        center_x, center_y, center_z = center_pos

        rate_hz = 10
        dt = 1.0 / rate_hz
        total_steps = int(duration / dt)
        angle_step = (speed / radius) * dt  # radians per step

        for step in range(total_steps):
            angle = step * angle_step
            target_x = center_x + radius * math.cos(angle)
            target_y = center_y + radius * math.sin(angle)
            target_z = center_z  # Maintain current altitude

            self.goto_position(target_x, target_y, target_z, duration=dt)

    def move_square(self, speed: float = 1.0, leg_s: float = 3.0):
        """
        Move square in WORLD frame (ENU)
        Args:
            speed: Speed in m/s
            leg_s: Time to move each leg in seconds
        """
        pos = self.mavros_subs.get_position()  # (x, y, z)
        x_0, y_0, z_0 = pos
        distance = speed * leg_s if speed and leg_s else 3.0

        corners = [
            (x_0 + distance, y_0, z_0),
            (x_0 + distance, y_0 + distance, z_0),
            (x_0, y_0 + distance, z_0),
            (x_0, y_0, z_0)
        ]

        for corner in corners:
            self.goto_position(corner[0], corner[1], corner[2], duration=leg_s)
    
    # ------------------------- Low-level missions --------------------------
    # Basic movements in body frame (FLU)
    def move_forward(self, speed: float, duration: float):
        """Move forward in body frame"""
        self.get_logger().info(f'Moving forward at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=speed, vy=0.0, vz=0.0, duration=duration)
    def move_backward(self, speed: float, duration: float):
        """Move backward in body frame"""
        self.get_logger().info(f'Moving backward at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=-speed, vy=0.0, vz=0.0, duration=duration)
    def move_right(self, speed: float, duration: float):
        """Move right in body frame"""
        self.get_logger().info(f'Moving right at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=0.0, vy=-speed, vz=0.0, duration=duration)
    def move_left(self, speed: float, duration: float):
        """Move left in body frame"""
        self.get_logger().info(f'Moving left at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=0.0, vy=speed, vz=0.0, duration=duration)
    def move_up(self, speed: float, duration: float):
        """Move up in body frame (FLU: negative Z is up)"""
        self.get_logger().info(f'Moving up at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=0.0, vy=0.0, vz=-speed, duration=duration)
    def move_down(self, speed: float, duration: float):
        """Move down in body frame (FLU: positive Z is down)"""
        self.get_logger().info(f'Moving down at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=0.0, vy=0.0, vz=speed, duration=duration)

    # Diagonal movements
    def move_diagonal_front_right(self, speed: float=1.0, duration: float=2.0):
        self.get_logger().info(f'Moving diagonal front right at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=+speed, vy=-speed, vz=0.0, duration=duration, yaw_rate=0.0)
    def move_diagonal_front_left(self, speed: float=1.0, duration: float=2.0):
        self.get_logger().info(f'Moving diagonal front left at {speed}m/s for {duration}s') 
        self.move_body_velocity(vx=+speed, vy=+speed, vz=0.0, duration=duration, yaw_rate=0.0)
    def move_diagonal_back_right(self, speed: float=1.0, duration: float=2.0):
        self.get_logger().info(f'Moving diagonal back right at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=-speed, vy=-speed, vz=0.0, duration=duration, yaw_rate=0.0)
    def move_diagonal_back_left(self, speed: float=1.0, duration: float=2.0):
        self.get_logger().info(f'Moving diagonal back left at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=-speed, vy=+speed, vz=0.0, duration=duration, yaw_rate=0.0)
    def move_diagonal_front_right_up(self, speed: float=0.5, duration: float=2.0):
        self.get_logger().info(f'Moving diagonal front right up at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=+speed, vy=-speed, vz=-speed, duration=duration, yaw_rate=0.0)
    def move_diagonal_front_right_down(self, speed: float=0.5, duration: float=2.0):
        self.get_logger().info(f'Moving diagonal front right down at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=+speed, vy=-speed, vz=+speed, duration=duration, yaw_rate=0.0)
    def move_diagonal_front_left_up(self, speed: float=0.5, duration: float=2.0):
        self.get_logger().info(f'Moving diagonal front left up at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=+speed, vy=+speed, vz=-speed, duration=duration, yaw_rate=0.0)
    def move_diagonal_front_left_down(self, speed: float=0.5, duration: float=2.0):
        self.get_logger().info(f'Moving diagonal front left down at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=+speed, vy=+speed, vz=+speed, duration=duration, yaw_rate=0.0)
    def move_diagonal_back_right_up(self, speed: float=0.5, duration: float=2.0):
        self.get_logger().info(f'Moving diagonal back right up at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=-speed, vy=-speed, vz=-speed, duration=duration, yaw_rate=0.0)
    def move_diagonal_back_right_down(self, speed: float=0.5, duration: float=2.0):
        self.get_logger().info(f'Moving diagonal back right down at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=-speed, vy=-speed, vz=+speed, duration=duration, yaw_rate=0.0)
    def move_diagonal_back_left_up(self, speed: float=0.5, duration: float=2.0):
        self.get_logger().info(f'Moving diagonal back left up at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=-speed, vy=+speed, vz=-speed, duration=duration, yaw_rate=0.0)
    def move_diagonal_back_left_down(self, speed: float=0.5, duration: float=2.0):
        self.get_logger().info(f'Moving diagonal back left down at {speed}m/s for {duration}s')
        self.move_body_velocity(vx=-speed, vy=+speed, vz=+speed, duration=duration, yaw_rate=0.0)

    # Rotation movements
    def rotate_left(self, yaw_rate: float, duration: float):
        """Rotate left (counter-clockwise)"""
        self.get_logger().info(f'Rotating left at {yaw_rate}°/s for {duration}s')
        self.move_body_velocity(vx=0.0, vy=0.0, vz=0.0, duration=duration, yaw_rate=yaw_rate)
    def rotate_right(self, yaw_rate: float, duration: float):
        """Rotate right (clockwise)"""
        self.get_logger().info(f'Rotating right at {yaw_rate}°/s for {duration}s')
        self.move_body_velocity(vx=0.0, vy=0.0, vz=0.0, duration=duration, yaw_rate=-yaw_rate)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDroneNode()

    try:
        # Wait for connection
        while not node.mavros_subs.is_connected():
            node.get_logger().info('Waiting for MAVROS connection...')
            rclpy.spin_once(node, timeout_sec=1.0)

        node.get_logger().info('MAVROS connected.')
        
        # Perform simple test mission
        if node.arm_and_takeoff(altitude=5.0):
            node.get_logger().info('Takeoff complete!')
            
            # Hover briefly
            node.hover(3.0)

            # # Test forward/backward
            # node.move_forward(speed=1.0, duration=3.0)
            # node.hover(2.0)
            # node.move_backward(speed=1.0, duration=3.0)
            # node.hover(2.0)

            # # Test left/right
            # node.move_right(speed=1.0, duration=3.0)
            # node.hover(2.0)
            # node.move_left(speed=1.0, duration=3.0)
            # node.hover(2.0)

            # # Test rotation
            # node.rotate_right(yaw_rate=30.0, duration=3.0)  # ~90° turn
            # node.hover(2.0)
            # node.rotate_left(yaw_rate=30.0, duration=3.0)   # Turn back
            # node.hover(2.0)
            
            # # Test circle movement
            # node.move_circle_global(radius=5.0, speed=1.0, duration=30.0)
            
            # Test square movement
            node.move_square(speed=1.0, leg_s=3.0)

            # Land
            node.land()
            node.get_logger().info('Mission complete!')
        else:
            node.get_logger().error('Takeoff failed!') 
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()