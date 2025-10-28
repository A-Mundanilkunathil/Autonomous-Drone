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

    def arm_and_takeoff(self, altitude: float) -> bool:
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

        # Takeoff
        if not self.mavros_srvs.takeoff(altitude):
            return False
        
        self.get_logger().info('Takeoff successful.')
        return True

    def move_body_velocity(self, vx: float, vy: float, vz: float, duration: float = 2.0):
        """
        Move with velocity for specified duration
        
        Why: Simple "move forward for 2 seconds" commands
        Usage: node.move_body_velocity(1.0, 0, 0, 5.0)  # Move forward 1m/s for 5s
        """
        import time
        rate_hz = 10
        dt = 1.0 / rate_hz
        end_time = time.time() + duration

        while time.time() < end_time:
            self.mavros_pubs.publish_velocity_body(vx, vy, vz)
            time.sleep(dt)
        
    def goto_position(self, x: float, y: float, z: float, duration: float = 5.0):
        """
        Go to position and hold
        
        Why: Waypoint navigation
        Usage: node.goto_position(10, 5, -5, 10.0)  # Go to (10,5,-5) over 10s
        """
        import time
        rate_hz = 10
        dt = 1.0 / rate_hz
        end_time = time.time() + duration

        while time.time() < end_time:
            self.mavros_pubs.publish_position(x, y, z)
            time.sleep(dt)

def main(args=None):
    rclpy.init(args=args)

    node = AutonomousDroneNode()

    try:
        # Wait for connection
        while not node.mavros_subs.is_connected():
            node.get_logger().info('Waiting for MAVROS connection...')
            rclpy.spin_once(node, timeout_sec=1.0)

        node.get_logger().info('MAVROS connected.')
        
        node.arm_and_takeoff(altitude=5.0)
        
        # Keep the node alive
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()