import sys
import os
import rclpy
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from node_interface import AutonomousDroneNode

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDroneNode()

    try:
        # Wait for MAVROS connection
        while not node.mavros_subs.is_connected():
            node.get_logger().info('Waiting for MAVROS connection...')
            rclpy.spin_once(node, timeout_sec=1.0)

        node.get_logger().info('MAVROS connected.')
        node.mavros_srvs.set_home()
        
        if node.arm_and_takeoff(altitude=4.0, timeout=60.0):
            node.start_follow()
            node._sleep_nonblocking(60.0)  # Follow for 30 seconds
            node.stop_follow()
            node.return_to_launch(pos_tol_m=1.0, alt_tol_m=1.0, timeout=180.0, smart=True)
            node.land()

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()