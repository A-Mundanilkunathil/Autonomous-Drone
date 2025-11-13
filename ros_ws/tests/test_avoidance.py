import sys
import os
import rclpy
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from node_interface import AutonomousDroneNode

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDroneNode()

    try:
        while not node.mavros_subs.is_connected():
            node.get_logger().info('Waiting for MAVROS connection...')
            rclpy.spin_once(node, timeout_sec=1.0)

        node.get_logger().info('MAVROS connected.')
        node.mavros_srvs.set_home()

        if node.arm_and_takeoff(altitude=2.0, timeout=60.0):
            node.get_logger().info('Takeoff complete!')
            node.get_logger().info('Hovering for 3s...')
            node._sleep_nonblocking(3, rate_hz=50.0)

            node.get_logger().info('Starting autonomous mission with obstacle avoidance...')
            node.start_mission()
            node._sleep_nonblocking(60, rate_hz=50)

            node.stop_mission()
            node.get_logger().info('Mission complete, returning home...')
            node.return_to_launch(pos_tol_m=1.0, alt_tol_m=1.0, timeout=180.0, smart=True)
            node.land()
            node.get_logger().info('All done!')
        else:
            node.get_logger().error('Takeoff failed!')
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt detected!')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()