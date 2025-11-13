import sys
import os
import rclpy
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../ros_ws')))
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
        
        if node.arm_and_takeoff(altitude=3.0, timeout=60.0):
            current_lat, current_lon, current_alt = node.mavros_subs.get_global_position()
            print(f"Current GPS position: lat={current_lat}, lon={current_lon}, alt={current_alt}")

            # Move to new GPS position
            new_lat = current_lat + 0.0001
            new_lon = current_lon + 0.0001
            node.goto_gps(target_lat=new_lat, target_lon=new_lon, target_alt=current_alt, timeout_s=60.0)
            node.get_logger().info(f"Moving to new GPS position: lat={new_lat}, lon={new_lon}, alt={current_alt}")
            node.return_to_launch(pos_tol_m=1.0, alt_tol_m=1.0, timeout=180.0, smart=True)
            node.land()

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()