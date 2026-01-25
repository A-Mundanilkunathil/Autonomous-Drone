#!/usr/bin/env python3
import sys
import os
import rclpy

# Add the installed package to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', 'install', 'autonomous_drone', 'lib', 'python3.12', 'site-packages'))

from autonomous_drone.node_interface import AutonomousDroneNode

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
        
        if node.arm_and_takeoff(altitude=10.0, timeout=60.0):
            node.move_circle_global(radius=2.0, speed=1.0, duration=20.0)
            node.get_logger().info('Takeoff complete!')
            
            # Get current GPS position
            current_lat, current_lon, _ = node.mavros_subs.get_global_position()
            current_alt = node.mavros_subs.get_relative_altitude()
            node.get_logger().info(f"Current GPS: lat={current_lat:.6f}, lon={current_lon:.6f}, alt={current_alt:.2f}m")

            # Define target GPS position 
            target_lat = current_lat + 0.00003 # 3 meters north
            target_lon = current_lon + 0.00003 # 3 meters east
            target_alt = current_alt # maintain altitude
            
            node.get_logger().info(f"Starting GPS mission to lat={target_lat:.6f}, lon={target_lon:.6f}")
            
            # Start GPS mission - this will navigate to the target using only GPS
            node.start_mission_gps(lat=target_lat, lon=target_lon, alt=target_alt, tolerance_m=2.5)
            node.wait_until_mission_complete_or_timeout(timeout_s=90.0)

            # Get current GPS position
            current_lat, current_lon, _ = node.mavros_subs.get_global_position()
            current_alt = node.mavros_subs.get_relative_altitude()
            node.get_logger().info(f"Current GPS: lat={current_lat:.6f}, lon={current_lon:.6f}, alt={current_alt:.2f}m")

            # Define target GPS position 
            target_lat = current_lat - 0.00003 # 3 meters south
            target_lon = current_lon - 0.00003 # 3 meters west
            target_alt = current_alt # maintain altitude
            
            node.start_mission_gps(lat=target_lat, lon=target_lon, alt=target_alt, tolerance_m=2.5)
            node.wait_until_mission_complete_or_timeout(timeout_s=90.0)

            # Stop mission
            node.stop_mission()
            node.get_logger().info('Mission complete, returning home...')
            node.return_to_launch(pos_tol_m=3.0, alt_tol_m=1.0, timeout=180.0, smart=True)
            node.land()
            node.get_logger().info('All done!')
        else:
            node.get_logger().error('Takeoff failed!')
            
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt detected!')
    finally:
        try:
            node.stop_mission()
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()