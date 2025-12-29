import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to MiDaS calibration file (in package)
    pkg_dir = get_package_share_directory('autonomous_drone')
    calib_path = os.path.join(pkg_dir, 'config', 'esp32_midas_calibration.npz')
    
    # Fallback to source path if package not installed yet
    if not os.path.exists(calib_path):
        calib_path = '/home/hp/Desktop/Autonomous-Drone/ros_ws/src/autonomous_drone/autonomous_drone/perception/esp32_midas_calibration.npz'
    
    return LaunchDescription([
        # Sim bridge for depth processing
        Node(
            package='autonomous_drone',
            executable='sim_bridge',
            name='sim_bridge',
            output='screen'
        ),
        
        # Object detector (YOLO)
        Node(
            package='autonomous_drone',
            executable='object_detector',
            name='object_detector',
            output='screen'
        ),
        
        # Object avoidance with obstacle detection
        Node(
            package='autonomous_drone',
            executable='object_avoidance',
            name='object_avoidance',
            output='screen',
            parameters=[{'midas_calib_npz': calib_path}]
        ),
        
        # Object following with tracking
        Node(
            package='autonomous_drone',
            executable='object_following',
            name='object_following',
            output='screen',
            parameters=[{'midas_calib_npz': calib_path}]
        ),
        
        # Main control node 
        Node(
            package='autonomous_drone',
            executable='node_interface',
            name='node_interface',
            output='screen'
        ),
    ])