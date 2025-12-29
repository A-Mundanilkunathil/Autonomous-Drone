import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('autonomous_drone')
    
    # Path to MiDaS calibration file for perception
    midas_calib_path = os.path.join(pkg_dir, 'config', 'esp32_midas_calibration.npz')
    if not os.path.exists(midas_calib_path):
        midas_calib_path = '/home/hp/Desktop/Autonomous-Drone/ros_ws/src/autonomous_drone/autonomous_drone/perception/esp32_midas_calibration.npz'
    
    # Path to camera calibration file for UDP receiver
    camera_calib_path = os.path.join(pkg_dir, 'config', 'camera_calib.npz')
    if not os.path.exists(camera_calib_path):
        camera_calib_path = '/home/hp/Desktop/Autonomous-Drone/ros_ws/src/autonomous_drone/autonomous_drone/bridges/camera_calib.npz'
    
    return LaunchDescription([
        # UDP custom receiver for real hardware camera stream
        Node(
            package='autonomous_drone',
            executable='udp_custom_receiver',
            name='udp_custom_receiver',
            output='screen',
            parameters=[
                {'calib_npz': camera_calib_path},
                {'port': 5005},
                {'expected_width': 640},
                {'expected_height': 480}
            ]
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
            parameters=[{'midas_calib_npz': midas_calib_path}]
        ),
        
        # Object following with tracking
        Node(
            package='autonomous_drone',
            executable='object_following',
            name='object_following',
            output='screen',
            parameters=[{'midas_calib_npz': midas_calib_path}]
        ),
        
        # Main control node 
        Node(
            package='autonomous_drone',
            executable='node_interface',
            name='node_interface',
            output='screen'
        ),
    ])
