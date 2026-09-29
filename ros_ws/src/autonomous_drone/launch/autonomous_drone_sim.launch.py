import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to MiDaS calibration file (in package)
    pkg_dir = get_package_share_directory('autonomous_drone')
    calib_path = os.path.join(pkg_dir, 'config', 'esp32_midas_calibration.npz')
    
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Sim bridge for depth processing
        Node(
            package='autonomous_drone',
            executable='sim_bridge',
            name='sim_bridge',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        
        # Object detector (YOLO)
        Node(
            package='autonomous_drone',
            executable='object_detector',
            name='object_detector',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Object avoidance with obstacle detection
        Node(
            package='autonomous_drone',
            executable='object_avoidance',
            name='object_avoidance',
            namespace=namespace,
            output='screen',
            parameters=[{'midas_calib_npz': calib_path}, {'use_sim_time': use_sim_time}]
        ),
        
        # Object following with tracking
        Node(
            package='autonomous_drone',
            executable='object_following',
            name='object_following',
            namespace=namespace,
            output='screen',
            parameters=[{'midas_calib_npz': calib_path}, {'use_sim_time': use_sim_time}]
        ),
        
        # VSLAM — visual odometry, sparse map, and virtual GPS
        Node(
            package='autonomous_drone',
            executable='vslam_node',
            name='vslam_node',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Main control node
        Node(
            package='autonomous_drone',
            executable='node_interface',
            name='node_interface',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
