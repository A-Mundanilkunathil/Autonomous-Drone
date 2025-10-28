from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{'fcu_url': 'udp://127.0.0.1:14550@14555'}]
        ),
        Node(
            package='autonomous_drone_nav',  
            executable='node_interface',
            name='autonomous_drone_node',
            output='screen'
        )
    ])