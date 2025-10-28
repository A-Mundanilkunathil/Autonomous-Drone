import rclpy
from rclpy.node import Node
from .publishers import MavrosPublishers
from .subscribers import MavrosSubscribers
from .services import MavrosServices

def main():
    rclpy.init()
    node = Node('autonomous_drone_node')

    pubs = MavrosPublishers(node)
    subs = MavrosSubscribers(node)
    srvs = MavrosServices(node)

    node.get_logger().info('MAVROS interface ready.')
    rclpy.spin(node)

if __name__ == '__main__':
    main()