import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        self.subscription = self.create_subscription(
            Image,
            '/world/iris_warehouse/model/camera/link/link/sensor/camera/image',
            self._listener_callback,
            10)
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.get_logger().info('ImageSaverNode started, forwarding images to /camera/image_raw')

    def _listener_callback(self, msg):
        try:
            self.publisher.publish(msg)
            self.get_logger().info('Published image to /camera/image_raw')
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
