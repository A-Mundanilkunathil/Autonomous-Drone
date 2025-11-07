import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('sim_bridge')
        self.camera_subscription = self.create_subscription(
            Image,
            '/world/iris_warehouse/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image',
            self._camera_callback,
            10)
        self.depth_subscription = self.create_subscription(
            Image,
            '/world/iris_warehouse/model/depth_camera/link/link/sensor/camera/depth_image',
            self._depth_callback,
            10)

        self.camera_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.depth_publisher = self.create_publisher(Image, '/camera/depth_map', 10)
        self.bridge = CvBridge()

        self.get_logger().info('ImageSaverNode started, forwarding images to /camera/image_raw and /camera/depth_map')

    def _camera_callback(self, msg):
        try:
            self.camera_publisher.publish(msg)
            self.get_logger().info('Published image to /camera/image_raw')
        except Exception as e:
            self.get_logger().error(f'Error publishing camera image: {e}')

    def _depth_callback(self, msg):
        try:
            self.depth_publisher.publish(msg)
            self.get_logger().info('Published depth image to /camera/depth_map')
        except Exception as e:
            self.get_logger().error(f'Error publishing depth image: {e}')

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
