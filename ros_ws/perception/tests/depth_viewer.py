import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class DepthViewer(Node):
    def __init__(self):
        super().__init__('depth_viewer')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth_map',
            self.listener_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                depth=10
            )
        )
        
    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to numpy array
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            # Normalize for display
            disp = np.nan_to_num(depth_img)
            disp = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX)
            disp = disp.astype(np.uint8)
            disp_color = cv2.applyColorMap(disp, cv2.COLORMAP_JET)
            cv2.imshow('Depth Map', disp_color)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Failed to display depth image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DepthViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
