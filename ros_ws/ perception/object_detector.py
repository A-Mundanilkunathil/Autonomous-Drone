import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector')

        self.bridge = CvBridge()

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publish detected objects
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detected_objects',
            10
        )

        # Load detection model
        self.detector = self.load_model()

    def load_model(self):
        # Load YOLOv8n model
        try:
            model = YOLO('yolov8n.pt')
            self.get_logger().info('Loaded YOLOv8n model')
            return model
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLOv8n model: {e}')
            return None
            
    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        if not self.detector:
            self.get_logger().error('Detector not initialized')
            return
        
        # Run detection
        results = self.detector.predict(cv_image)

        # Convert results to ROS messages
        detection_array_msg = Detection2DArray()
        detection_array_msg.header = msg.header

        # Iterate over detected objects
        for box in results[0].boxes:
            det = Detection2D()
            det.header = msg.header

            # Bounding box center and size
            det.bbox.center.x = float(box.xywh[0][0])
            det.bbox.center.y = float(box.xywh[0][1])
            det.bbox.size_x = float(box.xywh[0][2])
            det.bbox.size_y = float(box.xywh[0][3])

            # Class and confidence
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = int(box.cls[0])
            hypothesis.score = float(box.conf[0])
            det.results.append(hypothesis)

            detection_array_msg.detections.append(det)

        # Publish results
        self.detection_pub.publish(detection_array_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()