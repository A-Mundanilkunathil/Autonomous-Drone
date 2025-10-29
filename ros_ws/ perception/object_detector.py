import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector')

        self.bridge = CvBridge()
        self.camera_connected = False
        
        # QoS profile 
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile
        )
        
        # Statistics
        self.frame_count = 0
        self.detection_count = 0

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
        if not self.camera_connected:
            self.get_logger().info('Camera stream connected!')
            self.camera_connected = True

        # Convert ROS image to OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert ROS image to OpenCV: {e}')
            return

        if not self.detector:
            self.get_logger().error('Detector not initialized')
            return
        
        self.frame_count += 1
        
        # Run detection
        results = self.detector.predict(cv_image, verbose=False)

        # Convert results to ROS messages
        detection_array_msg = Detection2DArray()
        detection_array_msg.header = msg.header

        # Iterate over detected objects
        for box in results[0].boxes:
            det = Detection2D()
            det.header = msg.header

            # Bounding box center and size 
            det.bbox.center.position.x = float(box.xywh[0][0])
            det.bbox.center.position.y = float(box.xywh[0][1])
            det.bbox.center.theta = 0.0  # No rotation
            det.bbox.size_x = float(box.xywh[0][2])
            det.bbox.size_y = float(box.xywh[0][3])

            # Class and confidence
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(int(box.cls[0]))
            hypothesis.hypothesis.score = float(box.conf[0])
            det.results.append(hypothesis)

            detection_array_msg.detections.append(det)
            self.detection_count += 1

        # Publish results
        if len(detection_array_msg.detections) > 0:
            self.detection_pub.publish(detection_array_msg)
            self.get_logger().info(
                f'Frame {self.frame_count}: Detected {len(detection_array_msg.detections)} objects',
                throttle_duration_sec=2.0
            )

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Shutting down - Processed {node.frame_count} frames, '
            f'{node.detection_count} total detections'
        )
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()