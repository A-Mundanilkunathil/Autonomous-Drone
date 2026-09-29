import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO

class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector')

        self.bridge = CvBridge()
        self.camera_connected = False
        
        # QoS profile 
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            # Inference is slower than the camera. Keeping only the newest
            # frame prevents seconds of stale detections from accumulating.
            depth=1
        )

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            qos_profile
        )
        
        # Statistics
        self.frame_count = 0
        self.detection_count = 0

        # Publish detected objects
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            'detected_objects',
            10
        )

        self.model_path = self.declare_parameter('model_path', 'yolov8n.pt').value
        self.confidence_threshold = float(
            self.declare_parameter('confidence_threshold', 0.25).value
        )
        self.max_detections = int(
            self.declare_parameter('max_detections', 50).value
        )

        # Load detection model
        self.detector = self.load_model()
        if self.detector is None:
            raise RuntimeError(f'Unable to load detector model: {self.model_path}')

    def load_model(self):
        # Load YOLOv8s model
        try:
            model = YOLO(self.model_path)

            self.get_logger().info(f'Loaded detector model: {self.model_path}')
            return model
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLOv8s model: {e}')
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

        self.frame_count += 1

        # Convert results to ROS messages
        detection_array_msg = Detection2DArray()
        detection_array_msg.header = msg.header

        try:
            results = self.detector.predict(
                cv_image,
                conf=self.confidence_threshold,
                max_det=self.max_detections,
                verbose=False,
            )
        except Exception as e:
            # Publish an empty, correctly stamped result so consumers can
            # immediately age out an old target instead of flying on stale data.
            self.get_logger().error(
                f'Detector inference failed: {e}', throttle_duration_sec=2.0)
            self.detection_pub.publish(detection_array_msg)
            return

        if not results:
            self.detection_pub.publish(detection_array_msg)
            return

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
            # Get class name from model's names dictionary
            class_idx = int(box.cls[0])
            names = self.detector.names
            class_name = (names.get(class_idx, str(class_idx))
                          if isinstance(names, dict)
                          else names[class_idx])
            hypothesis.hypothesis.class_id = class_name
            hypothesis.hypothesis.score = float(box.conf[0])
            det.results.append(hypothesis)

            detection_array_msg.detections.append(det)
            self.detection_count += 1

        # Publish results
        self.detection_pub.publish(detection_array_msg)
        if len(detection_array_msg.detections) > 0:
            self.get_logger().info(
                f'Frame {self.frame_count}: Detected {len(detection_array_msg.detections)} objects',
                throttle_duration_sec=2.0
            )

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
