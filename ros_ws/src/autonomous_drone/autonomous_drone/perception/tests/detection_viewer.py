import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# COCO dataset class names (80 classes)
COCO_CLASSES = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
    'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat',
    'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack',
    'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
    'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
    'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
    'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
    'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
    'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator',
    'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
]

class DetectionViewer(Node):
    def __init__(self):
        super().__init__('detection_viewer')
        self.bridge = CvBridge()
        self.last_detections = []
        self.last_det_time = 0.0
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.fps = 0.0

        qos_image = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        qos_det = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(Image, '/camera/image_raw', self.image_cb, qos_image)
        self.create_subscription(Detection2DArray, '/detected_objects', self.det_cb, qos_det)
        self.get_logger().info('Detection viewer started')

    def det_cb(self, msg: Detection2DArray):
        self.last_detections = msg.detections
        self.last_det_time = time.time()

    def image_cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        now = time.time()
        self.frame_count += 1
        if now - self.last_fps_time >= 1.0:
            self.fps = self.frame_count / (now - self.last_fps_time)
            self.frame_count = 0
            self.last_fps_time = now

        # Draw detections with proper class names
        for det in self.last_detections:
            try:
                cx = float(det.bbox.center.position.x)
                cy = float(det.bbox.center.position.y)
                w = float(det.bbox.size_x)
                h = float(det.bbox.size_y)
                x1 = int(round(cx - w/2.0))
                y1 = int(round(cy - h/2.0))
                x2 = int(round(cx + w/2.0))
                y2 = int(round(cy + h/2.0))
                
                # Draw bounding box
                cv2.rectangle(cv_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # Get class name and confidence
                label = ""
                if det.results:
                    hyp = det.results[0].hypothesis
                    class_id = int(hyp.class_id) if hasattr(hyp, 'class_id') else 0
                    score = float(hyp.score) if hasattr(hyp, 'score') else 0.0
                    
                    # Get class name from COCO dataset
                    class_name = COCO_CLASSES[class_id] if 0 <= class_id < len(COCO_CLASSES) else f"class_{class_id}"
                    label = f"{class_name} {score:.2f}"
                
                if label:
                    # Draw label background
                    (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    cv2.rectangle(cv_img, (x1, y1 - label_h - 10), (x1 + label_w, y1), (0, 255, 0), -1)
                    # Draw label text
                    cv2.putText(cv_img, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            except Exception as e:
                self.get_logger().warning(f'Error drawing detection: {e}')
                continue

        # Overlay stats with better visibility
        det_age = 0.0 if self.last_det_time == 0 else (now - self.last_det_time)
        stats_text = f"FPS: {self.fps:.1f} | Detections: {len(self.last_detections)} | Age: {det_age:.2f}s"
        
        # Black background for stats
        cv2.rectangle(cv_img, (5, 5), (500, 30), (0, 0, 0), -1)
        cv2.putText(cv_img, stats_text, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        cv2.imshow('Detection Viewer', cv_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DetectionViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()