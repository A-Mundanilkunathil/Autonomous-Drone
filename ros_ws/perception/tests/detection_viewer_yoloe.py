import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class DetectionViewer(Node):
    def __init__(self):
        super().__init__('detection_viewer')
        self.bridge = CvBridge()
        self.last_detections = []
        self.last_det_time = 0.0
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.fps = 0.0
        
        # Your text prompt class names, ordered by class_id from YOLOE outputs
        self.prompt_classes =  ["can", "box", "shelf", "fan", "barrel", "floor"]

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

        for det in self.last_detections:
            try:
                hyp = det.results[0].hypothesis if det.results else None
                if not hyp:
                    continue
                class_id = int(hyp.class_id)
                if class_id < 0 or class_id >= len(self.prompt_classes):
                    continue  # Skip classes outside your prompted ones

                class_name = self.prompt_classes[class_id]
                score = float(hyp.score)

                # Bounding box
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

                label = f"{class_name} {score:.2f}"
                (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(cv_img, (x1, y1 - label_h - 10), (x1 + label_w, y1), (0, 255, 0), -1)
                cv2.putText(cv_img, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

            except Exception as e:
                self.get_logger().warning(f'Error drawing detection: {e}')
                continue

        det_age = 0.0 if self.last_det_time == 0 else (now - self.last_det_time)
        stats_text = f"FPS: {self.fps:.1f} | Detections: {len(self.last_detections)} | Age: {det_age:.2f}s"
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
