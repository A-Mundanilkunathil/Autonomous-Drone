import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
from queue import Queue, Empty
import threading

class SimBridgeNode(Node):
    def __init__(self):
        super().__init__('sim_bridge')
        
        # Load MiDaS model for depth estimation
        self.get_logger().info('Loading MiDaS model...')
        self.midas_model_type = "MiDaS_small"  # Fast model for real-time processing
        self.midas = torch.hub.load("intel-isl/MiDaS", self.midas_model_type)
        self.midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
        self.transform = self.midas_transforms.small_transform
        self.midas.eval()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.midas.to(self.device)
        try:
            torch.set_num_threads(1)
        except Exception:
            pass
        self.get_logger().info(f'MiDaS loaded on device: {self.device}')
        
        # Subscribe to Gazebo camera
        self.camera_subscription = self.create_subscription(
            Image,
            '/world/iris_warehouse/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image',
            self._camera_callback,
            10)
        
        # Publishers
        self.camera_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.depth_publisher = self.create_publisher(Image, '/camera/depth_map', 10)
        self.bridge = CvBridge()

        # Background depth processing
        self.depth_queue = Queue(maxsize=2)
        self.depth_running = True
        self.depth_thread = threading.Thread(target=self._depth_worker, daemon=True)
        self.depth_thread.start()

        self.get_logger().info('SimBridge started: forwarding camera to /camera/image_raw and computing depth with MiDaS')

    def _camera_callback(self, msg):
        try:
            # Forward raw image immediately
            self.camera_publisher.publish(msg)
            
            # Convert to OpenCV for depth processing
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Queue for depth processing (non-blocking)
            item = (frame_bgr, msg.header.stamp, msg.header.frame_id)
            if not self.depth_queue.full():
                self.depth_queue.put_nowait(item)
            else:
                # Drop oldest frame if queue is full
                try:
                    self.depth_queue.get_nowait()
                except Empty:
                    pass
                self.depth_queue.put_nowait(item)
                
        except Exception as e:
            self.get_logger().error(f'Error in camera callback: {e}')

    def _depth_worker(self):
        """Background thread for MiDaS depth estimation"""
        while self.depth_running:
            try:
                item = self.depth_queue.get(timeout=0.2)
            except Empty:
                continue

            if item is None:
                self.depth_queue.task_done()
                break

            frame_bgr, stamp, frame_id = item
            try:
                # Run MiDaS inference
                depth_map = self._run_midas(frame_bgr)

                # Convert to ROS Image message with encoding '32FC1'
                depth_msg = self.bridge.cv2_to_imgmsg(depth_map, encoding='32FC1')
                depth_msg.header.stamp = stamp
                depth_msg.header.frame_id = frame_id
                
                # Publish depth map
                self.depth_publisher.publish(depth_msg)
                
            except Exception as e:
                self.get_logger().error(f'Error in depth worker: {e}')
            finally:
                self.depth_queue.task_done()

    def _run_midas(self, frame_bgr) -> np.ndarray:
        """Run MiDaS depth estimation on a BGR image"""
        # Convert BGR to RGB
        img_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        # Transform input for MiDaS
        input_tensor = self.transform(img_rgb).to(self.device)
        
        # Run inference
        with torch.no_grad():
            depth_prediction = self.midas(input_tensor)

        # Resize depth map to original image size
        depth_map = torch.nn.functional.interpolate(
            depth_prediction.unsqueeze(1),
            size=img_rgb.shape[:2],
            mode='bicubic',
            align_corners=False
        ).squeeze().cpu().numpy().astype(np.float32)

        return depth_map
    
    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down SimBridge...')
        self.depth_running = False
        
        # Signal thread to exit
        try:
            self.depth_queue.put_nowait(None)
        except:
            pass
        
        # Wait for thread to finish
        if hasattr(self, 'depth_thread') and self.depth_thread.is_alive():
            self.depth_thread.join(timeout=2.0)
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
