import socket, struct, time, threading, collections
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch

# UDP packet structure
# uint32_t: frame_id
# uint16_t: packet_num
# uint16_t: total_packets
# uint32_t: frame_size
# uint16_t: data_size
HDR_FMT = "<IHHIH" # 4 + 2 + 2 + 4 + 2 = 14 bytes
HDR_SIZE = struct.calcsize(HDR_FMT)

class UdpFrameReceiver(Node):
    def __init__(self):
        super().__init__('udp_frame_receiver')

        # Load MiDaS model
        self.midas_model_type = "DPT_Hybrid"
        self.midas = torch.hub.load("intel-isl/MiDaS", self.midas_model_type)
        self.midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")

        if self.midas_model_type in ["DPT_Large", "DPT_Hybrid"]:
            self.transform =self.midas_transforms.dpt_transform
        else:
            self.transform = self.midas_transforms.small_transform
        
        self.midas.eval()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.midas.to(self.device)

        # Parameters
        self.port = int(self.declare_parameter('port', 5005).value)
        self.bind_ip = self.declare_parameter('bind_ip', '0.0.0.0').value
        self.frame_id_str = self.declare_parameter('frame_id', 'camera').value
        self.expected_width = int(self.declare_parameter('expected_width', 640).value)
        self.expected_height = int(self.declare_parameter('expected_height', 480).value)

        # QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ROS pubs
        self.pub_img = self.create_publisher(Image, '/camera/image_raw', qos_profile)
        self.pub_depth = self.create_publisher(Image, '/camera/depth_map', qos_profile)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/camera_info', qos_profile)

        self.bridge = CvBridge()

        # CameraInfo
        self.cam_info = CameraInfo()
        self.cam_info.width  = self.expected_width
        self.cam_info.height = self.expected_height
        self.cam_info.distortion_model = 'plumb_bob'
        self.cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        fx = fy = 600.0
        cx = self.cam_info.width  / 2.0
        cy = self.cam_info.height / 2.0
        self.cam_info.k = [fx, 0.0, cx,
                        0.0, fy, cy,
                        0.0, 0.0, 1.0]
        self.cam_info.r = [1.0,0.0,0.0,
                        0.0,1.0,0.0,
                        0.0,0.0,1.0]
        self.cam_info.p = [fx, 0.0, cx, 0.0,
                        0.0, fy, cy, 0.0,
                        0.0, 0.0, 1.0, 0.0]

        # Load calibration if provided
        calib_path = self.declare_parameter('calib_npz', '').value
        if calib_path and calib_path != '':
            try:
                self.cam_info = self.load_caminfo_from_npz(calib_path)
                self.get_logger().info(f'Loaded camera calibration from {calib_path}')
            except FileNotFoundError:
                self.get_logger().warning(f'Calibration file not found: {calib_path}')
            except Exception as e:
                self.get_logger().warning(f'Failed to load calibration: {e}')

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.bind_ip, self.port))
        self.sock.settimeout(0.5)

        # Reassembly buffers: frame_id -> dict(packet_num->bytes) + metadata
        self.frames = {} 
        self.lock = threading.Lock()
        
        # Statistics
        self.stats = {
            'frames_received': 0,
            'frames_decoded': 0,
            'frames_dropped': 0,
            'last_log_time': time.time()
        }

        # Background receiver thread
        self.running = True
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        # Periodic flush of stale frames
        self.timer = self.create_timer(0.5, self.flush_stale_frames)
        self.get_logger().info(f'UDP Frame Receiver initialized on {self.bind_ip}:{self.port}')
    
    def load_caminfo_from_npz(self, path: str) -> CameraInfo:
        data = np.load(path)
        
        # Camera matrix
        K = data.get('mtx')
        if K is None:
            K = data.get('K')
        if K is None:
            K = data.get('camera_matrix')
        
        # Distortion coefficients
        D = data.get('dist')
        if D is None:
            D = data.get('D')
        if D is None:
            D = data.get('dist_coeffs')

        if K is None or D is None:
            available_keys = list(data.keys())
            raise ValueError(f'Invalid calibration data in NPZ file. Available keys: {available_keys}. Expected keys: mtx/K/camera_matrix and dist/D/dist_coeffs')
        
        cam_info = CameraInfo()
        cam_info.width = int(data.get('width', self.expected_width))
        cam_info.height = int(data.get('height', self.expected_height))
        cam_info.distortion_model = 'plumb_bob'
        cam_info.k = K.astype(float).flatten().tolist()
        cam_info.d = D.astype(float).flatten().tolist()

        # Identity rectification and projection from K
        cam_info.r = [1.0, 0.0, 0.0,
                       0.0, 1.0, 0.0,
                       0.0, 0.0, 1.0]
        cam_info.p = [K[0,0], 0.0, K[0,2], 0.0,
                       0.0, K[1,1], K[1,2], 0.0,
                       0.0, 0.0, 1.0, 0.0]
        return cam_info

    def rx_loop(self):
        """Background thread to receive UDP packets"""
        while self.running:
            try:
                data, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:  # Only log if not shutting down
                    self.get_logger().error(f'UDP receive error: {e}')
                continue

            if len(data) < HDR_SIZE:
                continue
            
            try:
                # Parse header
                (fid, pkt_no, total_pkts, frame_size, data_size) = struct.unpack(HDR_FMT, data[:HDR_SIZE])
                payload = data[HDR_SIZE:HDR_SIZE+data_size]
                if len(payload) != data_size:
                    continue

                # Variable to hold completed frame data (outside lock)
                completed_frame = None

                # Store packet (only hold lock briefly)
                with self.lock:
                    f = self.frames.get(fid)
                    if f is None:
                        self.frames[fid] = f = {
                            'total': total_pkts,
                            'size': frame_size,
                            'parts': {},
                            'first_ts': time.time()
                        }
                    
                    # De-duplicate
                    if pkt_no not in f['parts']:
                        f['parts'][pkt_no] = payload
                        
                        # Check if complete (do this inside lock)
                        if len(f['parts']) == f['total']:
                            # Extract data we need, then release lock quickly
                            completed_frame = {
                                'fid': fid,
                                'parts': [f['parts'][i] for i in range(f['total'])],
                                'size': f['size']
                            }
                            del self.frames[fid]
                            self.stats['frames_received'] += 1

                if completed_frame is not None:
                    self.assemble_and_publish_async(completed_frame)
            
            except struct.error as e:
                self.get_logger().warning(f'Packet parse error: {e}')
            except Exception as e:
                self.get_logger().error(f'Unexpected error in rx_loop: {e}')

    def assemble_and_publish_async(self, frame_data):
        """Assemble and publish frame without holding the lock"""
        try:
            # Concatenate parts
            jpeg_bytes = b''.join(frame_data['parts'])

            # Sanity check
            if len(jpeg_bytes) != frame_data['size']:
                self.get_logger().warning(f"Frame size mismatch: expected {frame_data['size']}, got {len(jpeg_bytes)}")
                self.stats['frames_dropped'] += 1
                return

            # Decode JPEG -> BGR
            npbuf = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            frame = cv2.imdecode(npbuf, cv2.IMREAD_COLOR) 
            if frame is None:
                self.get_logger().warning('Failed to decode JPEG frame')
                self.stats['frames_dropped'] += 1
                return
            
            self.stats['frames_decoded'] += 1
            
            # --------------------------------- MiDaS depth map ---------------------------------
            # Convert BGR to RGB
            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Transform input for MiDaS
            input_tensor = self.transform(img_rgb).to(self.device)
            
            with torch.no_grad():
                depth_prediction = self.midas(input_tensor)

            depth_map = torch.nn.functional.interpolate(
                depth_prediction.unsqueeze(1),
                size=img_rgb.shape[:2],
                mode='bicubic',
                align_corners=False
            ).squeeze().cpu().numpy()

            # Convert to float32
            depth_raw = depth_map.astype(np.float32)

            # Convert to ROS Image message with encoding '32FC1'
            depth_msg = self.bridge.cv2_to_imgmsg(depth_raw, encoding='32FC1')
            # --------------------------------------------------------------------------------------

            # Log stats every 5 seconds
            current_time = time.time()
            if current_time - self.stats['last_log_time'] > 5.0:
                self.get_logger().info(
                    f"Stats: Received={self.stats['frames_received']}, "
                    f"Decoded={self.stats['frames_decoded']}, "
                    f"Dropped={self.stats['frames_dropped']}"
                )
                self.stats['last_log_time'] = current_time
            
            # Publish frame
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            now = self.get_clock().now().to_msg()
            img_msg.header.stamp = now
            img_msg.header.frame_id = self.frame_id_str

            # Publish CameraInfo
            cam_info = self.cam_info
            cam_info.header.stamp = now
            cam_info.header.frame_id = self.frame_id_str  

            self.pub_img.publish(img_msg)
            self.pub_info.publish(cam_info)

            # Publish depth map
            depth_msg.header.stamp = now
            depth_msg.header.frame_id = self.frame_id_str
            self.pub_depth.publish(depth_msg)
        
        except Exception as e:
            self.get_logger().error(f'Failed to assemble/publish frame: {e}')

    def flush_stale_frames(self):
        # Drop frames older than 0.5s
        now = time.time()
        stale = []
        with self.lock:
            for fid, f in self.frames.items():
                if now - f['first_ts'] > 0.5:
                    stale.append(fid)
            for fid in stale:
                del self.frames[fid]
    
    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down UDP receiver...')
        self.running = False
        
        # Close socket to unblock recvfrom
        try:
            self.sock.close()
        except:
            pass
        
        # Wait for thread to finish
        if self.rx_thread.is_alive():
            try:
                self.rx_thread.join(timeout=2.0)
            except:
                pass
        
        super().destroy_node()

def main():
    rclpy.init()
    node = UdpFrameReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
            
