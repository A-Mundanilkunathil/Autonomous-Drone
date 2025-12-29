import socket, struct, time, threading, os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
from queue import Queue, Empty
from ament_index_python.packages import get_package_share_directory

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
        self.midas_model_type = "MiDaS_small" # or "DPT_Large", "DPT_Hybrid"
        self.midas = torch.hub.load("intel-isl/MiDaS", self.midas_model_type)
        self.midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
        self.transform = (self.midas_transforms.dpt_transform
                          if self.midas_model_type in ["DPT_Large","DPT_Hybrid"]
                          else self.midas_transforms.small_transform)
        self.midas.eval()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.midas.to(self.device)
        try: torch.set_num_threads(1)
        except Exception: pass

        # MiDaS depth calibration 
        self.midas_scale = 140.0  
        self.midas_shift = 0.13  
        self._load_midas_calibration()

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
        try: # Set receive buffer
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024) # 4MB
        except socket.error:
            pass
        
        # Inflight frames
        self.max_inflight_frames = 8

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
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

        # Background decoder thread
        self.assembled_queue = Queue(maxsize=4)
        self.decode_running = True
        self.decode_thread = threading.Thread(target=self._decode_publish_worker, daemon=True)
        self.decode_thread.start()

        # Background depth thread
        self.depth_queue = Queue(maxsize=2)
        self.depth_running = True
        self.depth_thread = threading.Thread(target=self._depth_publish_worker, daemon=True)
        self.depth_thread.start()

        # Periodic flush of stale frames
        self.timer = self.create_timer(0.5, self.flush_stale_frames)
        self.get_logger().info(f'UDP Frame Receiver initialized on {self.bind_ip}:{self.port}')
    
    def _load_midas_calibration(self):
        """Load MiDaS calibration from .npz file for metric depth conversion"""
        try:
            pkg_share = get_package_share_directory('autonomous_drone')
            calib_path = os.path.join(pkg_share, 'config', 'esp32_midas_calibration.npz')
            
            if os.path.exists(calib_path):
                calib = np.load(calib_path)
                self.midas_scale = float(calib['scale'])
                self.midas_shift = float(calib['shift'])
                self.get_logger().info(
                    f'Loaded MiDaS calibration: scale={self.midas_scale:.2f}, shift={self.midas_shift:.3f}'
                )
            else:
                self.get_logger().warn(
                    f'MiDaS calibration not found at {calib_path}, using defaults'
                )
        except Exception as e:
            self.get_logger().warn(f'Failed to load MiDaS calibration: {e}, using defaults')

    def _midas_to_metric(self, midas_depth: np.ndarray) -> np.ndarray:
        """
        Convert MiDaS inverse depth to metric depth in meters.
        Formula: depth_meters = scale / midas_inverse_depth + shift
        """
        safe_depth = np.clip(midas_depth, 0.1, None)
        metric_depth = self.midas_scale / safe_depth + self.midas_shift
        metric_depth = np.clip(metric_depth, 0.1, 50.0)
        return metric_depth.astype(np.float32)

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

    def _rx_loop(self):
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
                        # If too many frames in flight, drop oldest
                        if len(self.frames) >= self.max_inflight_frames:
                            oldest_id = min(self.frames.items(), key=lambda kv: kv[1]['first_ts'])[0]
                            del self.frames[oldest_id]

                        self.frames[fid] = f = {
                            'total': total_pkts,
                            'size': frame_size,
                            'parts': [None] * total_pkts,
                            'filled': 0,
                            'first_ts': time.time()
                        }
                    
                    # De-duplicate and store
                    if 0 <= pkt_no < f['total'] and f['parts'][pkt_no] is None:
                        f['parts'][pkt_no] = payload # Store
                        f['filled'] += 1
                        
                        # Check if complete (do this inside lock)
                        if f['filled'] == f['total']:
                            # Extract data we need, then release lock quickly
                            completed_frame = {
                                'fid': fid,
                                'parts': f['parts'],
                                'size': f['size']
                            }
                            del self.frames[fid]
                            self.stats['frames_received'] += 1

                if completed_frame is not None:
                    if self.assembled_queue.full():
                        try:
                            self.assembled_queue.get_nowait()
                        except Empty:
                            pass
                    self.assembled_queue.put_nowait(completed_frame)

            except struct.error as e:
                self.get_logger().warning(f'Packet parse error: {e}')
            except Exception as e:
                self.get_logger().error(f'Unexpected error in rx_loop: {e}')
    
    def _decode_publish_worker(self):
        while self.decode_running:
            try:
                frame_data = self.assembled_queue.get(timeout=0.2)
            except Empty:
                continue

            if frame_data is None:
                self.assembled_queue.task_done()
                break

            try:
                # Assemble JPEG
                jpeg_bytes = b''.join(frame_data['parts'])

                # Sanity check
                if len(jpeg_bytes) != frame_data['size']:
                    self.get_logger().warning(f"Frame size mismatch: expected {frame_data['size']}, got {len(jpeg_bytes)}")
                    self.stats['frames_dropped'] += 1
                    continue
                
                # Decode JPEG -> BGR
                npbuf = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                frame_bgr = cv2.imdecode(npbuf, cv2.IMREAD_COLOR) 
                if frame_bgr is None:
                    self.get_logger().warning('Failed to decode JPEG frame')
                    self.stats['frames_dropped'] += 1
                    continue
                
                self.stats['frames_decoded'] += 1

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
                img_msg = self.bridge.cv2_to_imgmsg(frame_bgr, encoding='bgr8')
                now = self.get_clock().now().to_msg()
                img_msg.header.stamp = now
                img_msg.header.frame_id = self.frame_id_str
                self.pub_img.publish(img_msg)

                # Publish CameraInfo
                cam_info = self.cam_info
                cam_info.header.stamp = now
                cam_info.header.frame_id = self.frame_id_str  
                self.pub_info.publish(cam_info)

                # Hand off to depth worker
                self._enqueue_depth(frame_bgr, now)
            
            except Exception as e:
                self.get_logger().error(f'Unexpected error in decode_publish_worker: {e}')
            finally:
                self.assembled_queue.task_done()
    
    def _enqueue_depth(self, frame_bgr, stamp):
        item = (frame_bgr, stamp)
        if not self.depth_queue.full():
            self.depth_queue.put_nowait(item)
        else:
            # Drop oldest
            try:
                _ = self.depth_queue.get_nowait()
            except Empty:
                pass
            self.depth_queue.put_nowait(item)
    
    def _depth_publish_worker(self):
        while self.depth_running:
            try:
                item = self.depth_queue.get(timeout=0.2)
            except Empty:
                continue

            if item is None:
                self.depth_queue.task_done()
                break

            frame_bgr, stamp = item
            try:
                raw_depth = self._run_midas(frame_bgr)
                metric_depth = self._midas_to_metric(raw_depth)

                depth_msg = self.bridge.cv2_to_imgmsg(metric_depth, encoding='32FC1')
                depth_msg.header.stamp = stamp
                depth_msg.header.frame_id = self.frame_id_str
                self.pub_depth.publish(depth_msg)
            except Exception as e:
                self.get_logger().error(f'Unexpected error in depth_publish_worker: {e}')
            finally:
                self.depth_queue.task_done()

    def _run_midas(self, frame_bgr) -> np.ndarray:
        # Convert BGR to RGB
        img_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        # Transform input for MiDaS
        input_tensor = self.transform(img_rgb).to(self.device)
        
        with torch.no_grad():
            depth_prediction = self.midas(input_tensor)

        depth_map = torch.nn.functional.interpolate(
            depth_prediction.unsqueeze(1),
            size=img_rgb.shape[:2],
            mode='bicubic',
            align_corners=False
        ).squeeze().cpu().numpy().astype(np.float32)

        return depth_map

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
        self.decode_running = False
        self.depth_running = False

        # Close socket to unblock recvfrom
        try: self.sock.close()
        except: pass
        
        # Nudge thread to exit
        try: self.assembled_queue.put_nowait(None)
        except: pass
        try: self.depth_queue.put_nowait(None)
        except: pass

        # Wait for thread to finish
        if getattr(self, "rx_thread", None) and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=2.0)
        if getattr(self, "decode_thread", None) and self.decode_thread.is_alive():
            self.decode_thread.join(timeout=2.0)
        if getattr(self, "depth_thread", None) and self.depth_thread.is_alive():
            self.depth_thread.join(timeout=2.0)
        
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
            
