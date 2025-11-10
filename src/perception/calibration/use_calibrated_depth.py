#!/usr/bin/env python3

import cv2
import numpy as np
from calibrate_depth import load_midas_model, estimate_midas_depth, apply_calibration, load_midas_calibration
import socket
import struct
import threading
from collections import defaultdict
from queue import Queue, Empty
import time

UDP_IP = "192.168.1.89"
UDP_PORT = 5005
HEADER_SIZE = 14
SOCKET_BUFFER = 4096

frame_queue = Queue(maxsize=3)
raw_packet_queue = Queue(maxsize=500)
frames = defaultdict(dict)

def decode_header(data):
    if len(data) < HEADER_SIZE:
        return None
    return struct.unpack('<IHHIH', data[:HEADER_SIZE])

def udp_receiver():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8*1024*1024)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    while True:
        try:
            data, _ = sock.recvfrom(SOCKET_BUFFER)
            try:
                raw_packet_queue.put_nowait(data)
            except:
                continue
        except:
            continue

def packet_processor():
    while True:
        try:
            data = raw_packet_queue.get(timeout=1)
            header = decode_header(data)
            if header is None:
                continue
            
            frame_id, packet_num, total_packets, frame_size, data_size = header
            payload = data[HEADER_SIZE:HEADER_SIZE+data_size]
            
            frames[frame_id][packet_num] = payload
        
            if len(frames[frame_id]) == total_packets:
                frame_parts = [frames[frame_id][i] for i in range(total_packets)]
                frame_data = b''.join(frame_parts)
                
                if len(frame_data) >= 2 and frame_data[0] == 0xFF and frame_data[1] == 0xD8:
                    try:
                        frame_queue.put_nowait(frame_data)
                    except:
                        try:
                            frame_queue.get_nowait()
                            frame_queue.put_nowait(frame_data)
                        except:
                            pass
                
                del frames[frame_id]
                
                if frame_id % 10 == 0 and len(frames) > 3:
                    old_frames = [fid for fid in frames.keys() if fid < frame_id - 3]
                    for fid in old_frames:
                        del frames[fid]
        except Empty:
            continue
        except:
            continue

def get_esp32_image():
    try:
        frame_data = frame_queue.get_nowait()
        frame = cv2.imdecode(np.frombuffer(frame_data, np.uint8), cv2.IMREAD_COLOR)
        return frame
    except Empty:
        return None
    except:
        return None

def main():
    receiver_thread = threading.Thread(target=udp_receiver, daemon=True)
    receiver_thread.start()
    
    processor_thread = threading.Thread(target=packet_processor, daemon=True)
    processor_thread.start()
    
    print("Starting UDP receiver on {}:{}".format(UDP_IP, UDP_PORT))
    time.sleep(1)
    
    print("Loading MiDaS model and calibration...")
    midas, transform, device = load_midas_model('MiDaS_small')
    scale, shift = load_midas_calibration('src/perception/calibration/midas_calibration.npz')
    print(f"Calibration: scale={scale:.6f}, shift={shift:.6f}\n")
    
    print("Press 'q' to quit\n")
    
    while True:
        img = get_esp32_image()
        if img is None:
            time.sleep(0.01)
            continue
        
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        midas_depth = estimate_midas_depth(img_rgb, midas, transform, device)
        real_depth = apply_calibration(midas_depth, scale, shift)
        
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(real_depth * 50, alpha=1), 
            cv2.COLORMAP_JET
        )
        
        h, w = img.shape[:2]
        center_depth = real_depth[h//2, w//2]
        
        cv2.putText(img, f"Center: {center_depth:.2f}m", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        display = np.hstack([img, depth_colormap])
        cv2.imshow('ESP32 Real-World Depth', display)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
