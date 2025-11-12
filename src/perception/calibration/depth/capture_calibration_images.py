#!/usr/bin/env python3

import cv2
import numpy as np
import pyrealsense2 as rs
import socket
import struct
import threading
from collections import defaultdict
from queue import Queue, Empty
import time
import os

UDP_IP = "192.168.1.89"
UDP_PORT = 5005
HEADER_SIZE = 14
SOCKET_BUFFER = 4096
SAVE_DIR = "calibration_images"

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

def setup_realsense():
    pipeline = rs.pipeline()
    config = rs.config()
    
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    pipeline.start(config)
    
    return pipeline

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
    if not os.path.exists(SAVE_DIR):
        os.makedirs(SAVE_DIR)
    
    receiver_thread = threading.Thread(target=udp_receiver, daemon=True)
    receiver_thread.start()
    
    processor_thread = threading.Thread(target=packet_processor, daemon=True)
    processor_thread.start()
    
    print("Starting UDP receiver on {}:{}".format(UDP_IP, UDP_PORT))
    time.sleep(1)
    
    pipeline = setup_realsense()
    
    print("Press 's' to save image pair, 'q' to quit")
    print("Capture diverse scenes for better MiDaS calibration:")
    print("  - Indoor/outdoor environments")
    print("  - Different distances (0.5m to 10m)")
    print("  - Various lighting conditions")
    
    img_count = 0
    
    try:
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            
            if not depth_frame:
                continue
            
            realsense_depth = np.asanyarray(depth_frame.get_data())
            
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(realsense_depth, alpha=0.03), 
                cv2.COLORMAP_JET
            )
            
            esp32_img = get_esp32_image()
            
            if esp32_img is not None:
                display = np.hstack([
                    cv2.resize(esp32_img, (640, 480)),
                    depth_colormap
                ])
                
                cv2.putText(display, f"Images: {img_count}", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(display, "s: save | q: quit", (10, 460), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                cv2.imshow('MiDaS Calibration Capture', display)
                
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('s'):
                if esp32_img is not None:
                    esp32_path = os.path.join(SAVE_DIR, f'esp32_{img_count:02d}.png')
                    rs_depth_path = os.path.join(SAVE_DIR, f'realsense_depth_{img_count:02d}.npy')
                    
                    cv2.imwrite(esp32_path, esp32_img)
                    
                    depth_meters = realsense_depth.astype(np.float32) * 0.001
                    np.save(rs_depth_path, depth_meters)
                    
                    print(f"Saved pair {img_count}")
                    img_count += 1
                else:
                    print("ESP32 image not available")
            
            elif key == ord('q'):
                break
    
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print(f"\nCaptured {img_count} image pairs")
        print(f"Files saved in: {SAVE_DIR}")
        print("\nNext steps:")
        print("1. Copy images to calibration directory")
        print("2. Run: python calibrate_depth.py")
        print("3. Use midas_calibration.npz for real-world depth estimation")

if __name__ == "__main__":
    main()
