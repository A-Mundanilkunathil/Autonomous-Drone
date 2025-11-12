#!/usr/bin/env python3

import os
import time
import cv2
import socket
import struct
import threading
import csv
import numpy as np
from queue import Queue, Empty
from collections import defaultdict

UDP_IP = "192.168.1.89"
UDP_PORT = 5005
HEADER_SIZE = 14
SOCKET_BUFFER = 4096
SAVE_DIR = "esp32_calib_wall"
os.makedirs(SAVE_DIR, exist_ok=True)
MANIFEST = os.path.join(SAVE_DIR, "manifest.csv")

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
    print(f"Starting ESP32 UDP receiver on {UDP_IP}:{UDP_PORT}")
    threading.Thread(target=udp_receiver, daemon=True).start()
    threading.Thread(target=packet_processor, daemon=True).start()
    time.sleep(1)

    if not os.path.exists(MANIFEST):
        with open(MANIFEST, "w", newline="") as f:
            csv.writer(f).writerow(["filename", "distance_m"])

    hotkeys = {
        ord('1'): 0.5, ord('2'): 1.0, ord('3'): 1.5, ord('4'): 2.0,
        ord('5'): 2.5, ord('6'): 3.0, ord('7'): 4.0, ord('8'): 5.0
    }
    current_dist = 1.0
    idx = 0

    print("\nControls:")
    print("  1-8: Set distance (0.5m to 5.0m)")
    print("  [ ]: Decrease distance by 0.1m")
    print("  ]: Increase distance by 0.1m")
    print("  s: Save image with current distance")
    print("  q: Quit\n")
    
    while True:
        img = get_esp32_image()
        if img is None:
            cv2.waitKey(1)
            continue

        vis = img.copy()
        cv2.putText(vis, f"Distance: {current_dist:.2f}m", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(vis, "1-8: preset | [/]: adjust | s: save | q: quit", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("ESP32 Distance Calibration", vis)
        
        k = cv2.waitKey(1) & 0xFF
        if k in hotkeys:
            current_dist = float(hotkeys[k])
        elif k == ord('['):
            current_dist = max(0.2, current_dist - 0.1)
        elif k == ord(']'):
            current_dist = current_dist + 0.1
        elif k == ord('s'):
            fn = f"esp32_wall_{idx:03d}.png"
            path = os.path.join(SAVE_DIR, fn)
            cv2.imwrite(path, img)
            with open(MANIFEST, "a", newline="") as f:
                csv.writer(f).writerow([fn, f"{current_dist:.3f}"])
            print(f"Saved {fn} at {current_dist:.3f}m")
            idx += 1
        elif k == ord('q'):
            break

    cv2.destroyAllWindows()
    print(f"\nSaved {idx} images in: {SAVE_DIR}")
    print("Next step: Run calibrate_midas_scale.py")

if __name__ == "__main__":
    main()
