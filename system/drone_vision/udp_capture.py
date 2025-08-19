import socket
import cv2
import numpy as np
import struct
import threading
from collections import defaultdict
from queue import Queue

UDP_IP = "192.168.1.89"
UDP_PORT = 5005

# Global frame queue
frame_queue = Queue(maxsize=5)
frames = defaultdict(dict)

def decode_header(data):
    if len(data) < 14:
        return None
    return struct.unpack('<IHHIH', data[:14])

def udp_receiver():
    """Background thread to receive UDP packets"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4*1024*1024)  # 4MB buffer
    
    while True:
        try:
            data, addr = sock.recvfrom(4096)
            
            header = decode_header(data)
            if header is None:
                continue
                
            frame_id, packet_num, total_packets, frame_size, data_size = header
            payload = data[14:14+data_size]
            
            # Validate packet
            if len(payload) != data_size:
                print(f"Packet size mismatch: expected {data_size}, got {len(payload)}")
                continue
            
            frames[frame_id][packet_num] = payload
            
            if len(frames[frame_id]) == total_packets:
                # Check if all packets are present
                missing_packets = []
                for i in range(total_packets):
                    if i not in frames[frame_id]:
                        missing_packets.append(i)
                
                if missing_packets:
                    print(f"Frame {frame_id}: Missing packets {missing_packets}")
                    del frames[frame_id]  # Discard incomplete frame
                    continue
                
                # Reconstruct frame in correct order
                frame_data = bytearray()
                for i in range(total_packets):
                    frame_data.extend(frames[frame_id][i])
                
                # Validate reconstructed frame size
                if len(frame_data) != frame_size:
                    print(f"Frame {frame_id}: Size mismatch {len(frame_data)} != {frame_size}")
                    del frames[frame_id]
                    continue
                
                # Add to display queue (non-blocking)
                if not frame_queue.full():
                    frame_queue.put(bytes(frame_data))
                else:
                    # Drop oldest frame if queue full
                    try:
                        frame_queue.get_nowait()
                        frame_queue.put(bytes(frame_data))
                    except:
                        pass
                
                del frames[frame_id]
                
                # Cleanup old frames more aggressively
                if len(frames) > 5:
                    old_frames = list(frames.keys())[:-3]
                    for fid in old_frames:
                        del frames[fid]
                        
        except Exception as e:
            print(f"UDP receiver error: {e}")
            continue

# Start receiver thread
receiver_thread = threading.Thread(target=udp_receiver, daemon=True)
receiver_thread.start()

print("Listening for ESP32-CAM stream...")

# Main display loop
while True:
    try:
        # Get frame from queue (non-blocking)
        frame_data = frame_queue.get_nowait()
        
        # Validate JPEG header
        if len(frame_data) < 2 or frame_data[0] != 0xFF or frame_data[1] != 0xD8:
            print("Invalid JPEG header")
            continue
            
        frame = cv2.imdecode(np.frombuffer(frame_data, np.uint8), cv2.IMREAD_COLOR)
        if frame is not None:
            cv2.imshow("ESP32-CAM UDP", frame)
        else:
            print("Failed to decode JPEG")
    except:
        pass
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()