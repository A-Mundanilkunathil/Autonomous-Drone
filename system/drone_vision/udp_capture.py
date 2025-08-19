import socket
import cv2
import numpy as np
import struct
import threading
from collections import defaultdict
from queue import Queue, Empty

UDP_IP = "192.168.1.89"
UDP_PORT = 5005
HEADER_SIZE = 14  # Size of the header in bytes

# Frame queue for displaying frames
frame_queue = Queue(maxsize=3)
# Store incoming packet per frame
frames = defaultdict(dict)

# Decode UDP packet header
def decode_header(data):
    if len(data) < HEADER_SIZE:
        return None
    return struct.unpack('<IHHIH', data[:HEADER_SIZE])

# UDP receiver thread
def udp_receiver():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8*1024*1024) # 8 MB receive buffer
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Allow reuse of address

    while True:
        try:
            data, _ = sock.recvfrom(4096)
            
            if len(data) < HEADER_SIZE:
                continue

            frame_id, packet_num, total_packets, frame_size, data_size = decode_header(data)
            payload = data[HEADER_SIZE:HEADER_SIZE+data_size]
            
            frames[frame_id][packet_num] = payload
        
            if len(frames[frame_id]) == total_packets:
                # Reconstruct frame in correct order
                frame_parts = [frames[frame_id][i] for i in range(total_packets)]
                frame_data = b''.join(frame_parts)

                # Validate reconstructed frame size
                if len(frame_data) >= 2 and frame_data[0] == 0xFF and frame_data[1] == 0xD8:
                    try:
                        frame_queue.put_nowait(frame_data)
                    except:
                        try:
                            frame_queue.get_nowait()  # Remove oldest frame if full
                            frame_queue.put_nowait(frame_data) # Replace with new frame
                        except:
                            pass

                del frames[frame_id]
                
                # Cleanup old frames
                if frame_id % 10 == 0 and len(frames) > 3:
                    old_frames = [fid for fid in frames.keys() if fid < frame_id - 3]
                    for fid in old_frames:
                        del frames[fid]
                        
        except:
            continue

# Start receiver thread
receiver_thread = threading.Thread(target=udp_receiver, daemon=True)
receiver_thread.start()
print("Listening for ESP32-CAM stream...")

if __name__ == "__main__":
    # Main display loop
    while True:
        try:
            # Get frame from queue 
            frame_data = frame_queue.get_nowait()
            frame = cv2.imdecode(np.frombuffer(frame_data, np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                cv2.imshow("ESP32-CAM UDP", frame)
        except Empty:
            pass
        except:
            pass
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()