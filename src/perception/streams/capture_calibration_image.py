# https://calib.io/pages/camera-calibration-pattern-generator

import cv2
import os
import time
from src.perception.streams.udp_capture import frame_queue
from queue import Empty
import numpy as np

SAVE_DIR = "system/drone_vision/camera_calibration/calibration_images"
os.makedirs(SAVE_DIR, exist_ok=True)

count = 0
print("Press 's' to save frame, 'ESC' to exit.")

while True:
    try:
        frame_data = frame_queue.get(timeout=1)
        frame = cv2.imdecode(np.frombuffer(frame_data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            break
    except Empty:
        continue
    
    cv2.imshow('Camera Calibration', frame)
    key = cv2.waitKey(1) & 0xFF
    
    if key == ord('s'):  # Press 's' to save the image
        filename = f"{SAVE_DIR}calib_{count:02d}.jpg"
        cv2.imwrite(filename, frame)
        print(f"Saved calib_img{count}.jpg")
        count += 1
    elif key == 27:  # Press 'ESC' to exit
        break
    
cv2.destroyAllWindows()
print(f"Captured {count} calibration images.")