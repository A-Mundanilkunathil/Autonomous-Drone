# https://calib.io/pages/camera-calibration-pattern-generator

import cv2
import os
import time

STREAM_URL = "http://192.168.4.1/stream"  # Replace with your camera stream URL

SAVE_DIR = "calibration_images"
os.makedirs(SAVE_DIR, exist_ok=True)

cap = cv2.VideoCapture(STREAM_URL)  # Open the default camera
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    cv2.imshow('Camera Calibration', frame)
    key = cv2.waitKey(1) # 1 ms delay
    
    if key == ord('s'):  # Press 's' to save the image
        filename = f"{SAVE_DIR}calib_{count:02d}.jpg"
        cv2.imwrite(filename, frame)
        print(f"Saved calib_img{count}.jpg")
        count += 1
    elif key == 27:  # Press 'ESC' to exit
        break
    
cap.release()
cv2.destroyAllWindows()