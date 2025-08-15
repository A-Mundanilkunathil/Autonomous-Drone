import cv2
import numpy as np
import requests

# MJPEG stream from ESP32-CAM
url = 'http://192.168.1.156/stream' # Adjust IP address as needed
cap = cv2.VideoCapture(url)

# Load predefined dictionary
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50) 
parameters = cv2.aruco.DetectorParameters_create()

# Camera calibration 
camera_matrix = np.array([
    [320, 0, 160],
    [0, 320, 120],
    [0, 0, 1]
]) # Adjust these values based on your camera calibration

dist_coeffs = np.zeros((5,1)) # Assuming no lens distortion

def send_command(cmd):
    requests.get(f"http://192.168.1.156/cmd?val={cmd}")
    
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break
    
    # Detect ArUco markers
    corners, ids, _ = cv2.aruco.DetectMarkers(frame, aruco_dict, parameters=parameters)
    
    # If markers are detected
    if ids is not None and 23 in ids:
        idx = list(ids.flatten()).index(23) # Get index of marker ID 23
        
        # Estimate pose of the marker
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[idx], 0.1, camera_matrix, dist_coeffs)
        
        # Get x, y, z coordinates
        x,y,z = tvec[0][0]
        
        # Draw marker and axis
        cv2.aruco.drawDetectedMarkers(frame, corners)  # Draw detected markers
        
        # Draw axis
        cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
        
        # Center threshold
        if abs(x) < 0.05 and abs(y) < 0.05:
            print("Landing Zone Detected")
            send_command("x") 
        elif x < -0.05: 
            print("Move Left")
            send_command("a")
        elif x > 0.05: 
            print("Move Right")
            send_command("d")
            
        cv2.imshow("Landing View", frame)
        if cv2.waitKey(1) == 27: # Press 'ESC' to exit
            break

cap.release()
cv2.destroyAllWindows()

            
        