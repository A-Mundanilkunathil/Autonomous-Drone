import cv2
import numpy as np
import cv2.aruco as aruco

# Load camera calibration parameters
camera_matrix = np.load("camera_matrix.npy")
dist_coeffs = np.load("dist_coeffs.npy")

# ArUco dictionary & marker size
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters_create()
marker_length = 0.15 # Marker length in meters

cap = cv2.VideoCapture("http://192.168.1.156/stream")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture image")
        break
    
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect markers
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    if ids is not None:
        # Estimate pose of each marker
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
        
        # Draw markers and axes
        for i in range(len(ids)):
            aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1)
            print(f"Marker ID: {ids[i][0]}")
            
    cv2.imshow("ESP32-CAM Pose", frame)
    if cv2.waitKey(1) == 27:
        break
    
cap.release()
cv2.destroyAllWindows()