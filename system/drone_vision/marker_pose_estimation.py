import cv2
import numpy as np
import cv2.aruco as aruco
from queue import Empty
from udp_capture import frame_queue
import sys

# ---- CONFIG ----
NPZ_PATH = "system/drone_vision/calibration_images/camera_calib.npz"
MARKER_LENGTH_M = 0.15
GET_FRAME_TIMEOUT = 1.0

# ---- Load calibration ----
try:
    data = np.load(NPZ_PATH)
    camera_matrix = data["camera_matrix"]
    dist_coeffs = data["dist_coeffs"]
    print(f"Loaded calibration from {NPZ_PATH}")
except Exception as e:
    print("Failed to load calibration file:", e)
    sys.exit(1)

# ---- Build ArUco dictionary & detector ----
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

print("OpenCV version:", cv2.__version__)
print("Using modern ArUco API with ArucoDetector")

# ---- Main loop ----
print("Listening for ESP32-CAM stream... (press ESC in window to exit)")
while True:
    try:
        frame_data = frame_queue.get(timeout=GET_FRAME_TIMEOUT)
    except Empty:
        continue

    frame = cv2.imdecode(np.frombuffer(frame_data, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect markers
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is not None and len(corners) > 0:
        # Estimate pose for each marker
        for i in range(len(ids)):
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                [corners[i]], MARKER_LENGTH_M, camera_matrix, dist_coeffs
            )
            rvec = rvec[0][0]
            tvec = tvec[0][0]

            # Draw marker and axis
            aruco.drawDetectedMarkers(frame, [corners[i]], ids[i:i+1])
            aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, MARKER_LENGTH_M * 0.5)

            print(f"Marker ID: {int(ids[i][0])}, rvec: {rvec}, tvec: {tvec}")

    cv2.imshow("ESP32-CAM ArUco Pose", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cv2.destroyAllWindows()
print("Exited cleanly.")
