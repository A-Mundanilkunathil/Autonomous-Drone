import cv2
import numpy as np
from queue import Empty
from src.perception.streams.udp_capture import frame_queue
import sys

# ---- CONFIG ----
NPZ_PATH = "system/drone_vision/camera_calibration/camera_calib.npz"
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
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters() 
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

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
        # Define object points for the marker (square)
        obj_points = np.array([
            [-MARKER_LENGTH_M / 2, -MARKER_LENGTH_M / 2, 0],
            [ MARKER_LENGTH_M / 2, -MARKER_LENGTH_M / 2, 0],
            [ MARKER_LENGTH_M / 2,  MARKER_LENGTH_M / 2, 0],
            [-MARKER_LENGTH_M / 2,  MARKER_LENGTH_M / 2, 0]
        ], dtype=np.float32)

        for i in range(len(ids)):
            # Solve PnP
            retval, rvec, tvec = cv2.solvePnP(obj_points, corners[i], camera_matrix, dist_coeffs)
            if retval:
                # Draw marker and axis
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, MARKER_LENGTH_M * 0.5)
                # tvec = [x, y, z] # 3D position in meters
                # rvec = [rx, ry, rz] # 3D rotation in radians
                print(f"Marker ID: {int(ids[i][0])}, rvec: {rvec.ravel()}, tvec: {tvec.ravel()}")

    cv2.imshow("ESP32-CAM ArUco Pose", frame)
    if cv2.waitKey(1) & 0xFF == 27:  # ESC key
        break

cv2.destroyAllWindows()
print("Exited cleanly.")
