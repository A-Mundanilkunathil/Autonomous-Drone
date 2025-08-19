import cv2
import numpy as np
import glob

# Chessboard size (number of inner corners per row and column)
chessboard_size = (8, 5) # Inner corner for 9x6

# Size of each square in your printed chessboard (meters or mm)
square_size = 0.015  # 15 mm

# Prepare object points (0,0,0), (1,0,0), ..., (8,5,0)
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store object points and image points
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# Load calibration images
images = glob.glob("system/drone_vision/calibration_images/*.jpg")

if not images:
    raise FileNotFoundError("No calibration images found in calibration_images/*.jpg")

img_shape = None

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if img_shape is None:
        img_shape = gray.shape[::-1]  # (width, height)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        objpoints.append(objp)

        # Refine corner locations
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and show corners
        cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
        cv2.imshow('Corners', img)
        cv2.waitKey(200)

cv2.destroyAllWindows()

if not objpoints or not imgpoints:
    raise RuntimeError("No chessboard corners were detected. Check your calibration images.")

# Calibrate camera
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, img_shape, None, None
)

print("Camera matrix:\n", camera_matrix)
print("Distortion coefficients:\n", dist_coeffs)

# Save parameters
np.savez("system/drone_vision/calibration_images/camera_calib.npz", camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)
print("Calibration parameters saved to camera_calib.npz")
