import cv2
import cv2.aruco as aruco
import os

# Get the dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# Marker ID and size
marker_id = 40
side_pixels = 200

# Generate marker image
marker_img = aruco.generateImageMarker(aruco_dict, marker_id, side_pixels)

os.makedirs("system/drone_vision/aruco_markers", exist_ok=True)

# Save and show
cv2.imwrite(f"system/drone_vision/aruco_markers/marker_{marker_id}.png", marker_img)
cv2.imshow("Marker", marker_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
