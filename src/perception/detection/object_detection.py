import cv2
import numpy as np
import time
from queue import Empty
from sklearn.cluster import DBSCAN
from udp_capture import frame_queue

# Motion detection setup
bg_subtractor = cv2.createBackgroundSubtractorMOG2()

# Load Haar Cascade for face detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Variable to store previous frame for optical flow
prev_frame = None

def detect_faces(frame):
    """
    Detect faces in a given frame using OpenCV's Haar Cascade classifier.

    :param frame: A BGR image frame from the camera
    :return: A list of dictionaries containing the detected face information
    """
    # Convert to grayscale for faster processing
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Resize for even faster processing
    small_gray = cv2.resize(gray, (0, 0), fx=0.5, fy=0.5)
    
    # Detect faces - adjust parameters for speed vs accuracy tradeoff
    faces = face_cascade.detectMultiScale(
        small_gray, 
        scaleFactor=1.1, 
        minNeighbors=3,
        minSize=(20, 20),
        flags=cv2.CASCADE_SCALE_IMAGE
    )
    
    detected_objects = []
    for (x, y, w, h) in faces:
        # Scale coordinates back to original frame size
        x, y, w, h = x*2, y*2, w*2, h*2
        
        detected_objects.append({ 
            'label': 'face',
            'confidence': 1.0,
            'bbox': (x, y, w, h),
            'center': (x + w//2, y + h//2)
        })
        
        # Draw face detection
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, 'Face', (x, y - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    return detected_objects

def detect_with_farneback_optical_flow(prev_frame, current_frame, scale=0.5):
    """
    Detects movement in a given frame using Farneback optical flow.

    The function downscales the frame for faster processing, then calculates
    the optical flow between the previous and current frames. The flow is then
    visualized in a small region of the frame. The magnitude of the flow is
    analyzed to detect significant movement. If movement is detected, a
    bounding box is drawn around the region with the most movement, and the
    direction of the movement is annotated.

    :param prev_frame: The previous frame from the camera
    :param current_frame: The current frame from the camera
    :param scale: The scale at which to downsample the frames (default: 0.5)
    :return: A list of dictionaries containing the detected object information
    """
    # Downscale frames for faster processing
    small_prev = cv2.resize(prev_frame, (0, 0), fx=scale, fy=scale)
    small_curr = cv2.resize(current_frame, (0, 0), fx=scale, fy=scale)

    # Convert frames to grayscale
    prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
    curr_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    
    # Calculate optical flow
    flow = cv2.calcOpticalFlowFarneback(prev_gray, curr_gray, None, 
                                        pyr_scale=0.5, levels=3, winsize=15, 
                                        iterations=3, poly_n=5, poly_sigma=1.2,
                                        flags=0)

    # Scale flow back to original size
    flow = cv2.resize(flow, (current_frame.shape[1], current_frame.shape[0]))
    flow *= 1.0 / scale
    
    # Visualize the flow
    magnitude, angle = cv2.cartToPolar(flow[..., 0], flow[..., 1])
    hsv = np.zeros_like(current_frame)
    hsv[..., 1] = 255
    hsv[..., 0] = angle * 180 / np.pi / 2
    hsv[..., 2] = cv2.normalize(magnitude, None, 0, 255, cv2.NORM_MINMAX)
    flow_rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    
    # Show flow visualization
    small_flow = cv2.resize(flow_rgb, (160, 120))
    current_frame[10:130, 170:330] = small_flow
    
    # Analyze flow magnitude for movement detection
    avg_magnitude = np.mean(magnitude)
    
    detected_objects = []
    
    # Only detect significant movement
    if avg_magnitude > 2.0:  
        # Calculate dominant direction
        h, w = magnitude.shape
        regions = {
            "left": np.mean(magnitude[:, :w//3]),
            "center": np.mean(magnitude[:, w//3:2*w//3]),
            "right": np.mean(magnitude[:, 2*w//3:])
        }
        movement_direction = max(regions, key=regions.get)
        
        # Create a "detected object" for the area with most movement
        region_x = 0
        if movement_direction == "center":
            region_x = w//3
        elif movement_direction == "right":
            region_x = 2*w//3
            
        # Create bounding box for movement region
        x, y = region_x, 0
        w = w//3
        h = h
        
        detected_objects.append({
            'label': f'flow_{movement_direction}',
            'confidence': min(avg_magnitude / 10.0, 1.0),
            'bbox': (x, y, w, h),
            'center': (x + w//2, y + h//2)
        })
        
        # Draw movement detection
        cv2.rectangle(current_frame, (x, y), (x + w, y + h), (0, 165, 255), 2)
        cv2.putText(current_frame, f'Flow: {movement_direction}', (x, y - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
    
    # Add text showing average magnitude
    cv2.putText(current_frame, f"Movement: {avg_magnitude:.2f}", (170, 150),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    return detected_objects

def detect_with_lucas_kanade_optical_flow(prev_frame, current_frame, feature_params, lk_params, K):
    """Detect moving regions using Lucas-Kanade optical flow with motion compensation."""

    def _empty_result(vis_frame=None):
        return {
            "R": np.eye(3),
            "t": np.zeros((3, 1)),
            "tracked_points": np.empty((0, 1, 2), dtype=np.float32),
            "moving_points": np.empty((0, 2), dtype=np.float32),
            "clusters": [],
            "frame_vis": current_frame if vis_frame is None else vis_frame
        }

    prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
    curr_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

    prev_gray = cv2.GaussianBlur(prev_gray, (5, 5), 0)
    curr_gray = cv2.GaussianBlur(curr_gray, (5, 5), 0)

    prev_pts = cv2.goodFeaturesToTrack(prev_gray, mask=None, **feature_params)
    if prev_pts is None or len(prev_pts) == 0:
        return _empty_result()

    curr_pts, status, err = cv2.calcOpticalFlowPyrLK(prev_gray, curr_gray, prev_pts, None, **lk_params)
    if curr_pts is None or status is None:
        return _empty_result()

    status = status.reshape(-1)
    err = err.reshape(-1)
    valid_mask = (status == 1) & (err < 20.0)
    if not np.any(valid_mask):
        return _empty_result()

    good_prev = prev_pts[valid_mask]
    good_curr = curr_pts[valid_mask]

    if len(good_prev) < 4:
        return _empty_result()

    fb_pts, fb_status, fb_err = cv2.calcOpticalFlowPyrLK(curr_gray, prev_gray, good_curr, None, **lk_params)
    if fb_pts is not None and fb_status is not None:
        fb_status = fb_status.reshape(-1)
        fb_err = fb_err.reshape(-1)
        fb_dist = np.linalg.norm(good_prev.reshape(-1, 2) - fb_pts.reshape(-1, 2), axis=1)
        fb_mask = (fb_status == 1) & (fb_err < 20.0) & (fb_dist < 1.5)
        if np.count_nonzero(fb_mask) >= 4:
            good_prev = good_prev[fb_mask]
            good_curr = good_curr[fb_mask]

    if len(good_prev) < 4:
        return _empty_result()

    good_prev_xy = good_prev.reshape(-1, 2)
    good_curr_xy = good_curr.reshape(-1, 2)
    frame_h, frame_w = current_frame.shape[:2]
    frame_diag = float(np.hypot(frame_w, frame_h))

    affine, inliers = cv2.estimateAffinePartial2D(
        good_prev_xy,
        good_curr_xy,
        method=cv2.RANSAC,
        ransacReprojThreshold=2.5,
        maxIters=2000,
        confidence=0.99,
    )

    if affine is not None:
        predicted_curr = cv2.transform(good_prev_xy.reshape(-1, 1, 2), affine).reshape(-1, 2)
        residuals = good_curr_xy - predicted_curr
    else:
        residuals = good_curr_xy - good_prev_xy

    residual_mags = np.linalg.norm(residuals, axis=1)
    median_mag = np.median(residual_mags)
    mad = np.median(np.abs(residual_mags - median_mag)) * 1.4826 + 1e-6
    z_scores = np.abs(residual_mags - median_mag) / mad

    if inliers is not None:
        inliers = inliers.reshape(-1).astype(bool)
    else:
        inliers = np.ones_like(residual_mags, dtype=bool)

    # Balanced thresholds: detect real motion while rejecting noise
    # Use OR logic: either strong motion OR statistically significant outlier
    strong_motion = residual_mags > 1.8  # Absolute displacement threshold
    statistical_outlier = (~inliers) & (z_scores > 3.0)  # RANSAC outlier with significance
    moving_mask = strong_motion | statistical_outlier
    moving_indices = np.where(moving_mask)[0]

    moving_prev = good_prev_xy[moving_indices] if moving_indices.size else np.empty((0, 2))
    moving_curr = good_curr_xy[moving_indices] if moving_indices.size else np.empty((0, 2))
    moving_residuals = residuals[moving_indices] if moving_indices.size else np.empty((0, 2))

    clusters = []
    if moving_curr.shape[0] >= 2:  # Back to 2 minimum points
        try:
            eps = max(12.0, 0.025 * frame_diag)  # Looser clustering for better grouping
            clustering = DBSCAN(eps=eps, min_samples=2).fit(moving_curr)  # min_samples=2
            labels = clustering.labels_
            unique_labels = [lbl for lbl in np.unique(labels) if lbl != -1]
            for lbl in unique_labels:
                pts = moving_curr[labels == lbl]
                mag_subset = np.linalg.norm(moving_residuals[labels == lbl], axis=1)
                if pts.shape[0] == 0:
                    continue
                x, y, w, h = cv2.boundingRect(pts.astype(np.int32))
                pad = 10
                x = max(int(x - pad), 0)
                y = max(int(y - pad), 0)
                w = int(min(w + 2 * pad, current_frame.shape[1] - x))
                h = int(min(h + 2 * pad, current_frame.shape[0] - y))
                diag = float(np.hypot(w, h))
                area = float(w * h)
                density = pts.shape[0] / max(area, 1.0)
                # Relaxed filters: allow smaller, sparser clusters
                if (diag > 0.5 * frame_diag and pts.shape[0] < 4):  # Only reject huge sparse boxes
                    continue
                # Lower motion requirement
                avg_motion = mag_subset.mean()
                if avg_motion < 1.5:  # Lower threshold for motion
                    continue
                confidence = float(np.clip(avg_motion / 4.0, 0.2, 1.0))
                clusters.append({
                    "bbox": (x, y, w, h),
                    "score": confidence,
                    "count": int(pts.shape[0]),
                    "density": density
                })
        except Exception:
            pass

    if not clusters and moving_curr.shape[0] > 0:
        # Relaxed fallback: create detection for any sustained moving points
        avg_residual_mag = np.mean(np.linalg.norm(moving_residuals, axis=1))
        if avg_residual_mag < 1.5:  # Lower threshold
            return {
                "R": R,
                "t": t,
                "tracked_points": good_curr.reshape(-1, 1, 2),
                "moving_points": moving_curr,
                "clusters": [],
                "frame_vis": vis
            }
        
        center = moving_curr.mean(axis=0)
        spread = moving_curr.std(axis=0) * 2.5
        spread = np.maximum(spread, np.array([15.0, 15.0]))  # Slightly larger minimum
        x = int(np.clip(center[0] - spread[0], 0, frame_w - 1))
        y = int(np.clip(center[1] - spread[1], 0, frame_h - 1))
        w = int(np.clip(spread[0] * 2, 30, frame_w - x))  # Minimum 30px wide
        h = int(np.clip(spread[1] * 2, 30, frame_h - y))  # Minimum 30px tall
        diag = float(np.hypot(w, h))
        if diag <= 0.65 * frame_diag:  # Allow slightly larger boxes
            avg_mag = float(np.clip(avg_residual_mag / 4.0, 0.2, 1.0))
            clusters.append({
                "bbox": (x, y, w, h),
                "score": avg_mag,
                "count": int(moving_curr.shape[0]),
                "density": moving_curr.shape[0] / max(w * h, 1.0)
            })

    R = np.eye(3)
    t = np.zeros((3, 1))
    if good_curr_xy.shape[0] >= 8:
        E, mask_pose = cv2.findEssentialMat(good_curr_xy, good_prev_xy, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
        if E is not None:
            _, R, t, _ = cv2.recoverPose(E, good_curr_xy, good_prev_xy, K)

    vis = current_frame.copy()
    for (p_prev, p_curr, is_moving) in zip(good_prev_xy, good_curr_xy, moving_mask):
        x1, y1 = map(int, p_prev)
        x2, y2 = map(int, p_curr)
        color = (0, 0, 255) if is_moving else (0, 200, 0)
        cv2.line(vis, (x1, y1), (x2, y2), (80, 80, 80), 1, cv2.LINE_AA)
        cv2.circle(vis, (x2, y2), 3, color, -1)

    for cluster in clusters:
        x, y, w, h = cluster["bbox"]
        label = f"Move {cluster['count']}" if cluster.get("count", 0) else "Motion"
        cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 165, 255), 2)
        cv2.putText(vis, label, (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)

    return {
        "R": R,
        "t": t,
        "tracked_points": good_curr.reshape(-1, 1, 2),
        "moving_points": moving_curr,
        "clusters": clusters,
        "frame_vis": vis
    }

def detect_motion(frame):
    """
    Detects motion in a given frame using background subtraction and contour detection.
    
    The function applies a background subtractor (MOG2) to the frame to detect foreground pixels. It then applies thresholding to remove shadows, and erosion/dilation to remove noise. Contours are found in the resulting mask, and filtered to remove small detections. The remaining contours are returned as detected objects, with a confidence score based on the area of the contour.
    
    :param frame: The current frame from the camera
    :return: A list of detected objects with their bounding boxes and confidence scores
    """
    # First, resize frame for faster processing 
    frame_resized = cv2.resize(frame, (320, 240))
    
    # Apply Gaussian blur to reduce noise
    frame_blur = cv2.GaussianBlur(frame_resized, (5, 5), 0)
    
    # Apply background subtraction
    fg_mask = bg_subtractor.apply(frame_blur, learningRate=0.01)
    
    # Remove shadows (values of 127) from the mask
    _, fg_mask = cv2.threshold(fg_mask, 200, 255, cv2.THRESH_BINARY) 
    
    # Remove noise
    kernel = np.ones((5,5), np.uint8)
    fg_mask = cv2.erode(fg_mask, kernel, iterations=1)
    fg_mask = cv2.dilate(fg_mask, kernel, iterations=2)
    
    # Find contours
    contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Adjust scale factor to match original frame size
    scale_x = frame.shape[1] / frame_resized.shape[1]
    scale_y = frame.shape[0] / frame_resized.shape[0]
    
    detected_objects = []
    for contour in contours:
        area = cv2.contourArea(contour)
        
        if area > 500:
            x, y, w, h = cv2.boundingRect(contour)
            
            # Scale back to original frame size
            x, y = int(x * scale_x), int(y * scale_y)
            w, h = int(w * scale_x), int(h * scale_y)
            
            # Filter out very small detections
            if w > 20 and h > 20:
                detected_objects.append({
                    'label': 'motion',
                    'confidence': min(area / 10000, 1.0),
                    'bbox': (x, y, w, h),
                    'center': (x + w//2, y + h//2)
                })
                
                # Draw motion detection with confidence
                conf = min(area / 10000, 1.0)
                color = (0, int(255 * conf), 255)
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                cv2.putText(frame, f'Motion {conf:.2f}', (x, y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    
    # Display the processed mask in a corner for debugging
    small_mask = cv2.resize(fg_mask, (160, 120))
    frame[10:130, 10:170] = cv2.cvtColor(small_mask, cv2.COLOR_GRAY2BGR)
    
    return detected_objects

def detect_colors(frame):
    """
    Detect colored objects in the frame.

    This function detects objects in the frame with a color that matches one of the predefined color ranges.
    The color ranges are defined as (lower, upper) HSV values. The function returns a list of detected objects,
    each represented as a dictionary with the keys 'label', 'confidence', 'bbox', and 'center'.

    Parameters
    ----------
    frame : numpy.ndarray
        The input frame to detect objects in.

    Returns
    -------
    detected_objects : list of dict
        A list of detected objects, each represented as a dictionary with the keys 'label', 'confidence', 'bbox', and 'center'.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define color ranges
    color_ranges = {
        'red': ([0, 100, 100], [10, 255, 255]),
        'blue': ([100, 100, 100], [130, 255, 255]),
        'green': ([40, 100, 100], [80, 255, 255]),
        'yellow': ([20, 100, 100], [30, 255, 255])
    }
    
    detected_objects = []
    for color_name, (lower, upper) in color_ranges.items():
        lower = np.array(lower)
        upper = np.array(upper)
        mask = cv2.inRange(hsv, lower, upper)
        
        # Remove noise
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:
                x, y, w, h = cv2.boundingRect(contour)
                detected_objects.append({
                    'label': f"{color_name}_object",
                    'confidence': 1.0,
                    'bbox': (x, y, w, h),
                    'center': (x + w//2, y + h//2)
                })
                
                # Draw detection
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                cv2.putText(frame, color_name, (x, y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    
    return detected_objects

def detect_edges_contours(frame):
    """
    Detect objects in the frame by finding contours after edge detection.

    This function detects objects by first converting the frame to grayscale, then applying a Gaussian blur, followed by edge detection using the Canny algorithm. It then finds contours in the resulting image and filters them based on their area and aspect ratio. The function returns a list of detected objects, each represented as a dictionary with the keys 'label', 'confidence', 'bbox', and 'center'.

    Parameters
    ----------
    frame : numpy.ndarray
        The input frame to detect objects in.

    Returns
    -------
    detected_objects : list of dict
        A list of detected objects, each represented as a dictionary with the keys 'label', 'confidence', 'bbox', and 'center'.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Edge detection
    edges = cv2.Canny(blurred, 50, 150)
    
    # Find contours
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    detected_objects = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 1000:
            # Get bounding rectangle
            x, y, w, h = cv2.boundingRect(contour)
            
            # Calculate aspect ratio to filter objects
            aspect_ratio = float(w) / h
            if 0.2 < aspect_ratio < 5.0:
                detected_objects.append({
                    'label': 'edge_object',
                    'confidence': 1.0,
                    'bbox': (x, y, w, h),
                    'center': (x + w//2, y + h//2)
                })
                
                # Draw detection
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 255), 2)
                cv2.putText(frame, 'Object', (x, y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
    
    return detected_objects

def track_object(detected_objects, frame_center):
    """
    Determine the movement command to move the drone to center the detected object.

    Parameters
    ----------
    detected_objects : list of dict
        A list of detected objects, each represented as a dictionary with the keys 'label', 'confidence', 'bbox', and 'center'.
    frame_center : tuple of int
        The center coordinates of the frame (x, y).

    Returns
    -------
    command : str
        The movement command for the drone ("left", "right", "up", "down", or "hover").
    largest_obj : dict
        The largest detected object represented as a dictionary with the keys 'label', 'confidence', 'bbox', and 'center'.
    """
    
    if not detected_objects:
        return None
    
    # Find the largest object
    largest_obj = max(detected_objects, key=lambda obj: obj['bbox'][2] * obj['bbox'][3])
    
    obj_center = largest_obj['center']
    frame_center_x, frame_center_y = frame_center
    
    # Calculate offset from center
    offset_x = obj_center[0] - frame_center_x
    offset_y = obj_center[1] - frame_center_y
    
    # Thresholds for movement
    threshold_x = 60
    threshold_y = 60
    
    command = None
    
    # Determine movement command
    if abs(offset_x) > threshold_x:
        if offset_x > 0:
            command = "right"
        else:
            command = "left"
    elif abs(offset_y) > threshold_y:
        if offset_y > 0:
            command = "down"
        else:
            command = "up"
    else:
        command = "hover"
    
    return command, largest_obj

def main():
    """Main detection loop using UDP frames"""
    global bg_subtractor, prev_frame
    bg_subtractor = cv2.createBackgroundSubtractorMOG2(history=200, varThreshold=16, detectShadows=True)

    # Load camera calibration for optical flow
    # Default camera matrix (rough estimate for typical camera)
    camera_matrix = np.array([[800, 0, 320],
                              [0, 800, 240],
                              [0, 0, 1]], dtype=np.float32)
    
    try:
        calib_data = np.load("camera_calib.npz")
        camera_matrix = calib_data["camera_matrix"]
        print("Loaded camera calibration matrix")
    except Exception as e:
        print(f"Warning: Could not load camera calibration: {e}")
        print("Using default camera matrix for optical flow")
    
    # Parameters for Lucas-Kanade optical flow - optimized for motion detection
    feature_params = dict(maxCorners=150,        # More features for better coverage
                         qualityLevel=0.01,      # Lower quality = more corners (more sensitive)
                         minDistance=10,         # Spread features out more
                         blockSize=7)
    
    lk_params = dict(winSize=(21, 21),           # Larger window = more robust tracking
                    maxLevel=3,                  # More pyramid levels = handle larger motion
                    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 15, 0.02))

    print("Starting UDP OpenCV object detection...")
    print("Controls:")
    print("   'q' - Quit")
    print("   '1' - Face detection")
    print("   '2' - Motion detection")
    print("   '3' - Color detection")
    print("   '4' - Edge detection")
    print("   '5' - Optical Flow")
    print("   't' - Toggle tracking display")
    
    detection_mode = 1
    show_tracking = False
    frame_count = 0
    fps_counter = 0
    fps_start_time = time.time()
    
    while True:
        try:
            # Get frame from UDP queue instead of VideoCapture
            frame_data = frame_queue.get_nowait()
            frame = cv2.imdecode(np.frombuffer(frame_data, np.uint8), cv2.IMREAD_COLOR)
            
            if frame is None:
                continue
                
        except Empty:
            # No frame available, continue
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            continue
        
        frame_count += 1
        fps_counter += 1
        
        # Calculate FPS every 30 frames
        if fps_counter >= 30:
            elapsed = time.time() - fps_start_time
            fps = fps_counter / elapsed
            print(f"FPS: {fps:.1f}")
            fps_counter = 0
            fps_start_time = time.time()
        
        height, width = frame.shape[:2]
        frame_center = (width // 2, height // 2)
        
        # Draw center crosshair
        cv2.line(frame, (frame_center[0] - 20, frame_center[1]), 
                (frame_center[0] + 20, frame_center[1]), (255, 255, 255), 2)
        cv2.line(frame, (frame_center[0], frame_center[1] - 20), 
                (frame_center[0], frame_center[1] + 20), (255, 255, 255), 2)
        
        # Detect objects based on current mode
        if detection_mode == 1:
            detected_objects = detect_faces(frame)
            mode_name = "Face Detection"
        elif detection_mode == 2:
            detected_objects = detect_motion(frame)
            mode_name = "Motion Detection"
        elif detection_mode == 3:
            detected_objects = detect_colors(frame)
            mode_name = "Color Detection"
        elif detection_mode == 4:
            detected_objects = detect_edges_contours(frame)
            mode_name = "Edge Detection"
        elif detection_mode == 5:
            if prev_frame is not None:
                flow_result = detect_with_lucas_kanade_optical_flow(
                    prev_frame, frame, feature_params, lk_params, camera_matrix
                )
                detected_objects = []
                if isinstance(flow_result, dict):
                    clusters = flow_result.get('clusters', [])
                    for cluster in clusters:
                        x, y, w, h = cluster['bbox']
                        confidence = cluster.get('score', 0.5)
                        detected_objects.append({
                            'label': 'moving_object',
                            'confidence': confidence,
                            'bbox': (x, y, w, h),
                            'center': (x + w//2, y + h//2),
                            'count': cluster.get('count', 0)
                        })

                    if not detected_objects:
                        moving_pts = flow_result.get('moving_points', np.empty((0, 2)))
                        for pt in moving_pts:
                            x, y = map(int, pt)
                            detected_objects.append({
                                'label': 'moving_point',
                                'confidence': 0.4,
                                'bbox': (x - 6, y - 6, 12, 12),
                                'center': (x, y)
                            })

                    frame = flow_result.get('frame_vis', frame)
                mode_name = "Optical Flow"
            else:
                detected_objects = []
            prev_frame = frame.copy()
        else:
            detected_objects = []
            mode_name = "None"
        
        # Display info
        cv2.putText(frame, f"Mode: {mode_name}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"Objects: {len(detected_objects)}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"Tracking: {'ON' if show_tracking else 'OFF'}", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if show_tracking else (0, 0, 255), 2)
        cv2.putText(frame, f"UDP: Connected", (10, 120), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Show tracking info (no actions)
        if show_tracking and detected_objects:
            result = track_object(detected_objects, frame_center)
            if result:
                command, tracked_obj = result
                
                # Highlight tracked object
                x, y, w, h = tracked_obj['bbox']
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)
                cv2.putText(frame, f"TRACK: {command}", (x, y - 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # Show tracking info
                cv2.putText(frame, f"Command: {command}", (10, 150), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        # Display frame
        cv2.imshow('UDP OpenCV Detection', frame)
        
        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('1'):
            detection_mode = 1
            print("Switched to Face Detection")
        elif key == ord('2'):
            detection_mode = 2
            print("Switched to Motion Detection")
        elif key == ord('3'):
            detection_mode = 3
            print("Switched to Color Detection")
        elif key == ord('4'):
            detection_mode = 4
            print("Switched to Edge Detection")
        elif key == ord('5'):
            detection_mode = 5
            prev_frame = None
            print("Switched to Optical Flow Detection")
        elif key == ord('t'):
            show_tracking = not show_tracking
            print(f"Tracking display: {'ON' if show_tracking else 'OFF'}")
    
    cv2.destroyAllWindows()
    print("Detection stopped")

if __name__ == "__main__":
    main()