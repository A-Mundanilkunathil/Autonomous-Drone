import cv2
import numpy as np
import time
from queue import Empty
from udp_capture import frame_queue

# Motion detection setup
bg_subtractor = cv2.createBackgroundSubtractorMOG2()

# Load Haar Cascade for face detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Variable to store previous frame for optical flow
prev_frame = None

def detect_faces(frame):
    """Fast face detection using OpenCV's Haar Cascade"""
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

def detect_with_optical_flow(prev_frame, current_frame):
    """Track movement using optical flow"""
    # Convert frames to grayscale
    prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
    curr_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    
    # Calculate optical flow
    flow = cv2.calcOpticalFlowFarneback(prev_gray, curr_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    
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

def detect_motion(frame):
    """Improved motion detection with better filtering"""
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
    """Simple color-based object detection"""
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
    """Detect objects using edge detection and contours"""
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
    """Track the largest/closest object and return movement info"""
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
                detected_objects = detect_with_optical_flow(prev_frame, frame)
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