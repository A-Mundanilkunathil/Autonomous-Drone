import time
import threading
import os
from datetime import datetime
import numpy as np

try:
    import cv2
    CAMERA_AVAILABLE = True
except ImportError:
    print("OpenCV not available. Camera functionality will be disabled.")
    CAMERA_AVAILABLE = False
    
class DroneCamera:
    def __init__(self, camera_index=0, resolution=(640, 480), framerate=30):
        self.camera_index = camera_index
        self.resolution = resolution
        self.framerate = framerate
        
        self.camera = None  # Camera object
        self.is_running = False # Flag to indicate if the camera is running
        self.current_frame = None # Current frame from the camera
        self.frame_lock = threading.Lock() # Thread-safe lock for frame access

    def start(self):
        if not CAMERA_AVAILABLE:
            self.is_running = False
            return False
        
        try:
            self.camera = cv2.VideoCapture(self.camera_index)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            self.camera.set(cv2.CAP_PROP_FPS, self.framerate)
            
            if not self.camera.isOpened():
                print("Error: Camera not accessible.")
                return False
            
            # Start frame capture thread
            self.is_running = True
            self.capture_thread = threading.Thread(target=self._capture_loop)
            self.capture_thread.daemon = True
            self.capture_thread.start()
            
            print("Camera started.")
            return True
        except Exception as e:
            print(f"Error starting camera: {e}")
            return False
        
    def capture_loop(self):
        while self.is_running and self.camera.isOpened():
            ret, frame = self.camera.read()
            
            if ret:
                with self.frame_lock:
                    self.current_frame = frame
            else:
                print("Error capturing frame.")
                break
    
    def get_frame(self):
        if not CAMERA_AVAILABLE or not self.is_running:
            # Return a dummy frame in simulation mode
            height, width = self.resolution[1], self.resolution[0]
            dummy_frame = np.zeros((height, width, 3), dtype=np.uint8)
            cv2.putText(dummy_frame, "Camera not available", (50, height//2), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            return dummy_frame
        
        with self.frame_lock:
            if self.current_frame is not None:
                return self.current_frame.copy()
            else:
                # Return a dummy frame if no frame is available
                height, width = self.resolution[1], self.resolution[0]
                dummy_frame = np.zeros((height, width, 3), dtype=np.uint8)
                cv2.putText(dummy_frame, "No frame available", (50, height//2), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                return dummy_frame
            
    def stop(self):
        self.is_running = False
        if self.capture_thread and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=1.0)
            
        if CAMERA_AVAILABLE and self.camera:
            self.camera.release()
            self.camera = None
            print("Camera stopped.")