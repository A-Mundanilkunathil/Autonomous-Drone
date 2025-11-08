#!/usr/bin/env python3
import time
import cv2
import numpy as np
import pyrealsense2 as rs
from collections import deque

def has_sensor(profile, sensor_name_substr):
    for s in profile.get_device().sensors:
        if sensor_name_substr.lower() in s.get_info(rs.camera_info.name).lower():
            return True
    return False

def main():
    pipeline = rs.pipeline()
    config = rs.config()

    # Try typical D435i modes
    color_w, color_h, color_fps = 640, 480, 30
    depth_w, depth_h, depth_fps = 640, 480, 30

    # Enable streams
    config.enable_stream(rs.stream.depth, depth_w, depth_h, rs.format.z16, depth_fps)
    config.enable_stream(rs.stream.color, color_w, color_h, rs.format.bgr8, color_fps)

    # If the device has IMU (D435i), try enabling gyro+accel
    try:
        config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
        config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 63)
        imu_enabled = True
    except Exception:
        imu_enabled = False

    # Start streaming
    profile = pipeline.start(config)

    # Depth scale
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()  # meters per unit

    # Align depth to color
    align = rs.align(rs.stream.color)
    colorizer = rs.colorizer()  # for pretty depth visualization

    # Simple FPS tracker
    fps_hist = deque(maxlen=30)
    t0 = time.time()

    # Optional: set post-processing filters (comment out if not needed)
    spat = rs.spatial_filter()
    temp = rs.temporal_filter()

    save_idx = 0
    imu_last = {'gyro': None, 'accel': None}

    print("Press 'q' to quit, 's' to save frames.")
    while True:
        frames = pipeline.wait_for_frames()

        # Handle IMU if present (motion frames may arrive interleaved)
        if imu_enabled:
            for f in frames:
                if f.is_motion_frame():
                    md = f.as_motion_frame().get_motion_data()
                    if f.profile.stream_type() == rs.stream.gyro:
                        imu_last['gyro'] = (md.x, md.y, md.z)
                    elif f.profile.stream_type() == rs.stream.accel:
                        imu_last['accel'] = (md.x, md.y, md.z)

        # Align and fetch depth/color
        aligned = align.process(frames)
        depth = aligned.get_depth_frame()
        color = aligned.get_color_frame()

        if not depth or not color:
            continue

        # Optional filtering (improves depth stability)
        depth = spat.process(depth)
        depth = temp.process(depth)

        # Colorize depth for display
        depth_color = np.asanyarray(colorizer.colorize(depth).get_data())
        color_img = np.asanyarray(color.get_data())

        # Compose side-by-side view
        h = min(depth_color.shape[0], color_img.shape[0])
        depth_resized = cv2.resize(depth_color, (color_img.shape[1], color_img.shape[0]))
        stacked = np.hstack((color_img, depth_resized))

        # FPS overlay
        now = time.time()
        fps_hist.append(1.0 / max(now - t0, 1e-6))
        t0 = now
        fps = sum(fps_hist) / len(fps_hist)

        # IMU overlay (if available)
        overlay = f"FPS: {fps:5.1f} | Depth scale: {depth_scale*1000:.2f} mm/unit"
        if imu_enabled and (imu_last['gyro'] or imu_last['accel']):
            g = imu_last['gyro']
            a = imu_last['accel']
            if g:
                overlay += f" | Gyro (rad/s): [{g[0]:+.2f}, {g[1]:+.2f}, {g[2]:+.2f}]"
            if a:
                overlay += f" | Accel (m/s^2): [{a[0]:+.2f}, {a[1]:+.2f}, {a[2]:+.2f}]"

        cv2.putText(stacked, overlay, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2, cv2.LINE_AA)

        cv2.imshow("RealSense D435i • Color | Depth(aligned)", stacked)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite(f"realsense_color_{save_idx:03d}.png", color_img)
            cv2.imwrite(f"realsense_depth_colorized_{save_idx:03d}.png", depth_resized)
            # Also save raw depth as 16-bit PNG (depth units -> multiply by 1 to keep raw)
            depth_np = np.asanyarray(depth.get_data())
            cv2.imwrite(f"realsense_depth_raw_{save_idx:03d}.png", depth_np)
            print(f"Saved frame set #{save_idx}")
            save_idx += 1

    pipeline.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:  # macOS wheel doesn't expose rs.error
        print("RealSense error:", e)
