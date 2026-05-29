#!/usr/bin/env python3
"""Record a ROS sensor_msgs/Image topic to an mp4 file.

Used by run_sim_test.sh to capture the onboard camera during a test run. We
record the bridged camera topic rather than the screen because under WSLg's
rootless Xwayland an x11grab screen capture comes out black.

Usage:
    python3 record_camera.py <ros_image_topic> <output.mp4> [fps]
"""
import sys
import signal

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraRecorder(Node):
    def __init__(self, topic: str, out_path: str, fps: float):
        super().__init__('camera_recorder')
        self.bridge = CvBridge()
        self.writer = None
        self.out_path = out_path
        self.fps = fps
        self.count = 0
        self.create_subscription(Image, topic, self._on_image, 10)
        self.get_logger().info(f'Recording {topic} -> {out_path} @ {fps} fps')

    def _on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Frame convert failed: {e}')
            return

        if self.writer is None:
            h, w = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter(self.out_path, fourcc, self.fps, (w, h))
            self.get_logger().info(f'Writer opened: {w}x{h}')

        self.writer.write(frame)
        self.count += 1

    def close(self):
        if self.writer is not None:
            self.writer.release()
        self.get_logger().info(f'Wrote {self.count} frames to {self.out_path}')


_running = True


def _handler(signum, frame):
    # Set a flag and let the spin loop exit on its own. Calling rclpy.shutdown()
    # here can hang, since a blocking rclpy.spin() never yields to the handler.
    global _running
    _running = False


def main():
    if len(sys.argv) < 3:
        print('Usage: record_camera.py <ros_image_topic> <output.mp4> [fps]')
        sys.exit(1)

    topic = sys.argv[1]
    out_path = sys.argv[2]
    fps = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0

    rclpy.init()
    node = CameraRecorder(topic, out_path, fps)

    signal.signal(signal.SIGINT, _handler)
    signal.signal(signal.SIGTERM, _handler)

    try:
        # spin_once with a timeout returns to Python regularly, so the signal
        # handler runs and we can shut down promptly and flush the video.
        while _running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
