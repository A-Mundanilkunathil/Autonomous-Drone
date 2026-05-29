#!/usr/bin/env python3
"""Record the onboard camera with a perception/avoidance debug overlay.

Used by run_sim_test.sh for the perception tests so you can SEE what the drone
perceives and decides — since the onboard cam is first-person (the drone isn't
in its own view, so its body can't be boxed). Overlays:

  * depth "danger" shading  — near obstacles tinted red (what avoidance reacts to)
  * forward clearance bar    — /avoidance/forward_clearance
  * steering indicator       — /avoidance/cmd_vel (lateral + yaw)
  * detection boxes          — /detected_objects (YOLO; used by the follow test)

Usage:
    python3 record_overlay.py <camera_topic> <output.mp4> [fps]
"""
import sys
import signal

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge

NEAR_M = 2.5      # depth below this is shaded as "close"
DET_CONF = 0.30   # only draw detections at/above this confidence

_running = True


def _handler(signum, frame):
    global _running
    _running = False


class OverlayRecorder(Node):
    def __init__(self, camera_topic, out_path, fps):
        super().__init__('overlay_recorder')
        self.bridge = CvBridge()
        self.out_path = out_path
        self.fps = fps
        self.writer = None
        self.count = 0

        self.depth = None
        self.detections = None
        self.clearance = None
        self.cmd = None

        # BEST_EFFORT subscriber is compatible with both reliable and
        # best-effort publishers, so it works for every source topic.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)

        # Base image: the gz camera (bridged to ROS) — the topic we know
        # publishes reliably. The overlays come from the perception topics.
        self.create_subscription(Image, camera_topic, self._on_image, qos)
        self.create_subscription(Image, '/camera/depth_map', self._on_depth, qos)
        self.create_subscription(Detection2DArray, '/detected_objects', self._on_det, qos)
        self.create_subscription(Float32, '/avoidance/forward_clearance', self._on_clear, qos)
        self.create_subscription(TwistStamped, '/avoidance/cmd_vel', self._on_cmd, qos)

        self.get_logger().info(f'Overlay recorder: base={camera_topic} -> {out_path} @ {fps} fps')

    def _on_depth(self, msg):
        try:
            self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception:
            pass

    def _on_det(self, msg):
        self.detections = msg

    def _on_clear(self, msg):
        self.clearance = msg.data

    def _on_cmd(self, msg):
        self.cmd = msg.twist

    def _on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'image convert failed: {e}')
            return

        h, w = frame.shape[:2]

        # --- depth danger shading: tint pixels closer than NEAR_M red ---
        if self.depth is not None:
            d = self.depth
            if d.shape[:2] != (h, w):
                d = cv2.resize(d, (w, h), interpolation=cv2.INTER_NEAREST)
            near = np.isfinite(d) & (d > 0.05) & (d < NEAR_M)
            if near.any():
                tint = frame.copy()
                tint[near] = (0, 0, 255)
                frame = cv2.addWeighted(frame, 0.75, tint, 0.25, 0)

        # --- YOLO detection boxes ---
        if self.detections is not None:
            for det in self.detections.detections:
                if not det.results:
                    continue
                hyp = det.results[0].hypothesis
                if hyp.score < DET_CONF:
                    continue
                cx, cy = det.bbox.center.position.x, det.bbox.center.position.y
                bw, bh = det.bbox.size_x, det.bbox.size_y
                x1, y1 = int(cx - bw / 2), int(cy - bh / 2)
                x2, y2 = int(cx + bw / 2), int(cy + bh / 2)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f'{hyp.class_id} {hyp.score:.2f}', (x1, max(y1 - 5, 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        # --- clearance bar (top-left) ---
        if self.clearance is not None:
            c = max(0.0, min(self.clearance, 10.0))
            frac = c / 10.0
            col = (0, 0, 255) if c < NEAR_M else (0, 200, 0)
            cv2.rectangle(frame, (10, 10), (210, 30), (40, 40, 40), -1)
            cv2.rectangle(frame, (10, 10), (10 + int(200 * frac), 30), col, -1)
            cv2.putText(frame, f'clearance {c:.1f}m', (14, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # --- avoidance steering indicator (bottom-center) ---
        if self.cmd is not None:
            vy = self.cmd.linear.y     # lateral (FLU: +left)
            wz = self.cmd.angular.z    # yaw rate (+left)
            steer = vy + wz
            label = 'STRAIGHT'
            if steer > 0.05:
                label = 'AVOID  <-- LEFT'
            elif steer < -0.05:
                label = 'AVOID  RIGHT -->'
            cx0 = w // 2
            cv2.arrowedLine(frame, (cx0, h - 25),
                            (int(cx0 + np.clip(steer, -1, 1) * 120), h - 25),
                            (0, 255, 255), 3, tipLength=0.3)
            cv2.putText(frame, label, (cx0 - 80, h - 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA)

        if self.writer is None:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter(self.out_path, fourcc, self.fps, (w, h))
            self.get_logger().info(f'writer opened {w}x{h}')
        self.writer.write(frame)
        self.count += 1

    def close(self):
        if self.writer is not None:
            self.writer.release()
        self.get_logger().info(f'wrote {self.count} frames to {self.out_path}')


def main():
    if len(sys.argv) < 3:
        print('Usage: record_overlay.py <camera_topic> <output.mp4> [fps]')
        sys.exit(1)
    camera_topic = sys.argv[1]
    out_path = sys.argv[2]
    fps = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0

    rclpy.init()
    node = OverlayRecorder(camera_topic, out_path, fps)
    signal.signal(signal.SIGINT, _handler)
    signal.signal(signal.SIGTERM, _handler)
    try:
        while _running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
