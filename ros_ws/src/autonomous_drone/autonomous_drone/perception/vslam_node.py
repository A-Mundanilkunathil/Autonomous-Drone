import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, NavSatFix, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import math
from collections import deque


METERS_PER_DEG_LAT = 111319.9  # metres per degree of latitude (WGS-84)


class KeyFrame:
    """Lightweight container for a SLAM keyframe."""
    __slots__ = ['id', 'kp', 'des', 'pose']

    def __init__(self, kf_id: int, kp, des, pose: np.ndarray):
        self.id   = kf_id
        self.kp   = kp
        self.des  = des
        self.pose = pose.copy()   # 4x4 camera-to-world transform


class VSLAMNode(Node):
    """
    Visual SLAM node with:
      - ORB feature-based visual odometry (keyframe-to-keyframe)
      - Depth-scale recovery via depth image
      - Sparse 3D map accumulation (PointCloud2 on /vslam/map)
      - Lightweight loop-closure with linear pose-graph correction
      - Virtual GPS: converts VSLAM pose → NavSatFix using an initial
        GPS anchor + compass heading (/vslam/gps)

    Published topics
    ─────────────────
    /vslam/pose    PoseStamped   camera pose in odom frame
    /vslam/odom    Odometry      same pose as odometry
    /vslam/gps     NavSatFix     virtual GPS (needs real GPS anchor once)
    /vslam/map     PointCloud2   sparse 3-D landmark map
    /vslam/status  String        periodic diagnostics

    Subscribed topics
    ──────────────────
    /camera/image_raw                  Image       mono visual input
    /camera/depth_map                  Image(32FC1) metric depth
    /camera/camera_info                CameraInfo  intrinsics
    /mavros/global_position/global     NavSatFix   real GPS for anchor
    /mavros/global_position/compass_hdg Float64    drone heading (deg CW from N)
    """

    def __init__(self):
        super().__init__('vslam_node')

        self.bridge = CvBridge()

        # ── Camera intrinsics ────────────────────────────────────────────────
        self.fx = 600.0
        self.fy = 600.0
        self.cx = 320.0
        self.cy = 240.0

        # ── Feature detector / matchers ─────────────────────────────────────
        self.orb      = cv2.ORB_create(nfeatures=500)
        self.bf_cross = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.bf_knn   = cv2.BFMatcher(cv2.NORM_HAMMING)   # for loop-closure

        # ── Pose state ───────────────────────────────────────────────────────
        self.pose        = np.eye(4)   # camera-to-world transform
        self.frame_count = 0

        # ── Active keyframe (VO reference) ───────────────────────────────────
        self.kf_image       = None
        self.kf_kp          = None
        self.kf_des         = None
        self.kf_pose        = np.eye(4)
        self.frames_since_kf = 0
        self.kf_count        = 0

        # ── VO thresholds ────────────────────────────────────────────────────
        self.min_trans_m      = 0.05   # keyframe on translation (m)
        self.min_rot_deg      = 2.0    # keyframe on rotation (deg)
        self.min_px_disp      = 15.0   # skip update if features barely moved
        self.max_frames_no_kf = 30     # force keyframe after this many frames

        # ── Sparse 3-D map ───────────────────────────────────────────────────
        self.map_points: deque = deque(maxlen=5000)   # each element: [x, y, z]
        self._map_dirty = False

        # ── Keyframe database for loop closure ───────────────────────────────
        self.kf_db: list[KeyFrame] = []
        self.max_kf_db       = 60     # keep at most this many KFs
        self.loop_min_gap    = 6      # min KF-index gap to consider a loop
        self.loop_min_score  = 0.28   # min good-match fraction (Lowe ratio)
        self.last_loop_id    = -99    # prevents back-to-back corrections
        self.loop_count      = 0

        # ── Virtual GPS state ────────────────────────────────────────────────
        self.gps_origin    = None        # (lat, lon, alt) anchor
        self.vslam_at_gps  = None        # pose[:3,3] when anchor was set
        self.R_cam2enu     = np.eye(3)   # camera-frame → ENU rotation
        self.gps_ready     = False
        self._pending_gps  = None        # latest valid NavSatFix from MAVROS
        self._init_hdg     = 0.0         # compass heading at init (deg CW from N)
        self._hdg_set      = False

        # ── QoS ─────────────────────────────────────────────────────────────
        qos_be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(
            Image,      'camera/image_raw',
            self._img_cb,     qos_be)
        self.create_subscription(
            Image,      'camera/depth_map',
            self._depth_cb,   qos_be)
        self.create_subscription(
            CameraInfo, 'camera/camera_info',
            self._caminfo_cb, 10)
        self.create_subscription(
            NavSatFix,  'mavros/global_position/global',
            self._gps_cb,     qos_be)
        self.create_subscription(
            Float64,    'mavros/global_position/compass_hdg',
            self._hdg_cb,     qos_be)

        # ── Publishers ──────────────────────────────────────────────────────
        self.pose_pub   = self.create_publisher(PoseStamped, 'vslam/pose',   10)
        self.odom_pub   = self.create_publisher(Odometry,    'vslam/odom',   10)
        self.vgps_pub   = self.create_publisher(NavSatFix,   'vslam/gps',    10)
        self.map_pub    = self.create_publisher(PointCloud2, 'vslam/map',    10)
        self.status_pub = self.create_publisher(String,      'vslam/status', 10)

        # ── TF broadcaster ──────────────────────────────────────────────────
        self.tf_br = tf2_ros.TransformBroadcaster(self)

        # ── Depth buffer ─────────────────────────────────────────────────────
        self.current_depth = None

        # Map published on a timer so we don't flood the bus every frame
        self.create_timer(2.0, self._map_timer_cb)

        self.get_logger().info('VSLAM Node ready (map + virtual GPS)')

    # =========================================================================
    # Sensor callbacks
    # =========================================================================

    def _caminfo_cb(self, msg: CameraInfo):
        K = np.array(msg.k).reshape(3, 3)
        self.fx, self.fy = K[0, 0], K[1, 1]
        self.cx, self.cy = K[0, 2], K[1, 2]

    def _depth_cb(self, msg: Image):
        try:
            self.current_depth = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion error: {e}')

    def _gps_cb(self, msg: NavSatFix):
        """Cache latest GPS fix; try to set anchor once VSLAM is warm."""
        if msg.status.status < 0:
            return
        self._pending_gps = msg
        if not self.gps_ready and self.frame_count > 10:
            self._init_gps(msg)

    def _hdg_cb(self, msg: Float64):
        """Record compass heading at first receipt (used to align cam→ENU)."""
        if not self._hdg_set:
            self._init_hdg = msg.data
            self._hdg_set  = True
            self.get_logger().info(
                f'Compass heading locked at {self._init_hdg:.1f}°')

    # =========================================================================
    # Main VO pipeline
    # =========================================================================

    def _img_cb(self, msg: Image):
        """Process one camera frame through the visual-odometry pipeline."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return

        self.frame_count     += 1
        self.frames_since_kf += 1

        kp, des = self.orb.detectAndCompute(frame, None)

        # First frame — just register as keyframe, nothing to match against
        if self.kf_image is None or self.kf_des is None or des is None:
            self._register_keyframe(frame, kp, des, self.pose.copy())
            return

        # ── Match against active keyframe ────────────────────────────────────
        matches = self.bf_cross.match(self.kf_des, des)
        matches = sorted(matches, key=lambda m: m.distance)[:100]
        if len(matches) < 8:
            self.get_logger().warn('Too few matches for pose estimation')
            return

        pts1 = np.float32([self.kf_kp[m.queryIdx].pt for m in matches])
        pts2 = np.float32([kp[m.trainIdx].pt          for m in matches])

        avg_disp = np.mean(np.linalg.norm(pts2 - pts1, axis=1))
        if (avg_disp < self.min_px_disp and
                self.frames_since_kf < self.max_frames_no_kf):
            return   # camera barely moved; skip pose update

        K_mat = np.array([[self.fx, 0,       self.cx],
                          [0,       self.fy, self.cy],
                          [0,       0,       1      ]])

        E, mask = cv2.findEssentialMat(
            pts1, pts2, K_mat,
            method=cv2.RANSAC, prob=0.999, threshold=1.0)
        if E is None:
            return

        _, R, t, mask = cv2.recoverPose(E, pts1, pts2, K_mat)

        scale = (self._estimate_scale(pts1, pts2, mask)
                 if self.current_depth is not None else 1.0)

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3]  = (t * scale).flatten()

        # Cumulative pose: keyframe_pose × inv(T_kf_to_current)
        self.pose = self.kf_pose @ np.linalg.inv(T)
        self._publish_pose(msg.header.stamp)

        # ── Keyframe decision ────────────────────────────────────────────────
        trans    = np.linalg.norm(t * scale)
        rot_deg  = (np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
                    * 180 / np.pi)

        if (trans   > self.min_trans_m or
                rot_deg > self.min_rot_deg or
                self.frames_since_kf >= self.max_frames_no_kf):
            self._register_keyframe(frame, kp, des, self.pose.copy())

    # =========================================================================
    # Keyframe management & map building
    # =========================================================================

    def _register_keyframe(self, frame, kp, des, pose: np.ndarray):
        """Register a new keyframe, extend the 3-D map, check loop closure."""
        self.kf_image        = frame
        self.kf_kp           = kp
        self.kf_des          = des
        self.kf_pose         = pose.copy()
        self.frames_since_kf = 0
        self.kf_count       += 1

        # Lift feature pixels to 3-D world points and append to map
        if self.current_depth is not None and kp:
            pts3d = self._unproject(kp, pose)
            if len(pts3d):
                self.map_points.extend(pts3d.tolist())
                self._map_dirty = True

        # Add to KF database (used for loop closure)
        kf = KeyFrame(self.kf_count, kp, des, pose)
        self.kf_db.append(kf)
        if len(self.kf_db) > self.max_kf_db:
            self.kf_db.pop(0)

        # Loop-closure check
        if des is not None and self.kf_count > self.loop_min_gap * 2:
            self._check_loop(kf)

        # Lazy GPS anchor init (GPS may arrive after VSLAM starts)
        if (not self.gps_ready and
                self._pending_gps is not None and
                self.frame_count > 10):
            self._init_gps(self._pending_gps)

    def _unproject(self, kp: list, pose: np.ndarray) -> np.ndarray:
        """
        Back-project keypoint pixels to 3-D world coordinates using the
        current depth image.

        Returns (N, 3) float32 array of world-frame points.
        """
        h, w = self.current_depth.shape
        R, t = pose[:3, :3], pose[:3, 3]
        pts  = []
        for k in kp:
            u, v = int(k.pt[0]), int(k.pt[1])
            if not (0 <= u < w and 0 <= v < h):
                continue
            d = float(self.current_depth[v, u])
            if not (0.2 < d < 20.0):
                continue
            # Camera-frame 3-D point
            X = (u - self.cx) * d / self.fx
            Y = (v - self.cy) * d / self.fy
            # World-frame point
            pts.append(R @ np.array([X, Y, d]) + t)

        return (np.array(pts, dtype=np.float32)
                if pts else np.empty((0, 3), dtype=np.float32))

    # =========================================================================
    # Depth-based scale estimation (unchanged from original)
    # =========================================================================

    def _estimate_scale(self, pts1, pts2, mask) -> float:
        scales = []
        h, w   = self.current_depth.shape

        for i, (p1, p2) in enumerate(zip(pts1, pts2)):
            if mask[i] == 0:
                continue
            x1, y1 = int(p1[0]), int(p1[1])
            if not (0 <= x1 < w and 0 <= y1 < h):
                continue
            d1 = float(self.current_depth[y1, x1])
            if not (0.2 < d1 < 20.0):
                continue
            X1 = (p1[0] - self.cx) * d1 / self.fx
            Y1 = (p1[1] - self.cy) * d1 / self.fy

            x2, y2 = int(p2[0]), int(p2[1])
            if not (0 <= x2 < w and 0 <= y2 < h):
                continue
            d2 = float(self.current_depth[y2, x2])
            if not (0.2 < d2 < 20.0):
                continue
            X2 = (p2[0] - self.cx) * d2 / self.fx
            Y2 = (p2[1] - self.cy) * d2 / self.fy

            dist = math.sqrt((X2-X1)**2 + (Y2-Y1)**2 + (d2-d1)**2)
            if dist > 0.01:
                scales.append(dist)

        if len(scales) < 5:
            return 1.0

        a   = np.array(scales)
        med = float(np.median(a))
        mad = float(np.median(np.abs(a - med)))

        if mad < 1e-6:
            return med

        inliers = a[np.abs(a - med) < 2.5 * mad * 1.4826]
        return float(np.mean(inliers)) if len(inliers) > 3 else med

    # =========================================================================
    # Loop-closure (lightweight bag-of-features)
    # =========================================================================

    def _check_loop(self, curr: KeyFrame):
        """
        Compare curr against older keyframes in the database.
        A loop is declared when Lowe-ratio-filtered match fraction ≥
        loop_min_score against a non-adjacent keyframe.
        """
        best_score, best_kf = 0.0, None

        # Only check KFs that are at least loop_min_gap steps old
        for kf in self.kf_db[:-self.loop_min_gap]:
            if (kf.des is None or curr.des is None or
                    abs(kf.id - curr.id) < self.loop_min_gap or
                    len(kf.des) < 2 or len(curr.des) < 2):
                continue
            try:
                raw = self.bf_knn.knnMatch(kf.des, curr.des, k=2)
            except cv2.error:
                continue

            good = sum(1 for pair in raw
                       if len(pair) == 2
                       and pair[0].distance < 0.75 * pair[1].distance)
            score = good / len(kf.des)
            if score > best_score:
                best_score, best_kf = score, kf

        if (best_score >= self.loop_min_score and
                best_kf is not None and
                curr.id - self.last_loop_id > self.loop_min_gap):
            self._apply_loop_correction(curr, best_kf, best_score)

    def _apply_loop_correction(self, curr: KeyFrame,
                               loop_kf: KeyFrame, score: float):
        """
        Linear pose-graph correction when a loop is detected.

        The positional drift at curr relative to loop_kf is spread
        proportionally across all keyframes between the two, and also
        applied to the live tracking state.
        """
        self.loop_count    += 1
        self.last_loop_id   = curr.id

        delta = loop_kf.pose[:3, 3] - curr.pose[:3, 3]
        alpha = min(score * 0.6, 0.50)  # cap correction at 50 %

        try:
            li = self.kf_db.index(loop_kf)
            ci = self.kf_db.index(curr)
        except ValueError:
            return  # one of them was pruned

        span = max(ci - li, 1)
        for i in range(li + 1, ci + 1):
            blend = (i - li) / span       # 0 near loop_kf, 1 at curr
            self.kf_db[i].pose[:3, 3] += delta * alpha * blend

        # Correct live state as well
        self.pose[:3, 3]    += delta * alpha
        self.kf_pose[:3, 3] += delta * alpha

        self.get_logger().info(
            f'Loop #{self.loop_count}: KF{curr.id}→KF{loop_kf.id} '
            f'score={score:.2f}  correction={np.linalg.norm(delta*alpha):.3f}m')

    # =========================================================================
    # Virtual GPS
    # =========================================================================

    def _init_gps(self, msg: NavSatFix):
        """
        Anchor the VSLAM origin to a real GPS coordinate.

        Camera frame conventions (forward-facing camera):
          x = right in image  →  East  (when drone faces North)
          y = down in image   →  -Up
          z = out of lens     →  North (when drone faces North)

        With compass heading H (degrees CW from North) the rotation
        from camera frame to ENU is:

            R_cam2enu = [[ cos H,  0,  sin H ],   # East
                         [-sin H,  0,  cos H ],   # North
                         [  0,    -1,   0    ]]   # Up
        """
        self.gps_origin   = (msg.latitude, msg.longitude, msg.altitude)
        self.vslam_at_gps = self.pose[:3, 3].copy()

        H = math.radians(self._init_hdg if self._hdg_set else 0.0)
        self.R_cam2enu = np.array([
            [ math.cos(H), 0.0,  math.sin(H)],
            [-math.sin(H), 0.0,  math.cos(H)],
            [ 0.0,        -1.0,  0.0         ],
        ])
        self.gps_ready = True
        self.get_logger().info(
            f'GPS anchor set: ({msg.latitude:.6f}, {msg.longitude:.6f}, '
            f'{msg.altitude:.1f} m)  hdg={self._init_hdg:.1f}°')

    def _make_vgps(self, stamp) -> NavSatFix:
        """Convert current VSLAM pose to a virtual NavSatFix."""
        # VSLAM displacement since the GPS anchor, rotated to ENU
        delta_cam = self.pose[:3, 3] - self.vslam_at_gps
        enu = self.R_cam2enu @ delta_cam   # [East, North, Up]

        lat0, lon0, alt0 = self.gps_origin
        lat_rad = math.radians(lat0)

        msg = NavSatFix()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'vslam_link'
        msg.latitude   = lat0 + enu[1] / METERS_PER_DEG_LAT
        msg.longitude  = lon0 + enu[0] / (METERS_PER_DEG_LAT * math.cos(lat_rad))
        msg.altitude   = alt0 + enu[2]
        msg.status.status  = 0   # STATUS_FIX
        msg.status.service = 1   # SERVICE_GPS

        # Covariance grows slowly with distance traveled (VO drift model)
        dist = float(np.linalg.norm(delta_cam))
        cov  = (0.5 + dist * 0.015) ** 2
        msg.position_covariance = [cov, 0.0, 0.0,
                                   0.0, cov, 0.0,
                                   0.0, 0.0, cov * 4.0]
        msg.position_covariance_type = 2   # COVARIANCE_TYPE_DIAGONAL_KNOWN
        return msg

    # =========================================================================
    # Map publishing
    # =========================================================================

    def _map_timer_cb(self):
        """Publish the sparse 3-D map when new points have been added."""
        if not self._map_dirty or not self.map_points:
            return
        pts = np.array(self.map_points, dtype=np.float32)
        if pts.ndim == 2 and pts.shape[1] == 3:
            self.map_pub.publish(self._make_pc2(pts))
            self._map_dirty = False

    def _make_pc2(self, pts: np.ndarray) -> PointCloud2:
        """Build a minimal XYZ PointCloud2 from an (N, 3) float32 array."""
        msg = PointCloud2()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.height     = 1
        msg.width      = len(pts)
        msg.fields     = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step   = 12
        msg.row_step     = 12 * len(pts)
        msg.is_dense     = True
        msg.data         = pts.tobytes()
        return msg

    # =========================================================================
    # Pose publishing
    # =========================================================================

    def _publish_pose(self, stamp):
        pos  = self.pose[:3, 3]
        quat = self._rot2quat(self.pose[:3, :3])

        # ── PoseStamped ──────────────────────────────────────────────────────
        pose_msg = PoseStamped()
        pose_msg.header.stamp    = stamp
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose.position.x = float(pos[0])
        pose_msg.pose.position.y = float(pos[1])
        pose_msg.pose.position.z = float(pos[2])
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        self.pose_pub.publish(pose_msg)

        # ── Odometry ─────────────────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp       = stamp
        odom.header.frame_id    = 'odom'
        odom.child_frame_id     = 'base_link'
        odom.pose.pose          = pose_msg.pose
        self.odom_pub.publish(odom)

        # ── TF ───────────────────────────────────────────────────────────────
        tf_msg = TransformStamped()
        tf_msg.header.stamp      = stamp
        tf_msg.header.frame_id   = 'odom'
        tf_msg.child_frame_id    = 'vslam_link'
        tf_msg.transform.translation.x = float(pos[0])
        tf_msg.transform.translation.y = float(pos[1])
        tf_msg.transform.translation.z = float(pos[2])
        tf_msg.transform.rotation      = pose_msg.pose.orientation
        self.tf_br.sendTransform(tf_msg)

        # ── Virtual GPS ───────────────────────────────────────────────────────
        if self.gps_ready:
            self.vgps_pub.publish(self._make_vgps(stamp))

        # ── Status log ───────────────────────────────────────────────────────
        if self.frame_count % 60 == 0:
            s = (f'frames={self.frame_count} kf={self.kf_count} '
                 f'map={len(self.map_points)} loops={self.loop_count} '
                 f'vgps={self.gps_ready}')
            self.get_logger().info(f'VSLAM: {s}', throttle_duration_sec=5.0)
            sm = String()
            sm.data = s
            self.status_pub.publish(sm)

    # =========================================================================
    # Utilities
    # =========================================================================

    @staticmethod
    def _rot2quat(R: np.ndarray) -> np.ndarray:
        """
        Shepperd's method: 3×3 rotation matrix → quaternion [x, y, z, w].
        Numerically stable across all rotation angles.
        """
        trace = float(np.trace(R))
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            return np.array([(R[2, 1] - R[1, 2]) * s,
                              (R[0, 2] - R[2, 0]) * s,
                              (R[1, 0] - R[0, 1]) * s,
                              0.25 / s])
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            return np.array([0.25 * s,
                              (R[0, 1] + R[1, 0]) / s,
                              (R[0, 2] + R[2, 0]) / s,
                              (R[2, 1] - R[1, 2]) / s])
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            return np.array([(R[0, 1] + R[1, 0]) / s,
                              0.25 * s,
                              (R[1, 2] + R[2, 1]) / s,
                              (R[0, 2] - R[2, 0]) / s])
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            return np.array([(R[0, 2] + R[2, 0]) / s,
                              (R[1, 2] + R[2, 1]) / s,
                              0.25 * s,
                              (R[1, 0] - R[0, 1]) / s])


def main(args=None):
    rclpy.init(args=args)
    node = VSLAMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
