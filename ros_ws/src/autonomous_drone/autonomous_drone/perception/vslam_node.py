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
from collections import OrderedDict, deque
from autonomous_drone.core.vision import estimate_rgbd_transform


METERS_PER_DEG_LAT = 111319.9  # metres per degree of latitude (WGS-84)


class KeyFrame:
    """Lightweight container for a SLAM keyframe."""
    __slots__ = ['id', 'kp', 'des', 'pose', 'depth']

    def __init__(self, kf_id: int, kp, des, pose: np.ndarray, depth: np.ndarray):
        self.id   = kf_id
        self.kp   = kp
        self.des  = des
        self.pose = pose.copy()   # 4x4 camera-to-world transform
        self.depth = depth.copy()


class VSLAMNode(Node):
    """
    Experimental RGB-D visual odometry node with:
      - Timestamp-paired RGB and metric-depth processing
      - ORB feature matching and metric PnP motion estimation
      - Sparse 3D map accumulation (PointCloud2 on /vslam/map)
      - Optional experimental loop-closure correction
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

        self.enable_loop_closure = bool(
            self.declare_parameter('enable_loop_closure', False).value)
        self.enable_virtual_gps = bool(
            self.declare_parameter('enable_virtual_gps', False).value)
        self.max_pending_frames = int(
            self.declare_parameter('max_pending_frames', 120).value)
        self.min_pnp_inliers = int(
            self.declare_parameter('min_pnp_inliers', 8).value)
        self.max_frame_translation_m = float(
            self.declare_parameter('max_frame_translation_m', 2.0).value)

        # ── Camera intrinsics ────────────────────────────────────────────────
        self.fx = 600.0
        self.fy = 600.0
        self.cx = 320.0
        self.cy = 240.0
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        # ── Feature detector / matchers ─────────────────────────────────────
        self.orb      = cv2.ORB_create(nfeatures=500)
        self.bf_cross = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.bf_knn   = cv2.BFMatcher(cv2.NORM_HAMMING)   # for loop-closure

        # ── Pose state ───────────────────────────────────────────────────────
        self.pose        = np.eye(4)   # camera-to-world transform
        self.frame_count = 0
        self._last_inlier_count = 0

        # ── Active keyframe (VO reference) ───────────────────────────────────
        self.kf_image       = None
        self.kf_kp          = None
        self.kf_des         = None
        self.kf_pose        = np.eye(4)
        self.kf_depth       = None
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
            depth=1)

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

        # Pair RGB and depth by source timestamp. Both camera bridges preserve
        # the RGB stamp on the generated depth image, so stale frames are never
        # mixed into a metric pose estimate.
        self._pending_images = OrderedDict()
        self._pending_depths = OrderedDict()

        # Map published on a timer so we don't flood the bus every frame
        self.create_timer(2.0, self._map_timer_cb)

        self.get_logger().info(
            'RGB-D visual odometry ready '
            f'(loop_closure={self.enable_loop_closure}, '
            f'virtual_gps={self.enable_virtual_gps})')

    # =========================================================================
    # Sensor callbacks
    # =========================================================================

    def _caminfo_cb(self, msg: CameraInfo):
        K = np.array(msg.k).reshape(3, 3)
        if K[0, 0] <= 0.0 or K[1, 1] <= 0.0:
            self.get_logger().warn('Ignoring invalid camera intrinsics')
            return
        self.fx, self.fy = K[0, 0], K[1, 1]
        self.cx, self.cy = K[0, 2], K[1, 2]
        if msg.d:
            self.dist_coeffs = np.asarray(msg.d, dtype=np.float64).reshape(-1, 1)

    def _depth_cb(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion error: {e}')
            return
        key = self._stamp_ns(msg.header.stamp)
        self._pending_depths[key] = (depth, msg.header.stamp)
        self._trim_pending(self._pending_depths)
        self._process_pair(key)

    def _gps_cb(self, msg: NavSatFix):
        """Cache latest GPS fix; try to set anchor once VSLAM is warm."""
        if msg.status.status < 0:
            return
        self._pending_gps = msg
        if (self.enable_virtual_gps and not self.gps_ready
                and self.frame_count > 10 and self._hdg_set):
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
        """Cache an RGB frame until depth with the same source stamp arrives."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return
        key = self._stamp_ns(msg.header.stamp)
        self._pending_images[key] = (frame, msg.header.stamp)
        self._trim_pending(self._pending_images)
        self._process_pair(key)

    @staticmethod
    def _stamp_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _trim_pending(self, pending: OrderedDict):
        while len(pending) > self.max_pending_frames:
            pending.popitem(last=False)

    def _process_pair(self, key: int):
        image_item = self._pending_images.get(key)
        depth_item = self._pending_depths.get(key)
        if image_item is None or depth_item is None:
            return
        frame, stamp = self._pending_images.pop(key)
        depth, _ = self._pending_depths.pop(key)
        if frame.shape[:2] != depth.shape[:2]:
            self.get_logger().warn('Dropping RGB/depth pair with mismatched dimensions')
            return
        self._process_frame(frame, depth, stamp)

    def _process_frame(self, frame: np.ndarray, depth: np.ndarray, stamp):
        """Process one synchronized RGB-D frame through visual odometry."""

        self.frame_count     += 1
        self.frames_since_kf += 1

        kp, des = self.orb.detectAndCompute(frame, None)

        if des is None or kp is None or len(kp) < self.min_pnp_inliers:
            self.get_logger().warn(
                'RGB-D frame has too few visual features',
                throttle_duration_sec=2.0)
            return

        # First usable frame — register it as the tracking keyframe.
        if self.kf_image is None or self.kf_des is None:
            self._register_keyframe(frame, depth, kp, des, self.pose.copy())
            return

        # ── Match against active keyframe ────────────────────────────────────
        try:
            matches = self.bf_cross.match(self.kf_des, des)
        except cv2.error as exc:
            self.get_logger().warn(f'Feature matching failed: {exc}')
            return
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

        transform, inlier_count = self._estimate_rgbd_transform(
            pts1, pts2, self.kf_depth, K_mat)
        if transform is None:
            self.get_logger().warn(
                'RGB-D pose rejected: insufficient geometrically consistent matches',
                throttle_duration_sec=2.0)
            return
        self._last_inlier_count = inlier_count

        T = transform
        R = T[:3, :3]
        translation = T[:3, 3]
        trans = float(np.linalg.norm(translation))
        if not np.isfinite(trans) or trans > self.max_frame_translation_m:
            self.get_logger().warn(
                f'RGB-D pose rejected: implausible translation {trans:.2f} m',
                throttle_duration_sec=2.0)
            return

        # Cumulative pose: keyframe_pose × inv(T_kf_to_current)
        self.pose = self.kf_pose @ np.linalg.inv(T)
        self._publish_pose(stamp)

        # ── Keyframe decision ────────────────────────────────────────────────
        rot_deg  = (np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
                    * 180 / np.pi)

        if (trans   > self.min_trans_m or
                rot_deg > self.min_rot_deg or
                self.frames_since_kf >= self.max_frames_no_kf):
            self._register_keyframe(frame, depth, kp, des, self.pose.copy())

    # =========================================================================
    # Keyframe management & map building
    # =========================================================================

    def _register_keyframe(self, frame, depth, kp, des, pose: np.ndarray):
        """Register a new keyframe, extend the 3-D map, check loop closure."""
        self.kf_image        = frame
        self.kf_kp           = kp
        self.kf_des          = des
        self.kf_pose         = pose.copy()
        self.kf_depth        = depth.copy()
        self.frames_since_kf = 0
        self.kf_count       += 1

        # Lift feature pixels to 3-D world points and append to map
        if kp:
            pts3d = self._unproject(kp, pose, depth)
            if len(pts3d):
                self.map_points.extend(pts3d.tolist())
                self._map_dirty = True

        # Add to KF database (used for loop closure)
        kf = KeyFrame(self.kf_count, kp, des, pose, depth)
        self.kf_db.append(kf)
        if len(self.kf_db) > self.max_kf_db:
            self.kf_db.pop(0)

        # Loop-closure check
        if (self.enable_loop_closure and des is not None
                and self.kf_count > self.loop_min_gap * 2):
            self._check_loop(kf)

        # Lazy GPS anchor init (GPS may arrive after VSLAM starts)
        if (self.enable_virtual_gps and not self.gps_ready and
                self._pending_gps is not None and
                self.frame_count > 10 and self._hdg_set):
            self._init_gps(self._pending_gps)

    def _unproject(self, kp: list, pose: np.ndarray,
                   depth: np.ndarray) -> np.ndarray:
        """
        Back-project keypoint pixels to 3-D world coordinates using the
        current depth image.

        Returns (N, 3) float32 array of world-frame points.
        """
        h, w = depth.shape
        R, t = pose[:3, :3], pose[:3, 3]
        pts  = []
        pixels = np.asarray([k.pt for k in kp], dtype=np.float32)
        K_mat = np.array([[self.fx, 0.0, self.cx],
                          [0.0, self.fy, self.cy],
                          [0.0, 0.0, 1.0]])
        rays = cv2.undistortPoints(
            pixels.reshape(-1, 1, 2), K_mat,
            self.dist_coeffs).reshape(-1, 2)
        for k, ray in zip(kp, rays):
            u, v = int(k.pt[0]), int(k.pt[1])
            if not (0 <= u < w and 0 <= v < h):
                continue
            d = float(depth[v, u])
            if not (np.isfinite(d) and 0.2 < d < 20.0):
                continue
            # Camera-frame 3-D point
            X = float(ray[0]) * d
            Y = float(ray[1]) * d
            # World-frame point
            pts.append(R @ np.array([X, Y, d]) + t)

        return (np.array(pts, dtype=np.float32)
                if pts else np.empty((0, 3), dtype=np.float32))

    # =========================================================================
    # Metric RGB-D motion estimation
    # =========================================================================

    def _estimate_rgbd_transform(self, pts1, pts2, keyframe_depth, K_mat):
        """Estimate keyframe→current metric transform with RGB-D PnP."""
        if keyframe_depth is None:
            return None, 0
        return estimate_rgbd_transform(
            pts1,
            pts2,
            keyframe_depth,
            K_mat,
            self.dist_coeffs,
            min_inliers=self.min_pnp_inliers,
        )

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
        # This pose belongs to the camera/VO frame. Publishing it as base_link
        # would silently ignore the camera-to-body extrinsic transform.
        odom.child_frame_id     = 'vslam_link'
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
                 f'inliers={self._last_inlier_count} '
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
