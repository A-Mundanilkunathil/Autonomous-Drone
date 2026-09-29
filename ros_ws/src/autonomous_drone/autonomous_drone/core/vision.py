"""Pure geometry helpers shared by perception nodes.

This module deliberately has no ROS imports, so metric visual-odometry math can
be unit tested without a ROS installation.
"""

from __future__ import annotations

import cv2
import numpy as np


def conservative_depth_percentile(
    region: np.ndarray,
    *,
    percentile: float = 20.0,
    min_valid_fraction: float = 0.10,
) -> float:
    """Return a robust clearance estimate, treating sparse depth as danger."""
    values = np.asarray(region)
    if values.size == 0:
        return 0.0
    finite = values[np.isfinite(values)]
    if finite.size / values.size < min_valid_fraction:
        return 0.0
    return float(np.percentile(finite, percentile, method='lower'))


def estimate_rgbd_transform(
    keyframe_pixels: np.ndarray,
    current_pixels: np.ndarray,
    keyframe_depth: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray | None = None,
    *,
    min_inliers: int = 8,
    min_depth_m: float = 0.2,
    max_depth_m: float = 20.0,
    reprojection_error_px: float = 2.0,
) -> tuple[np.ndarray | None, int]:
    """Estimate the metric keyframe-to-current camera transform using PnP.

    Depth is sampled only from the keyframe associated with ``keyframe_pixels``.
    This is important: sampling both frames from whichever depth image arrived
    last produces a plausible-looking but physically meaningless scale.
    """
    pts1 = np.asarray(keyframe_pixels, dtype=np.float32).reshape(-1, 2)
    pts2 = np.asarray(current_pixels, dtype=np.float32).reshape(-1, 2)
    depth_image = np.asarray(keyframe_depth)
    K = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    if len(pts1) != len(pts2) or depth_image.ndim != 2:
        return None, 0

    fx, fy = float(K[0, 0]), float(K[1, 1])
    if fx <= 0.0 or fy <= 0.0:
        return None, 0

    height, width = depth_image.shape
    dist = (np.zeros((5, 1), dtype=np.float64) if distortion is None
            else np.asarray(distortion, dtype=np.float64).reshape(-1, 1))
    normalized = cv2.undistortPoints(
        pts1.reshape(-1, 1, 2), K, dist).reshape(-1, 2)
    object_points = []
    image_points = []
    for p1, p2, ray in zip(pts1, pts2, normalized):
        u, v = int(round(float(p1[0]))), int(round(float(p1[1])))
        if not (0 <= u < width and 0 <= v < height):
            continue
        depth = float(depth_image[v, u])
        if not (np.isfinite(depth) and min_depth_m < depth < max_depth_m):
            continue
        object_points.append([float(ray[0]) * depth,
                              float(ray[1]) * depth,
                              depth])
        image_points.append(p2)

    if len(object_points) < min_inliers:
        return None, 0

    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        np.asarray(object_points, dtype=np.float32),
        np.asarray(image_points, dtype=np.float32),
        K,
        dist,
        iterationsCount=100,
        reprojectionError=float(reprojection_error_px),
        confidence=0.999,
        flags=cv2.SOLVEPNP_EPNP,
    )
    inlier_count = 0 if inliers is None else len(inliers)
    if not ok or inlier_count < min_inliers:
        return None, inlier_count

    rotation, _ = cv2.Rodrigues(rvec)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(tvec).reshape(3)
    return transform, inlier_count
