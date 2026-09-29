import cv2
import numpy as np

from autonomous_drone.core.vision import (
    conservative_depth_percentile,
    estimate_rgbd_transform,
    sanitize_metric_depth,
)


def test_depth_sanitization_does_not_turn_infinity_into_open_space():
    sanitized = sanitize_metric_depth(
        np.array([[np.inf, np.nan, 0.0, 100.0]], dtype=np.float32),
        max_depth_m=50.0,
    )

    assert np.isinf(sanitized[0, 0])
    assert np.isinf(sanitized[0, 1])
    assert np.isinf(sanitized[0, 2])
    assert sanitized[0, 3] == 50.0


def test_clearance_fails_closed_when_depth_is_sparse():
    region = np.full((10, 10), np.inf, dtype=np.float32)
    region.flat[:5] = 4.0

    assert conservative_depth_percentile(
        region, min_valid_fraction=0.10) == 0.0


def test_clearance_uses_lower_depth_percentile():
    region = np.full((10, 10), 5.0, dtype=np.float32)
    region.flat[:20] = 0.6

    assert np.isclose(conservative_depth_percentile(region), 0.6)


def test_rgbd_pnp_recovers_metric_transform():
    camera_matrix = np.array([
        [400.0, 0.0, 160.0],
        [0.0, 400.0, 120.0],
        [0.0, 0.0, 1.0],
    ])
    keyframe_pixels = np.array([
        [80, 60], [120, 60], [160, 60], [200, 60], [240, 60],
        [80, 100], [120, 100], [160, 100], [200, 100], [240, 100],
        [80, 140], [120, 140], [160, 140], [200, 140], [240, 140],
    ], dtype=np.float32)
    depths = np.linspace(3.0, 5.0, len(keyframe_pixels), dtype=np.float32)
    depth_image = np.full((240, 320), np.nan, dtype=np.float32)
    object_points = []
    for (u, v), depth in zip(keyframe_pixels, depths):
        depth_image[int(v), int(u)] = depth
        object_points.append([
            (u - 160.0) * depth / 400.0,
            (v - 120.0) * depth / 400.0,
            depth,
        ])

    expected_rvec = np.array([0.01, -0.02, 0.015], dtype=np.float64)
    expected_translation = np.array([0.12, -0.04, 0.08], dtype=np.float64)
    current_pixels, _ = cv2.projectPoints(
        np.asarray(object_points, dtype=np.float32),
        expected_rvec,
        expected_translation,
        camera_matrix,
        np.zeros(5),
    )

    transform, inliers = estimate_rgbd_transform(
        keyframe_pixels,
        current_pixels.reshape(-1, 2),
        depth_image,
        camera_matrix,
        min_inliers=8,
    )

    assert transform is not None
    assert inliers >= 8
    np.testing.assert_allclose(
        transform[:3, 3], expected_translation, atol=2e-3)
    expected_rotation, _ = cv2.Rodrigues(expected_rvec)
    np.testing.assert_allclose(transform[:3, :3], expected_rotation, atol=2e-3)


def test_rgbd_pnp_rejects_missing_depth():
    pixels = np.array([[10.0, 10.0]] * 8, dtype=np.float32)
    transform, inliers = estimate_rgbd_transform(
        pixels,
        pixels,
        np.full((20, 20), np.nan, dtype=np.float32),
        np.eye(3),
        min_inliers=8,
    )

    assert transform is None
    assert inliers == 0
