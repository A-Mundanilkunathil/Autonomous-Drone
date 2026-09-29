import math
import unittest

from autonomous_drone.core.navigation import (
    haversine_distance_m,
    target_reached,
    wrap_pi,
)


class NavigationTests(unittest.TestCase):
    def test_haversine_returns_zero_for_same_point(self):
        self.assertEqual(haversine_distance_m(37.0, -122.0, 37.0, -122.0), 0.0)

    def test_haversine_is_symmetric(self):
        forward = haversine_distance_m(37.0, -122.0, 37.001, -121.999)
        reverse = haversine_distance_m(37.001, -121.999, 37.0, -122.0)
        self.assertAlmostEqual(forward, reverse)

    def test_target_reached_uses_absolute_altitude_error(self):
        self.assertFalse(target_reached(1.0, 1.5, -5.0, 0.8))
        self.assertTrue(target_reached(1.0, 1.5, -0.5, 0.8))

    def test_wrap_pi(self):
        self.assertAlmostEqual(wrap_pi(3.0 * math.pi), math.pi)


if __name__ == '__main__':
    unittest.main()
