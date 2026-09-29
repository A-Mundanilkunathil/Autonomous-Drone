import unittest

from autonomous_drone.core.safety import forward_speed_for_clearance


class SafetyTests(unittest.TestCase):
    def speed(self, clearance):
        return forward_speed_for_clearance(
            clearance,
            0.6,
            stop_distance_m=0.5,
            caution_distance_m=0.8,
        )

    def test_missing_clearance_stops(self):
        self.assertEqual(self.speed(None), 0.0)
        self.assertEqual(self.speed(float('inf')), 0.0)
        self.assertEqual(self.speed(float('nan')), 0.0)

    def test_obstacle_inside_stop_distance_stops(self):
        self.assertEqual(self.speed(0.5), 0.0)

    def test_open_path_allows_mission_speed(self):
        self.assertEqual(self.speed(1.5), 0.6)

    def test_caution_zone_interpolates_speed(self):
        speed = self.speed(0.65)
        self.assertGreater(speed, 0.2)
        self.assertLess(speed, 0.6)


if __name__ == '__main__':
    unittest.main()
