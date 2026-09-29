import unittest

from autonomous_drone.core.control import (
    body_tracking_corrections,
    damped_distance_speed,
)


class ControlTests(unittest.TestCase):
    def test_far_target_commands_forward_motion(self):
        speed = damped_distance_speed(2.0, 0.0, 1.2, 0.3)
        self.assertGreater(speed, 0.0)

    def test_closing_rate_reduces_forward_motion(self):
        steady = damped_distance_speed(2.0, 0.0, 1.2, 0.3)
        closing = damped_distance_speed(2.0, -1.0, 1.2, 0.3)
        self.assertLess(closing, steady)

    def test_target_to_right_commands_right_and_clockwise(self):
        lateral, _, yaw_rate = body_tracking_corrections(0.5, 0.0, 0.8, 0.8, 0.6)
        self.assertLess(lateral, 0.0)
        self.assertLess(yaw_rate, 0.0)

    def test_target_below_commands_down(self):
        _, vertical, _ = body_tracking_corrections(0.0, 0.5, 0.8, 0.8, 0.6)
        self.assertLess(vertical, 0.0)


if __name__ == '__main__':
    unittest.main()
