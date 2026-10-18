"""Unit tests for the pure command->velocity mapping logic."""

import unittest
import math

from robot_control.controller import (
    action_to_velocity,
    validate_command,
)


class TestValidateCommand(unittest.TestCase):
    def test_unknown_action(self):
        ok, err = validate_command("fly", 0.1)
        self.assertFalse(ok)
        self.assertIn("unknown action", err)

    def test_navigate_not_available_yet(self):
        ok, err = validate_command("navigate")
        self.assertFalse(ok)
        self.assertIn("Navigation2", err)

    def test_speed_limits(self):
        self.assertFalse(validate_command("move", linear_x=2.0)[0])
        self.assertFalse(validate_command("rotate", angular_z=5.0)[0])
        self.assertTrue(validate_command("move", linear_x=0.5, duration=1.0)[0])

    def test_negative_duration(self):
        self.assertFalse(validate_command("move", linear_x=0.3, duration=-1.0)[0])

    def test_motion_duration_must_be_finite_positive_and_bounded(self):
        for duration in (0.0, math.nan, math.inf, 30.01):
            with self.subTest(duration=duration):
                self.assertFalse(validate_command("move", linear_x=0.3, duration=duration)[0])
        self.assertTrue(validate_command("move", linear_x=0.3, duration=30.0)[0])

    def test_motion_values_must_be_finite(self):
        self.assertFalse(validate_command("rotate", angular_z=math.nan, duration=1.0)[0])
        self.assertFalse(validate_command("move", linear_x=math.inf, duration=1.0)[0])


class TestActionToVelocity(unittest.TestCase):
    def test_move_forward(self):
        vel = action_to_velocity("move", linear_x=0.3, duration=2.0)
        self.assertAlmostEqual(vel.linear_x, 0.3)
        self.assertEqual(vel.duration, 2.0)

    def test_stop_zeroes(self):
        vel = action_to_velocity("stop")
        self.assertTrue(vel.is_zero())

    def test_rotate(self):
        vel = action_to_velocity("rotate", angular_z=0.5, duration=1.57)
        self.assertAlmostEqual(vel.angular_z, 0.5)
        self.assertAlmostEqual(vel.linear_x, 0.0)

    def test_invalid_returns_error_message(self):
        vel = action_to_velocity("navigate")
        self.assertTrue(vel.is_zero())
        self.assertIn("Navigation2", vel.message)


if __name__ == "__main__":
    unittest.main()
