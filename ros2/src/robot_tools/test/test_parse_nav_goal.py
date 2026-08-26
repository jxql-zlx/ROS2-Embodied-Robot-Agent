"""Unit tests for navigation goal string parsing."""

import unittest

from robot_tools.ros2_robot_tool import parse_nav_goal_string


class TestParseNavGoalString(unittest.TestCase):
    def test_x_y(self):
        self.assertEqual(parse_nav_goal_string("1.0,0.5"), (1.0, 0.5, 0.0))

    def test_x_y_yaw(self):
        self.assertEqual(parse_nav_goal_string("1.0,0.5,1.57"), (1.0, 0.5, 1.57))

    def test_x_only(self):
        self.assertEqual(parse_nav_goal_string("2.0"), (2.0, 0.0, 0.0))

    def test_empty(self):
        self.assertIsNone(parse_nav_goal_string(""))
        self.assertIsNone(parse_nav_goal_string("   "))

    def test_malformed(self):
        self.assertIsNone(parse_nav_goal_string("abc"))
        self.assertIsNone(parse_nav_goal_string("1.0,abc"))

    def test_whitespace_tolerated(self):
        self.assertEqual(parse_nav_goal_string(" 1.0 , 0.5 "), (1.0, 0.5, 0.0))


if __name__ == "__main__":
    unittest.main()
