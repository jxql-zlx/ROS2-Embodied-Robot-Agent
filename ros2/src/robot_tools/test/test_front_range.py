"""Forward laser range filtering and freshness tests."""

import math
import unittest
from unittest.mock import MagicMock, patch

from sensor_msgs.msg import LaserScan
from robot_tools.ros2_robot_tool import Ros2RobotTool


class TestFrontRange(unittest.TestCase):
    def scan(self):
        scan = LaserScan()
        scan.header.frame_id = "lidar_link"
        scan.angle_min = -math.pi / 2
        scan.angle_increment = math.pi / 2
        scan.range_min = 0.12
        scan.range_max = 12.0
        scan.ranges = [0.2, 0.7, 0.3]
        return scan

    def test_only_forward_returns_are_used(self):
        self.assertAlmostEqual(Ros2RobotTool.front_range(self.scan()), 0.7, places=5)

    def test_invalid_returns_and_unknown_frame_are_unavailable(self):
        scan = self.scan()
        for value in (float("nan"), 0.01, 13.0):
            scan.ranges = [0.2, value, 0.3]
            self.assertIsNone(Ros2RobotTool.front_range(scan))
        scan = self.scan()
        scan.header.frame_id = "unknown"
        self.assertIsNone(Ros2RobotTool.front_range(scan))

    def test_infinite_returns_mean_clear_to_sensor_maximum(self):
        scan = self.scan()
        scan.ranges = [0.2, float("inf"), 0.3]
        self.assertEqual(Ros2RobotTool.front_range(scan), 12.0)

    @patch("robot_tools.ros2_robot_tool.time.monotonic", return_value=10.0)
    def test_stale_scan_is_unavailable(self, _clock):
        tool = object.__new__(Ros2RobotTool)
        tool._latest_scan = self.scan()
        tool._scan_received_at = 8.0
        self.assertFalse(tool.get_sensor("front_range")["available"])

    @patch("robot_tools.ros2_robot_tool.time.monotonic", side_effect=[0.0, 0.1])
    def test_wait_ignores_braking_scan_and_requires_stable_later_scans(self, _clock):
        tool = object.__new__(Ros2RobotTool)
        tool._scan_condition = MagicMock()
        tool._scan_history = [(0.1, 0.12)]
        def receive_scans(_timeout):
            tool._scan_history.extend([(0.6, 0.81), (0.7, 0.82), (0.8, 0.81)])
        tool._scan_condition.wait.side_effect = receive_scans
        result = tool.wait_for_front_range()
        self.assertTrue(result["available"])
        self.assertEqual(result["distance"], 0.81)

    @patch("robot_tools.ros2_robot_tool.time.monotonic", side_effect=[0.0, 4.0])
    def test_inconsistent_ranges_do_not_confirm_distance(self, _clock):
        tool = object.__new__(Ros2RobotTool)
        tool._scan_condition = MagicMock()
        tool._scan_history = [(0.6, 0.12), (0.7, 0.8), (0.8, 0.4)]
        self.assertFalse(tool.wait_for_front_range()["available"])

    @patch("robot_tools.ros2_robot_tool.time.monotonic", side_effect=[0.0, 0.1])
    def test_wait_does_not_accept_stable_near_blind_zone_echo(self, _clock):
        tool = object.__new__(Ros2RobotTool)
        tool._latest_scan = self.scan()
        tool._scan_condition = MagicMock()
        tool._scan_history = [(0.6, 0.12), (0.7, 0.12), (0.8, 0.12)]
        def receive_scans(_timeout):
            tool._scan_history.extend([(0.9, 0.81), (1.0, 0.81), (1.1, 0.82)])
        tool._scan_condition.wait.side_effect = receive_scans
        result = tool.wait_for_front_range()
        self.assertEqual(result["distance"], 0.81)


if __name__ == "__main__":
    unittest.main()
