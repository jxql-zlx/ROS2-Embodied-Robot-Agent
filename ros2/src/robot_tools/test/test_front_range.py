"""Forward laser range filtering and freshness tests."""

import math
import unittest
from collections import OrderedDict
from threading import Condition, Event
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

    @staticmethod
    def _tool():
        tool = object.__new__(Ros2RobotTool)
        tool._status_condition = Condition()
        tool._status_history = OrderedDict()
        tool._latest_status = None
        tool._latest_status_received_at = None
        tool._status_clock_epoch = 0
        tool._last_status_ros_now_ns = None
        tool._scan_condition = Condition()
        tool._scan_history = []
        tool._last_scan_stamp_ns = None
        tool._last_scan_ros_now_ns = None
        tool._latest_scan_stamp_ns = None
        tool._latest_scan = None
        tool._scan_received_at = 0.0
        return tool

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

    @patch.object(Ros2RobotTool, "_ros_time_ns", return_value=4_000_000_010)
    @patch("robot_tools.ros2_robot_tool.time.monotonic", return_value=2.0)
    def test_duplicate_or_unstamped_scan_cannot_add_confirmation_sample(self, _mono, _ros_clock):
        tool = self._tool()
        scan = self.scan()
        scan.header.stamp.sec = 4
        scan.header.stamp.nanosec = 2
        tool._on_scan(scan)
        self.assertEqual(len(tool._scan_history), 1)
        tool._on_scan(scan)
        self.assertEqual(len(tool._scan_history), 1)
        unstamped = self.scan()
        tool._on_scan(unstamped)
        self.assertEqual(len(tool._scan_history), 1)
        scan.header.stamp.nanosec = 3
        tool._on_scan(scan)
        self.assertEqual(len(tool._scan_history), 2)

    @patch.object(Ros2RobotTool, "_ros_time_ns", return_value=10_000_000_000)
    @patch("robot_tools.ros2_robot_tool.time.monotonic", return_value=10.0)
    def test_stale_scan_is_unavailable(self, _mono, _ros_clock):
        tool = self._tool()
        tool._latest_scan = self.scan()
        tool._latest_scan_stamp_ns = 9_500_000_000
        tool._scan_received_at = 8.0
        self.assertFalse(tool.get_sensor("front_range")["available"])

    @patch.object(Ros2RobotTool, "_ros_time_ns", return_value=10_000_000_000)
    @patch("robot_tools.ros2_robot_tool.time.monotonic", return_value=10.0)
    def test_newly_received_old_source_frame_is_rejected(self, _mono, _ros_clock):
        tool = self._tool()
        scan = self.scan()
        scan.header.stamp.sec = 8
        tool._on_scan(scan)
        self.assertIsNone(tool._latest_scan)
        self.assertFalse(tool.get_sensor("front_range")["available"])

    @patch.object(Ros2RobotTool, "_ros_time_ns", return_value=10_000_000_000)
    @patch("robot_tools.ros2_robot_tool.time.monotonic", return_value=10.0)
    def test_overly_future_source_frame_is_rejected(self, _mono, _ros_clock):
        tool = self._tool()
        scan = self.scan()
        scan.header.stamp.sec = 11
        tool._on_scan(scan)
        self.assertIsNone(tool._latest_scan)

    def test_out_of_order_frame_does_not_clear_current_scan_session(self):
        tool = self._tool()
        current = self.scan()
        current.header.stamp.sec = 10
        old = self.scan()
        old.header.stamp.sec = 9
        with patch.object(Ros2RobotTool, "_ros_time_ns", return_value=10_000_000_000), \
                patch("robot_tools.ros2_robot_tool.time.monotonic", side_effect=[1.0, 1.1]):
            tool._on_scan(current)
            tool._on_scan(old)
        self.assertIs(tool._latest_scan, current)
        self.assertEqual(tool._latest_scan_stamp_ns, 10_000_000_000)
        self.assertEqual(len(tool._scan_history), 1)

    def test_ros_clock_rollback_starts_a_new_scan_session(self):
        tool = self._tool()
        before = self.scan()
        before.header.stamp.sec = 100
        after = self.scan()
        after.header.stamp.sec = 5
        with patch.object(Ros2RobotTool, "_ros_time_ns", side_effect=[100_000_000_000, 5_000_000_000]), \
                patch("robot_tools.ros2_robot_tool.time.monotonic", side_effect=[1.0, 1.1]):
            tool._on_scan(before)
            tool._on_scan(after)
        self.assertIs(tool._latest_scan, after)
        self.assertEqual(len(tool._scan_history), 1)

    @patch.object(Ros2RobotTool, "_ros_time_ns", return_value=1_000_000_000)
    @patch("robot_tools.ros2_robot_tool.time.monotonic", side_effect=[0.0, 1.0, 1.1])
    def test_wait_ignores_braking_scan_and_requires_stable_later_scans(self, _mono, _ros_clock):
        tool = self._tool()
        tool._scan_condition = MagicMock()
        tool._scan_history = [(0.1, 0.12, 800_000_000)]
        def receive_scans(_timeout):
            tool._scan_history.extend([
                (0.6, 0.81, 900_000_000), (0.7, 0.82, 950_000_000),
                (0.8, 0.81, 1_000_000_000),
            ])
        tool._scan_condition.wait.side_effect = receive_scans
        result = tool.wait_for_front_range()
        self.assertTrue(result["available"])
        self.assertEqual(result["distance"], 0.81)

    @patch.object(Ros2RobotTool, "_ros_time_ns", return_value=1_000_000_000)
    @patch("robot_tools.ros2_robot_tool.time.monotonic", side_effect=[0.0, 4.0])
    def test_inconsistent_ranges_do_not_confirm_distance(self, _mono, _ros_clock):
        tool = self._tool()
        tool._scan_condition = MagicMock()
        tool._scan_history = [
            (0.6, 0.12, 900_000_000), (0.7, 0.8, 950_000_000),
            (0.8, 0.4, 1_000_000_000),
        ]
        self.assertFalse(tool.wait_for_front_range()["available"])

    @patch.object(Ros2RobotTool, "_ros_time_ns", return_value=1_000_000_000)
    @patch("robot_tools.ros2_robot_tool.time.monotonic", side_effect=[0.0, 1.5, 1.6])
    def test_wait_does_not_accept_stable_near_blind_zone_echo(self, _mono, _ros_clock):
        tool = self._tool()
        tool._latest_scan = self.scan()
        tool._scan_condition = MagicMock()
        tool._scan_history = [
            (0.6, 0.12, 900_000_000), (0.7, 0.12, 950_000_000),
            (0.8, 0.12, 1_000_000_000),
        ]
        def receive_scans(_timeout):
            tool._scan_history.extend([
                (0.9, 0.81, 900_000_000), (1.0, 0.81, 950_000_000),
                (1.1, 0.82, 1_000_000_000),
            ])
        tool._scan_condition.wait.side_effect = receive_scans
        result = tool.wait_for_front_range()
        self.assertEqual(result["distance"], 0.81)

    @patch.object(Ros2RobotTool, "_ros_time_ns", return_value=1_000_000_000)
    @patch("robot_tools.ros2_robot_tool.time.monotonic", side_effect=[0.0, 10.2])
    def test_paused_ros_clock_does_not_keep_old_received_scans_fresh(self, _mono, _ros_clock):
        tool = self._tool()
        tool._scan_condition = MagicMock()
        tool._scan_history = [
            (0.6, 0.81, 900_000_000), (0.7, 0.82, 950_000_000),
            (0.8, 0.81, 1_000_000_000),
        ]
        self.assertFalse(tool.wait_for_front_range(timeout_sec=1.0)["available"])

    def test_clock_jump_callback_clears_scan_session(self):
        tool = self._tool()
        scan = self.scan()
        scan.header.stamp.sec = 1
        tool._latest_scan = scan
        tool._latest_scan_stamp_ns = 1_000_000_000
        tool._last_scan_stamp_ns = 1_000_000_000
        tool._scan_history.append((1.0, 0.8, 1_000_000_000))
        tool._on_ros_clock_jump(None)
        self.assertIsNone(tool._latest_scan)
        self.assertEqual(len(tool._scan_history), 0)

    def test_cancelled_wait_returns_without_scans(self):
        tool = object.__new__(Ros2RobotTool)
        tool._scan_condition = Condition()
        tool._scan_history = []
        cancel_event = Event()
        cancel_event.set()
        result = tool.wait_for_front_range(timeout_sec=10.0, cancel_event=cancel_event)
        self.assertFalse(result["available"])
        self.assertTrue(result["cancelled"])


if __name__ == "__main__":
    unittest.main()
