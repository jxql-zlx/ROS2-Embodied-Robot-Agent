"""Robot status wait helper tests without a running ROS graph."""

import unittest
from collections import OrderedDict
from threading import Condition, Event, Thread
from unittest.mock import patch

from robot_interfaces.msg import RobotStatus

from robot_tools.ros2_robot_tool import Ros2RobotTool


class TestMotionStatusWait(unittest.TestCase):
    def setUp(self):
        self.tool = object.__new__(Ros2RobotTool)
        self.tool._status_condition = Condition()
        self.tool._scan_condition = Condition()
        self.tool._scan_history = []
        self.tool._latest_scan = None
        self.tool._latest_scan_stamp_ns = None
        self.tool._last_scan_stamp_ns = None
        self.tool._last_scan_ros_now_ns = None
        self.tool._scan_received_at = 0.0
        self.tool._latest_status = None
        self.tool._latest_command_id = "expected"
        self.tool._status_history = OrderedDict()
        self.tool._status_clock_epoch = 0
        self.tool._last_status_ros_now_ns = None
        self.clock = {"now": 4_000_000_000}
        self.tool._ros_time_ns = lambda: self.clock["now"]

    @staticmethod
    def _status(action, state, stamp, command_id="expected"):
        msg = RobotStatus()
        msg.action = action
        msg.state = state
        msg.stamp.sec, msg.stamp.nanosec = stamp
        msg.command_id = command_id
        return msg

    def test_wait_requires_new_terminal_motion_status(self):
        self.tool._on_status(self._status("move", "done", (1, 0), "previous"))
        self.assertFalse(self.tool.wait_for_motion_complete((1, 0), 0.001, command_id="previous"))
        self.tool._on_status(self._status("move", "running", (2, 0)))
        self.assertFalse(self.tool.wait_for_motion_complete((1, 0), 0.001, command_id="expected"))
        self.tool._on_status(self._status("move", "done", (3, 0)))
        self.assertTrue(self.tool.wait_for_motion_complete((1, 0), 0.001, command_id="expected"))

    def test_error_status_fails_motion_wait(self):
        self.tool._on_status(self._status("rotate", "error", (2, 0)))
        self.assertFalse(self.tool.wait_for_motion_complete((1, 0), 0.001, command_id="expected"))

    def test_status_from_different_command_does_not_complete_wait(self):
        self.tool._on_status(self._status("move", "done", (2, 0), "older-command"))
        self.assertFalse(self.tool.wait_for_motion_complete((1, 0), 0.001, command_id="expected"))
        self.tool._on_status(self._status("move", "done", (3, 0), "expected"))
        self.assertTrue(self.tool.wait_for_motion_complete((1, 0), 0.001, command_id="expected"))

    def test_cancelled_motion_wait_exits_immediately(self):
        cancel_event = Event()
        cancel_event.set()
        self.assertFalse(self.tool.wait_for_motion_complete((1, 0), 10.0, cancel_event))

    def test_missing_command_id_fails_closed(self):
        self.tool._on_status(self._status("move", "done", (2, 0), ""))
        self.assertFalse(self.tool.wait_for_motion_complete((1, 0), 0.001))
        self.assertFalse(self.tool.wait_for_stop_confirmation((1, 0), 0.001))

    def test_completion_survives_interleaved_command_statuses(self):
        self.tool._on_status(self._status("move", "done", (3, 0), "expected"))
        self.tool._on_status(self._status("move", "running", (4, 0), "other"))
        self.assertTrue(self.tool.wait_for_motion_complete((1, 0), 0.001, command_id="expected"))

    def test_terminal_status_is_not_overwritten_by_late_status(self):
        self.tool._on_status(self._status("move", "done", (3, 0), "expected"))
        self.tool._on_status(self._status("move", "error", (4, 0), "expected"))
        self.assertTrue(self.tool.wait_for_motion_complete((1, 0), 0.001, command_id="expected"))

    def test_terminal_status_is_retained_before_wait_starts(self):
        self.tool._on_status(self._status("stop", "stopped", (3, 0), "expected"))
        self.assertTrue(self.tool.wait_for_stop_confirmation((1, 0), 0.001, command_id="expected"))

    def test_status_history_is_bounded(self):
        for index in range(self.tool.STATUS_HISTORY_LIMIT + 5):
            self.tool._on_status(self._status("move", "done", (index + 1, 0), f"id-{index}"))
        self.assertLessEqual(len(self.tool._status_history), self.tool.STATUS_HISTORY_LIMIT)

    def test_status_stamp_clears_old_epoch_and_new_stop_can_confirm(self):
        self.clock["now"] = 100_000_000_000
        self.tool._on_status(self._status("move", "done", (100, 0), "old"))
        self.assertEqual(self.tool.status_stamp(), (100, 0))
        self.clock["now"] = 5_000_000_000
        self.assertIsNone(self.tool.status_stamp())
        self.assertIsNone(self.tool._latest_status)

        self.tool._on_status(self._status("stop", "stopped", (5, 0), "new-stop"))
        self.assertTrue(self.tool.wait_for_stop_confirmation(
            None, 0.01, command_id="new-stop"
        ))

    def test_wait_started_before_clock_reset_fails_even_if_matching_id_arrives(self):
        class SignalingCondition(Condition):
            def __init__(self):
                super().__init__()
                self.waiting = Event()

            def wait(self, timeout=None):
                self.waiting.set()
                return super().wait(timeout)

        condition = SignalingCondition()
        self.tool._status_condition = condition
        self.clock["now"] = 100_000_000_000
        result = []
        worker = Thread(target=lambda: result.append(
            self.tool.wait_for_stop_confirmation(None, 5.0, command_id="same-id")
        ))
        worker.start()
        self.assertTrue(condition.waiting.wait(1.0), "waiter did not enter its condition wait")

        self.clock["now"] = 5_000_000_000
        self.tool._on_status(self._status("stop", "stopped", (5, 0), "same-id"))
        worker.join(timeout=1.0)
        self.assertFalse(worker.is_alive())
        self.assertEqual(result, [False])

    def test_status_history_ttl_expires_old_receipt(self):
        self.clock["now"] = 100_000_000_000
        with patch("robot_tools.ros2_robot_tool.time.monotonic", return_value=0.0):
            self.tool._on_status(self._status("stop", "stopped", (100, 0), "expired"))
        with patch("robot_tools.ros2_robot_tool.time.monotonic",
                   side_effect=[61.0, 61.0, 61.002]):
            self.assertFalse(self.tool.wait_for_stop_confirmation(
                None, 0.001, command_id="expired"
            ))

    def test_forward_ros_clock_jump_invalidates_status_cache(self):
        self.tool._on_status(self._status("stop", "stopped", (4, 0), "old-stop"))
        self.tool._on_ros_clock_jump(None)
        self.assertEqual(len(self.tool._status_history), 0)
        self.assertIsNone(self.tool._latest_status)
        self.assertFalse(self.tool.wait_for_stop_confirmation(
            None, 0.001, command_id="old-stop"
        ))

    def test_old_source_status_after_forward_jump_cannot_confirm(self):
        self.clock["now"] = 40_000_000_000
        self.tool._on_status(self._status("stop", "stopped", (4, 0), "late-old"))
        self.assertNotIn("late-old", self.tool._status_history)
        self.assertFalse(self.tool.wait_for_stop_confirmation(
            None, 0.001, command_id="late-old"
        ))

    def test_paused_ros_clock_does_not_keep_old_receipt_fresh(self):
        with patch("robot_tools.ros2_robot_tool.time.monotonic", return_value=0.0):
            self.tool._on_status(self._status("stop", "stopped", (4, 0), "paused-old"))
        with patch("robot_tools.ros2_robot_tool.time.monotonic", side_effect=[6.0, 6.0]):
            self.assertFalse(self.tool.wait_for_stop_confirmation(
                None, 0.0, command_id="paused-old"
            ))

    def test_missing_source_stamp_fails_closed(self):
        status = self._status("stop", "stopped", (0, 0), "unstamped")
        self.tool._on_status(status)
        self.assertNotIn("unstamped", self.tool._status_history)
        self.assertFalse(self.tool.wait_for_stop_confirmation(
            None, 0.001, command_id="unstamped"
        ))


if __name__ == "__main__":
    unittest.main()
