"""Robot status wait helper tests without a running ROS graph."""

import unittest
from threading import Condition, Event

from robot_interfaces.msg import RobotStatus

from robot_tools.ros2_robot_tool import Ros2RobotTool


class TestMotionStatusWait(unittest.TestCase):
    def setUp(self):
        self.tool = object.__new__(Ros2RobotTool)
        self.tool._status_condition = Condition()
        self.tool._latest_status = None
        self.tool._latest_command_id = "expected"

    @staticmethod
    def _status(action, state, stamp, command_id="expected"):
        msg = RobotStatus()
        msg.action = action
        msg.state = state
        msg.stamp.sec, msg.stamp.nanosec = stamp
        msg.command_id = command_id
        return msg

    def test_wait_requires_new_terminal_motion_status(self):
        self.tool._on_status(self._status("move", "done", (1, 0)))
        self.assertFalse(self.tool.wait_for_motion_complete((1, 0), 0.001))
        self.tool._on_status(self._status("move", "running", (2, 0)))
        self.assertFalse(self.tool.wait_for_motion_complete((1, 0), 0.001))
        self.tool._on_status(self._status("move", "done", (3, 0)))
        self.assertTrue(self.tool.wait_for_motion_complete((1, 0), 0.001))

    def test_error_status_fails_motion_wait(self):
        self.tool._on_status(self._status("rotate", "error", (2, 0)))
        self.assertFalse(self.tool.wait_for_motion_complete((1, 0), 0.001))

    def test_status_from_different_command_does_not_complete_wait(self):
        self.tool._on_status(self._status("move", "done", (2, 0), "older-command"))
        self.assertFalse(self.tool.wait_for_motion_complete((1, 0), 0.001))
        self.tool._on_status(self._status("move", "done", (3, 0), "expected"))
        self.assertTrue(self.tool.wait_for_motion_complete((1, 0), 0.001))

    def test_cancelled_motion_wait_exits_immediately(self):
        cancel_event = Event()
        cancel_event.set()
        self.assertFalse(self.tool.wait_for_motion_complete((1, 0), 10.0, cancel_event))


if __name__ == "__main__":
    unittest.main()
