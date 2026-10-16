"""Robot status wait helper tests without a running ROS graph."""

import unittest
from threading import Condition

from robot_interfaces.msg import RobotStatus

from robot_tools.ros2_robot_tool import Ros2RobotTool


class TestMotionStatusWait(unittest.TestCase):
    def setUp(self):
        self.tool = object.__new__(Ros2RobotTool)
        self.tool._status_condition = Condition()
        self.tool._latest_status = None

    @staticmethod
    def _status(action, state, stamp):
        msg = RobotStatus()
        msg.action = action
        msg.state = state
        msg.stamp.sec, msg.stamp.nanosec = stamp
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


if __name__ == "__main__":
    unittest.main()
