"""Navigation action result handling tests using fake action futures."""

import unittest
from types import SimpleNamespace
from unittest.mock import patch

from action_msgs.msg import GoalStatus

from robot_tools.ros2_robot_tool import Ros2RobotTool


class _Future:
    def __init__(self, value=None, invoke_callback=True):
        self.value = value
        self.invoke_callback = invoke_callback

    def result(self):
        return self.value

    def add_done_callback(self, callback):
        if self.invoke_callback:
            callback(self)


class _GoalHandle:
    def __init__(self, accepted=True, status=GoalStatus.STATUS_SUCCEEDED, complete_result=True):
        self.accepted = accepted
        self.status = status
        self.complete_result = complete_result
        self.cancelled = False

    def get_result_async(self):
        return _Future(SimpleNamespace(status=self.status), self.complete_result)

    def cancel_goal_async(self):
        self.cancelled = True


class _ActionClient:
    def __init__(self, goal_handle=None, invoke_callback=True):
        self.goal_handle = goal_handle
        self.invoke_callback = invoke_callback

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal):
        return _Future(self.goal_handle, self.invoke_callback)


class TestNavigationResult(unittest.TestCase):
    def _tool(self, handle=None, invoke_callback=True):
        tool = object.__new__(Ros2RobotTool)
        tool._nav_client = _ActionClient(handle, invoke_callback)
        return tool

    @patch("robot_tools.ros2_robot_tool.NavigateToPose")
    @patch.object(Ros2RobotTool, "_parse_nav_goal", return_value=object())
    @patch.object(Ros2RobotTool, "get_logger", return_value=SimpleNamespace(
        warn=lambda *_: None, info=lambda *_: None, error=lambda *_: None
    ))
    def test_returns_true_only_when_nav2_succeeds(self, *_mocks):
        tool = self._tool(_GoalHandle(status=GoalStatus.STATUS_SUCCEEDED))
        self.assertTrue(tool.navigate("1,2", timeout=0.1))

    @patch("robot_tools.ros2_robot_tool.NavigateToPose")
    @patch.object(Ros2RobotTool, "_parse_nav_goal", return_value=object())
    @patch.object(Ros2RobotTool, "get_logger", return_value=SimpleNamespace(
        warn=lambda *_: None, info=lambda *_: None, error=lambda *_: None
    ))
    def test_returns_false_when_nav2_aborts(self, *_mocks):
        tool = self._tool(_GoalHandle(status=GoalStatus.STATUS_ABORTED))
        self.assertFalse(tool.navigate("1,2", timeout=0.1))

    @patch("robot_tools.ros2_robot_tool.NavigateToPose")
    @patch.object(Ros2RobotTool, "_parse_nav_goal", return_value=object())
    @patch.object(Ros2RobotTool, "get_logger", return_value=SimpleNamespace(
        warn=lambda *_: None, info=lambda *_: None, error=lambda *_: None
    ))
    def test_timeout_returns_false_and_cancels_goal(self, *_mocks):
        handle = _GoalHandle()
        tool = self._tool(handle)
        handle.complete_result = False
        self.assertFalse(tool.navigate("1,2", timeout=0.001))
        self.assertTrue(handle.cancelled)

    @patch("robot_tools.ros2_robot_tool.NavigateToPose")
    @patch.object(Ros2RobotTool, "_parse_nav_goal", return_value=object())
    @patch.object(Ros2RobotTool, "get_logger", return_value=SimpleNamespace(
        warn=lambda *_: None, info=lambda *_: None, error=lambda *_: None
    ))
    def test_rejected_goal_returns_false(self, *_mocks):
        tool = self._tool(_GoalHandle(accepted=False))
        self.assertFalse(tool.navigate("1,2", timeout=0.1))


if __name__ == "__main__":
    unittest.main()
