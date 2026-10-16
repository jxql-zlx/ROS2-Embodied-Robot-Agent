"""Navigation action result handling tests using fake action futures."""

import unittest
from types import SimpleNamespace
from unittest.mock import patch

from action_msgs.msg import GoalStatus

from robot_tools.ros2_robot_tool import Ros2RobotTool


class _Future:
    def __init__(self, value=None, invoke_callback=True):
        self.value = value
        self.done = invoke_callback
        self.callbacks = []

    def result(self):
        return self.value

    def add_done_callback(self, callback):
        if self.done:
            callback(self)
        else:
            self.callbacks.append(callback)

    def complete(self, value=None):
        self.value = value
        self.done = True
        callbacks, self.callbacks = self.callbacks, []
        for callback in callbacks:
            callback(self)


class _GoalHandle:
    def __init__(self, accepted=True, status=GoalStatus.STATUS_SUCCEEDED, complete_result=True):
        self.accepted = accepted
        self.status = status
        self.complete_result = complete_result
        self.cancelled = False
        self.result_future = _Future(SimpleNamespace(status=status), complete_result)

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancelled = True
        return _Future(SimpleNamespace(), True)


class _ActionClient:
    def __init__(self, goal_handle=None, invoke_callback=True):
        self.goal_handle = goal_handle
        self.invoke_callback = invoke_callback
        self.send_future = None

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal):
        self.send_future = _Future(self.goal_handle, self.invoke_callback)
        return self.send_future


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
        handle.result_future.done = False
        navigation_states = []
        self.assertFalse(tool.navigate(
            "1,2", timeout=0.001, cancel_timeout=0.1,
            navigation_state_callback=navigation_states.append,
        ))
        self.assertTrue(handle.cancelled)
        self.assertEqual(navigation_states, [True])

    @patch("robot_tools.ros2_robot_tool.NavigateToPose")
    @patch.object(Ros2RobotTool, "_parse_nav_goal", return_value=object())
    @patch.object(Ros2RobotTool, "get_logger", return_value=SimpleNamespace(
        warn=lambda *_: None, info=lambda *_: None, error=lambda *_: None
    ))
    def test_cancel_waits_for_terminal_result_before_releasing_navigation(self, *_mocks):
        import threading
        import time

        handle = _GoalHandle(complete_result=False)
        tool = self._tool(handle)
        cancel_event = threading.Event()
        navigation_states = []
        result = []
        worker = threading.Thread(target=lambda: result.append(tool.navigate(
            "1,2", timeout=5.0, cancel_timeout=1.0, cancel_event=cancel_event,
            navigation_state_callback=navigation_states.append,
        )))
        worker.start()
        deadline = time.monotonic() + 1.0
        while not handle.result_future.callbacks and time.monotonic() < deadline:
            time.sleep(0.01)
        cancel_event.set()
        time.sleep(0.1)
        self.assertTrue(worker.is_alive(), "navigate returned before Nav2 reached a terminal state")
        self.assertEqual(navigation_states, [True])
        handle.result_future.complete(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
        worker.join(timeout=1.0)
        self.assertFalse(worker.is_alive())
        self.assertEqual(result, [False])
        self.assertTrue(handle.cancelled)
        self.assertEqual(navigation_states, [True, False])

    @patch("robot_tools.ros2_robot_tool.NavigateToPose")
    @patch.object(Ros2RobotTool, "_parse_nav_goal", return_value=object())
    @patch.object(Ros2RobotTool, "get_logger", return_value=SimpleNamespace(
        warn=lambda *_: None, info=lambda *_: None, error=lambda *_: None
    ))
    def test_late_accepted_goal_is_cancelled_before_result_returns(self, *_mocks):
        import threading
        import time

        handle = _GoalHandle(complete_result=False)
        tool = self._tool(handle, invoke_callback=False)
        cancel_event = threading.Event()
        navigation_states = []
        result = []
        worker = threading.Thread(target=lambda: result.append(tool.navigate(
            "1,2", timeout=5.0, cancel_timeout=1.0, cancel_event=cancel_event,
            navigation_state_callback=navigation_states.append,
        )))
        worker.start()
        deadline = time.monotonic() + 1.0
        while tool._nav_client.send_future is None and time.monotonic() < deadline:
            time.sleep(0.01)
        goal_future = tool._nav_client.send_future
        self.assertIsNotNone(goal_future)
        cancel_event.set()
        time.sleep(0.1)
        self.assertTrue(worker.is_alive())
        goal_future.complete(handle)
        deadline = time.monotonic() + 1.0
        while not handle.result_future.callbacks and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(handle.cancelled)
        handle.result_future.complete(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
        worker.join(timeout=1.0)
        self.assertFalse(worker.is_alive())
        self.assertEqual(result, [False])
        self.assertEqual(navigation_states, [True, False])

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
