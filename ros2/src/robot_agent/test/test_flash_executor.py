"""Tests for command acceptance and navigation completion handling."""

import unittest

from robot_agent.executor.flash_executor import FlashExecutor
from robot_agent.llm import MockLLM
from robot_agent.planner.task_schema import Task


class _Robot:
    def __init__(self, nav_result=True, action_result=True):
        self.nav_result = nav_result
        self.action_result = action_result
        self.calls = []

    def navigate(self, goal, **kwargs):
        self.calls.append(("navigate", goal, kwargs))
        return self.nav_result

    def execute_action(self, action):
        self.calls.append(("execute_action", action))
        return self.action_result


class _StaleVision:
    def find_object(self, _name):
        return {"found": False, "objects": [], "stale": True}


class TestFlashExecutor(unittest.TestCase):
    def test_navigation_task_reports_final_success(self):
        robot = _Robot(nav_result=True)
        result = FlashExecutor(robot_tool=robot).execute(
            Task(id=1, tool="nav", action="navigate", target="1.0,0.5"), {}
        )
        self.assertTrue(result.success)
        self.assertEqual(result.message, "navigation completed")
        self.assertEqual(robot.calls[0][0:2], ("navigate", "1.0,0.5"))

    def test_navigation_failure_is_not_reported_as_success(self):
        robot = _Robot(nav_result=False)
        result = FlashExecutor(robot_tool=robot).execute(
            Task(id=1, tool="robot", action="navigate", target="1.0,0.5"), {}
        )
        self.assertFalse(result.success)
        self.assertEqual(result.message, "navigation failed")

    def test_inferred_command_rejection_is_reported(self):
        robot = _Robot(action_result=False)
        result = FlashExecutor(robot_tool=robot, llm=MockLLM()).execute(
            Task(id=1, tool="robot", action="infer", target="导航到 1.0,0.5"), {}
        )
        self.assertFalse(result.success)
        self.assertIn("rejected", result.message)

    def test_stale_vision_does_not_count_as_a_valid_observation(self):
        result = FlashExecutor(vision_tool=_StaleVision()).execute(
            Task(id=1, tool="vision", action="find_object", target="cup"), {}
        )
        self.assertFalse(result.success)
        self.assertEqual(result.message, "vision result is stale")


if __name__ == "__main__":
    unittest.main()
