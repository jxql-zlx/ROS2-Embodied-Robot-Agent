"""Tests for command acceptance and navigation completion handling."""

import unittest
import math
import time

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


class _EvidenceRobot(_Robot):
    def __init__(self):
        super().__init__()
        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.last_navigation_goal_id = "nav-goal-uuid"

    def get_pose(self):
        return dict(self.pose)


class _StaleVision:
    def find_object(self, _name):
        return {"found": False, "objects": [], "stale": True}


class _CenteredVision:
    def find_object(self, _name):
        return {"found": True, "objects": [{"name": "cup", "direction": "center"}]}


class _PayloadVision:
    def __init__(self, payload):
        self.payload = payload

    def find_object(self, _name):
        return self.payload


class TestFlashExecutor(unittest.TestCase):
    def test_navigation_task_reports_final_success(self):
        robot = _Robot(nav_result=True)
        result = FlashExecutor(robot_tool=robot).execute(
            Task(id=1, tool="nav", action="navigate", target="1.0,0.5"), {}
        )
        self.assertTrue(result.success)
        self.assertEqual(result.message, "navigation completed")
        self.assertEqual(robot.calls[0][0:2], ("navigate", "1.0,0.5"))

    def test_navigation_result_includes_stage_pose_and_goal_uuid(self):
        robot = _EvidenceRobot()
        result = FlashExecutor(robot_tool=robot).execute(
            Task(id=1, tool="nav", action="navigate", target="1.0,0.5"), {}
        )
        self.assertTrue(result.success)
        self.assertEqual(result.data["stage_start_pose"], {"x": 0.0, "y": 0.0, "yaw": 0.0})
        self.assertEqual(result.data["stage_end_pose"], {"x": 0.0, "y": 0.0, "yaw": 0.0})
        self.assertEqual(result.data["navigation_goal_id"], "nav-goal-uuid")

    def test_vision_stage_does_not_inherit_previous_navigation_goal_uuid(self):
        robot = _EvidenceRobot()
        result = FlashExecutor(robot_tool=robot, vision_tool=_PayloadVision({
            "found": True,
            "objects": [],
            "stale": True,
        })).execute(Task(id=2, tool="vision", action="find_object", target="cup"), {})
        self.assertNotIn("navigation_goal_id", result.data)

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

    def test_fresh_vision_requires_ros_observation_metadata(self):
        payload = {
            "found": True,
            "objects": [{"name": "cup"}],
            "stale": False,
            "observed_at": time.time(),
            "age_sec": 0.01,
        }
        result = FlashExecutor(vision_tool=_PayloadVision(payload)).execute(
            Task(id=1, tool="vision", action="find_object", target="cup"), {}
        )
        self.assertTrue(result.success)

        wrong_target = FlashExecutor(
            vision_tool=_PayloadVision({**payload, "objects": [{"name": "chair"}]})
        ).execute(Task(id=1, tool="vision", action="find_object", target="cup"), {})
        self.assertFalse(wrong_target.success)
        self.assertEqual(wrong_target.message, "malformed vision result")

        for malformed in (
            {key: value for key, value in payload.items() if key != "stale"},
            {**payload, "observed_at": math.nan},
            {**payload, "observed_at": time.time() + 1.0},
        ):
            with self.subTest(payload=malformed):
                result = FlashExecutor(vision_tool=_PayloadVision(malformed)).execute(
                    Task(id=1, tool="vision", action="find_object", target="cup"), {}
                )
                self.assertFalse(result.success)
                self.assertEqual(result.message, "malformed vision result")

    def test_expired_vision_payload_is_rejected_as_stale(self):
        payload = {
            "found": True,
            "objects": [{"name": "cup"}],
            "stale": False,
            "observed_at": time.time() - 3.0,
            "age_sec": 3.0,
        }
        result = FlashExecutor(vision_tool=_PayloadVision(payload)).execute(
            Task(id=1, tool="vision", action="find_object", target="cup"), {}
        )
        self.assertFalse(result.success)
        self.assertEqual(result.message, "vision result is stale")

    def test_find_object_requires_explicit_target(self):
        result = FlashExecutor(vision_tool=_PayloadVision({})).execute(
            Task(id=1, tool="vision", action="find_object", target=""), {}
        )
        self.assertFalse(result.success)
        self.assertEqual(result.message, "vision target is required")

    def test_legacy_visual_move_cannot_bypass_feedback_requirements(self):
        robot = _Robot()
        result = FlashExecutor(robot, _CenteredVision(), MockLLM()).execute(
            Task(id=1, tool="robot", action="move"),
            {"goal": "find cup", "vision": {"objects": [{"name": "cup"}]}},
        )
        self.assertFalse(result.success)
        self.assertFalse(result.data["recoverable"])
        self.assertEqual(robot.calls, [])


if __name__ == "__main__":
    unittest.main()
