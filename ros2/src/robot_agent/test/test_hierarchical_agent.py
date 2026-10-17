"""Integration tests for the HierarchicalAgent (Pro -> Manager -> Flash)."""

import unittest
import threading

from robot_agent.hierarchical_agent import build_hierarchical_agent
from robot_agent.executor.base_executor import BaseExecutor, TaskResult
from robot_agent.hierarchical_agent import HierarchicalAgent
from robot_agent.planner.base_planner import BasePlanner
from robot_agent.planner.task_schema import Plan, Task


class _FakeRobotTool:
    def __init__(self):
        self.actions = []
        self.range_available = True

    def status_stamp(self):
        return (0, len(self.actions))

    def wait_for_motion_complete(self, *_args):
        return True

    def get_sensor(self, _name):
        moved = any(a.get("action") == "move" for a in self.actions)
        return {"available": self.range_available, "distance": 0.6 if moved else 1.0}

    def execute_action(self, action):
        self.actions.append(action)
        return True

    def move(self, linear_x=0.0, linear_y=0.0, angular_z=0.0, duration=0.0):
        self.actions.append(
            {"action": "move", "linear_x": linear_x, "linear_y": linear_y, "angular_z": angular_z, "duration": duration}
        )
        return True

    def rotate(self, angular_z, duration=0.0):
        self.actions.append({"action": "rotate", "angular_z": angular_z, "duration": duration})
        return True

    def stop(self):
        self.actions.append({"action": "stop"})
        return True


class _FakeVisionTool:
    def __init__(self, found=True, direction="center"):
        self.found = found
        self.direction = direction
        self.last_query = None

    def find_object(self, name):
        self.last_query = ("find", name)
        if not self.found:
            return {"found": False, "objects": []}
        return {
            "found": True,
            "objects": [
                {"name": name, "confidence": 0.9, "center_x": 320.0, "center_y": 240.0, "direction": self.direction}
            ],
        }

    def get_objects(self):
        self.last_query = ("scene", "")
        return {"found": self.found, "objects": []}

    def wait_for_new_scene(self, _after, timeout_sec):
        return self.find_object("cup")

    def query_scene(self, scene, _name):
        return scene

    def get_scene(self):
        return {"found": self.found, "objects": []}


class _FixedPlanner(BasePlanner):
    def __init__(self):
        self.replan_calls = 0

    def plan(self, goal):
        return Plan(
            goal=goal,
            tasks=[
                Task(id=1, tool="robot", action="first"),
                Task(id=2, tool="robot", action="second"),
            ],
        )

    def replan(self, goal, task, result):
        self.replan_calls += 1
        return Plan(goal=goal, tasks=[Task(id=1, tool="robot", action="retry")])


class _FixedExecutor(BaseExecutor):
    def __init__(self, success=True):
        self.success = success

    def execute(self, task, context):
        return TaskResult(task.id, self.success, "done" if self.success else "failed")


class _CancelDuringTaskExecutor(BaseExecutor):
    def __init__(self, cancel_event):
        self.cancel_event = cancel_event
        self.calls = []

    def execute(self, task, context):
        self.calls.append(task.id)
        self.cancel_event.set()
        return TaskResult(task.id, False, "interrupted")


class TestHierarchicalAgent(unittest.TestCase):
    def test_navigation_then_find_and_approach_is_ordered_and_dependent(self):
        robot = _FakeRobotTool()
        vision = _FakeVisionTool()
        events = []
        original_find = vision.find_object

        def find(target):
            events.append("find")
            return original_find(target)

        vision.find_object = find
        agent = build_hierarchical_agent({"provider": "mock"}, robot_tool=robot, vision_tool=vision)
        goal = "导航到 1.0,0.5，然后找杯子并靠近"
        plan = agent._planner.plan(goal)
        self.assertEqual([(task.tool, task.action) for task in plan.tasks], [
            ("nav", "navigate"), ("vision", "find_object"), ("robot", "approach"),
        ])
        self.assertEqual([task.depends_on for task in plan.tasks], [[], [1], [2]])

        results = agent.run(goal, context={
            "_navigate": lambda target, _timeout: events.append(("navigate", target)) or True,
        })
        self.assertTrue(all(result.success for result in results), [r.message for r in results])
        self.assertEqual(events[0], ("navigate", "1.0,0.5"))
        self.assertEqual(events[1], "find")
        self.assertTrue(any(action.get("action") == "move" for action in robot.actions))

    def test_navigation_failure_prevents_following_find_and_approach(self):
        robot = _FakeRobotTool()
        vision = _FakeVisionTool()
        agent = build_hierarchical_agent({"provider": "mock"}, robot_tool=robot, vision_tool=vision)
        results = agent.run(
            "导航到 1.0,0.5，然后找杯子并靠近",
            context={"_navigate": lambda *_args: False},
        )
        self.assertFalse(all(result.success for result in results))
        self.assertIsNone(vision.last_query)
        self.assertEqual(robot.actions, [])

    def test_missing_target_after_navigation_does_not_approach(self):
        robot = _FakeRobotTool()
        vision = _FakeVisionTool(found=False)
        agent = build_hierarchical_agent({"provider": "mock"}, robot_tool=robot, vision_tool=vision)
        results = agent.run(
            "导航到 1.0,0.5，然后找杯子并靠近",
            context={"_navigate": lambda *_args: True},
        )
        self.assertFalse(all(result.success for result in results))
        self.assertEqual(vision.last_query, ("find", "cup"))
        self.assertEqual(robot.actions, [])

    def test_malformed_vision_result_blocks_approach_and_recovery_motion(self):
        class MalformedVision(_FakeVisionTool):
            def find_object(self, _name):
                return {"found": True, "objects": "not-a-list"}

        robot = _FakeRobotTool()
        agent = build_hierarchical_agent(
            {"provider": "mock"}, robot_tool=robot, vision_tool=MalformedVision()
        )

        results = agent.run("找到杯子然后移动过去")

        self.assertTrue(any(not result.success for result in results))
        self.assertIn("malformed vision result", " ".join(r.message for r in results))
        self.assertEqual(robot.actions, [])

    def test_vision_exception_does_not_trigger_recovery_motion(self):
        class RaisingVision(_FakeVisionTool):
            def find_object(self, _name):
                raise RuntimeError("camera stream unavailable")

        robot = _FakeRobotTool()
        agent = build_hierarchical_agent(
            {"provider": "mock"}, robot_tool=robot, vision_tool=RaisingVision()
        )

        results = agent.run("找到杯子然后移动过去")

        self.assertTrue(any(not result.success for result in results))
        self.assertIn("camera stream unavailable", " ".join(r.message for r in results))
        self.assertEqual(robot.actions, [])

    def test_cancellation_after_navigation_prevents_following_find(self):
        cancel_event = threading.Event()
        robot = _FakeRobotTool()
        vision = _FakeVisionTool()
        agent = build_hierarchical_agent({"provider": "mock"}, robot_tool=robot, vision_tool=vision)

        def navigate(*_args):
            cancel_event.set()
            return True

        results = agent.run(
            "导航到 1.0,0.5，然后找杯子并靠近",
            context={"_navigate": navigate}, cancel_event=cancel_event,
        )
        self.assertTrue(results[-1].data["cancelled"])
        self.assertIsNone(vision.last_query)
        self.assertEqual(robot.actions, [])

    def test_hierarchical_follow_up_navigation_reports_nav2_completion(self):
        robot = _FakeRobotTool()
        navigation_calls = []

        def execute_action(action):
            if action.get("action") == "navigate":
                navigation_calls.append(action["goal"])
                return True  # The node callback returns only after Nav2 succeeds.
            return robot.execute_action(action)

        agent = build_hierarchical_agent({"provider": "mock"}, robot_tool=robot)
        results = agent.run(
            "Navigate to 1.0,0.5",
            context={"_execute_action": execute_action},
        )

        self.assertEqual(navigation_calls, ["1.0,0.5"])
        self.assertEqual(len(results), 1)
        self.assertTrue(results[0].success)
        self.assertEqual(results[0].message, "navigation completed")

    def test_hierarchical_follow_up_navigation_failure_is_not_success(self):
        agent = build_hierarchical_agent({"provider": "mock"}, robot_tool=_FakeRobotTool())
        results = agent.run(
            "Navigate to 1.0,0.5",
            context={"_execute_action": lambda _action: False},
        )

        self.assertTrue(results)
        self.assertFalse(results[0].success)
        self.assertIn("rejected", results[0].message)
        self.assertFalse(all(result.success for result in results))

    def test_cancelled_task_does_not_replan_or_start_dependent_tasks(self):
        cancel_event = threading.Event()
        planner = _FixedPlanner()
        executor = _CancelDuringTaskExecutor(cancel_event)
        agent = HierarchicalAgent(planner, executor)

        results = agent.run("查找目标然后移动", cancel_event=cancel_event)

        self.assertEqual(executor.calls, [1])
        self.assertEqual(planner.replan_calls, 0)
        self.assertTrue(results[-1].data["cancelled"])

    def test_complex_find_then_move(self):
        vision = _FakeVisionTool(found=True, direction="center")
        robot = _FakeRobotTool()
        agent = build_hierarchical_agent({"provider": "mock"}, robot_tool=robot, vision_tool=vision)
        results = agent.run("找到杯子然后移动过去")
        self.assertTrue(results)
        self.assertEqual(vision.last_query, ("find", "cup"))
        self.assertTrue(any(a.get("action") == "move" for a in robot.actions))
        self.assertTrue(all(r.success for r in results))

    def test_simple_stop_goes_straight_to_flash(self):
        robot = _FakeRobotTool()
        agent = build_hierarchical_agent({"provider": "mock"}, robot_tool=robot, vision_tool=None)
        results = agent.run("停止机器人")
        self.assertEqual(len(results), 1)
        self.assertTrue(results[0].success)
        self.assertEqual(robot.actions[0].get("action"), "stop")

    def test_find_cup_alone_uses_closed_loop_approach(self):
        robot = _FakeRobotTool()
        agent = build_hierarchical_agent(robot_tool=robot, vision_tool=_FakeVisionTool())
        results = agent.run("找杯子")
        self.assertTrue(all(r.success for r in results))
        self.assertEqual(results[-1].data["approach"]["distance"], 0.6)
        self.assertEqual([a["action"] for a in robot.actions], ["move", "stop"])

    def test_range_failure_does_not_trigger_blind_search_rotation(self):
        robot = _FakeRobotTool()
        robot.range_available = False
        agent = build_hierarchical_agent(robot_tool=robot, vision_tool=_FakeVisionTool())
        results = agent.run("找到杯子然后移动过去")
        self.assertTrue(any(not r.success for r in results))
        self.assertEqual([a["action"] for a in robot.actions], ["stop"])

    def test_absent_target_does_not_trigger_unrequested_search_motion(self):
        vision = _FakeVisionTool(found=False)
        robot = _FakeRobotTool()
        agent = build_hierarchical_agent({"provider": "mock"}, robot_tool=robot, vision_tool=vision)
        results = agent.run("找到杯子然后移动过去")
        self.assertTrue(any(not r.success for r in results))
        self.assertEqual(robot.actions, [])

    def test_step_limit_reports_remaining_tasks_as_failure(self):
        agent = HierarchicalAgent(_FixedPlanner(), _FixedExecutor(), max_steps=1)
        results = agent.run("navigate multi-step goal")
        self.assertFalse(results[-1].success)
        self.assertIn("max steps reached", results[-1].message)
        self.assertTrue(results[-1].data["pending_task_ids"])

    def test_recovery_limit_stops_replanning(self):
        planner = _FixedPlanner()
        agent = HierarchicalAgent(
            planner, _FixedExecutor(success=False), max_steps=10, max_recoveries=1
        )
        results = agent.run("navigate multi-step goal")
        self.assertEqual(planner.replan_calls, 1)
        self.assertTrue(any("recovery limit reached" in result.message for result in results))


if __name__ == "__main__":
    unittest.main()
