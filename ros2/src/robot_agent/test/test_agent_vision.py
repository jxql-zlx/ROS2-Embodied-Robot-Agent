"""Integration-style test: the agent routes vision/approach actions to tools."""

import unittest

import rclpy

from robot_agent.llm import MockLLM
from robot_agent.action_plan import ActionPlan
from robot_agent.robot_agent_node import RobotAgentNode


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
                {
                    "name": name,
                    "confidence": 0.93,
                    "position": {"x": 320.0, "y": 240.0},
                    "center_x": 320.0,
                    "center_y": 240.0,
                    "direction": self.direction,
                    "distance": None,
                }
            ],
        }

    def get_objects(self):
        self.last_query = ("scene", "")
        return {"found": True, "objects": []}


class _FakeRobotTool:
    def __init__(self, result=True):
        self.actions = []
        self.result = result

    def execute_action(self, action):
        self.actions.append(action)
        return self.result


class _LoopVisionTool:
    def __init__(self, scenes):
        self.scenes = scenes
        self.index = 0
        self.wait_count = 0

    def find_object(self, _name):
        return dict(self.scenes[self.index])

    def wait_for_new_scene(self, after_observed_at, timeout_sec):
        self.wait_count += 1
        if self.index + 1 >= len(self.scenes):
            return None
        self.index += 1
        return dict(self.scenes[self.index])


class _LoopRobotTool(_FakeRobotTool):
    def __init__(self):
        super().__init__()
        self.wait_count = 0

    def status_stamp(self):
        return (0, self.wait_count)

    def wait_for_motion_complete(self, _after_stamp, _timeout_sec):
        self.wait_count += 1
        return True


class TestAgentApproachRouting(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def tearDown(self):
        if getattr(self, "agent", None):
            self.agent.destroy_node()

    def test_approach_center_moves_forward(self):
        vision = _FakeVisionTool(direction="center")
        robot = _FakeRobotTool()
        self.agent = RobotAgentNode(llm=MockLLM(), tool=robot, vision_tool=vision)
        ok, response, action_json = self.agent.handle_task("帮我找杯子")
        self.assertTrue(ok)
        self.assertEqual(vision.last_query, ("find", "cup"))
        self.assertEqual(robot.actions[0]["action"], "move")
        self.assertIn("cup", response)

    def test_approach_no_target_does_not_move(self):
        vision = _FakeVisionTool(found=False)
        robot = _FakeRobotTool()
        self.agent = RobotAgentNode(llm=MockLLM(), tool=robot, vision_tool=vision)
        ok, response, action_json = self.agent.handle_task("帮我找杯子")
        self.assertTrue(ok)
        self.assertEqual(robot.actions, [])
        self.assertIn("没有发现目标", response)

    def test_scene_query_describes_only(self):
        vision = _FakeVisionTool()
        robot = _FakeRobotTool()
        self.agent = RobotAgentNode(llm=MockLLM(), tool=robot, vision_tool=vision)
        ok, response, action_json = self.agent.handle_task("场景中有什么物体？")
        self.assertTrue(ok)
        self.assertEqual(vision.last_query, ("scene", ""))
        self.assertEqual(robot.actions, [])
        self.assertIn("未检测到", response)

    def test_rejected_navigation_is_reported_as_failure(self):
        robot = _FakeRobotTool(result=False)
        self.agent = RobotAgentNode(llm=MockLLM(), tool=robot, vision_tool=_FakeVisionTool())
        ok, response, _ = self.agent.handle_task("导航到 1.0,0.5")
        self.assertFalse(ok)
        self.assertEqual(response, "执行层受理失败")

    def test_approach_uses_new_frame_after_each_completed_motion(self):
        def scene(stamp, direction):
            return {
                "found": True,
                "stale": False,
                "observed_at": stamp,
                "objects": [{"name": "cup", "confidence": 0.9, "direction": direction}],
            }

        vision = _LoopVisionTool([
            scene(1, "left"), scene(2, "center"), scene(3, "center"), scene(4, "center")
        ])
        robot = _LoopRobotTool()
        self.agent = RobotAgentNode(llm=MockLLM(), tool=robot, vision_tool=vision)
        ok, response, _ = self.agent._handle_approach(
            ActionPlan("approach", goal="cup"), "帮我找杯子"
        )
        self.assertFalse(ok)
        self.assertIn("步数上限", response)
        self.assertEqual(vision.wait_count, 3)
        self.assertEqual(robot.wait_count, 3)
        self.assertEqual(robot.actions[0]["action"], "rotate")
        self.assertEqual(robot.actions[1]["action"], "move")
        self.assertLessEqual(robot.actions[1]["duration"], 0.5)

    def test_approach_stops_when_no_new_frame_arrives(self):
        vision = _LoopVisionTool([
            {"found": True, "stale": False, "observed_at": 1, "objects": [
                {"name": "cup", "confidence": 0.9, "direction": "left"}
            ]}
        ])
        robot = _LoopRobotTool()
        self.agent = RobotAgentNode(llm=MockLLM(), tool=robot, vision_tool=vision)
        ok, response, _ = self.agent._handle_approach(
            ActionPlan("approach", goal="cup"), "帮我找杯子"
        )
        self.assertFalse(ok)
        self.assertIn("未收到新视觉帧", response)
        self.assertEqual(len(robot.actions), 1)


if __name__ == "__main__":
    unittest.main()
