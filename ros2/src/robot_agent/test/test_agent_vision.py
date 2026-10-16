"""Integration-style test: the agent routes vision/approach actions to tools."""

import unittest

import rclpy

from robot_agent.llm import MockLLM
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


if __name__ == "__main__":
    unittest.main()
