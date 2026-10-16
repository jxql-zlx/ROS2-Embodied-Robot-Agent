"""Integration-style test: the agent routes vision/approach actions to tools."""

import unittest
from unittest.mock import patch

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
        self.cutoffs = []
        self.queried_stamps = []

    def find_object(self, _name):
        return dict(self.scenes[self.index])

    def wait_for_new_scene(self, after_observed_at, timeout_sec):
        self.wait_count += 1
        self.cutoffs.append(after_observed_at)
        if self.index + 1 >= len(self.scenes):
            return None
        self.index += 1
        return dict(self.scenes[self.index])

    def query_scene(self, scene, _name):
        self.queried_stamps.append(scene["observed_at"])
        return dict(scene)


class _LoopRobotTool(_FakeRobotTool):
    def __init__(self, distances=None):
        super().__init__()
        self.wait_count = 0
        self.distances = distances or [10.0]

    def get_sensor(self, _name):
        distance = self.distances[min(self.wait_count, len(self.distances) - 1)]
        return {"available": distance is not None, "distance": distance}

    def stop(self):
        self.actions.append({"action": "stop"})
        return True

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
        vision = _LoopVisionTool([
            {"found": True, "observed_at": stamp, "objects": [
                {"name": "cup", "confidence": 0.9, "direction": "center"}
            ]} for stamp in (1, 2)
        ])
        robot = _LoopRobotTool(distances=[1.0, 0.6])
        self.agent = RobotAgentNode(llm=MockLLM(), tool=robot, vision_tool=vision)
        ok, response, action_json = self.agent.handle_task("帮我找杯子")
        self.assertTrue(ok)
        self.assertEqual(robot.actions[0]["action"], "move")
        self.assertIn("停止距离", response)

    def test_approach_no_target_does_not_move(self):
        vision = _FakeVisionTool(found=False)
        robot = _FakeRobotTool()
        self.agent = RobotAgentNode(llm=MockLLM(), tool=robot, vision_tool=vision)
        ok, response, action_json = self.agent.handle_task("帮我找杯子")
        self.assertFalse(ok)
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

    @patch("robot_agent.approach.time.time", return_value=100.0)
    def test_approach_uses_new_frame_after_each_completed_motion(self, _clock):
        def scene(stamp, direction):
            return {
                "found": True,
                "stale": False,
                "observed_at": stamp,
                "objects": [{"name": "cup", "confidence": 0.9, "direction": direction}],
            }

        vision = _LoopVisionTool([
            scene(1, "left"), scene(101, "center"), scene(102, "center"), scene(103, "center")
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
        self.assertEqual(vision.cutoffs, [100.0, 100.0, 100.0])
        self.assertEqual(vision.queried_stamps, [101, 102, 103])
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

    def test_approach_succeeds_at_measured_standoff_distance(self):
        scenes = [
            {"found": True, "observed_at": stamp, "objects": [
                {"name": "cup", "direction": "center"}
            ]} for stamp in (1, 2)
        ]
        robot = _LoopRobotTool(distances=[1.0, 0.6])
        self.agent = RobotAgentNode(llm=MockLLM(), tool=robot, vision_tool=_LoopVisionTool(scenes))
        ok, response, _ = self.agent._handle_approach(ActionPlan("approach", goal="cup"), "找杯子")
        self.assertTrue(ok)
        self.assertIn("停止距离", response)
        self.assertEqual([a["action"] for a in robot.actions], ["move", "stop"])

    def test_approach_does_not_move_without_range_feedback(self):
        robot = _LoopRobotTool(distances=[None])
        vision = _LoopVisionTool([{"found": True, "objects": [{"direction": "center"}]}])
        self.agent = RobotAgentNode(llm=MockLLM(), tool=robot, vision_tool=vision)
        ok, response, _ = self.agent._handle_approach(ActionPlan("approach", goal="cup"), "找杯子")
        self.assertFalse(ok)
        self.assertIn("测距不可用", response)
        self.assertEqual(robot.actions, [])

    def test_near_blind_zone_return_is_not_reported_as_arrival(self):
        robot = _LoopRobotTool(distances=[0.12])
        vision = _LoopVisionTool([{"found": True, "objects": [{"direction": "center"}]}])
        self.agent = RobotAgentNode(llm=MockLLM(), tool=robot, vision_tool=vision)
        ok, response, _ = self.agent._handle_approach(ActionPlan("approach", goal="cup"), "找杯子")
        self.assertFalse(ok)
        self.assertIn("距离不足", response)
        self.assertEqual([a["action"] for a in robot.actions], ["stop"])

    def test_final_motion_step_is_shortened_to_remaining_clearance(self):
        scenes = [{"found": True, "observed_at": stamp, "objects": [
            {"name": "cup", "direction": "center"}
        ]} for stamp in (1, 2)]
        robot = _LoopRobotTool(distances=[0.7, 0.62])
        self.agent = RobotAgentNode(llm=MockLLM(), tool=robot, vision_tool=_LoopVisionTool(scenes))
        ok, _, _ = self.agent._handle_approach(ActionPlan("approach", goal="cup"), "找杯子")
        self.assertTrue(ok)
        self.assertAlmostEqual(robot.actions[0]["duration"], (0.7 - 0.65) / 0.3)


if __name__ == "__main__":
    unittest.main()
