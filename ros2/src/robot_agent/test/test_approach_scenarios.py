"""Scenario regressions for the shared visual approach controller."""

import unittest
import threading

from robot_agent.approach import ApproachController
from robot_agent.llm import MockLLM


class Robot:
    def __init__(self, ranges):
        self.ranges = ranges
        self.step = 0
        self.actions = []

    def get_sensor(self, _name):
        value = self.ranges[min(self.step, len(self.ranges) - 1)]
        return {"available": value is not None, "distance": value}

    def wait_for_front_range(self):
        return self.get_sensor("front_range")

    def status_stamp(self):
        return (0, self.step)

    def execute_action(self, command):
        self.actions.append(command)
        return True

    def wait_for_motion_complete(self, *_args):
        self.step += 1
        return True

    def stop(self):
        self.actions.append({"action": "stop"})
        return True


class Vision:
    def __init__(self, directions):
        self.directions = directions
        self.step = 0

    def find_object(self, _target):
        direction = self.directions[min(self.step, len(self.directions) - 1)]
        return {
            "found": direction is not None,
            "objects": [{"name": "cup", "direction": direction}] if direction else [],
            "observed_at": float(self.step + 1),
        }

    def wait_for_new_scene(self, _after, timeout_sec):
        self.step += 1
        if self.step >= len(self.directions):
            return None
        return self.find_object("cup")

    def query_scene(self, scene, _target):
        return scene


class TestApproachScenarios(unittest.TestCase):
    def test_side_targets_rotate_then_advance_and_stop(self):
        for direction, sign in (("left", 1), ("right", -1)):
            with self.subTest(direction=direction):
                robot = Robot([12.0, 1.0, 0.6])
                result = ApproachController(robot, Vision([direction, "center", "center"]), MockLLM()).run("cup", "find cup")
                self.assertTrue(result[0])
                self.assertEqual([a["action"] for a in robot.actions], ["rotate", "move", "stop"])
                self.assertGreater(robot.actions[0]["angular_z"] * sign, 0)

    def test_absent_target_never_starts_motion(self):
        robot = Robot([1.0])
        result = ApproachController(robot, Vision([None]), MockLLM()).run("cup", "find cup")
        self.assertFalse(result[0])
        self.assertEqual(robot.actions, [{"action": "stop"}])

    def test_near_and_far_starting_distances_complete_within_step_budget(self):
        for ranges in ([0.8, 0.6], [1.3, 1.1, 0.9, 0.7, 0.6]):
            with self.subTest(ranges=ranges):
                robot = Robot(ranges)
                result = ApproachController(robot, Vision(["center"] * len(ranges)), MockLLM()).run("cup", "find cup")
                self.assertTrue(result[0])
                self.assertEqual(robot.actions[-1]["action"], "stop")

    def test_close_obstacle_causes_failure_without_forward_motion(self):
        robot = Robot([0.3])
        result = ApproachController(robot, Vision(["center"]), MockLLM()).run("cup", "find cup")
        self.assertFalse(result[0])
        self.assertIn("距离不足", result[1])
        self.assertEqual(robot.actions, [{"action": "stop"}])

    def test_vision_dropout_stops_after_completed_short_step(self):
        robot = Robot([1.0])
        result = ApproachController(robot, Vision(["center"]), MockLLM()).run("cup", "find cup")
        self.assertFalse(result[0])
        self.assertIn("未收到新视觉帧", result[1])
        self.assertEqual([a["action"] for a in robot.actions], ["move", "stop"])

    def test_range_dropout_stops_before_second_motion(self):
        robot = Robot([1.0, None])
        result = ApproachController(robot, Vision(["center", "center"]), MockLLM()).run("cup", "find cup")
        self.assertFalse(result[0])
        self.assertIn("未获得稳定", result[1])
        self.assertEqual([a["action"] for a in robot.actions], ["move", "stop"])

    def test_target_lost_in_new_frame_stops_before_second_motion(self):
        robot = Robot([1.0])
        result = ApproachController(robot, Vision(["center", None]), MockLLM()).run("cup", "find cup")
        self.assertFalse(result[0])
        self.assertIn("无法重新定位", result[1])
        self.assertEqual([a["action"] for a in robot.actions], ["move", "stop"])

    def test_range_expires_during_visual_reobservation(self):
        robot = Robot([1.0])
        vision = Vision(["center", "center"])
        query_scene = vision.query_scene

        def expire_range(scene, target):
            robot.ranges = [None]
            return query_scene(scene, target)

        vision.query_scene = expire_range
        result = ApproachController(robot, vision, MockLLM()).run("cup", "find cup")
        self.assertFalse(result[0])
        self.assertIn("已过期", result[1])
        self.assertEqual([a["action"] for a in robot.actions], ["move", "stop"])

    def test_cancel_during_motion_wait_exits_without_another_move(self):
        cancel_event = threading.Event()
        robot = Robot([1.0])

        def wait_for_cancel(_stamp, _timeout, cancel_event=None):
            cancel_event.wait(2.0)
            return False

        robot.wait_for_motion_complete = wait_for_cancel
        result_box = []
        controller = ApproachController(
            robot, Vision(["center"]), MockLLM(), cancel_event=cancel_event,
        )
        worker = threading.Thread(target=lambda: result_box.append(controller.run("cup", "find cup")))
        worker.start()
        for _ in range(100):
            if robot.actions:
                break
            threading.Event().wait(0.01)
        cancel_event.set()
        robot.stop()
        worker.join(1.0)
        self.assertFalse(worker.is_alive())
        self.assertFalse(result_box[0][0])
        self.assertIn("任务已取消", result_box[0][1])
        self.assertEqual([a["action"] for a in robot.actions], ["move", "stop"])

    def test_temporary_missed_frame_is_rechecked_without_more_motion(self):
        robot = Robot([1.0, 0.6])
        vision = Vision(["center", None, "center"])
        result = ApproachController(robot, vision, MockLLM()).run("cup", "find cup")
        self.assertTrue(result[0])
        self.assertEqual(vision.step, 2)
        self.assertEqual([a["action"] for a in robot.actions], ["move", "stop"])


if __name__ == "__main__":
    unittest.main()
