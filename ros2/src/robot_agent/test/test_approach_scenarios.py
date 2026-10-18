"""Scenario regressions for the shared visual approach controller."""

import unittest
import threading
import time
import json
from types import SimpleNamespace

from robot_agent.action_plan import ActionPlan
from robot_agent.approach import ApproachController, approach_safe_stop_distance
from robot_agent.llm import MockLLM


class Robot:
    def __init__(self, ranges, stop_result=True):
        self.ranges = ranges
        self.step = 0
        self.actions = []
        self.stop_result = stop_result
        self.latest_command_id = None

    def get_sensor(self, _name):
        value = self.ranges[min(self.step, len(self.ranges) - 1)]
        return {"available": value is not None, "distance": value}

    def wait_for_front_range(self):
        return self.get_sensor("front_range")

    def status_stamp(self):
        return (0, self.step)

    def execute_action(self, command):
        self.actions.append(command)
        self.latest_command_id = f"motion-{len(self.actions)}"
        return True

    def wait_for_motion_complete(self, *_args, **_kwargs):
        self.step += 1
        return True

    def stop(self):
        self.actions.append({"action": "stop"})
        self.latest_command_id = f"stop-{len(self.actions)}"
        return self.stop_result

    def wait_for_stop_confirmation(self, *_args, **_kwargs):
        return True


class Vision:
    def __init__(self, directions, target_name="cup", max_age_sec=2.5):
        self.directions = directions
        self.step = 0
        self.target_name = target_name
        self.max_age_sec = max_age_sec

    def find_object(self, _target):
        direction = self.directions[min(self.step, len(self.directions) - 1)]
        return {
            "found": direction is not None,
            "objects": [{"name": self.target_name, "direction": direction}] if direction else [],
            "observed_at": time.time(),
            "age_sec": 0.0,
            "stale": False,
        }

    def wait_for_new_scene(self, _after, timeout_sec):
        self.step += 1
        if self.step >= len(self.directions):
            return None
        return self.find_object("cup")

    def query_scene(self, scene, _target):
        return scene

    def wait_for_stop_confirmation(self, *_args, **_kwargs):
        return self.stop_result


class TestApproachScenarios(unittest.TestCase):
    def test_empty_or_mismatched_target_never_moves(self):
        empty_robot = Robot([1.0])
        empty = ApproachController(empty_robot, Vision(["center"]), MockLLM()).run("", "approach")
        self.assertFalse(empty[0])
        self.assertEqual([a["action"] for a in empty_robot.actions], ["stop"])

        mismatch_robot = Robot([1.0])
        mismatch = ApproachController(
            mismatch_robot, Vision(["center"], target_name="chair"), MockLLM()
        ).run("cup", "approach cup")
        self.assertFalse(mismatch[0])
        self.assertEqual([a["action"] for a in mismatch_robot.actions], ["stop"])

    def test_failed_stop_is_not_reported_as_confirmed(self):
        robot = Robot([0.3], stop_result=False)
        result = ApproachController(robot, Vision(["center"]), MockLLM()).run("cup", "find cup")
        self.assertFalse(result[0])
        self.assertIn("停止状态未确认", result[1])

    def test_observation_expiring_during_planning_prevents_motion(self):
        class SlowLLM(MockLLM):
            def plan_motion(self, task, vision):
                time.sleep(0.02)
                return super().plan_motion(task, vision)

        robot = Robot([2.0])
        vision = Vision(["center"], max_age_sec=0.001)
        result = ApproachController(robot, vision, SlowLLM()).run("cup", "find cup")
        self.assertFalse(result[0])
        self.assertIn("规划完成后已过期", result[1])
        self.assertEqual([a["action"] for a in robot.actions], ["stop"])
    def test_side_targets_rotate_then_advance_and_stop(self):
        for direction, sign in (("left", 1), ("right", -1)):
            with self.subTest(direction=direction):
                robot = Robot([12.0, 1.0, 0.6])
                result = ApproachController(robot, Vision([direction, "center", "center"]), MockLLM()).run("cup", "find cup")
                self.assertFalse(result[0])
                self.assertIn("目标到达未确认", result[1])
                self.assertEqual([a["action"] for a in robot.actions], ["rotate", "move", "stop"])
                self.assertGreater(robot.actions[0]["angular_z"] * sign, 0)

    def test_absent_target_never_starts_motion(self):
        robot = Robot([1.0])
        result = ApproachController(robot, Vision([None]), MockLLM()).run("cup", "find cup")
        self.assertFalse(result[0])
        self.assertEqual(robot.actions, [{"action": "stop"}])

    def test_near_and_far_starting_distances_stop_at_safe_boundary(self):
        for ranges in ([0.8, 0.6], [1.3, 1.1, 0.9, 0.85, 0.6]):
            with self.subTest(ranges=ranges):
                robot = Robot(ranges)
                result = ApproachController(robot, Vision(["center"] * len(ranges)), MockLLM()).run("cup", "find cup")
                self.assertFalse(result[0])
                self.assertIn("目标到达未确认", result[1])
                self.assertEqual(robot.actions[-1]["action"], "stop")

    def test_close_obstacle_causes_failure_without_forward_motion(self):
        robot = Robot([0.3])
        result = ApproachController(robot, Vision(["center"]), MockLLM()).run("cup", "find cup")
        self.assertFalse(result[0])
        self.assertIn("距离不足", result[1])
        self.assertEqual(robot.actions, [{"action": "stop"}])
        self.assertEqual(json.loads(result[2])["approach_state"], "clearance_below_stop_distance")

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

        def wait_for_cancel(_stamp, _timeout, cancel_event=None, command_id=None):
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

    def test_cancel_after_planning_blocks_command_dispatch(self):
        cancel_event = threading.Event()
        robot = Robot([2.0])

        class CancellingLLM(MockLLM):
            def plan_motion(self, task, vision):
                motion = super().plan_motion(task, vision)
                cancel_event.set()
                return motion

        result = ApproachController(
            robot, Vision(["center"]), CancellingLLM(), cancel_event=cancel_event
        ).run("cup", "find cup")
        self.assertFalse(result[0])
        self.assertIn("任务已取消", result[1])
        self.assertEqual(robot.actions, [])

    def test_temporary_missed_frame_is_rechecked_without_more_motion(self):
        robot = Robot([1.0, 0.6])
        vision = Vision(["center", None, "center"])
        result = ApproachController(robot, vision, MockLLM()).run("cup", "find cup")
        self.assertFalse(result[0])
        self.assertIn("目标到达未确认", result[1])
        self.assertEqual(vision.step, 2)
        self.assertEqual([a["action"] for a in robot.actions], ["move", "stop"])

    def test_safe_stop_boundary_is_exact_and_not_target_arrival(self):
        boundary = approach_safe_stop_distance(0.65)
        self.assertAlmostEqual(boundary, 0.79)
        for distance in (boundary - 1e-6, boundary):
            with self.subTest(distance=distance):
                robot = Robot([distance])
                result = ApproachController(robot, Vision(["center"]), MockLLM()).run("cup", "find cup")
                self.assertFalse(result[0])
                evidence = json.loads(result[2])
                self.assertEqual(evidence["approach_state"], "safe_stop_boundary_reached")
                self.assertFalse(evidence["target_arrival_confirmed"])
                self.assertEqual(evidence["target_association"], "unknown")
                self.assertEqual([action["action"] for action in robot.actions], ["stop"])
                self.assertIsNone(evidence["motion_command_id"])
                self.assertEqual(evidence["stop_command_id"], "stop-1")

    def test_invalid_stop_distance_fails_closed_before_motion(self):
        for distance in (0.0, -0.1, float("nan"), float("inf")):
            with self.subTest(distance=distance):
                with self.assertRaises(ValueError):
                    approach_safe_stop_distance(distance)
                robot = Robot([2.0])
                result = ApproachController(
                    robot, Vision(["center"]), MockLLM(), stop_distance=distance
                ).run("cup", "find cup")
                self.assertFalse(result[0])
                self.assertIn("停止距离必须是有限正数", result[1])
                self.assertEqual([action["action"] for action in robot.actions], ["stop"])

    def test_just_above_boundary_dispatches_only_a_safe_control_period(self):
        distance = approach_safe_stop_distance(0.65) + 1e-5
        robot = Robot([distance, distance])
        result = ApproachController(robot, Vision(["center"]), MockLLM()).run("cup", "find cup")
        self.assertFalse(result[0])
        self.assertEqual([action["action"] for action in robot.actions], ["move", "stop"])
        self.assertLessEqual(robot.actions[0]["duration"], 0.5)
        self.assertGreaterEqual(robot.actions[0]["duration"], 0.1)
        evidence = json.loads(result[2])
        self.assertEqual(evidence["motion_command_id"], "motion-1")
        self.assertEqual(evidence["stop_command_id"], "stop-2")
        self.assertEqual(evidence["executed_motion"]["action"], "move")
        self.assertEqual(len(evidence["executed_motions"]), 1)

    def test_unaccepted_plan_is_not_reported_as_executed_motion(self):
        class RejectedRobot(Robot):
            def execute_action(self, command):
                self.actions.append(command)
                return False

        class LongPlanLLM(MockLLM):
            def plan_motion(self, task, vision):
                return ActionPlan("move", linear_x=0.3, duration=1.6666667)

        robot = RejectedRobot([2.0])
        result = ApproachController(robot, Vision(["center"]), LongPlanLLM()).run("cup", "find cup")
        evidence = json.loads(result[2])
        self.assertFalse(result[0])
        self.assertAlmostEqual(evidence["planned_motion"]["duration"], 1.6666667)
        self.assertEqual(evidence["dispatch_attempted_motion"]["duration"], 0.5)
        self.assertIsNone(evidence["executed_motion"])
        self.assertIsNone(evidence["motion_command_id"])
        self.assertEqual(evidence["stop_command_id"], "stop-2")

    def test_duplicate_target_names_fail_closed_before_motion(self):
        class DuplicateVision(Vision):
            def find_object(self, _target):
                return {
                    "found": True,
                    "objects": [
                        {"name": "cup", "direction": "center"},
                        {"name": "cup", "direction": "left"},
                    ],
                    "observed_at": time.time(), "age_sec": 0.0, "stale": False,
                }

        robot = Robot([2.0])
        result = ApproachController(robot, DuplicateVision(["center"]), MockLLM()).run("cup", "find cup")
        self.assertFalse(result[0])
        self.assertEqual([action["action"] for action in robot.actions], ["stop"])

    def test_motion_direction_speed_and_translation_are_validated(self):
        cases = (
            (ActionPlan("move", linear_x=-0.1, duration=0.2), "center"),
            (ActionPlan("move", linear_x=0.31, duration=0.2), "center"),
            (ActionPlan("move", linear_x=0.1, linear_y=0.01, duration=0.2), "center"),
            (SimpleNamespace(action="move", linear_x=float("nan"), linear_y=0.0,
                             angular_z=0.0, duration=0.2), "center"),
            (ActionPlan("rotate", angular_z=-0.2, duration=0.2), "left"),
        )
        for motion, direction in cases:
            with self.subTest(motion=motion):
                valid, _ = ApproachController._validate_motion(motion, direction)
                self.assertFalse(valid)

    def test_lower_legal_speed_and_custom_stop_distance_keep_same_contract(self):
        class SlowLLM(MockLLM):
            def plan_motion(self, task, vision):
                return ActionPlan("move", linear_x=0.1, duration=0.2)

        stop_distance = 0.7
        boundary = approach_safe_stop_distance(stop_distance)
        self.assertAlmostEqual(boundary, 0.84)
        robot = Robot([1.0])
        controller = ApproachController(
            robot, Vision(["center"]), SlowLLM(), stop_distance=stop_distance
        )
        result = controller.run("cup", "find cup")
        self.assertFalse(result[0])
        self.assertEqual(robot.actions[0]["linear_x"], 0.1)
        self.assertEqual(robot.actions[0]["duration"], 0.2)


if __name__ == "__main__":
    unittest.main()
