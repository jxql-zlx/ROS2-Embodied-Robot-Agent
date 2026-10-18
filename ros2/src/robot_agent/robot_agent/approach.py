"""Shared visual approach controller for simple and hierarchical agents."""

from __future__ import annotations

import json
import logging
import math
import time

from robot_agent.llm import LLMError

APPROACH_MAX_STEP_DURATION_SEC = 0.5


class ApproachController:
    def __init__(
        self, robot, vision, llm, stop_distance=0.65, logger=None, max_steps=8,
        cancel_event=None, execute_action=None,
    ):
        self._robot = robot
        self._vision = vision
        self._llm = llm
        self._stop_distance = float(stop_distance)
        self._logger = logger if logger is not None else logging.getLogger(__name__)
        self._max_steps = max(1, min(int(max_steps), 20))
        self._cancel_event = cancel_event
        self._execute_action = execute_action

    def _cancelled(self) -> bool:
        return self._cancel_event is not None and self._cancel_event.is_set()

    def run(self, goal: str, task: str) -> tuple[bool, str, str]:
        if self._cancelled():
            return False, "任务已取消", ""
        result = self._run(goal, task)
        if self._cancelled():
            return False, "任务已取消", result[2]
        if not result[0]:
            if not self._stop_and_confirm():
                result = (False, f"{result[1]}；停止状态未确认", result[2])
        return result

    def _stop_and_confirm(self) -> bool:
        stop = getattr(self._robot, "stop", None)
        confirm = getattr(self._robot, "wait_for_stop_confirmation", None)
        stamp = getattr(self._robot, "status_stamp", None)
        if not callable(stop) or not callable(confirm) or not callable(stamp):
            return False
        try:
            before_stamp = stamp()
            if not stop():
                return False
            command_id = getattr(self._robot, "latest_command_id", None)
            return bool(confirm(before_stamp, timeout_sec=2.0, command_id=command_id))
        except Exception as exc:
            self._logger.error(f"failed to confirm stop after approach: {exc}")
            return False

    def _vision_is_fresh(self, vision) -> bool:
        if not isinstance(vision, dict) or vision.get("stale") is not False:
            return False
        observed_at = vision.get("observed_at")
        age_sec = vision.get("age_sec")
        if (not isinstance(observed_at, (int, float)) or isinstance(observed_at, bool)
                or not math.isfinite(observed_at) or observed_at <= 0 or observed_at > time.time()
                or not isinstance(age_sec, (int, float)) or isinstance(age_sec, bool)
                or not math.isfinite(age_sec) or age_sec < 0):
            return False
        max_age_sec = float(getattr(self._vision, "max_age_sec", 2.5))
        return max(time.time() - observed_at, age_sec) <= max_age_sec

    @staticmethod
    def _contains_target(vision, goal: str) -> bool:
        if not goal or not isinstance(vision, dict):
            return False
        objects = vision.get("objects")
        if not isinstance(objects, list):
            return False
        target = goal.strip().casefold()
        return any(
            isinstance(obj, dict) and isinstance(obj.get("name"), str)
            and target in obj["name"].casefold()
            for obj in objects
        )

    def _run(self, goal: str, task: str) -> tuple[bool, str, str]:
        goal = (goal or "").strip()
        try:
            if not goal:
                return False, "接近任务缺少目标名称。", ""
            if self._cancelled():
                return False, "任务已取消", ""
            find_target = lambda: (
                self._vision.find_object(goal)
                if goal else self._vision.get_objects()
            )
            vision = find_target()
            if self._cancelled():
                return False, "任务已取消", ""
            if not self._vision_is_fresh(vision):
                return False, "视觉结果已过期，请等待新画面。", json.dumps(vision, ensure_ascii=False)
            if not vision.get("found") or not self._contains_target(vision, goal):
                response = f"没有发现目标 {goal}。" if goal else "当前场景未检测到任何物体。"
                return False, response, json.dumps(vision, ensure_ascii=False)

            wait_for_scene = getattr(self._vision, "wait_for_new_scene", None)
            wait_for_motion = getattr(self._robot, "wait_for_motion_complete", None)
            closed_loop = callable(wait_for_scene) and callable(wait_for_motion)
            if not closed_loop or not all(callable(getattr(self._robot, name, None)) for name in (
                "status_stamp", "get_sensor", "stop", "wait_for_stop_confirmation"
            )):
                return False, "接近需要完整的视觉、运动和测距反馈接口", ""
            max_steps = self._max_steps
            last_action_json = ""
            clearance = None

            for step in range(max_steps + 1):
                if self._cancelled():
                    return False, "任务已取消", last_action_json
                get_sensor = getattr(self._robot, "get_sensor", None)
                if closed_loop and callable(get_sensor):
                    # Visual re-observation can take seconds. Recheck range
                    # freshness before each decision, even after a stable scan.
                    clearance = get_sensor("front_range")
                    if not clearance.get("available"):
                        return False, "前方激光测距不可用或已过期，停止接近", ""
                    objects = vision.get("objects", [])
                    centered = bool(objects) and objects[0].get("direction") == "center"
                    stop_distance = self._stop_distance
                    if float(clearance["distance"]) <= stop_distance:
                        if centered and float(clearance["distance"]) >= stop_distance - 0.25:
                            if self._stop_and_confirm():
                                return True, "已到达目标前方的接近停止距离", json.dumps(clearance, ensure_ascii=False)
                        failure = {
                            **clearance,
                            "target_direction": objects[0].get("direction") if objects else None,
                            "stop_distance": stop_distance,
                        }
                        return False, "前方距离不足，已停止接近", json.dumps(failure, ensure_ascii=False)
                if step == max_steps:
                    return False, "达到视觉接近步数上限，目标仍未确认到达", last_action_json
                motion = self._llm.plan_motion(task, vision)
                last_action_json = json.dumps(motion.to_command_dict(), ensure_ascii=False)
                if motion.action == "stop":
                    return False, "找到目标，但当前视觉信息无法安全规划接近动作。", last_action_json

                motion.duration = min(
                    max(0.0, motion.duration) or APPROACH_MAX_STEP_DURATION_SEC,
                    APPROACH_MAX_STEP_DURATION_SEC,
                )
                if clearance and motion.action == "move" and motion.linear_x > 0:
                    margin = max(0.0, float(clearance["distance"]) - stop_distance)
                    # The controller updates at 10 Hz; shorten the last step
                    # to the remaining distance, with one update as minimum.
                    motion.duration = min(motion.duration, max(0.1, margin / motion.linear_x))

                # Planning may take seconds (or longer for a remote provider).
                # Do not dispatch a command from a vision/range sample that aged
                # out while the planner was responding.
                if not self._vision_is_fresh(vision):
                    return False, "视觉观测在规划完成后已过期", last_action_json
                clearance = get_sensor("front_range")
                if not clearance.get("available"):
                    return False, "规划完成后前方测距已过期", last_action_json
                if float(clearance["distance"]) <= stop_distance:
                    objects = vision.get("objects", [])
                    centered = bool(objects) and objects[0].get("direction") == "center"
                    if (centered and float(clearance["distance"]) >= stop_distance - 0.25
                            and self._stop_and_confirm()):
                        return True, "已到达目标前方的接近停止距离", json.dumps(clearance, ensure_ascii=False)
                    return False, "规划完成后前方距离不足，停止接近", json.dumps(clearance, ensure_ascii=False)
                if motion.action == "move" and motion.linear_x > 0:
                    margin = max(0.0, float(clearance["distance"]) - stop_distance)
                    motion.duration = min(
                        motion.duration, max(0.1, margin / motion.linear_x)
                    )
                last_action_json = json.dumps(motion.to_command_dict(), ensure_ascii=False)

                status_stamp = self._robot.status_stamp()
                if self._cancelled():
                    return False, "任务已取消", last_action_json
                dispatch = self._execute_action or self._robot.execute_action
                if not dispatch(motion.to_command_dict()):
                    return False, "执行层未接受运动命令", last_action_json
                self._logger.info(f"approach '{task}' step={step + 1} -> {last_action_json}")

                duration = max(0.0, motion.duration)
                if self._cancel_event is None:
                    motion_done = wait_for_motion(status_stamp, max(3.0, duration + 2.0))
                else:
                    motion_done = wait_for_motion(
                        status_stamp, max(3.0, duration + 2.0), cancel_event=self._cancel_event
                    )
                if not motion_done:
                    if self._cancelled():
                        return False, "任务已取消", last_action_json
                    return False, "运动未能在时限内完成", last_action_json

                wait_for_range = getattr(self._robot, "wait_for_front_range", None)
                if callable(wait_for_range):
                    clearance = (wait_for_range() if self._cancel_event is None else
                                 wait_for_range(cancel_event=self._cancel_event))
                else:
                    clearance = None
                if self._cancelled():
                    return False, "任务已取消", last_action_json
                if clearance is not None and not clearance.get("available"):
                    return False, "运动后未获得稳定的前方测距", ""

                # A frame from during the motion is newer than the previous
                # observation but cannot verify the completed motion.
                motion_completed_at = time.time()
                for observation in range(3):
                    if self._cancel_event is None:
                        scene = wait_for_scene(motion_completed_at, timeout_sec=5.0)
                    else:
                        scene = wait_for_scene(
                            motion_completed_at, timeout_sec=5.0, cancel_event=self._cancel_event
                        )
                    if self._cancelled():
                        return False, "任务已取消", last_action_json
                    if scene is None:
                        message = "运动后未收到新视觉帧" if observation == 0 else f"运动后无法重新定位目标 {goal}"
                        return False, message, last_action_json
                    query_scene = getattr(self._vision, "query_scene", None)
                    vision = query_scene(scene, goal) if callable(query_scene) else find_target()
                    if not self._vision_is_fresh(vision):
                        return False, "运动后的视觉结果已过期", json.dumps(vision, ensure_ascii=False)
                    if vision.get("found") and self._contains_target(vision, goal):
                        break
                    # Re-observe while stationary. Never issue another motion
                    # based on the previous detection after a missed frame.
                    motion_completed_at = float(scene.get("observed_at", time.time()))
                if not vision.get("found"):
                    return False, f"运动后无法重新定位目标 {goal}", json.dumps(vision, ensure_ascii=False)

            return False, "达到视觉接近步数上限，目标仍未确认到达", last_action_json
        except LLMError as exc:
            if self._cancelled():
                return False, "任务已取消", ""
            self._logger.warn(f"approach LLM error for task '{task}': {exc}")
            return False, str(exc), ""
        except Exception as exc:  # keep the node alive on unexpected errors
            if self._cancelled():
                return False, "任务已取消", ""
            self._logger.error(f"approach error for task '{task}': {exc}")
            return False, f"内部错误：{exc}", ""

