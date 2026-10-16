"""Shared visual approach controller for simple and hierarchical agents."""

from __future__ import annotations

import json
import logging
import time

from robot_agent.llm import LLMError

APPROACH_MAX_STEP_DURATION_SEC = 0.5


class ApproachController:
    def __init__(self, robot, vision, llm, stop_distance=0.65, logger=None, max_steps=8):
        self._robot = robot
        self._vision = vision
        self._llm = llm
        self._stop_distance = float(stop_distance)
        self._logger = logger if logger is not None else logging.getLogger(__name__)
        self._max_steps = max(1, min(int(max_steps), 20))

    def run(self, goal: str, task: str) -> tuple[bool, str, str]:
        result = self._run(goal, task)
        if not result[0]:
            stop = getattr(self._robot, "stop", None)
            if callable(stop):
                try:
                    stop()
                except Exception as exc:
                    self._logger.error(f"failed to stop after approach failure: {exc}")
        return result

    def _run(self, goal: str, task: str) -> tuple[bool, str, str]:
        goal = (goal or "").strip()
        try:
            find_target = lambda: (
                self._vision.find_object(goal)
                if goal else self._vision.get_objects()
            )
            vision = find_target()
            if vision.get("stale"):
                return False, "视觉结果已过期，请等待新画面。", json.dumps(vision, ensure_ascii=False)
            if not vision.get("found"):
                response = f"没有发现目标 {goal}。" if goal else "当前场景未检测到任何物体。"
                return False, response, json.dumps(vision, ensure_ascii=False)

            wait_for_scene = getattr(self._vision, "wait_for_new_scene", None)
            wait_for_motion = getattr(self._robot, "wait_for_motion_complete", None)
            closed_loop = callable(wait_for_scene) and callable(wait_for_motion)
            if not closed_loop or not all(callable(getattr(self._robot, name, None)) for name in (
                "status_stamp", "get_sensor", "stop"
            )):
                return False, "接近需要完整的视觉、运动和测距反馈接口", ""
            max_steps = self._max_steps
            last_action_json = ""
            clearance = None

            for step in range(max_steps + 1):
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
                            if self._robot.stop():
                                return True, "已到达目标前方的接近停止距离", json.dumps(clearance, ensure_ascii=False)
                        return False, "前方距离不足，已停止接近", json.dumps(clearance, ensure_ascii=False)
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
                last_action_json = json.dumps(motion.to_command_dict(), ensure_ascii=False)

                status_stamp = self._robot.status_stamp()
                if not self._robot.execute_action(motion.to_command_dict()):
                    return False, "执行层未接受运动命令", last_action_json
                self._logger.info(f"approach '{task}' step={step + 1} -> {last_action_json}")

                duration = max(0.0, motion.duration)
                if not wait_for_motion(status_stamp, max(3.0, duration + 2.0)):
                    return False, "运动未能在时限内完成", last_action_json

                wait_for_range = getattr(self._robot, "wait_for_front_range", None)
                clearance = wait_for_range() if callable(wait_for_range) else None
                if clearance is not None and not clearance.get("available"):
                    return False, "运动后未获得稳定的前方测距", ""

                # A frame from during the motion is newer than the previous
                # observation but cannot verify the completed motion.
                motion_completed_at = time.time()
                for observation in range(3):
                    scene = wait_for_scene(motion_completed_at, timeout_sec=5.0)
                    if scene is None:
                        message = "运动后未收到新视觉帧" if observation == 0 else f"运动后无法重新定位目标 {goal}"
                        return False, message, last_action_json
                    query_scene = getattr(self._vision, "query_scene", None)
                    vision = query_scene(scene, goal) if callable(query_scene) else find_target()
                    if vision.get("stale"):
                        return False, "运动后的视觉结果已过期", json.dumps(vision, ensure_ascii=False)
                    if vision.get("found"):
                        break
                    # Re-observe while stationary. Never issue another motion
                    # based on the previous detection after a missed frame.
                    motion_completed_at = float(scene.get("observed_at", time.time()))
                if not vision.get("found"):
                    return False, f"运动后无法重新定位目标 {goal}", json.dumps(vision, ensure_ascii=False)

            return False, "达到视觉接近步数上限，目标仍未确认到达", last_action_json
        except LLMError as exc:
            self._logger.warn(f"approach LLM error for task '{task}': {exc}")
            return False, str(exc), ""
        except Exception as exc:  # keep the node alive on unexpected errors
            self._logger.error(f"approach error for task '{task}': {exc}")
            return False, f"内部错误：{exc}", ""

