"""Shared visual approach controller for simple and hierarchical agents."""

from __future__ import annotations

import json
import logging
import math
import time

from robot_agent.llm import LLMError

APPROACH_MAX_STEP_DURATION_SEC = 0.5
APPROACH_MAX_LINEAR_SPEED = 0.3
APPROACH_MAX_ANGULAR_SPEED = 0.5
APPROACH_CONTROL_PERIOD_SEC = 0.1
APPROACH_BRAKING_MARGIN_M = 0.08


def approach_safe_stop_distance(stop_distance: float) -> float:
    """Threshold leaving margin for braking and two 10 Hz control cycles.

    One cycle covers sensor/dispatch latency and the second is the next
    controller update. At defaults: 0.65 + 0.08 + 2 * 0.3 * 0.1 = 0.79 m.
    """
    stop_distance = float(stop_distance)
    if not math.isfinite(stop_distance) or stop_distance <= 0.0:
        raise ValueError("approach stop distance must be finite and greater than zero")
    return (stop_distance + APPROACH_BRAKING_MARGIN_M
            + APPROACH_MAX_LINEAR_SPEED * 2.0 * APPROACH_CONTROL_PERIOD_SEC)


class ApproachController:
    def __init__(
        self, robot, vision, llm, stop_distance=0.65, logger=None, max_steps=8,
        cancel_event=None, execute_action=None,
        stop_and_confirm=None, motion_wait_managed=False,
    ):
        self._robot = robot
        self._vision = vision
        self._llm = llm
        self._stop_distance = float(stop_distance)
        self._logger = logger if logger is not None else logging.getLogger(__name__)
        self._max_steps = max(1, min(int(max_steps), 20))
        self._cancel_event = cancel_event
        self._execute_action = execute_action
        self._stop_callback = stop_and_confirm
        self._motion_wait_managed = bool(motion_wait_managed)
        self._last_stop_confirmation = "unknown"
        self._stop_attempted = False
        self._stop_command_id = None
        self._planned_motion = None
        self._dispatch_attempted_motion = None
        self._executed_motion = None
        self._executed_motions = []
        self._motion_command_id = None

    def _cancelled(self) -> bool:
        return self._cancel_event is not None and self._cancel_event.is_set()

    def run(self, goal: str, task: str) -> tuple[bool, str, str]:
        if self._cancelled():
            return False, "任务已取消", self._result_evidence("")
        result = self._run(goal, task)
        if self._cancelled():
            return False, "任务已取消", self._result_evidence(result[2])
        if not result[0]:
            if not self._stop_attempted and not self._stop_and_confirm():
                result = (False, f"{result[1]}；停止状态未确认", result[2])
        return (result[0], result[1], self._result_evidence(result[2]))

    def _result_evidence(self, payload: str) -> str:
        try:
            evidence = json.loads(payload) if payload else {}
        except (TypeError, ValueError):
            evidence = {"detail": payload}
        if not isinstance(evidence, dict):
            evidence = {"detail": evidence}
        evidence["planned_motion"] = self._planned_motion
        evidence["dispatch_attempted_motion"] = self._dispatch_attempted_motion
        evidence["executed_motion"] = self._executed_motion
        evidence["executed_motions"] = list(self._executed_motions)
        evidence["motion_command_id"] = self._motion_command_id
        evidence["stop_command_id"] = self._stop_command_id
        evidence["stop_confirmation_type"] = self._last_stop_confirmation
        evidence["physical_stop_confirmed"] = False
        return json.dumps(evidence, ensure_ascii=False)

    def _stop_and_confirm(self) -> bool:
        self._stop_attempted = True
        command_id_before = getattr(self._robot, "latest_command_id", None)
        if callable(self._stop_callback):
            try:
                confirmed = bool(self._stop_callback("approach"))
                self._last_stop_confirmation = "control_ack" if confirmed else "unconfirmed"
                command_id_after = getattr(self._robot, "latest_command_id", None)
                if command_id_after != command_id_before:
                    self._stop_command_id = command_id_after
                return confirmed
            except Exception as exc:
                self._logger.error(f"failed to confirm stop after approach: {exc}")
                self._last_stop_confirmation = "unconfirmed"
                return False
        stop = getattr(self._robot, "stop", None)
        confirm = getattr(self._robot, "wait_for_stop_confirmation", None)
        stamp = getattr(self._robot, "status_stamp", None)
        if not callable(stop) or not callable(confirm) or not callable(stamp):
            return False
        try:
            before_stamp = stamp()
            accepted = stop()
            command_id = getattr(self._robot, "latest_command_id", None)
            if command_id != command_id_before:
                self._stop_command_id = command_id
            if not accepted:
                self._last_stop_confirmation = "unconfirmed"
                return False
            if command_id is None or command_id == command_id_before:
                self._last_stop_confirmation = "unconfirmed"
                return False
            confirmed = bool(confirm(before_stamp, timeout_sec=2.0, command_id=command_id))
            self._last_stop_confirmation = "control_ack" if confirmed else "unconfirmed"
            return confirmed
        except Exception as exc:
            self._logger.error(f"failed to confirm stop after approach: {exc}")
            self._last_stop_confirmation = "unconfirmed"
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
    def _matching_target(vision, goal: str):
        if not goal or not isinstance(vision, dict):
            return None
        objects = vision.get("objects")
        if not isinstance(objects, list):
            return None
        target = goal.strip().casefold()
        valid = [obj for obj in objects if isinstance(obj, dict)
                 and isinstance(obj.get("name"), str)]
        exact = [obj for obj in valid if obj["name"].strip().casefold() == target]
        if len(exact) == 1:
            return exact[0]
        matches = [obj for obj in valid if target in obj["name"].casefold()]
        return matches[0] if len(matches) == 1 else None

    @classmethod
    def _contains_target(cls, vision, goal: str) -> bool:
        return cls._matching_target(vision, goal) is not None

    @staticmethod
    def _finite_number(value) -> bool:
        return (isinstance(value, (int, float)) and not isinstance(value, bool)
                and math.isfinite(value))

    @classmethod
    def _validate_motion(cls, motion, direction: str) -> tuple[bool, str]:
        if getattr(motion, "action", None) not in ("move", "rotate"):
            return False, "接近阶段只允许前进或原地转向动作"
        values = (getattr(motion, "linear_x", None), getattr(motion, "linear_y", None),
                  getattr(motion, "angular_z", None), getattr(motion, "duration", None))
        if not all(cls._finite_number(value) for value in values):
            return False, "接近动作参数必须是有限数值"
        linear_x, linear_y, angular_z, duration = values
        if duration <= 0:
            return False, "接近动作时长必须大于零"
        if motion.action == "move":
            if direction != "center" or not 0 < linear_x <= APPROACH_MAX_LINEAR_SPEED:
                return False, "目标未居中或前进速度超出接近安全范围"
            if linear_y != 0 or angular_z != 0:
                return False, "接近阶段不允许侧移或边前进边转向"
        elif motion.action == "rotate":
            if direction not in ("left", "right"):
                return False, "只在目标位于左右侧时允许原地转向"
            if linear_x != 0 or linear_y != 0:
                return False, "接近阶段转向必须保持底盘线速度为零"
            if (direction == "left" and not 0 < angular_z <= APPROACH_MAX_ANGULAR_SPEED) or (
                    direction == "right" and not -APPROACH_MAX_ANGULAR_SPEED <= angular_z < 0):
                return False, "接近转向方向与目标方位不一致或角速度超限"
        return True, ""

    @staticmethod
    def _clearance_failure(clearance, direction, stop_distance, safe_stop_distance):
        distance = float(clearance["distance"])
        too_close = distance < stop_distance
        evidence = {
            **clearance,
            "target_direction": direction,
            "stop_distance": stop_distance,
            "safe_stop_distance": safe_stop_distance,
            "approach_state": (
                "clearance_below_stop_distance" if too_close
                else "safe_stop_boundary_reached"
            ),
            "target_arrival_confirmed": False,
            "target_association": "unknown",
        }
        message = ("前方距离不足，已停止接近；目标到达未确认" if too_close
                   else "已到安全停车边界；目标到达未确认")
        return False, message, json.dumps(evidence, ensure_ascii=False)

    def _run(self, goal: str, task: str) -> tuple[bool, str, str]:
        goal = (goal or "").strip()
        try:
            if not goal:
                return False, "接近任务缺少目标名称。", ""
            if not math.isfinite(self._stop_distance) or self._stop_distance <= 0.0:
                return False, "接近停止距离必须是有限正数，停止接近", ""
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
            if vision.get("found") is not True or not self._contains_target(vision, goal):
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
                    if (not isinstance(clearance, dict) or clearance.get("available") is not True
                            or not self._finite_number(clearance.get("distance"))):
                        return False, "前方激光测距不可用或已过期，停止接近", ""
                    target_object = self._matching_target(vision, goal)
                    direction = target_object.get("direction") if target_object else None
                    centered = direction == "center"
                    stop_distance = self._stop_distance
                    safe_stop_distance = approach_safe_stop_distance(stop_distance)
                    if float(clearance["distance"]) <= safe_stop_distance:
                        return self._clearance_failure(
                            clearance, direction, stop_distance, safe_stop_distance
                        )
                if step == max_steps:
                    return False, "达到视觉接近步数上限，目标仍未确认到达", last_action_json
                motion = self._llm.plan_motion(task, vision)
                self._planned_motion = motion.to_command_dict()
                last_action_json = json.dumps(self._planned_motion, ensure_ascii=False)
                target_object = self._matching_target(vision, goal)
                direction = target_object.get("direction") if target_object else None
                valid_motion, motion_error = self._validate_motion(motion, direction)
                if not valid_motion:
                    return False, motion_error, last_action_json
                motion.duration = min(motion.duration, APPROACH_MAX_STEP_DURATION_SEC)

                # Planning may take seconds (or longer for a remote provider).
                # Do not dispatch a command from a vision/range sample that aged
                # out while the planner was responding.
                if not self._vision_is_fresh(vision):
                    return False, "视觉观测在规划完成后已过期", last_action_json
                clearance = get_sensor("front_range")
                if (not isinstance(clearance, dict) or clearance.get("available") is not True
                        or not self._finite_number(clearance.get("distance"))):
                    return False, "规划完成后前方测距已过期", last_action_json
                safe_stop_distance = approach_safe_stop_distance(stop_distance)
                if float(clearance["distance"]) <= safe_stop_distance:
                    return self._clearance_failure(
                        clearance, direction, stop_distance, safe_stop_distance
                    )
                if motion.action == "move" and motion.linear_x > 0:
                    usable_margin = (float(clearance["distance"]) - stop_distance
                                     - APPROACH_BRAKING_MARGIN_M
                                     - motion.linear_x * APPROACH_CONTROL_PERIOD_SEC)
                    safe_duration = usable_margin / motion.linear_x
                    if safe_duration < APPROACH_CONTROL_PERIOD_SEC:
                        return False, "剩余净空不足以安全完成一个控制周期，停止接近", last_action_json
                    motion.duration = min(motion.duration, safe_duration)
                last_action_json = json.dumps(motion.to_command_dict(), ensure_ascii=False)

                status_stamp = self._robot.status_stamp()
                if self._cancelled():
                    return False, "任务已取消", last_action_json
                dispatch = self._execute_action or self._robot.execute_action
                command_id_before = getattr(self._robot, "latest_command_id", None)
                self._dispatch_attempted_motion = motion.to_command_dict()
                if not dispatch(motion.to_command_dict()):
                    return False, "执行层未接受运动命令", last_action_json
                command_id = getattr(self._robot, "latest_command_id", None)
                if command_id == command_id_before:
                    command_id = None
                self._executed_motion = motion.to_command_dict()
                self._motion_command_id = command_id
                self._executed_motions.append({
                    "command": self._executed_motion,
                    "command_id": command_id,
                })
                self._logger.info(f"approach '{task}' step={step + 1} -> {last_action_json}")

                duration = max(0.0, motion.duration)
                if self._motion_wait_managed:
                    motion_done = True
                elif self._cancel_event is None:
                    motion_done = wait_for_motion(
                        status_stamp, max(3.0, duration + 2.0), command_id=command_id
                    )
                else:
                    motion_done = wait_for_motion(
                        status_stamp, max(3.0, duration + 2.0),
                        cancel_event=self._cancel_event, command_id=command_id,
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
                    if vision.get("found") is True and self._contains_target(vision, goal):
                        break
                    # Re-observe while stationary. Never issue another motion
                    # based on the previous detection after a missed frame.
                    motion_completed_at = float(scene.get("observed_at", time.time()))
                if vision.get("found") is not True or not self._contains_target(vision, goal):
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

