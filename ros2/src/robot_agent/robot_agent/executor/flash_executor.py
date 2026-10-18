"""FlashExecutor: dispatch one Task to VisionTool / RobotTool / NavTool."""

from __future__ import annotations

import math
import time
from typing import Any, Dict, Optional
import json

from robot_agent.approach import ApproachController
from robot_agent.executor.base_executor import BaseExecutor, TaskResult
from robot_agent.llm import BaseLLM, LLMError
from robot_agent.planner.task_schema import Task


class FlashExecutor(BaseExecutor):
    """Executes a single task, calling tools directly (fast, low-level)."""

    def __init__(
        self,
        robot_tool: Any = None,
        vision_tool: Any = None,
        llm: Optional[BaseLLM] = None,
        approach_stop_distance: float = 0.65,
        approach_max_steps: int = 8,
    ) -> None:
        self._robot = robot_tool
        self._vision = vision_tool
        self._llm = llm
        self._approach_stop_distance = approach_stop_distance
        self._approach_max_steps = approach_max_steps

    def _run_approach(self, task: Task, context: Dict[str, Any], target: str) -> TaskResult:
        if self._robot is None or self._vision is None or self._llm is None:
            return TaskResult(task.id, False, "approach tools unavailable", {"recoverable": False})
        success, message, payload = ApproachController(
            self._robot, self._vision, self._llm, self._approach_stop_distance,
            max_steps=self._approach_max_steps,
            cancel_event=context.get("_cancel_event"),
            execute_action=context.get("_execute_approach_action") or context.get("_execute_action"),
            stop_and_confirm=context.get("_stop_and_confirm"),
            motion_wait_managed=callable(
                context.get("_execute_approach_action") or context.get("_execute_action")
            ),
        ).run(target, context.get("goal", task.name or target))
        data = {
            "approach": json.loads(payload) if payload else {},
            "recoverable": False,
            "task_tool": task.tool,
            "task_action": task.action,
            "target": target,
        }
        return TaskResult(task.id, success, message, data)

    def execute(self, task: Task, context: Dict[str, Any]) -> TaskResult:
        try:
            cancel_event = context.get("_cancel_event")
            if cancel_event is not None and cancel_event.is_set():
                return TaskResult(task.id, False, "任务已取消", {"cancelled": True, "recoverable": False})
            start_pose = self._pose_snapshot()
            start_command_id = getattr(self._robot, "latest_command_id", None)
            start_navigation_goal_id = getattr(self._robot, "last_navigation_goal_id", None)
            if task.tool == "vision":
                result = self._run_vision(task)
            elif task.tool == "robot":
                result = self._run_robot(task, context)
            elif task.tool == "nav":
                if self._robot is None:
                    result = TaskResult(task.id, False, "robot tool unavailable")
                elif task.action != "navigate":
                    result = TaskResult(task.id, False, f"unknown nav action: {task.action}")
                else:
                    success = self._navigate(task, context)
                    message = "navigation completed" if success else "navigation failed"
                    result = TaskResult(task.id, success, message, {"recoverable": False})
            else:
                result = TaskResult(task.id, False, f"unknown tool: {task.tool}")
            stage_evidence = {}
            result.data.setdefault("task_tool", task.tool)
            result.data.setdefault("task_action", task.action)
            result.data.setdefault("target", task.target)
            end_pose = self._pose_snapshot()
            if start_pose is not None:
                stage_evidence["stage_start_pose"] = start_pose
            if end_pose is not None:
                stage_evidence["stage_end_pose"] = end_pose
            goal_id = getattr(self._robot, "last_navigation_goal_id", None)
            # A vision/approach stage can follow navigation in the same task.
            # Do not attribute that earlier goal UUID to this stage.
            if goal_id and (task.action == "navigate" or goal_id != start_navigation_goal_id):
                stage_evidence["navigation_goal_id"] = goal_id
            command_id = getattr(self._robot, "latest_command_id", None)
            if (command_id and command_id != start_command_id
                    and not isinstance(result.data.get("approach"), dict)):
                stage_evidence["last_command_id"] = command_id
            if stage_evidence:
                result.data.update(stage_evidence)
            return result
        except Exception as exc:  # keep the loop alive on unexpected errors
            # An exception is not evidence that repeating the operation is
            # safe. In particular, a failed perception call must not trigger
            # the planner's generic rotate-and-observe recovery sequence.
            return TaskResult(
                task.id, False, f"executor error: {exc}", {"recoverable": False}
            )

    def _pose_snapshot(self) -> Optional[Dict[str, float]]:
        getter = getattr(self._robot, "get_pose", None)
        if not callable(getter):
            return None
        pose = getter()
        if not isinstance(pose, dict):
            return None
        try:
            values = {key: float(pose[key]) for key in ("x", "y", "yaw")}
        except (KeyError, TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in values.values()):
            return None
        return values

    def _run_vision(self, task: Task) -> TaskResult:
        if self._vision is None:
            return TaskResult(task.id, False, "vision tool unavailable")
        if task.action in ("find_object", "find"):
            if not task.target.strip():
                return TaskResult(
                    task.id, False, "vision target is required", {"recoverable": False}
                )
            data = self._vision.find_object(task.target)
            if not self._valid_vision_result(
                data, require_found=True, expected_target=task.target
            ):
                return TaskResult(
                    task.id, False, "malformed vision result", {"recoverable": False}
                )
            if self._vision_result_is_stale(data):
                return TaskResult(task.id, False, "vision result is stale", data={**data, "recoverable": False})
            if not data.get("found"):
                return TaskResult(task.id, False, f"no target found: {task.target}", data={**data, "recoverable": False})
            return TaskResult(task.id, True, "vision ok", data={"vision": data})
        elif task.action in ("observe", "scene", "get_objects"):
            data = self._vision.get_objects()
        else:
            data = self._vision.get_scene()
        if not self._valid_vision_result(data):
            return TaskResult(task.id, False, "malformed vision result", {"recoverable": False})
        if self._vision_result_is_stale(data):
            return TaskResult(
                task.id, False, "vision result is stale", {**data, "recoverable": False}
            )
        return TaskResult(task.id, True, "vision ok", data={"vision": data})

    def _valid_vision_result(
        self, data: Any, require_found: bool = False, expected_target: str = ""
    ) -> bool:
        """Reject malformed perception payloads before dependent work can run."""
        if not isinstance(data, dict):
            return False
        objects = data.get("objects")
        if not isinstance(objects, list):
            return False
        if any(not isinstance(item, dict) or not isinstance(item.get("name"), str)
               for item in objects):
            return False
        if require_found and not isinstance(data.get("found"), bool):
            return False
        if require_found and data.get("found") and expected_target:
            target = expected_target.strip().casefold()
            if not any(target in item["name"].casefold() for item in objects):
                return False
        if not isinstance(data.get("stale"), bool):
            return False
        # Ros2VisionTool derives these fields from the producer's Unix
        # observed_at stamp. Fresh payloads must retain that contract through
        # the planner handoff; stale payloads are handled as stale below.
        if not data["stale"]:
            observed_at = data.get("observed_at")
            age_sec = data.get("age_sec")
            if (not isinstance(observed_at, (int, float)) or isinstance(observed_at, bool)
                    or not math.isfinite(observed_at) or observed_at <= 0
                    or observed_at > time.time()):
                return False
            if (not isinstance(age_sec, (int, float)) or isinstance(age_sec, bool)
                    or not math.isfinite(age_sec) or age_sec < 0):
                return False
        return True

    def _vision_result_is_stale(self, data: Dict[str, Any]) -> bool:
        if data["stale"]:
            return True
        max_age_sec = float(getattr(self._vision, "max_age_sec", 2.5))
        observed_at = float(data["observed_at"])
        age_sec = float(data["age_sec"])
        return max(time.time() - observed_at, age_sec) > max_age_sec

    def _run_robot(self, task: Task, context: Dict[str, Any]) -> TaskResult:
        if self._robot is None:
            return TaskResult(task.id, False, "robot tool unavailable")
        if task.action == "approach":
            return self._run_approach(task, context, task.target)
        if task.action == "infer":
            if self._llm is None:
                return TaskResult(task.id, False, "no LLM for infer action")
            plan = self._llm.generate_action(task.target)
            if plan.action == "approach":
                return self._run_approach(task, context, plan.goal)
            if plan.action == "vision":
                return TaskResult(task.id, False, "infer resolved to non-motion action")
            success = self._dispatch(plan.to_command_dict(), context)
            if not success:
                message = f"{plan.action} was rejected"
            elif plan.action == "navigate":
                # The mock planner's response describes navigation as a future
                # integration. The dispatch callback actually waits for Nav2,
                # so report its terminal outcome instead of that stale prompt.
                message = "navigation completed"
            else:
                message = plan.response or plan.action
            return TaskResult(task.id, bool(success), message, data={"action": plan.to_command_dict()})

        if task.action == "navigate":
            success = self._navigate(task, context)
            return TaskResult(
                task.id, bool(success), "navigation completed" if success else "navigation failed",
                {"recoverable": False},
            )

        if task.action in ("move", "rotate", "stop"):
            if task.action == "stop":
                cancel_task = context.get("_cancel_task")
                if callable(cancel_task):
                    success, message, _ = cancel_task()
                    return TaskResult(task.id, False, message, {"cancelled": True, "recoverable": False})
                success = self._robot.stop()
                return TaskResult(task.id, bool(success), "stopped" if success else "stop was rejected")
            if task.action == "rotate":
                success = self._dispatch({
                    "action": "rotate",
                    "angular_z": float(task.params.get("angular_z", 0.5)),
                    "duration": float(task.params.get("duration", 1.0)),
                }, context)
                return TaskResult(task.id, bool(success), "rotated" if success else "rotation was rejected")
            # move: use vision context when available, else explicit params
            if self._llm is not None and context.get("vision") is not None:
                objects = context["vision"].get("objects", [])
                target = task.target or (objects[0].get("name", "") if objects else "")
                return self._run_approach(task, context, target)
            success = self._dispatch({
                "action": "move",
                "linear_x": float(task.params.get("linear_x", 0.3)),
                "linear_y": float(task.params.get("linear_y", 0.0)),
                "angular_z": float(task.params.get("angular_z", 0.0)),
                "duration": float(task.params.get("duration", 1.0)),
            }, context)
            return TaskResult(task.id, bool(success), "moved" if success else "move was rejected")

        return TaskResult(task.id, False, f"unknown robot action: {task.action}")

    def _dispatch(self, action: Dict[str, Any], context: Dict[str, Any]) -> bool:
        cancel_event = context.get("_cancel_event")
        if cancel_event is not None and cancel_event.is_set():
            return False
        callback = context.get("_execute_action")
        if callable(callback):
            return bool(callback(action))
        return bool(self._robot.execute_action(action))

    def _navigate(self, task: Task, context: Dict[str, Any]) -> bool:
        callback = context.get("_navigate")
        timeout = task.params.get("timeout", 120.0)
        if callable(callback):
            return bool(callback(task.target, timeout))
        cancel_event = context.get("_cancel_event")
        return bool(self._robot.navigate(task.target, timeout=timeout, cancel_event=cancel_event))
