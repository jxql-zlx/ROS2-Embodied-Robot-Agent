"""FlashExecutor: dispatch one Task to VisionTool / RobotTool / NavTool."""

from __future__ import annotations

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
            execute_action=context.get("_execute_action"),
        ).run(target, context.get("goal", task.name or target))
        data = {"approach": json.loads(payload) if payload else {}, "recoverable": False}
        return TaskResult(task.id, success, message, data)

    def execute(self, task: Task, context: Dict[str, Any]) -> TaskResult:
        try:
            cancel_event = context.get("_cancel_event")
            if cancel_event is not None and cancel_event.is_set():
                return TaskResult(task.id, False, "任务已取消", {"cancelled": True, "recoverable": False})
            if task.tool == "vision":
                return self._run_vision(task)
            if task.tool == "robot":
                return self._run_robot(task, context)
            if task.tool == "nav":
                if self._robot is None:
                    return TaskResult(task.id, False, "robot tool unavailable")
                if task.action != "navigate":
                    return TaskResult(task.id, False, f"unknown nav action: {task.action}")
                success = self._navigate(task, context)
                message = "navigation completed" if success else "navigation failed"
                return TaskResult(task.id, success, message)
            return TaskResult(task.id, False, f"unknown tool: {task.tool}")
        except Exception as exc:  # keep the loop alive on unexpected errors
            return TaskResult(task.id, False, f"executor error: {exc}")

    def _run_vision(self, task: Task) -> TaskResult:
        if self._vision is None:
            return TaskResult(task.id, False, "vision tool unavailable")
        if task.action in ("find_object", "find"):
            data = self._vision.find_object(task.target)
            if isinstance(data, dict) and data.get("stale"):
                return TaskResult(task.id, False, "vision result is stale", data={**data, "recoverable": False})
            if isinstance(data, dict) and not data.get("found", True):
                return TaskResult(task.id, False, f"no target found: {task.target}", data={**data, "recoverable": False})
            return TaskResult(task.id, True, "vision ok", data={"vision": data})
        elif task.action in ("observe", "scene", "get_objects"):
            data = self._vision.get_objects()
        else:
            data = self._vision.get_scene()
        if data is None:
            return TaskResult(task.id, False, "vision result unavailable")
        if isinstance(data, dict) and data.get("stale"):
            return TaskResult(task.id, False, "vision result is stale", data=data)
        return TaskResult(task.id, True, "vision ok", data={"vision": data})

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
            return TaskResult(task.id, bool(success), "navigation completed" if success else "navigation failed")

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
