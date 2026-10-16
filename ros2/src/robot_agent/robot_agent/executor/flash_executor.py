"""FlashExecutor: dispatch one Task to VisionTool / RobotTool / NavTool."""

from __future__ import annotations

from typing import Any, Dict, Optional

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
    ) -> None:
        self._robot = robot_tool
        self._vision = vision_tool
        self._llm = llm

    def execute(self, task: Task, context: Dict[str, Any]) -> TaskResult:
        try:
            if task.tool == "vision":
                return self._run_vision(task)
            if task.tool == "robot":
                return self._run_robot(task, context)
            if task.tool == "nav":
                if self._robot is None:
                    return TaskResult(task.id, False, "robot tool unavailable")
                if task.action != "navigate":
                    return TaskResult(task.id, False, f"unknown nav action: {task.action}")
                success = self._robot.navigate(task.target, timeout=task.params.get("timeout", 120.0))
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
                return TaskResult(task.id, False, "vision result is stale", data=data)
            if isinstance(data, dict) and not data.get("found", True):
                return TaskResult(task.id, False, f"no target found: {task.target}", data=data)
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
        if task.action == "infer":
            if self._llm is None:
                return TaskResult(task.id, False, "no LLM for infer action")
            plan = self._llm.generate_action(task.target)
            if plan.action == "vision" or plan.action == "approach":
                return TaskResult(task.id, False, "infer resolved to non-motion action")
            success = self._robot.execute_action(plan.to_command_dict())
            message = plan.response or plan.action if success else f"{plan.action} was rejected"
            return TaskResult(task.id, bool(success), message, data={"action": plan.to_command_dict()})

        if task.action == "navigate":
            success = self._robot.navigate(task.target, timeout=task.params.get("timeout", 120.0))
            return TaskResult(task.id, bool(success), "navigation completed" if success else "navigation failed")

        if task.action in ("move", "rotate", "stop"):
            if task.action == "stop":
                success = self._robot.stop()
                return TaskResult(task.id, bool(success), "stopped" if success else "stop was rejected")
            if task.action == "rotate":
                success = self._robot.rotate(
                    float(task.params.get("angular_z", 0.5)),
                    float(task.params.get("duration", 1.0)),
                )
                return TaskResult(task.id, bool(success), "rotated" if success else "rotation was rejected")
            # move: use vision context when available, else explicit params
            if self._llm is not None and context.get("vision") is not None:
                motion = self._llm.plan_motion(context.get("goal", task.target), context["vision"])
                if motion.action == "stop":
                    return TaskResult(task.id, False, "cannot plan motion from vision")
                success = self._robot.execute_action(motion.to_command_dict())
                message = motion.response or "moved" if success else "motion was rejected"
                return TaskResult(task.id, bool(success), message, data={"action": motion.to_command_dict()})
            success = self._robot.move(
                float(task.params.get("linear_x", 0.3)),
                float(task.params.get("linear_y", 0.0)),
                float(task.params.get("angular_z", 0.0)),
                float(task.params.get("duration", 1.0)),
            )
            return TaskResult(task.id, bool(success), "moved" if success else "move was rejected")

        return TaskResult(task.id, False, f"unknown robot action: {task.action}")
