"""robot_agent node: NL task -> LLM -> structured action -> Ros2RobotTool.

Inputs:
  - topic   /task_input   (std_msgs/String)
  - service /task_execute (robot_interfaces/TaskExecute)
Outputs:
  - topic   /robot_command (RobotCommand, via Ros2RobotTool)
"""

from __future__ import annotations

import json
import threading
from typing import Any, Dict, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_interfaces.srv import CancelTask, TaskExecute
from std_msgs.msg import String

from robot_agent.action_plan import ActionPlan
from robot_agent.approach import ApproachController
from robot_agent.hierarchical_agent import HierarchicalAgent, build_hierarchical_agent
from robot_agent.llm import BaseLLM, LLMError, load_llm
from robot_agent.vision_response import format_vision_result
from robot_tools.ros2_robot_tool import Ros2RobotTool
from robot_tools.ros2_vision_tool import Ros2VisionTool



class RobotAgentNode(Node):
    def __init__(
        self,
        llm: Optional[BaseLLM] = None,
        tool: Optional[Ros2RobotTool] = None,
        vision_tool: Optional[Ros2VisionTool] = None,
        hierarchical: Optional[HierarchicalAgent] = None,
    ) -> None:
        super().__init__("robot_agent")

        self.declare_parameter("task_input_topic", "/task_input")
        self.declare_parameter("task_service", "/task_execute")
        self.declare_parameter("cancel_service", "/task_cancel")
        self.declare_parameter("llm_config", "")
        self.declare_parameter("llm_provider", "")  # optional override of config
        self.declare_parameter("tool_command_topic", "/robot_command")
        self.declare_parameter("tool_status_topic", "/robot_status")
        self.declare_parameter("tool_odom_topic", "/odom")
        self.declare_parameter("tool_scan_topic", "/scan")
        self.declare_parameter("vision_detections_topic", "/vision/detections")
        self.declare_parameter("mode", "simple")
        self.declare_parameter("agent_config", "")
        self.declare_parameter("approach_stop_distance", 0.65)
        self.declare_parameter("approach_max_steps", 8)

        task_input_topic = self.get_parameter("task_input_topic").value
        task_service = self.get_parameter("task_service").value

        self._llm = llm if llm is not None else self._make_llm()
        self._tool = tool if tool is not None else Ros2RobotTool(
            command_topic=self.get_parameter("tool_command_topic").value,
            status_topic=self.get_parameter("tool_status_topic").value,
            odom_topic=self.get_parameter("tool_odom_topic").value,
            scan_topic=self.get_parameter("tool_scan_topic").value,
        )
        self._vision_tool = vision_tool if vision_tool is not None else Ros2VisionTool(
            detections_topic=self.get_parameter("vision_detections_topic").value
        )
        self._mode = str(self.get_parameter("mode").value).lower()
        self._hierarchical = hierarchical
        self._task_guard = threading.Lock()
        self._active_cancel_event: Optional[threading.Event] = None
        self._task_callback_group = ReentrantCallbackGroup()
        self._cancel_callback_group = ReentrantCallbackGroup()

        self._task_sub = self.create_subscription(
            String, task_input_topic, self._on_task_input, 10,
            callback_group=self._task_callback_group,
        )
        self._task_srv = self.create_service(
            TaskExecute, task_service, self._on_task_service,
            callback_group=self._task_callback_group,
        )
        self._cancel_srv = self.create_service(
            CancelTask, str(self.get_parameter("cancel_service").value),
            self._on_cancel_service, callback_group=self._cancel_callback_group,
        )

        self.get_logger().info(
            f"robot_agent ready: topic={task_input_topic}, service={task_service}, "
            f"llm={type(self._llm).__name__}"
        )

    # ------------------------------------------------------------------ #

    @property
    def tool(self) -> Ros2RobotTool:
        return self._tool

    @property
    def vision_tool(self) -> Ros2VisionTool:
        return self._vision_tool

    def _make_llm(self) -> BaseLLM:
        config_path = str(self.get_parameter("llm_config").value)
        config = {}
        if config_path:
            try:
                import yaml
            except ImportError as exc:  # pragma: no cover
                raise LLMError("PyYAML not installed; cannot read LLM config") from exc
            with open(config_path, "r", encoding="utf-8") as f:
                config = yaml.safe_load(f) or {}
        provider_override = str(self.get_parameter("llm_provider").value)
        if provider_override:
            config["provider"] = provider_override
        return load_llm(config)

    @staticmethod
    def _is_stop_request(task: str) -> bool:
        return (task or "").strip().lower() in {
            "stop", "停止", "停下", "停", "取消", "取消任务", "停止任务",
            "cancel", "cancel task", "abort", "急停",
        }

    def _cancel_active_task(self) -> tuple[bool, str, str]:
        # Serialize cancellation with command submission so any action already
        # being published precedes the stop, and later actions see the event.
        try:
            with self._task_guard:
                event = self._active_cancel_event
                if event is not None:
                    event.set()
                stopped = bool(self._tool.stop())
                if stopped and self._active_cancel_event is event:
                    self._active_cancel_event = None
        except Exception as exc:
            self.get_logger().error(f"stop command failed: {exc}")
            return False, f"停止指令发送失败：{exc}", ""
        if not stopped:
            return False, "停止指令发送失败", ""
        response = "任务已取消并已发送停止" if event is not None else "已发送停止，当前没有运行任务"
        return True, response, json.dumps({"action": "stop"}, ensure_ascii=False)

    def _dispatch_action(self, event: threading.Event, action: Dict[str, Any]) -> bool:
        with self._task_guard:
            if event.is_set():
                return False
            return bool(self._tool.execute_action(action))

    def _dispatch_navigation(self, event: threading.Event, goal: str, timeout: float) -> bool:
        if event.is_set():
            return False
        return bool(self._tool.navigate(goal, timeout=timeout, cancel_event=event,
                                        command_guard=self._task_guard))

    def handle_task(self, task: str) -> tuple[bool, str, str]:
        """Run one NL task. Returns (success, response, action_json)."""
        if self._is_stop_request(task):
            return self._cancel_active_task()
        event = threading.Event()
        with self._task_guard:
            if self._active_cancel_event is not None:
                return False, "已有任务正在执行", ""
            self._active_cancel_event = event
        try:
            result = self._run_task(task, event)
            if event.is_set():
                return False, "任务已取消", result[2]
            return result
        finally:
            with self._task_guard:
                if self._active_cancel_event is event:
                    self._active_cancel_event = None

    def _run_task(self, task: str, cancel_event: threading.Event) -> tuple[bool, str, str]:
        try:
            if cancel_event.is_set():
                return False, "任务已取消", ""
            if self._mode == "hierarchical" or self._hierarchical is not None:
                return self._handle_hierarchical(task, cancel_event)
            plan = self._llm.generate_action(task)
            if cancel_event.is_set():
                return False, "任务已取消", ""
            if plan.action == "vision":
                return self._handle_vision(plan, task, cancel_event)
            if plan.action == "approach":
                return self._handle_approach(plan, task, cancel_event)
            action_json = json.dumps(plan.to_command_dict(), ensure_ascii=False)
            if plan.action == "navigate" and callable(getattr(self._tool, "navigate", None)):
                ok = self._dispatch_navigation(
                    cancel_event, plan.goal, float(plan.params.get("timeout", 120.0))
                )
            else:
                ok = self._dispatch_action(cancel_event, plan.to_command_dict())
            if ok:
                self.get_logger().info(f"task='{task}' -> {action_json}")
                response = (
                    "导航已到达目标点" if plan.action == "navigate"
                    else plan.response or f"已执行动作：{plan.action}"
                )
                return True, response, action_json
            return False, "执行层受理失败", action_json
        except LLMError as exc:
            if cancel_event.is_set():
                return False, "任务已取消", ""
            self.get_logger().warn(f"LLM error for task '{task}': {exc}")
            return False, str(exc), ""
        except Exception as exc:  # keep the node alive on unexpected errors
            if cancel_event.is_set():
                return False, "任务已取消", ""
            self.get_logger().error(f"unexpected error for task '{task}': {exc}")
            return False, f"内部错误：{exc}", ""

    def _handle_hierarchical(self, task: str, cancel_event: threading.Event) -> tuple[bool, str, str]:
        agent = self._hierarchical or self._build_hierarchical()
        try:
            context = {
                "_cancel_event": cancel_event,
                "_execute_action": lambda action: self._dispatch_action(cancel_event, action),
                "_navigate": lambda goal, timeout: self._dispatch_navigation(cancel_event, goal, timeout),
                "_cancel_task": self._cancel_active_task,
            }
            results = agent.run(task, context=context, cancel_event=cancel_event)
            if not results:
                return False, "无任务", ""
            success = all(r.success for r in results)
            response = "；".join(r.message for r in results if r.message) or "任务完成"
            action_json = json.dumps(
                [{"task_id": r.task_id, "success": r.success, "data": r.data} for r in results],
                ensure_ascii=False,
            )
            return success, response, action_json
        except Exception as exc:
            self.get_logger().error(f"hierarchical error for task '{task}': {exc}")
            return False, f"分层规划错误：{exc}", ""

    def _build_hierarchical(self) -> HierarchicalAgent:
        config: Dict[str, Any] = {}
        path = str(self.get_parameter("agent_config").value)
        if path:
            try:
                import yaml
            except ImportError as exc:
                raise LLMError("PyYAML not installed; cannot read agent config") from exc
            with open(path, "r", encoding="utf-8") as f:
                config = yaml.safe_load(f) or {}
        config["approach_stop_distance"] = self.get_parameter("approach_stop_distance").value
        config["approach_max_steps"] = self.get_parameter("approach_max_steps").value
        return build_hierarchical_agent(
            config, robot_tool=self._tool, vision_tool=self._vision_tool
        )

    def _handle_vision(self, plan: ActionPlan, task: str, cancel_event: threading.Event) -> tuple[bool, str, str]:
        goal = (plan.goal or "").strip()
        try:
            if cancel_event.is_set():
                return False, "任务已取消", ""
            if goal:
                result = self._vision_tool.find_object(goal)
            else:
                result = self._vision_tool.get_objects()
            response = format_vision_result(result, goal)
            action_json = json.dumps(result, ensure_ascii=False)
            return True, response, action_json
        except Exception as exc:  # keep the node alive on unexpected errors
            self.get_logger().error(f"vision error for task '{task}': {exc}")
            return False, f"视觉查询失败：{exc}", ""

    def _handle_approach(
        self, plan: ActionPlan, task: str,
        cancel_event: Optional[threading.Event] = None,
    ) -> tuple[bool, str, str]:
        dispatch = (self._tool.execute_action if cancel_event is None else
                    lambda action: self._dispatch_action(cancel_event, action))
        return ApproachController(
            self._tool, self._vision_tool, self._llm,
            stop_distance=self.get_parameter("approach_stop_distance").value,
            logger=self.get_logger(),
            max_steps=self.get_parameter("approach_max_steps").value,
            cancel_event=cancel_event,
            execute_action=dispatch,
        ).run(plan.goal, task)

    def _on_task_input(self, msg: String) -> None:
        success, response, _ = self.handle_task(msg.data)
        self.get_logger().info(f"[task done] success={success} response={response}")

    def _on_task_service(self, request, response) -> TaskExecute.Response:
        response.success, response.response, response.action_json = self.handle_task(request.task)
        return response

    def _on_cancel_service(self, _request, response) -> CancelTask.Response:
        response.success, response.response, _ = self._cancel_active_task()
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    agent = RobotAgentNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(agent)
    executor.add_node(agent.tool)
    executor.add_node(agent.vision_tool)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        agent.tool.destroy_node()
        agent.vision_tool.destroy_node()
        agent.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
