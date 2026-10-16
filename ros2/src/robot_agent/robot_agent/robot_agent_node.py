"""robot_agent node: NL task -> LLM -> structured action -> Ros2RobotTool.

Inputs:
  - topic   /task_input   (std_msgs/String)
  - service /task_execute (robot_interfaces/TaskExecute)
Outputs:
  - topic   /robot_command (RobotCommand, via Ros2RobotTool)
"""

from __future__ import annotations

import json
from typing import Any, Dict, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_interfaces.srv import TaskExecute
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
        self.declare_parameter("llm_config", "")
        self.declare_parameter("llm_provider", "")  # optional override of config
        self.declare_parameter("tool_command_topic", "/robot_command")
        self.declare_parameter("tool_status_topic", "/robot_status")
        self.declare_parameter("tool_odom_topic", "/odom")
        self.declare_parameter("vision_detections_topic", "/vision/detections")
        self.declare_parameter("mode", "simple")
        self.declare_parameter("agent_config", "")
        self.declare_parameter("approach_stop_distance", 0.65)

        task_input_topic = self.get_parameter("task_input_topic").value
        task_service = self.get_parameter("task_service").value

        self._llm = llm if llm is not None else self._make_llm()
        self._tool = tool if tool is not None else Ros2RobotTool(
            command_topic=self.get_parameter("tool_command_topic").value,
            status_topic=self.get_parameter("tool_status_topic").value,
            odom_topic=self.get_parameter("tool_odom_topic").value,
        )
        self._vision_tool = vision_tool if vision_tool is not None else Ros2VisionTool(
            detections_topic=self.get_parameter("vision_detections_topic").value
        )
        self._mode = str(self.get_parameter("mode").value).lower()
        self._hierarchical = hierarchical

        self._task_sub = self.create_subscription(
            String, task_input_topic, self._on_task_input, 10
        )
        self._task_srv = self.create_service(
            TaskExecute, task_service, self._on_task_service
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

    def handle_task(self, task: str) -> tuple[bool, str, str]:
        """Run one NL task. Returns (success, response, action_json)."""
        try:
            if self._mode == "hierarchical" or self._hierarchical is not None:
                return self._handle_hierarchical(task)
            plan = self._llm.generate_action(task)
            if plan.action == "vision":
                return self._handle_vision(plan, task)
            if plan.action == "approach":
                return self._handle_approach(plan, task)
            action_json = json.dumps(plan.to_command_dict(), ensure_ascii=False)
            ok = self._tool.execute_action(plan.to_command_dict())
            if ok:
                self.get_logger().info(f"task='{task}' -> {action_json}")
                response = (
                    "导航已到达目标点" if plan.action == "navigate"
                    else plan.response or f"已执行动作：{plan.action}"
                )
                return True, response, action_json
            return False, "执行层受理失败", action_json
        except LLMError as exc:
            self.get_logger().warn(f"LLM error for task '{task}': {exc}")
            return False, str(exc), ""
        except Exception as exc:  # keep the node alive on unexpected errors
            self.get_logger().error(f"unexpected error for task '{task}': {exc}")
            return False, f"内部错误：{exc}", ""

    def _handle_hierarchical(self, task: str) -> tuple[bool, str, str]:
        agent = self._hierarchical or self._build_hierarchical()
        try:
            results = agent.run(task)
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
        return build_hierarchical_agent(
            config, robot_tool=self._tool, vision_tool=self._vision_tool
        )

    def _handle_vision(self, plan: ActionPlan, task: str) -> tuple[bool, str, str]:
        goal = (plan.goal or "").strip()
        try:
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

    def _handle_approach(self, plan: ActionPlan, task: str) -> tuple[bool, str, str]:
        return ApproachController(
            self._tool, self._vision_tool, self._llm,
            stop_distance=self.get_parameter("approach_stop_distance").value,
            logger=self.get_logger(),
        ).run(plan.goal, task)

    def _on_task_input(self, msg: String) -> None:
        success, response, _ = self.handle_task(msg.data)
        self.get_logger().info(f"[task done] success={success} response={response}")

    def _on_task_service(self, request, response) -> TaskExecute.Response:
        response.success, response.response, response.action_json = self.handle_task(request.task)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    agent = RobotAgentNode()
    executor = MultiThreadedExecutor()
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
