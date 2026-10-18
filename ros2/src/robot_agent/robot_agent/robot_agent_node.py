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
import time
from collections import OrderedDict
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
        # Serialize command publication independently from task admission and
        # status waits. Stop publication must not hold _task_guard while a
        # publisher is slow or blocked.
        self._command_guard = threading.Lock()
        self._cancel_guard = threading.Lock()
        self._active_cancel_event: Optional[threading.Event] = None
        self._active_task_done: Optional[threading.Event] = None
        self._active_task_id: Optional[str] = None
        self._task_generation = 0
        self._navigation_unresolved = False
        self._cancellation_pending = False
        self._cancellation_lease = 0
        self._cancellation_owner_task_id: Optional[str] = None
        self._cancellation_owner_generation: Optional[int] = None
        self._cancellation_stop_confirmed = False
        self._cancellation_explicit = False
        self._task_id_history: OrderedDict[str, tuple[str, float]] = OrderedDict()
        self._last_stop_failure_reason = ""
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

    def _prune_task_id_history(self) -> None:
        now = time.monotonic()
        while self._task_id_history:
            _key, (_state, expiry) = next(iter(self._task_id_history.items()))
            if expiry > now and len(self._task_id_history) <= 256:
                break
            self._task_id_history.popitem(last=False)

    def _remember_task_id(self, task_id: str, state: str) -> None:
        if not task_id:
            return
        self._prune_task_id_history()
        self._task_id_history.pop(task_id, None)
        self._task_id_history[task_id] = (state, time.monotonic() + 600.0)
        self._prune_task_id_history()

    def _release_cancellation_lease(self, lease: int) -> None:
        with self._task_guard:
            if (self._cancellation_pending and self._cancellation_lease == lease
                    and self._cancellation_stop_confirmed
                    and (not self._cancellation_explicit or self._active_cancel_event is None)
                    and not self._navigation_unresolved):
                owner = self._cancellation_owner_task_id
                if owner:
                    previous = self._task_id_history.get(owner, ("", 0.0))[0]
                    if previous != "cancelled_before_start":
                        self._remember_task_id(owner, "cancelled")
                self._cancellation_pending = False
                self._cancellation_owner_task_id = None
                self._cancellation_owner_generation = None
                self._cancellation_stop_confirmed = False
                self._cancellation_explicit = False

    def _cancel_active_task(self, task_id: str = "") -> tuple[bool, str, str]:
        # Install the admission lease before waiting for the cancellation
        # serializer. This prevents new work entering while a cancel queues.
        deadline = time.monotonic() + 12.0
        with self._task_guard:
            self._prune_task_id_history()
            current_state = self._task_id_history.get(task_id, ("", 0.0))[0] if task_id else ""
            active_match = bool(task_id and self._active_task_id == task_id)
            pending_match = bool(
                task_id and self._cancellation_pending
                and self._cancellation_owner_task_id == task_id
            )
            if task_id and not active_match and not pending_match:
                if current_state == "cancelled_before_start":
                    return True, "该任务已取消且未执行", ""
                if current_state in ("completed", "cancelled"):
                    return True, "取消请求对应的任务已结束", ""
                # A cancel may arrive before its execute request. Keep a bounded
                # tombstone so a late request with this UUID cannot execute.
                # Scoped cancellation never sends a global stop for an unknown
                # UUID, which could otherwise interrupt an unrelated task.
                self._remember_task_id(task_id, "cancelled_before_start")
                return True, "任务取消已登记；迟到的执行请求将被拒绝", ""
            elif self._cancellation_pending and self._cancellation_explicit:
                requested_generation = self._cancellation_owner_generation
                lease = self._cancellation_lease
                if self._cancellation_stop_confirmed:
                    # A duplicate cancel for the same lease is idempotent.
                    # The execution thread may still be unwinding, so leave
                    # admission closed and avoid publishing another stop.
                    return True, "停止已收到控制层确认；任务执行线程仍在退出", json.dumps({
                        "action": "stop",
                        "stop_confirmation_type": "control_ack",
                        "physical_stop_confirmed": False,
                        "task_execution_converged": self._active_cancel_event is None,
                    }, ensure_ascii=False)
            elif self._cancellation_pending:
                # Promote a stop initiated by an ordinary motion/approach
                # failure into an explicit cancellation lease. The earlier
                # waiter must no longer be able to release this admission gate.
                requested_generation = self._task_generation
                self._cancellation_lease += 1
                lease = self._cancellation_lease
                self._cancellation_owner_task_id = task_id or self._active_task_id
                self._cancellation_owner_generation = requested_generation
                self._cancellation_stop_confirmed = False
                self._cancellation_explicit = True
            else:
                requested_generation = self._task_generation
                self._cancellation_lease += 1
                lease = self._cancellation_lease
                self._cancellation_pending = True
                self._cancellation_owner_task_id = task_id or self._active_task_id
                self._cancellation_owner_generation = requested_generation
                self._cancellation_stop_confirmed = False
                self._cancellation_explicit = True

            if requested_generation is None:
                requested_generation = self._task_generation
        # Revalidate after serialization. The same lease can be retried after
        # the task's finally block has cleared its active UUID.
        remaining = max(0.0, deadline - time.monotonic())
        if not self._cancel_guard.acquire(timeout=remaining):
            return False, "取消等待超时，停止状态未知；新任务暂不可执行", ""
        try:
            return self._cancel_active_task_serialized(
                requested_generation, task_id, lease, deadline
            )
        finally:
            self._cancel_guard.release()

    def _cancel_active_task_serialized(
        self, requested_generation: int, task_id: str = "", lease: Optional[int] = None,
        deadline: Optional[float] = None,
    ) -> tuple[bool, str, str]:
        # Serialize cancellation with command submission. Keep the active-task
        # lease until its execution thread observes cancellation and exits.
        self.get_logger().info("cancel requested")
        try:
            with self._task_guard:
                if (lease is None or self._cancellation_lease != lease
                        or not self._cancellation_pending
                        or self._task_generation != requested_generation
                        or (task_id and self._active_task_id != task_id
                            and self._cancellation_owner_task_id != task_id)):
                    return True, "取消请求对应的任务已结束", ""
                event = self._active_cancel_event
                task_done = self._active_task_done
                navigation_unresolved = self._navigation_unresolved
                navigation_was_unresolved = navigation_unresolved
                # Retain admission until a task-correlated terminal stop is
                # confirmed. The confirmation wait happens after this lock.
                if event is not None:
                    event.set()
            self.get_logger().info(
                f"cancel signaled: task_active={event is not None}, "
                f"navigation_unresolved={navigation_unresolved}"
            )
        except Exception as exc:
            self.get_logger().error(f"cancel setup failed: {exc}")
            with self._task_guard:
                self._cancellation_pending = True
            return False, f"停止状态未确认：{exc}", ""
        if deadline is None:
            deadline = time.monotonic() + 12.0
        if time.monotonic() >= deadline:
            return False, "取消请求已超时，停止状态未知；新任务暂不可执行", ""
        if not self._stop_safely("for task cancellation", release_pending=False,
                                 timeout_sec=max(0.0, deadline - time.monotonic())):
            reason = self._last_stop_failure_reason or "停止状态未确认"
            return False, f"{reason}；新任务暂不可执行", ""
        if navigation_unresolved:
            if task_done is None:
                return False, "Nav2 尚未确认取消；新任务暂不可执行", ""
            if not task_done.wait(max(0.0, deadline - time.monotonic())):
                self.get_logger().warn("cancel wait timed out before task released navigation")
                return False, "已发送停止，但 Nav2 尚未确认取消；新任务暂不可执行", ""
            with self._task_guard:
                navigation_unresolved = self._navigation_unresolved
            if navigation_unresolved:
                self.get_logger().warn("Nav2 cancellation remains unconfirmed")
                return False, "Nav2 尚未确认取消；新任务暂不可执行", ""
            if not self._stop_safely(
                    "after Nav2 reached a terminal state", release_pending=False,
                    timeout_sec=max(0.0, deadline - time.monotonic())):
                reason = self._last_stop_failure_reason or "最终停止状态未确认"
                return False, f"Nav2 已结束，但{reason}", ""
        with self._task_guard:
            if self._cancellation_lease == lease:
                self._cancellation_stop_confirmed = True
        self._release_cancellation_lease(lease)
        response = "导航已取消并收到控制层停止确认" if navigation_was_unresolved else (
            "任务已取消并收到控制层停止确认" if event is not None
            else "停止指令已收到控制层确认"
        )
        stop_result = {
            "action": "stop",
            "stop_confirmation_type": "control_ack",
            "physical_stop_confirmed": False,
        }
        if navigation_was_unresolved:
            stop_result["nav2_terminal_confirmed"] = True
        return True, response, json.dumps(stop_result, ensure_ascii=False)

    def _stop_safely(
        self, context: str, *, release_pending: bool = True,
        timeout_sec: float = 2.0,
    ) -> bool:
        """Send a fresh stop and confirm its correlated terminal status.

        Any missing, rejected, timed out, or errored confirmation keeps task
        admission closed. A later explicit stop/cancel request can retry it.
        """
        before_stamp = None
        command_id = None
        owns_lease = False
        lease = None
        self._last_stop_failure_reason = ""
        acquired_command_guard = False
        try:
            command_timeout = max(0.0, timeout_sec)
            acquired_command_guard = self._command_guard.acquire(timeout=command_timeout)
            if not acquired_command_guard:
                self._last_stop_failure_reason = "停止指令等待命令通道超时"
                with self._task_guard:
                    self._cancellation_pending = True
                return False
            try:
                with self._task_guard:
                    if not self._cancellation_pending:
                        self._cancellation_lease += 1
                        self._cancellation_pending = True
                        self._cancellation_owner_task_id = self._active_task_id
                        self._cancellation_owner_generation = self._task_generation
                        self._cancellation_stop_confirmed = False
                        self._cancellation_explicit = False
                        owns_lease = True
                    lease = self._cancellation_lease
                    stamp = getattr(self._tool, "status_stamp", None)
                    before_stamp = stamp() if callable(stamp) else None
                accepted = bool(self._tool.stop())
                command_id = getattr(self._tool, "latest_command_id", None)
            finally:
                self._command_guard.release()
                acquired_command_guard = False
        except Exception as exc:
            self.get_logger().error(f"stop command raised {context}: {exc}")
            self._last_stop_failure_reason = f"停止指令失败：{exc}"
            with self._task_guard:
                self._cancellation_pending = True
            return False
        if not accepted:
            self.get_logger().error(f"stop command rejected {context}")
            self._last_stop_failure_reason = "停止指令被拒绝"
            return False

        waiter = getattr(self._tool, "wait_for_stop_confirmation", None)
        # At startup the status stream may not have emitted any sample yet.
        # A unique command ID still proves that the subsequent status belongs
        # to this stop, even when there is no timestamp baseline to compare.
        if not callable(waiter) or not command_id:
            self.get_logger().error(
                f"stop feedback unavailable {context}: command_id/confirmation missing"
            )
            self._last_stop_failure_reason = "停止反馈不可用"
            return False
        try:
            confirmed = bool(waiter(
                before_stamp, timeout_sec=max(0.0, timeout_sec), command_id=command_id
            ))
        except Exception as exc:
            self.get_logger().error(f"stop confirmation failed {context}: {exc}")
            self._last_stop_failure_reason = f"停止状态确认失败：{exc}"
            confirmed = False
        if confirmed and owns_lease and release_pending and lease is not None:
            with self._task_guard:
                if self._cancellation_lease == lease:
                    self._cancellation_stop_confirmed = True
            self._release_cancellation_lease(lease)
        if not confirmed:
            self.get_logger().error(f"stop status was not confirmed {context}")
            if not self._last_stop_failure_reason:
                self._last_stop_failure_reason = "停止状态未确认"
        return confirmed

    def _dispatch_action(
        self, event: threading.Event, action: Dict[str, Any], wait_for_completion: bool = True
    ) -> bool:
        if action.get("action") == "navigate" and callable(getattr(self._tool, "navigate", None)):
            params = action.get("params")
            timeout_value = action.get("timeout")
            if timeout_value is None and isinstance(params, dict):
                timeout_value = params.get("timeout")
            return self._dispatch_navigation(
                event,
                str(action.get("goal", "")),
                float(120.0 if timeout_value is None else timeout_value),
            )
        is_motion = action.get("action") in ("move", "rotate")
        before_stamp = None
        with self._command_guard:
            with self._task_guard:
                if event.is_set() or self._cancellation_pending:
                    return False
                if is_motion:
                    stamp = getattr(self._tool, "status_stamp", None)
                    before_stamp = stamp() if callable(stamp) else None
            accepted = bool(self._tool.execute_action(action))
            command_id = getattr(self._tool, "latest_command_id", None)
        if not accepted:
            return False
        if not is_motion or not wait_for_completion:
            return True

        wait_motion = getattr(self._tool, "wait_for_motion_complete", None)
        if not callable(wait_motion):
            self._stop_safely("motion feedback unavailable")
            return False
        try:
            duration = float(action.get("duration", 0.0))
            completed = wait_motion(
                before_stamp, max(3.0, duration + 2.0), cancel_event=event,
                command_id=command_id,
            )
        except Exception as exc:
            self.get_logger().error(f"motion feedback failed: {exc}")
            completed = False
        if completed:
            return True
        if not event.is_set():
            if not self._stop_safely("motion failed or timed out"):
                self.get_logger().error("motion failed and stop remains unconfirmed; admission closed")
        return False

    def _dispatch_navigation(self, event: threading.Event, goal: str, timeout: float) -> bool:
        if event.is_set():
            return False
        return bool(self._tool.navigate(goal, timeout=timeout, cancel_event=event,
                                        command_guard=self._command_guard,
                                        navigation_state_callback=self._set_navigation_unresolved))

    def _set_navigation_unresolved(self, active: bool) -> None:
        with self._task_guard:
            self._navigation_unresolved = bool(active)
            needs_retry = not active and self._cancellation_pending
        if needs_retry:
            self.get_logger().warn(
                "Nav2 reached a terminal state; cancellation stop must be explicitly retried"
            )

    def handle_task(self, task: str, task_id: str = "") -> tuple[bool, str, str]:
        """Run one NL task. Returns (success, response, action_json)."""
        if self._is_stop_request(task):
            # TaskExecute.task_id identifies this stop request itself. Scoped
            # cancellation targets are carried only by CancelTask.task_id.
            return self._cancel_active_task()
        event = threading.Event()
        task_done = threading.Event()
        with self._task_guard:
            if (self._active_cancel_event is not None or self._navigation_unresolved or
                    self._cancellation_pending):
                return False, "已有任务正在执行", ""
            if task_id:
                self._prune_task_id_history()
                previous_state = self._task_id_history.get(task_id, ("", 0.0))[0]
                if previous_state == "cancelled_before_start":
                    return False, "该任务已在取消请求后到达，未执行；请使用新 task_id 重试", ""
                if previous_state:
                    return False, "task_id 已使用；请使用新的 task_id 提交任务", ""
            self._task_generation += 1
            self._active_task_id = task_id or f"task-{self._task_generation}"
            self._remember_task_id(self._active_task_id, "active")
            self._active_cancel_event = event
            self._active_task_done = task_done
        try:
            result = self._run_task(task, event)
            if event.is_set():
                return False, "任务已取消", result[2]
            return result
        finally:
            with self._task_guard:
                task_id = self._active_task_id
                if self._active_cancel_event is event:
                    self._active_cancel_event = None
                    self._active_task_done = None
                    self._active_task_id = None
                    if task_id and self._cancellation_owner_task_id != task_id:
                        self._remember_task_id(task_id, "completed")
                task_done.set()
                lease = self._cancellation_lease
            self._release_cancellation_lease(lease)

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
                "_execute_action": lambda action: self._dispatch_action(
                    cancel_event, action, wait_for_completion=True
                ),
                "_execute_approach_action": lambda action: self._dispatch_action(
                    cancel_event, action, wait_for_completion=True
                ),
                "_stop_and_confirm": lambda context="approach": self._stop_safely(
                    f"{context}"
                ),
                "_navigate": lambda goal, timeout: self._dispatch_navigation(cancel_event, goal, timeout),
                "_cancel_task": self._cancel_active_task,
            }
            results = agent.run(task, context=context, cancel_event=cancel_event)
            if not results:
                return False, "无任务", ""
            success = all(r.success for r in results)
            response = "；".join(r.message for r in results if r.message) or "任务完成"
            action_json = json.dumps(
                [{
                    "task_id": r.task_id,
                    "task_tool": r.data.get("task_tool", ""),
                    "task_action": r.data.get("task_action", ""),
                    "target": r.data.get("target", ""),
                    "success": r.success,
                    "data": r.data,
                } for r in results],
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
            stop_and_confirm=(None if cancel_event is None else
                              lambda context="approach": self._stop_safely(context)),
            motion_wait_managed=cancel_event is not None,
        ).run(plan.goal, task)

    def _on_task_input(self, msg: String) -> None:
        success, response, _ = self.handle_task(msg.data)
        self.get_logger().info(f"[task done] success={success} response={response}")

    def _on_task_service(self, request, response) -> TaskExecute.Response:
        response.success, response.response, response.action_json = self.handle_task(
            request.task, request.task_id
        )
        return response

    def _on_cancel_service(self, request, response) -> CancelTask.Response:
        response.success, response.response, _ = self._cancel_active_task(request.task_id)
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
