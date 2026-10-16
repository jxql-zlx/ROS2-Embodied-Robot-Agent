"""HierarchicalAgent: orchestrates ProPlanner -> TaskManager -> FlashExecutor."""

from __future__ import annotations

from typing import Any, Dict, List

from robot_agent.executor.base_executor import BaseExecutor, TaskResult
from robot_agent.executor.flash_executor import FlashExecutor
from robot_agent.llm import ChatCompletionsLLM, MockLLM
from robot_agent.planner.base_planner import BasePlanner
from robot_agent.planner.pro_planner import ProPlanner
from robot_agent.planner.task_schema import Task
from robot_agent.task_manager.task_manager import TaskManager
from robot_agent.task_manager.task_state import TaskStatus


_COMPLEX_MARKERS = (
    "找", "find", "search", "导航", "navigate", "避障", "obstacle",
    "规划", "plan", "然后", "then", "并且", "再", "巡逻", "探索",
    "explore", "返回", "return", "靠近", "approach",
)
_SIMPLE_MARKERS = (
    "停止", "停下", "前进", "后退", "左转", "右转", "状态", "查询",
    "stop", "move", "forward", "backward", "rotate", "turn", "status",
)


def is_simple(goal: str) -> bool:
    """Heuristic complexity gate: multi-step goals go to the planner."""
    text = (goal or "").lower()
    if any(marker in text for marker in _COMPLEX_MARKERS):
        return False
    if any(marker in text for marker in _SIMPLE_MARKERS):
        return True
    return True  # default to fast path


class HierarchicalAgent:
    def __init__(
        self,
        planner: BasePlanner,
        executor: BaseExecutor,
        task_manager: TaskManager | None = None,
        max_steps: int = 20,
        max_recoveries: int = 3,
    ) -> None:
        self._planner = planner
        self._executor = executor
        self._task_manager = task_manager or TaskManager()
        self._max_steps = max_steps
        self._max_recoveries = max(0, max_recoveries)

    def run(self, goal: str) -> List[TaskResult]:
        if is_simple(goal):
            task = Task(id=0, tool="robot", action="infer", target=goal, name=goal)
            return [self._executor.execute(task, {"goal": goal})]

        plan = self._planner.plan(goal)
        self._task_manager.add_plan(plan)
        results: List[TaskResult] = []
        context = {"goal": goal}
        recoveries = 0
        steps_taken = 0

        for _ in range(self._max_steps):
            task = self._task_manager.next_task()
            if task is None:
                break
            self._task_manager.set_status(task.id, TaskStatus.RUNNING)
            result = self._executor.execute(task, context)
            steps_taken += 1
            results.append(result)
            if result.success:
                self._task_manager.mark_completed(task.id)
                context.update(result.data or {})
            else:
                self._task_manager.mark_failed(task.id)
                if recoveries >= self._max_recoveries:
                    results.append(
                        TaskResult(
                            task.id,
                            False,
                            f"recovery limit reached ({self._max_recoveries})",
                        )
                    )
                    continue
                recovery = self._planner.replan(goal, task, result)
                self._task_manager.add_plan(recovery)
                recoveries += 1

        pending = [
            task for task in self._task_manager.tasks()
            if task.status == TaskStatus.PENDING.value
        ]
        if pending:
            if steps_taken >= self._max_steps:
                message = f"max steps reached ({self._max_steps}) with pending tasks"
            else:
                message = "pending tasks are blocked by unresolved dependencies"
            results.append(
                TaskResult(-1, False, message, data={"pending_task_ids": [t.id for t in pending]})
            )
        return results


def build_hierarchical_agent(
    config: Dict[str, Any] | None = None,
    robot_tool: Any = None,
    vision_tool: Any = None,
) -> HierarchicalAgent:
    """Build a hierarchical agent from a config dict (defaults to offline mock)."""
    config = config or {}
    provider = str(config.get("provider", "mock")).lower()

    def _make_llm(section: str):
        section_cfg = config.get(section) or {}
        if provider in ("deepseek", "openai"):
            return ChatCompletionsLLM(
                model=str(section_cfg.get("model", "deepseek-chat")),
                base_url=str(section_cfg.get("base_url", "https://api.deepseek.com/v1")),
                api_key_env=str(section_cfg.get("api_key_env", "DEEPSEEK_API_KEY")),
                temperature=0.0,
                timeout=20.0,
            )
        return MockLLM()

    planner = ProPlanner(_make_llm("planner"))
    executor = FlashExecutor(robot_tool=robot_tool, vision_tool=vision_tool, llm=_make_llm("executor"))
    return HierarchicalAgent(planner=planner, executor=executor)
