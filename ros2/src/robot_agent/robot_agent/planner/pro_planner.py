"""ProPlanner: provider-agnostic high-level planner backed by a BaseLLM."""

from __future__ import annotations

from typing import Any

from robot_agent.executor.base_executor import TaskResult
from robot_agent.llm import BaseLLM
from robot_agent.planner.base_planner import BasePlanner
from robot_agent.planner.task_schema import Plan, Task


class ProPlanner(BasePlanner):
    """High-level planner that asks a (pro) LLM to decompose the goal.

    The concrete model is supplied by the configured LLM provider; the planner
    itself never touches ROS2 or the tools.
    """

    def __init__(self, llm: BaseLLM) -> None:
        self._llm = llm

    def plan(self, goal: str) -> Plan:
        return self._llm.plan_goal(goal)

    def replan(self, goal: str, task: Task, result: TaskResult) -> Plan:
        # Generic recovery: expand the search and re-observe before retrying.
        nid = task.id * 100 + 1
        return Plan(
            goal=f"recover:{goal}",
            tasks=[
                Task(
                    id=nid,
                    tool="robot",
                    action="rotate",
                    target="",
                    name="expand search (rotate)",
                    importance=80,
                    urgency=70,
                    complexity=40,
                    dependency=task.dependency,
                    params={"angular_z": 0.5, "duration": 1.0},
                    depends_on=[],
                ),
                Task(
                    id=nid + 1,
                    tool="vision",
                    action="observe",
                    target=task.target,
                    name="re-observe environment",
                    importance=90,
                    urgency=80,
                    complexity=60,
                    dependency=task.dependency,
                    depends_on=[nid],
                ),
            ],
        )
