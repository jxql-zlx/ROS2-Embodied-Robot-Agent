"""Structured task / plan schemas shared by planner, manager and executor."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List


@dataclass
class Task:
    """One low-level unit of work dispatched to a tool."""

    id: int
    tool: str  # "vision" | "robot" | "nav"
    action: str
    target: str = ""
    name: str = ""
    importance: float = 50.0
    urgency: float = 50.0
    complexity: float = 50.0
    dependency: float = 0.0
    status: str = "pending"
    params: Dict[str, Any] = field(default_factory=dict)
    depends_on: List[int] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Task":
        return cls(
            id=int(data.get("id", 0)),
            tool=str(data.get("tool", "")),
            action=str(data.get("action", "")),
            target=str(data.get("target", "")),
            name=str(data.get("name", "")),
            importance=float(data.get("importance", 50.0)),
            urgency=float(data.get("urgency", 50.0)),
            complexity=float(data.get("complexity", 50.0)),
            dependency=float(data.get("dependency", 0.0)),
            status=str(data.get("status", "pending")),
            params=dict(data.get("params") or {}),
            depends_on=[int(task_id) for task_id in data.get("depends_on", [])],
        )


@dataclass
class Plan:
    """A high-level goal decomposed into an ordered list of Tasks."""

    goal: str
    tasks: List[Task] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {"goal": self.goal, "tasks": [t.to_dict() for t in self.tasks]}

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Plan":
        return cls(
            goal=str(data.get("goal", "")),
            tasks=[Task.from_dict(t) for t in data.get("tasks", [])],
        )
