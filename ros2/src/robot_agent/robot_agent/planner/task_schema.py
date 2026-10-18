"""Structured task / plan schemas shared by planner, manager and executor."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import math
from typing import Any, Dict, List


class PlanValidationError(ValueError):
    """Raised when a planner returns a malformed or unsafe task graph."""


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
        if not isinstance(data, dict):
            raise PlanValidationError("task must be an object")
        allowed = {
            "id", "tool", "action", "target", "name", "importance", "urgency",
            "complexity", "dependency", "status", "params", "depends_on",
        }
        extra = set(data) - allowed
        if extra:
            raise PlanValidationError(f"unknown task fields: {sorted(extra)}")
        for required in ("id", "tool", "action"):
            if required not in data:
                raise PlanValidationError(f"task missing required field: {required}")
        for key in ("id",):
            if isinstance(data[key], bool) or not isinstance(data[key], int):
                raise PlanValidationError(f"task {key} must be an integer")
        for key in ("tool", "action", "target", "name", "status"):
            if key in data and not isinstance(data[key], str):
                raise PlanValidationError(f"task {key} must be a string")
        for key in ("importance", "urgency", "complexity", "dependency"):
            if key in data and (isinstance(data[key], bool) or
                                not isinstance(data[key], (int, float))):
                raise PlanValidationError(f"task {key} must be numeric")
        if "params" in data and not isinstance(data["params"], dict):
            raise PlanValidationError("task params must be an object")
        if "depends_on" in data and not isinstance(data["depends_on"], list):
            raise PlanValidationError("task depends_on must be an array")
        if any(
            isinstance(value, bool) or not isinstance(value, int)
            for value in data.get("depends_on", [])
        ):
            raise PlanValidationError("task dependency ids must be integers")
        return cls(
            id=data["id"],
            tool=data["tool"],
            action=data["action"],
            target=data.get("target", ""),
            name=data.get("name", ""),
            importance=float(data.get("importance", 50.0)),
            urgency=float(data.get("urgency", 50.0)),
            complexity=float(data.get("complexity", 50.0)),
            dependency=float(data.get("dependency", 0.0)),
            status=data.get("status", "pending"),
            params=dict(data.get("params") or {}),
            depends_on=list(data.get("depends_on", [])),
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
        if not isinstance(data, dict):
            raise PlanValidationError("plan must be an object")
        if set(data) - {"goal", "tasks"}:
            raise PlanValidationError(f"unknown plan fields: {sorted(set(data) - {'goal', 'tasks'})}")
        if not isinstance(data.get("goal"), str):
            raise PlanValidationError("plan goal must be a string")
        if not isinstance(data.get("tasks"), list):
            raise PlanValidationError("plan tasks must be an array")
        plan = cls(goal=data["goal"], tasks=[Task.from_dict(t) for t in data["tasks"]])
        validate_plan(plan)
        return plan


def _finite_tree(value: Any) -> bool:
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return True
    if isinstance(value, (int, float)):
        return math.isfinite(value)
    if isinstance(value, list):
        return all(_finite_tree(item) for item in value)
    if isinstance(value, dict):
        return all(isinstance(key, str) and _finite_tree(item) for key, item in value.items())
    return False


def validate_plan(plan: Plan) -> None:
    """Validate a plan before it can enter TaskManager or reach an executor."""
    if not isinstance(plan, Plan) or not isinstance(plan.goal, str) or not plan.goal.strip():
        raise PlanValidationError("plan goal must be a non-empty string")
    if not isinstance(plan.tasks, list) or not 1 <= len(plan.tasks) <= 64:
        raise PlanValidationError("plan must contain between 1 and 64 tasks")
    allowed = {
        "nav": {"navigate"},
        "vision": {"find_object", "find", "observe", "scene", "get_objects"},
        "robot": {"move", "rotate", "stop", "approach", "infer"},
    }
    ids = []
    by_id = {}
    for task in plan.tasks:
        if not isinstance(task, Task):
            raise PlanValidationError("plan tasks must be Task objects")
        if isinstance(task.id, bool) or not isinstance(task.id, int) or task.id <= 0:
            raise PlanValidationError("task ids must be positive integers")
        if task.id in by_id:
            raise PlanValidationError(f"duplicate task id: {task.id}")
        if task.tool not in allowed or task.action not in allowed[task.tool]:
            raise PlanValidationError(f"unsupported tool/action pair: {task.tool}/{task.action}")
        if not isinstance(task.target, str) or not isinstance(task.name, str):
            raise PlanValidationError(f"task {task.id} target/name must be strings")
        if task.action in ("find_object", "find", "approach") and not task.target.strip():
            raise PlanValidationError(f"task {task.id} requires a non-empty target")
        if not isinstance(task.params, dict) or not _finite_tree(task.params):
            raise PlanValidationError(f"task {task.id} params must contain finite JSON values")
        if task.status != "pending":
            raise PlanValidationError(f"task {task.id} must start pending")
        for field_name in ("importance", "urgency", "complexity", "dependency"):
            value = getattr(task, field_name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise PlanValidationError(f"task {task.id} {field_name} must be finite")
            if not 0 <= value <= 100:
                raise PlanValidationError(f"task {task.id} {field_name} must be between 0 and 100")
        if not isinstance(task.depends_on, list) or any(
            isinstance(dep, bool) or not isinstance(dep, int) for dep in task.depends_on
        ):
            raise PlanValidationError(f"task {task.id} depends_on must contain integer ids")
        if len(set(task.depends_on)) != len(task.depends_on) or task.id in task.depends_on:
            raise PlanValidationError(f"task {task.id} has duplicate or self dependencies")
        if task.action in ("move", "rotate") and "duration" in task.params:
            duration = task.params["duration"]
            if (isinstance(duration, bool) or not isinstance(duration, (int, float))
                    or not math.isfinite(duration) or not 0 < duration <= 30):
                raise PlanValidationError(f"task {task.id} has unsafe motion duration")
        ids.append(task.id)
        by_id[task.id] = task
    known = set(ids)
    for task in plan.tasks:
        missing = set(task.depends_on) - known
        if missing:
            raise PlanValidationError(f"task {task.id} depends on missing tasks: {sorted(missing)}")
    visiting = set()
    visited = set()

    def visit(task_id: int) -> None:
        if task_id in visiting:
            raise PlanValidationError("plan dependencies contain a cycle")
        if task_id in visited:
            return
        visiting.add(task_id)
        for dependency in by_id[task_id].depends_on:
            visit(dependency)
        visiting.remove(task_id)
        visited.add(task_id)

    for task_id in ids:
        visit(task_id)
