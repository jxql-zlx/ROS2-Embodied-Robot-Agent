"""Structured action plan produced by the LLM / agent."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Any, Dict

#: Actions the agent may generate. The robot execution layer supports the first
#: four; "vision" only perceives, and "approach" perceives then moves.
ALLOWED_ACTIONS = ("move", "stop", "rotate", "navigate", "vision", "approach")
_ACTION_FIELDS = {
    "action", "linear_x", "linear_y", "angular_z", "duration", "goal", "params", "response",
}
_NUMERIC_FIELDS = ("linear_x", "linear_y", "angular_z", "duration")


def _is_finite(value: int | float) -> bool:
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


@dataclass
class ActionPlan:
    """Validated, normalized action plan."""

    action: str
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0
    duration: float = 0.0
    goal: str = ""
    params: Dict[str, Any] = field(default_factory=dict)
    response: str = ""  # natural-language feedback for the user

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        """Recheck the plan at use sites, including after accidental mutation."""
        if not isinstance(self.action, str):
            raise ValueError("action must be a string")
        self.action = self.action.strip().lower()
        if self.action not in ALLOWED_ACTIONS:
            raise ValueError(f"unsupported action: {self.action!r}")
        if not isinstance(self.goal, str) or not isinstance(self.response, str):
            raise ValueError("goal and response must be strings")
        if not isinstance(self.params, dict):
            raise ValueError("params must be an object")
        if any(not isinstance(key, str) for key in self.params):
            raise ValueError("params keys must be strings")
        for name in _NUMERIC_FIELDS:
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be numeric")
            if not _is_finite(value):
                raise ValueError(f"{name} must be finite")
        if abs(self.linear_x) > 1.0 or abs(self.linear_y) > 1.0:
            raise ValueError("linear speed must be between -1 and 1 m/s")
        if abs(self.angular_z) > 2.0:
            raise ValueError("angular speed must be between -2 and 2 rad/s")
        if self.action in ("move", "rotate"):
            if not 0 < self.duration <= 30.0:
                raise ValueError("move/rotate duration must be positive and at most 30 seconds")
        elif any((self.linear_x, self.linear_y, self.angular_z, self.duration)):
            raise ValueError(f"{self.action} action cannot contain motion values")
        if self.action == "approach" and not self.goal.strip():
            raise ValueError("approach action requires a non-empty goal")
        if self.action == "navigate" and not self.goal.strip():
            raise ValueError("navigate action requires a non-empty goal")
        allowed_params = {"timeout"} if self.action == "navigate" else set()
        extra = set(self.params) - allowed_params
        if extra:
            raise ValueError(f"unknown {self.action} params: {sorted(extra, key=repr)}")
        if "timeout" in self.params:
            timeout = self.params["timeout"]
            if (isinstance(timeout, bool) or not isinstance(timeout, (int, float))
                    or not _is_finite(timeout) or not 0 < timeout <= 600):
                raise ValueError("navigate timeout must be finite and between 0 and 600 seconds")

    def to_dict(self) -> Dict[str, Any]:
        self.validate()
        return asdict(self)

    def to_command_dict(self) -> Dict[str, Any]:
        """Dict that maps 1:1 onto robot_interfaces/RobotCommand fields."""
        self.validate()
        return {k: v for k, v in asdict(self).items() if k != "response"}

    @classmethod
    def from_dict(cls, data: Dict[str, Any], response: str = "") -> "ActionPlan":
        if not isinstance(data, dict):
            raise ValueError("action plan must be an object")
        extra = set(data) - _ACTION_FIELDS
        if extra:
            raise ValueError(f"unknown action plan fields: {sorted(extra)}")
        action = data.get("action", "")
        if not isinstance(action, str):
            raise ValueError("action must be a string")
        numeric = {}
        for name in _NUMERIC_FIELDS:
            value = data.get(name, 0.0)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be numeric")
            numeric[name] = value
        goal = data.get("goal", "")
        params = data.get("params", {})
        response_value = data.get("response", response)
        if not isinstance(goal, str):
            raise ValueError("goal must be a string")
        if not isinstance(params, dict):
            raise ValueError("params must be an object")
        if not isinstance(response_value, str):
            raise ValueError("response must be a string")
        if any(not isinstance(key, str) for key in params):
            raise ValueError("params keys must be strings")
        return cls(
            action=action, **numeric, goal=goal, params=dict(params),
            response=response_value,
        )
