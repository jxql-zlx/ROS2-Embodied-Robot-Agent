"""Pure logic mapping RobotCommand -> velocity setpoint (no rclpy dependency).

Kept dependency-free so it can be unit-tested outside a ROS environment.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

MAX_COMMAND_DURATION_SEC = 30.0

#: High-level actions accepted by the execution layer (extensible).
ALLOWED_ACTIONS = ("move", "stop", "rotate", "navigate")

#: Actions that require Navigation2 (not available in this milestone yet).
NAV2_ACTIONS = ("navigate",)


@dataclass
class VelocityCommand:
    """Velocity setpoint for the robot base."""

    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0
    duration: float = 0.0  # seconds; 0 = indefinite until next command
    message: str = ""

    def is_zero(self) -> bool:
        return abs(self.linear_x) < 1e-9 and abs(self.linear_y) < 1e-9 and abs(self.angular_z) < 1e-9


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def validate_command(
    action: str,
    linear_x: float = 0.0,
    linear_y: float = 0.0,
    angular_z: float = 0.0,
    duration: float = 0.0,
    max_linear: float = 1.0,
    max_angular: float = 2.0,
) -> tuple[bool, str]:
    """Validate a high-level command. Returns (ok, error_message)."""
    action = (action or "").strip().lower()
    if action not in ALLOWED_ACTIONS:
        return False, f"unknown action '{action}', expected one of {ALLOWED_ACTIONS}"
    if action in NAV2_ACTIONS:
        return False, "navigate requires Navigation2 (planned for the next milestone)"
    values = (linear_x, linear_y, angular_z, duration, max_linear, max_angular)
    if not all(math.isfinite(value) for value in values):
        return False, "command values must be finite"
    if abs(linear_x) > max_linear or abs(linear_y) > max_linear:
        return False, f"linear speed exceeds max {max_linear} m/s"
    if abs(angular_z) > max_angular:
        return False, f"angular speed exceeds max {max_angular} rad/s"
    if action in ("move", "rotate") and duration <= 0:
        return False, "move/rotate duration must be finite and positive"
    if duration < 0:
        return False, "duration cannot be negative"
    if duration > MAX_COMMAND_DURATION_SEC:
        return False, f"duration exceeds max {MAX_COMMAND_DURATION_SEC} seconds"
    return True, ""


def action_to_velocity(
    action: str,
    linear_x: float = 0.0,
    linear_y: float = 0.0,
    angular_z: float = 0.0,
    duration: float = 0.0,
    max_linear: float = 1.0,
    max_angular: float = 2.0,
) -> VelocityCommand:
    """Map a high-level action to a velocity setpoint (zeros + error message on failure)."""
    action = (action or "").strip().lower()
    ok, err = validate_command(
        action, linear_x, linear_y, angular_z, duration, max_linear, max_angular
    )
    if not ok:
        return VelocityCommand(message=err)

    if action == "stop":
        return VelocityCommand(message="stopped")

    if action == "rotate":
        return VelocityCommand(
            angular_z=_clamp(angular_z, -max_angular, max_angular),
            duration=duration,
            message=f"rotate at {angular_z} rad/s",
        )

    if action == "move":
        return VelocityCommand(
            linear_x=_clamp(linear_x, -max_linear, max_linear),
            linear_y=_clamp(linear_y, -max_linear, max_linear),
            angular_z=_clamp(angular_z, -max_angular, max_angular),
            duration=duration,
            message=f"move vx={linear_x} vy={linear_y} wz={angular_z}",
        )

    return VelocityCommand(message=f"unsupported action {action}")
