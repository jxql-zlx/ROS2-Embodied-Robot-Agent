"""robot_control node: subscribe RobotCommand, publish cmd_vel Twist.

Flow:  /robot_command (RobotCommand)
         -> action_to_velocity()
         -> /cmd_vel (Twist) published at update_rate
         -> /robot_status (RobotStatus) feedback
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from robot_interfaces.msg import RobotCommand, RobotStatus
from robot_interfaces.srv import ExecuteCommand

from robot_control.controller import VelocityCommand, action_to_velocity, validate_command


class RobotControlNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_control")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("robot_command_topic", "/robot_command")
        self.declare_parameter("robot_status_topic", "/robot_status")
        self.declare_parameter("execute_service", "/execute_command")
        self.declare_parameter("update_rate", 10.0)
        self.declare_parameter("max_linear_speed", 1.0)
        self.declare_parameter("max_angular_speed", 2.0)

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        cmd_topic = self.get_parameter("robot_command_topic").value
        status_topic = self.get_parameter("robot_status_topic").value
        exec_service = self.get_parameter("execute_service").value
        self._update_rate = float(self.get_parameter("update_rate").value)
        self._max_linear = float(self.get_parameter("max_linear_speed").value)
        self._max_angular = float(self.get_parameter("max_angular_speed").value)

        # RELIABLE (default) so it matches the gazebo diff-drive subscriber
        self._cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._cmd_sub = self.create_subscription(
            RobotCommand, cmd_topic, self._on_command, 10
        )
        self._status_pub = self.create_publisher(RobotStatus, status_topic, 10)
        self._exec_srv = self.create_service(
            ExecuteCommand, exec_service, self._on_execute_command
        )

        # Active velocity state
        self._active: VelocityCommand | None = None
        self._remaining: float = 0.0
        self._last_action: str = ""
        self._last_command_id: str = ""

        self._timer = self.create_timer(1.0 / self._update_rate, self._tick)
        self.get_logger().info(
            f"robot_control ready: cmd_vel={cmd_vel_topic}, cmd={cmd_topic}, "
            f"max_linear={self._max_linear}, max_angular={self._max_angular}"
        )

    # ------------------------------------------------------------------ #

    def _publish_status(self, state: str, message: str, remaining: float) -> None:
        status = RobotStatus()
        status.action = self._last_action
        status.state = state
        status.message = message
        status.remaining = remaining
        status.stamp = self.get_clock().now().to_msg()
        status.command_id = self._last_command_id
        self._status_pub.publish(status)

    def _publish_twist(self, vel: VelocityCommand) -> None:
        twist = Twist()
        twist.linear.x = vel.linear_x
        twist.linear.y = vel.linear_y
        twist.angular.z = vel.angular_z
        self._cmd_vel_pub.publish(twist)

    def _stop(self, state: str = "stopped", message: str = "stopped") -> None:
        self._active = None
        self._remaining = 0.0
        self._publish_twist(VelocityCommand())
        self._publish_status(state, message, 0.0)

    def _handle_command(
        self,
        action: str,
        linear_x: float,
        linear_y: float,
        angular_z: float,
        duration: float,
        goal: str,
        command_id: str = "",
    ) -> tuple[bool, str]:
        self._last_command_id = command_id
        ok, err = validate_command(
            action, linear_x, linear_y, angular_z, duration, self._max_linear, self._max_angular
        )
        if not ok:
            self._last_action = action
            self._stop(state="error", message=err)
            return False, err

        vel = action_to_velocity(
            action, linear_x, linear_y, angular_z, duration, self._max_linear, self._max_angular
        )
        self._last_action = action
        if action.strip().lower() == "stop":
            self._stop(state="stopped", message="stopped")
            return True, "stopped"
        self._active = vel
        self._remaining = vel.duration if vel.duration > 0 else -1.0
        self._publish_twist(vel)
        self._publish_status("running", vel.message, self._remaining)
        self.get_logger().info(
            f"[{action}] {vel.message} (duration={vel.duration}s, goal={goal or '-'})"
        )
        return True, vel.message

    def _on_command(self, msg: RobotCommand) -> None:
        self._handle_command(
            msg.action, msg.linear_x, msg.linear_y, msg.angular_z, msg.duration,
            msg.goal, msg.command_id,
        )

    def _on_execute_command(self, request, response) -> ExecuteCommand.Response:
        success, message = self._handle_command(
            request.command.action,
            request.command.linear_x,
            request.command.linear_y,
            request.command.angular_z,
            request.command.duration,
            request.command.goal,
            request.command.command_id,
        )
        response.success = success
        response.message = message
        return response

    def _tick(self) -> None:
        if self._active is None:
            return
        self._publish_twist(self._active)
        if self._remaining < 0:
            return  # indefinite run
        self._remaining -= 1.0 / self._update_rate
        if self._remaining <= 0.0:
            self.get_logger().info("command finished, stopping")
            self._stop(state="done", message="command finished")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
