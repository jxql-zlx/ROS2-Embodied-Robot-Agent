#!/usr/bin/env python3
"""Exercise agent cancellation while a Nav2 goal is actively moving the robot."""

import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_interfaces.msg import RobotCommand
from robot_interfaces.srv import CancelTask, TaskExecute


class NavigationCancellationScenario(Node):
    def __init__(self):
        super().__init__("navigation_cancel_scenario")
        self.commands = []
        self.pose = None
        self.speed = None
        self.navigation_statuses = set()
        self.navigation_status_history = []
        self.odom_samples = []
        self.task_client = self.create_client(TaskExecute, "/task_execute")
        self.cancel_client = self.create_client(CancelTask, "/task_cancel")
        self.create_subscription(RobotCommand, "/robot_command", self._on_command, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(
            GoalStatusArray, "/navigate_to_pose/_action/status", self._on_navigation_status, 10
        )

    def _on_command(self, message):
        self.commands.append((time.monotonic(), message.action))

    def _on_odom(self, message):
        position = message.pose.pose.position
        self.pose = (position.x, position.y)
        twist = message.twist.twist
        self.speed = max(abs(twist.linear.x), abs(twist.linear.y), abs(twist.angular.z))
        self.odom_samples.append((time.monotonic(), self.pose, self.speed))

    def _on_navigation_status(self, message):
        self.navigation_statuses = {item.status for item in message.status_list}
        self.navigation_status_history.append((time.monotonic(), sorted(self.navigation_statuses)))

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return bool(predicate())


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "simple"
    if mode not in ("simple", "hierarchical"):
        raise SystemExit("usage: navigation_cancel_scenario.py [simple|hierarchical]")

    rclpy.init()
    node = NavigationCancellationScenario()
    try:
        if not node.task_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/task_execute is unavailable")
        if not node.cancel_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/task_cancel is unavailable")
        if not node.spin_until(lambda: node.pose is not None, 10.0):
            raise RuntimeError("odometry is unavailable")

        start_pose = node.pose
        start_sample_index = len(node.odom_samples) - 1
        request = TaskExecute.Request()
        request.task = "导航到 2.5,0,0"
        task_future = node.task_client.call_async(request)
        active_statuses = {GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING}
        saw_active_navigation = node.spin_until(
            lambda: bool(node.navigation_statuses & active_statuses), 10.0
        )
        moved = saw_active_navigation and node.spin_until(
            lambda: node.pose is not None and
            math.hypot(node.pose[0] - start_pose[0], node.pose[1] - start_pose[1]) >= 0.25,
            45.0,
        )
        if not moved:
            samples = node.odom_samples[start_sample_index:]
            max_displacement = max(
                (math.hypot(pose[0] - start_pose[0], pose[1] - start_pose[1])
                 for _, pose, _ in samples),
                default=0.0,
            )
            recent_statuses = node.navigation_status_history[-20:]
            raise RuntimeError(
                "navigation did not become active and move at least 0.25 m; "
                f"saw_active={saw_active_navigation}, start_pose={start_pose}, "
                f"last_pose={node.pose}, max_odom_displacement={max_displacement:.3f} m, "
                f"last_speed={node.speed}, recent_statuses={recent_statuses}"
            )

        print(f"active Nav2 statuses: {sorted(node.navigation_statuses)}; pose={node.pose}", flush=True)
        cancel_future = node.cancel_client.call_async(CancelTask.Request())
        if not node.spin_until(lambda: cancel_future.done(), 20.0):
            raise RuntimeError(
                f"cancel service did not finish within 20 seconds; statuses="
                f"{sorted(node.navigation_statuses)}"
            )
        cancel_response = cancel_future.result()
        if not cancel_response.success:
            raise RuntimeError(f"cancel was not confirmed: {cancel_response.response}")
        if not node.spin_until(lambda: task_future.done(), 3.0):
            raise RuntimeError("cancelled navigation task did not return")
        task_response = task_future.result()
        if task_response.success or "取消" not in task_response.response:
            raise RuntimeError(f"task did not report cancellation: {task_response.response}")

        if not node.spin_until(
            lambda: not (node.navigation_statuses & {
                GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING
            }),
            3.0,
        ):
            raise RuntimeError("Nav2 still reports an accepted or executing goal")
        if not node.spin_until(lambda: any(action == "stop" for _, action in node.commands), 2.0):
            raise RuntimeError("stop command was not observed")

        stop_index = max(index for index, (_, action) in enumerate(node.commands) if action == "stop")
        if not node.spin_until(lambda: node.speed is not None and node.speed < 0.02, 5.0):
            raise RuntimeError(f"robot did not settle after cancellation; speed={node.speed}")
        stopped_pose = node.pose
        quiet_deadline = time.monotonic() + 1.0
        while time.monotonic() < quiet_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        later_commands = [action for _, action in node.commands[stop_index + 1:]]
        if any(action in ("move", "rotate") for action in later_commands):
            raise RuntimeError(f"motion command followed final stop: {later_commands}")
        drift = math.hypot(node.pose[0] - stopped_pose[0], node.pose[1] - stopped_pose[1])
        if drift > 0.03:
            raise RuntimeError(f"robot drifted {drift:.3f} m after cancellation")

        followup = TaskExecute.Request()
        followup.task = "导航到 0,0,0"
        followup_future = node.task_client.call_async(followup)
        if not node.spin_until(lambda: followup_future.done(), 90.0):
            raise RuntimeError("follow-up navigation did not finish")
        followup_response = followup_future.result()
        if not followup_response.success:
            raise RuntimeError(f"follow-up task failed: {followup_response.response}")

        print(f"mode: {mode}")
        print(f"cancel_response: {cancel_response.response}")
        print(f"cancelled_task_response: {task_response.response}")
        print(f"followup_response: {followup_response.response}")
        print(f"commands: {[action for _, action in node.commands]}")
        print(f"post_stop_drift: {drift:.3f} m")
        print("PASS: Nav2 cancellation reached terminal state and follow-up navigation succeeded")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
