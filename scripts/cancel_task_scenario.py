#!/usr/bin/env python3
"""Cancel a live approach through ROS services and verify that it stays stopped."""

import math
import sys
import time
import uuid

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_interfaces.msg import RobotCommand
from robot_interfaces.srv import CancelTask, TaskExecute


class CancellationScenario(Node):
    def __init__(self):
        super().__init__("cancel_task_scenario")
        self.commands = []
        self.pose = None
        self._task_client = self.create_client(TaskExecute, "/task_execute")
        self._cancel_client = self.create_client(CancelTask, "/task_cancel")
        self.create_subscription(RobotCommand, "/robot_command", self._on_command, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

    def _on_command(self, message):
        self.commands.append((time.monotonic(), message.action))

    def _on_odom(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        self.pose = (position.x, position.y, yaw)

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return predicate()


def main():
    task = sys.argv[1] if len(sys.argv) > 1 else "找杯子"
    rclpy.init()
    node = CancellationScenario()
    try:
        if not node._task_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/task_execute is unavailable")
        if not node._cancel_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/task_cancel is unavailable")

        if not node.spin_until(lambda: node.pose is not None, 5.0):
            raise RuntimeError("odometry is unavailable")

        request = TaskExecute.Request()
        request.task = task
        request.task_id = uuid.uuid4().hex
        task_future = node._task_client.call_async(request)
        if not node.spin_until(
            lambda: any(action in ("move", "rotate") for _, action in node.commands)
            or task_future.done(),
            25.0,
        ):
            raise RuntimeError("task produced no motion command")
        if not any(action in ("move", "rotate") for _, action in node.commands):
            raise RuntimeError("task finished before motion started")

        cancel_started = time.monotonic()
        cancel_request = CancelTask.Request()
        cancel_request.task_id = request.task_id
        cancel_future = node._cancel_client.call_async(cancel_request)
        if not node.spin_until(lambda: cancel_future.done(), 2.0):
            raise RuntimeError("cancel service did not respond within two seconds")
        cancel_response = cancel_future.result()
        if not cancel_response.success:
            raise RuntimeError(f"cancel service rejected request: {cancel_response.response}")
        cancel_latency = time.monotonic() - cancel_started

        if not node.spin_until(lambda: task_future.done(), 3.0):
            raise RuntimeError("cancelled task did not return within three seconds")
        task_response = task_future.result()
        if task_response.success or "取消" not in task_response.response:
            raise RuntimeError(f"task did not report cancellation: {task_response.response}")

        if not node.spin_until(lambda: any(action == "stop" for _, action in node.commands), 1.0):
            raise RuntimeError("robot stop command was not observed")
        response_pose = node.pose
        stop_index = next(index for index, (_, action) in enumerate(node.commands) if action == "stop")
        quiet_until = time.monotonic() + 0.8
        while time.monotonic() < quiet_until:
            rclpy.spin_once(node, timeout_sec=0.05)
        after_pose = node.pose
        after_stop = [action for _, action in node.commands[stop_index + 1:]]
        if any(action in ("move", "rotate") for action in after_stop):
            raise RuntimeError(f"motion command followed stop: {after_stop}")
        displacement = math.hypot(after_pose[0] - response_pose[0], after_pose[1] - response_pose[1])
        yaw_delta = math.atan2(math.sin(after_pose[2] - response_pose[2]),
                               math.cos(after_pose[2] - response_pose[2]))
        if displacement > 0.02 or abs(yaw_delta) > 0.03:
            raise RuntimeError(
                f"robot continued moving after cancellation: {displacement:.3f}m, "
                f"{yaw_delta:.3f}rad"
            )
        print(f"cancel_response: {cancel_response.response}")
        print(f"task_response: {task_response.response}")
        print(f"commands: {[action for _, action in node.commands]}")
        print(f"cancel_latency: {cancel_latency:.3f}s")
        print(f"post_response_drift: {displacement:.3f}m, {yaw_delta:.3f}rad")
        print("PASS: cancellation stopped the active task")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
