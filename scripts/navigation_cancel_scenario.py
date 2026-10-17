#!/usr/bin/env python3
"""Exercise agent cancellation while a Nav2 goal is actively moving the robot."""

import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default
from robot_interfaces.msg import RobotCommand
from robot_interfaces.srv import CancelTask, TaskExecute

GOAL_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


class NavigationCancellationScenario(Node):
    def __init__(self):
        super().__init__("navigation_cancel_scenario")
        self.commands = []
        self.pose = None
        self.speed = None
        self.last_odom_received = None
        self.last_odom_stamp = None
        self.navigation_statuses = {}
        self.last_goal_statuses = {}
        self.navigation_status_history = []
        self.odom_samples = []
        self.task_client = self.create_client(TaskExecute, "/task_execute")
        self.cancel_client = self.create_client(CancelTask, "/task_cancel")
        self.create_subscription(RobotCommand, "/robot_command", self._on_command, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._on_navigation_status,
            qos_profile_action_status_default,
        )

    def _on_command(self, message):
        self.commands.append((time.monotonic(), message.action))

    def _on_odom(self, message):
        position = message.pose.pose.position
        self.pose = (position.x, position.y)
        twist = message.twist.twist
        self.speed = max(abs(twist.linear.x), abs(twist.linear.y), abs(twist.angular.z))
        self.last_odom_received = time.monotonic()
        self.last_odom_stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        self.odom_samples.append(
            (self.last_odom_received, self.pose, self.speed, self.last_odom_stamp)
        )

    def _on_navigation_status(self, message):
        self.navigation_statuses = {
            bytes(item.goal_info.goal_id.uuid).hex(): item.status
            for item in message.status_list
        }
        self.last_goal_statuses.update(self.navigation_statuses)
        self.navigation_status_history.append(
            (time.monotonic(), dict(self.navigation_statuses))
        )

    def goal_status(self, goal_id):
        return self.last_goal_statuses.get(goal_id)

    def goal_status_history(self, goal_id):
        return [
            statuses[goal_id]
            for _, statuses in self.navigation_status_history
            if goal_id in statuses
        ]

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
        known_goal_ids = set(node.navigation_statuses)
        request = TaskExecute.Request()
        request.task = "导航到 2.5,0,0"
        task_future = node.task_client.call_async(request)
        active_statuses = {GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING}
        scenario_goal = {}

        def find_new_active_goal():
            for goal_id, status in node.navigation_statuses.items():
                if goal_id not in known_goal_ids and status in active_statuses:
                    scenario_goal["id"] = goal_id
                    scenario_goal["status"] = status
                    return True
            return False

        saw_active_navigation = node.spin_until(find_new_active_goal, 10.0)
        goal_id = scenario_goal.get("id")
        moved = saw_active_navigation and node.spin_until(
            lambda: node.goal_status(goal_id) in active_statuses and node.pose is not None and
            math.hypot(node.pose[0] - start_pose[0], node.pose[1] - start_pose[1]) >= 0.25,
            45.0,
        )
        if not moved:
            samples = node.odom_samples[start_sample_index:]
            max_displacement = max(
                (math.hypot(pose[0] - start_pose[0], pose[1] - start_pose[1])
                 for _, pose, _, _ in samples),
                default=0.0,
            )
            recent_statuses = node.navigation_status_history[-20:]
            raise RuntimeError(
                "navigation did not become active and move at least 0.25 m; "
                f"saw_active={saw_active_navigation}, start_pose={start_pose}, "
                f"last_pose={node.pose}, max_odom_displacement={max_displacement:.3f} m, "
                f"last_speed={node.speed}, scenario_goal_id={goal_id}, "
                f"scenario_goal_status={node.goal_status(goal_id)}, "
                f"recent_statuses={recent_statuses}"
            )

        print(
            f"scenario_goal_id: {goal_id}; status={node.goal_status(goal_id)}; pose={node.pose}",
            flush=True,
        )
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

        terminal_statuses = {
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED,
        }
        if not node.spin_until(lambda: node.goal_status(goal_id) in terminal_statuses, 5.0):
            raise RuntimeError(
                f"scenario goal {goal_id} has no observed terminal status; "
                f"status={node.goal_status(goal_id)}, history={node.navigation_status_history[-20:]}"
            )
        if node.goal_status(goal_id) != GoalStatus.STATUS_CANCELED:
            raise RuntimeError(
                f"scenario goal {goal_id} ended with status {node.goal_status(goal_id)}, "
                "expected canceled"
            )
        if not node.spin_until(lambda: any(action == "stop" for _, action in node.commands), 2.0):
            raise RuntimeError("stop command was not observed")

        stop_index = max(index for index, (_, action) in enumerate(node.commands) if action == "stop")
        stop_received = node.commands[stop_index][0]
        if not node.spin_until(
            lambda: node.last_odom_received is not None
            and node.last_odom_received > stop_received
            and node.speed is not None and node.speed < 0.02,
            5.0,
        ):
            raise RuntimeError(
                f"no fresh low-speed odometry after stop; received={node.last_odom_received}, "
                f"stop={stop_received}, speed={node.speed}, stamp={node.last_odom_stamp}"
            )
        settle_start = node.last_odom_received
        stopped_pose = node.pose
        quiet_deadline = time.monotonic() + 1.0
        while time.monotonic() < quiet_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        settle_samples = [sample for sample in node.odom_samples if sample[0] >= settle_start]
        if len(settle_samples) < 5:
            raise RuntimeError(f"too few fresh odometry samples during settling: {len(settle_samples)}")
        if time.monotonic() - node.last_odom_received > 0.5:
            raise RuntimeError(
                f"odometry became stale while checking stop: last received {node.last_odom_received}"
            )
        if any(speed >= 0.02 for _, _, speed, _ in settle_samples):
            raise RuntimeError("robot speed rose above stop threshold during settling window")
        later_commands = [action for _, action in node.commands[stop_index + 1:]]
        if any(action in ("move", "rotate") for action in later_commands):
            raise RuntimeError(f"motion command followed final stop: {later_commands}")
        drift = max(
            math.hypot(pose[0] - stopped_pose[0], pose[1] - stopped_pose[1])
            for _, pose, _, _ in settle_samples
        )
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
        followup_goal = (0.0, 0.0)
        if not node.spin_until(
            lambda: math.hypot(node.pose[0] - followup_goal[0], node.pose[1] - followup_goal[1])
            <= 0.35,
            5.0,
        ):
            error = math.hypot(node.pose[0] - followup_goal[0], node.pose[1] - followup_goal[1])
            raise RuntimeError(
                f"follow-up reported success but robot is {error:.3f} m from "
                f"goal {followup_goal}; pose={node.pose}; response={followup_response.response}"
            )
        followup_error = math.hypot(
            node.pose[0] - followup_goal[0], node.pose[1] - followup_goal[1]
        )

        print(f"mode: {mode}")
        print(f"cancel_response: {cancel_response.response}")
        goal_states = list(dict.fromkeys(node.goal_status_history(goal_id)))
        print(f"goal_status_history: {[GOAL_STATUS_NAMES.get(status, status) for status in goal_states]}")
        print(f"goal_terminal_status: {GOAL_STATUS_NAMES.get(node.goal_status(goal_id))}")
        print(f"cancelled_task_response: {task_response.response}")
        print(f"followup_response: {followup_response.response}")
        print(f"followup_pose: {node.pose}; goal_error: {followup_error:.3f} m")
        print(f"commands: {[action for _, action in node.commands]}")
        print(f"post_stop_drift: {drift:.3f} m")
        print(f"fresh_odom_samples_after_stop: {len(settle_samples)}")
        print("PASS: Nav2 cancellation reached terminal state and follow-up navigation succeeded")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
