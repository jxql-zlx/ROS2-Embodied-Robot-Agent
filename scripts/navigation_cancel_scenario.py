#!/usr/bin/env python3
"""Exercise agent cancellation while a Nav2 goal is actively moving the robot."""

import math
import sys
import time
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from robot_agent.navigation_validation import validate_stop_samples

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
        super().__init__("navigation_cancel_scenario", parameter_overrides=[
            Parameter("use_sim_time", value=True),
        ])
        self.commands = []
        self.pose = None
        self.speed = None
        self.last_odom_received = None
        self.last_odom_stamp = None
        self.invalid_odom_samples = 0
        self.navigation_statuses = {}
        self.last_navigation_status_received = None
        self.last_goal_statuses = {}
        self.navigation_status_history = []
        self.odom_samples = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
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
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        pose = (position.x, position.y)
        twist = message.twist.twist
        speed = max(abs(twist.linear.x), abs(twist.linear.y), abs(twist.angular.z))
        stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        received = time.monotonic()
        stamp_ns = stamp[0] * 1_000_000_000 + stamp[1]
        previous_ns = (None if self.last_odom_stamp is None else
                       self.last_odom_stamp[0] * 1_000_000_000 + self.last_odom_stamp[1])
        valid = all(math.isfinite(value) for value in (*pose, yaw, speed))
        valid = valid and stamp_ns > 0 and (previous_ns is None or stamp_ns > previous_ns)
        self.odom_samples.append((received, pose, speed, stamp, yaw, valid))
        if not valid:
            self.invalid_odom_samples += 1
            return
        self.pose = pose
        self.yaw = yaw
        self.speed = speed
        self.last_odom_received = received
        self.last_odom_stamp = stamp

    def map_pose(self):
        transform = self.tf_buffer.lookup_transform("map", "base_footprint", Time())
        p = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        stamp = transform.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        age = max(0.0, (self.get_clock().now().nanoseconds - stamp_ns) / 1_000_000_000)
        if age > 0.5:
            raise RuntimeError(f"map pose TF is stale ({age:.3f}s)")
        return (p.x, p.y, yaw), age

    def _on_navigation_status(self, message):
        self.last_navigation_status_received = time.monotonic()
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

    def navigation_status_diagnostic(self):
        received_age = (None if self.last_navigation_status_received is None else
                        time.monotonic() - self.last_navigation_status_received)
        publishers = self.get_publishers_info_by_topic("/navigate_to_pose/_action/status")
        return {"publisher_count": len(publishers), "last_message_age_sec": received_age,
                "current_statuses": dict(self.navigation_statuses)}

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return bool(predicate())


def _map_pose_available(node):
    try:
        node.map_pose()
        return True
    except Exception:
        return False


def _map_goal_error(node, goal):
    pose, _ = node.map_pose()
    return math.hypot(pose[0] - goal[0], pose[1] - goal[1])


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "simple"
    if mode not in ("simple", "hierarchical"):
        raise SystemExit("usage: navigation_cancel_scenario.py [simple|hierarchical]")

    rclpy.init()
    node = NavigationCancellationScenario()
    task_future = None
    scenario_passed = False
    try:
        if not node.task_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/task_execute is unavailable")
        if not node.cancel_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/task_cancel is unavailable")
        if not node.spin_until(lambda: node.pose is not None, 10.0):
            raise RuntimeError("odometry is unavailable")

        if not node.spin_until(lambda: _map_pose_available(node), 10.0):
            raise RuntimeError("fresh map-to-base_footprint transform is unavailable")
        start_map_pose, _ = node.map_pose()
        start_pose = node.pose
        navigation_goal_x = 0.0 if start_map_pose[0] > 0.5 else 2.5
        start_sample_index = len(node.odom_samples) - 1
        known_goal_ids = set(node.navigation_statuses)
        request = TaskExecute.Request()
        request.task = f"导航到 {navigation_goal_x},0,0"
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
                 for _, pose, _, _, _, valid in samples if valid),
                default=0.0,
            )
            recent_statuses = node.navigation_status_history[-20:]
            raise RuntimeError(
                "navigation did not become active and move at least 0.25 m; "
                f"saw_active={saw_active_navigation}, start_pose={start_pose}, "
                f"last_pose={node.pose}, max_odom_displacement={max_displacement:.3f} m, "
                f"last_speed={node.speed}, scenario_goal_id={goal_id}, "
                f"scenario_goal_status={node.goal_status(goal_id)}, "
                f"recent_statuses={recent_statuses}, "
                f"status_diagnostic={node.navigation_status_diagnostic()}"
            )

        print(
            f"scenario_goal_id: {goal_id}; requested_goal_x={navigation_goal_x}; "
            f"status={node.goal_status(goal_id)}; pose={node.pose}",
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
        quiet_deadline = time.monotonic() + 1.0
        while time.monotonic() < quiet_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        settle_samples = [sample for sample in node.odom_samples if sample[0] >= settle_start]
        if time.monotonic() - node.last_odom_received > 0.5:
            raise RuntimeError(
                f"odometry became stale while checking stop: last received {node.last_odom_received}"
            )
        drift, yaw_drift = validate_stop_samples(settle_samples)
        later_commands = [action for _, action in node.commands[stop_index + 1:]]
        if any(action in ("move", "rotate") for action in later_commands):
            raise RuntimeError(f"motion command followed final stop: {later_commands}")

        followup_start_pose, _ = node.map_pose()
        # Both goals are known reachable in the saved smoke map. Alternate
        # direction from the current pose with enough margin beyond tolerance.
        followup_goal = (1.0 if followup_start_pose[0] < 0.5 else 0.0, 0.0)
        start_goal_distance = math.hypot(
            followup_start_pose[0] - followup_goal[0],
            followup_start_pose[1] - followup_goal[1],
        )
        if start_goal_distance <= 0.5:
            raise RuntimeError(
                f"follow-up goal must start outside tolerance; distance={start_goal_distance:.3f}m"
            )
        known_followup_goals = set(node.last_goal_statuses)
        followup = TaskExecute.Request()
        followup.task = f"导航到 {followup_goal[0]},{followup_goal[1]},0"
        followup_future = node.task_client.call_async(followup)
        if not node.spin_until(lambda: followup_future.done(), 90.0):
            raise RuntimeError("follow-up navigation did not finish")
        followup_response = followup_future.result()
        if not followup_response.success:
            raise RuntimeError(f"follow-up task failed: {followup_response.response}")
        active_statuses = {GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING}

        def followup_goal_ids():
            ids = set()
            for _, statuses in node.navigation_status_history:
                for candidate_id, status in statuses.items():
                    if candidate_id not in known_followup_goals and status in active_statuses:
                        ids.add(candidate_id)
            return ids

        if not node.spin_until(
            lambda: bool(followup_goal_ids()),
            2.0,
        ):
            raise RuntimeError(
                "follow-up navigation did not publish a new active goal UUID; "
                f"status_diagnostic={node.navigation_status_diagnostic()}"
            )
        followup_goal_id = next(iter(followup_goal_ids()))
        terminal_statuses = {
            GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED,
        }
        if not node.spin_until(lambda: node.goal_status(followup_goal_id) in terminal_statuses, 90.0):
            raise RuntimeError(f"follow-up goal {followup_goal_id} has no terminal Nav2 status")
        if node.goal_status(followup_goal_id) != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"follow-up goal {followup_goal_id} ended with "
                f"{GOAL_STATUS_NAMES.get(node.goal_status(followup_goal_id))}"
            )
        if not node.spin_until(
            lambda: _map_goal_error(node, followup_goal) <= 0.35,
            5.0,
        ):
            final_map_pose, _ = node.map_pose()
            error = math.hypot(final_map_pose[0] - followup_goal[0], final_map_pose[1] - followup_goal[1])
            raise RuntimeError(
                f"follow-up reported success but robot is {error:.3f} m from "
                f"map goal {followup_goal}; pose={final_map_pose}; response={followup_response.response}"
            )
        final_map_pose, _ = node.map_pose()
        followup_error = math.hypot(
            final_map_pose[0] - followup_goal[0], final_map_pose[1] - followup_goal[1]
        )
        actual_followup_motion = math.hypot(
            final_map_pose[0] - followup_start_pose[0],
            final_map_pose[1] - followup_start_pose[1],
        )
        if actual_followup_motion < 0.1:
            raise RuntimeError(
                f"follow-up goal succeeded without measurable motion ({actual_followup_motion:.3f}m)"
            )

        print(f"mode: {mode}")
        print(f"cancel_response: {cancel_response.response}")
        goal_states = list(dict.fromkeys(node.goal_status_history(goal_id)))
        print(f"goal_status_history: {[GOAL_STATUS_NAMES.get(status, status) for status in goal_states]}")
        print(f"goal_terminal_status: {GOAL_STATUS_NAMES.get(node.goal_status(goal_id))}")
        print(f"followup_goal_id: {followup_goal_id}; status_history: "
              f"{[GOAL_STATUS_NAMES.get(s, s) for s in node.goal_status_history(followup_goal_id)]}")
        print(f"cancelled_task_response: {task_response.response}")
        print(f"followup_response: {followup_response.response}")
        print(f"followup_map_pose: {final_map_pose}; actual_motion: {actual_followup_motion:.3f} m; "
              f"goal_error: {followup_error:.3f} m")
        print(f"commands: {[action for _, action in node.commands]}")
        print(f"post_stop_drift: {drift:.3f} m")
        print(f"fresh_odom_samples_after_stop: {len(settle_samples)}")
        print("PASS: Nav2 cancellation reached terminal state and follow-up navigation succeeded")
        scenario_passed = True
        return 0
    finally:
        if task_future is not None and not scenario_passed:
            try:
                cleanup_future = node.cancel_client.call_async(CancelTask.Request())
                node.spin_until(lambda: cleanup_future.done(), 3.0)
                node.spin_until(lambda: task_future.done(), 3.0)
                print("failure_cleanup_cancel: requested; "
                      f"cancel_done={cleanup_future.done()}, task_done={task_future.done()}", flush=True)
            except Exception as exc:
                print(f"failure_cleanup_cancel_error: {exc}", flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
