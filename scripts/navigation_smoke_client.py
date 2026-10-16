#!/usr/bin/env python3
"""Small ROS client used by the saved-map navigation smoke test."""

import argparse
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class NavigationSmokeClient(Node):
    def __init__(self):
        super().__init__(
            "navigation_smoke_client",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.pose = None
        self.pose_stamp = None
        self.odom_pose = None
        self.odom_speed = None
        self.ground_truth = None
        self.trajectory = []
        self.record_trajectory = False
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_pose, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(ModelStates, "/gazebo/model_states", self._on_model_states, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.bt_state = self.create_client(GetState, "/bt_navigator/get_state")

    def _on_pose(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        self.pose = (position.x, position.y, yaw)
        self.pose_stamp = message.header.stamp

    def _on_odom(self, message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        twist = message.twist.twist
        self.odom_pose = (p.x, p.y, yaw)
        self.odom_speed = max(abs(twist.linear.x), abs(twist.linear.y), abs(twist.angular.z))

    def _on_model_states(self, message):
        try:
            index = message.name.index("simple_diffbot")
        except ValueError:
            return
        pose = message.pose[index]
        q = pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.ground_truth = (pose.position.x, pose.position.y, yaw)

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
        pose = (p.x, p.y, yaw)
        return pose, age


def spin_until(node, predicate, timeout, description, report_timeout=True):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.record_trajectory:
            try:
                pose, _ = node.map_pose()
                node.trajectory.append(pose)
            except Exception:
                pass
        if predicate():
            return True
    if report_timeout:
        print(f"TIMEOUT: {description}", flush=True)
    return False


def spin_for(node, duration):
    deadline = time.monotonic() + duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)


def map_pose_available(node):
    try:
        node.map_pose()
        return True
    except Exception:
        return False


def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("check-localization")
    nav = subparsers.add_parser("navigate")
    nav.add_argument("x", type=float)
    nav.add_argument("y", type=float)
    nav.add_argument("yaw", type=float)
    nav.add_argument("--timeout", type=float, default=90.0)
    trip = subparsers.add_parser("round-trip")
    trip.add_argument("out_x", type=float)
    trip.add_argument("out_y", type=float)
    trip.add_argument("out_yaw", type=float)
    trip.add_argument("back_x", type=float)
    trip.add_argument("back_y", type=float)
    trip.add_argument("back_yaw", type=float)
    trip.add_argument("--timeout", type=float, default=90.0)
    obstacle = subparsers.add_parser("obstacle-trip")
    obstacle.add_argument("--timeout", type=float, default=120.0)
    failures = subparsers.add_parser("failure-suite")
    failures.add_argument("--timeout", type=float, default=90.0)
    args = parser.parse_args()

    rclpy.init()
    node = NavigationSmokeClient()
    try:
        if not spin_until(node, lambda: node.ground_truth is not None and map_pose_available(node),
                          30.0, "map-to-base transform and Gazebo ground truth"):
            return 1
        start, start_age = node.map_pose()
        print(f"map TF pose: x={start[0]:.3f} y={start[1]:.3f} yaw={start[2]:.3f} "
              f"(age={start_age:.3f}s); ground truth x={node.ground_truth[0]:.3f} "
              f"y={node.ground_truth[1]:.3f}; AMCL sample="
              f"{'available' if node.pose is not None else 'not received yet'}", flush=True)
        if args.command == "check-localization":
            error = math.hypot(start[0], start[1])
            print(f"initial position error={error:.3f}m (limit 0.35m)", flush=True)
            return 0 if error <= 0.35 else 1
        if args.command in ("round-trip", "obstacle-trip"):
            error = math.hypot(start[0], start[1])
            print(f"initial position error={error:.3f}m (limit 0.35m)", flush=True)
            if error > 0.35:
                return 1

        if not node.bt_state.wait_for_service(timeout_sec=20.0):
            print("FAIL: Nav2 lifecycle state service unavailable", flush=True)
            return 1
        active = False
        state_deadline = time.monotonic() + 90.0
        while time.monotonic() < state_deadline and not active:
            state_future = node.bt_state.call_async(GetState.Request())
            if not spin_until(node, lambda: state_future.done(), 20.0,
                              "bt_navigator lifecycle state", report_timeout=False):
                continue
            active = state_future.result().current_state.id == 3
            if not active:
                spin_for(node, 0.25)
        if not active:
            print("FAIL: bt_navigator did not become active", flush=True)
            return 1
        print("Nav2 bt_navigator is active", flush=True)

        if not node.client.wait_for_server(timeout_sec=10.0):
            print("FAIL: NavigateToPose action server unavailable", flush=True)
            return 1
        if args.command == "navigate":
            goals = [(args.x, args.y, args.yaw)]
        elif args.command == "round-trip":
            goals = [(args.out_x, args.out_y, args.out_yaw),
                     (args.back_x, args.back_y, args.back_yaw)]
        elif args.command == "obstacle-trip":
            goals = [(2.5, 0.0, 0.0)]
        elif args.command == "failure-suite":
            goals = [(1.8, 0.0, 0.0), (20.0, 20.0, 0.0), (0.0, 0.0, 0.0)]
        for index, (x, y, yaw) in enumerate(goals, start=1):
            start_map, start_tf_age = node.map_pose()
            start_odom = node.odom_pose
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

            send_future = node.client.send_goal_async(goal)
            if not spin_until(node, lambda: send_future.done(), 10.0, "goal acceptance"):
                return 1
            handle = send_future.result()
            if handle is None or not handle.accepted:
                print(f"FAIL: navigation goal {index} rejected", flush=True)
                return 1
            node.trajectory = []
            node.record_trajectory = True
            result_future = handle.get_result_async()
            goal_timeout = (0.35 if args.command == "failure-suite" and index == 3
                            else args.timeout)
            completed = spin_until(node, lambda: result_future.done(), goal_timeout, "navigation result")
            if not completed:
                cancel_future = handle.cancel_goal_async()
                if not spin_until(node, lambda: cancel_future.done(), 5.0, "goal cancellation response"):
                    return 1
                completed = spin_until(node, lambda: result_future.done(), 10.0,
                                       "cancelled navigation result")
                if not completed:
                    print("FAIL: navigation did not stop after timeout cancellation", flush=True)
                    return 1
            node.record_trajectory = False
            result = result_future.result()
            if args.command == "failure-suite":
                if result.status == GoalStatus.STATUS_SUCCEEDED:
                    print(f"FAIL: expected goal {index} to fail, status=SUCCEEDED", flush=True)
                    return 1
                expected_status = (GoalStatus.STATUS_CANCELED if index == 3
                                   else GoalStatus.STATUS_ABORTED)
                if result.status != expected_status:
                    print(f"FAIL: goal {index} ended with status={result.status}; "
                          f"expected={expected_status}", flush=True)
                    return 1
                if not spin_until(node, lambda: node.odom_speed is not None and node.odom_speed < 0.02,
                                  10.0, f"robot stop after failed goal {index}"):
                    return 1
                print(f"expected failure {index} stopped; Nav2 status={result.status}", flush=True)
                continue
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                print(f"FAIL: navigation goal {index} ended with status={result.status}", flush=True)
                return 1
            if not spin_until(
                node,
                lambda: node.odom_speed is not None and node.odom_speed < 0.02,
                10.0,
                "robot to stop after navigation",
            ):
                return 1
            stopped_odom_pose = node.odom_pose
            spin_for(node, 0.5)
            odom_drift = math.hypot(
                node.odom_pose[0] - stopped_odom_pose[0],
                node.odom_pose[1] - stopped_odom_pose[1],
            )
            final, final_tf_age = node.map_pose()
            odom_travel = math.hypot(
                node.odom_pose[0] - start_odom[0],
                node.odom_pose[1] - start_odom[1],
            )
            distance = math.hypot(final[0] - x, final[1] - y)
            yaw_error = abs(math.atan2(math.sin(final[2] - yaw), math.cos(final[2] - yaw)))
            print(
                f"goal {index} succeeded; final map pose x={final[0]:.3f} y={final[1]:.3f} "
                f"yaw={final[2]:.3f}; xy_error={distance:.3f}m yaw_error={yaw_error:.3f}rad "
                f"odom_travel={odom_travel:.3f}m stopped={node.odom_speed < 0.02} "
                f"odom_drift={odom_drift:.3f}m tf_age={final_tf_age:.3f}s "
                f"tf_start_age={start_tf_age:.3f}s",
                flush=True,
            )
            gt_error = math.hypot(final[0] - node.ground_truth[0], final[1] - node.ground_truth[1])
            if node.pose_stamp is not None:
                amcl_stamp_ns = node.pose_stamp.sec * 1_000_000_000 + node.pose_stamp.nanosec
                amcl_age = max(0.0, (node.get_clock().now().nanoseconds - amcl_stamp_ns) / 1_000_000_000)
                amcl_age_text = f"{amcl_age:.3f}s"
            else:
                amcl_age_text = "unavailable"
            print(f"ground-truth pose x={node.ground_truth[0]:.3f} y={node.ground_truth[1]:.3f}; "
                  f"map-to-truth error={gt_error:.3f}m amcl_age={amcl_age_text}", flush=True)
            if gt_error > 0.20:
                print("FAIL: map TF differs from Gazebo ground truth by more than 0.20m", flush=True)
                return 1
            if args.command == "obstacle-trip" and index == 1:
                lateral = max((abs(py) for _, py, _ in node.trajectory), default=0.0)
                print(f"obstacle route max lateral displacement={lateral:.3f}m", flush=True)
                if lateral < 0.30:
                    print("FAIL: trajectory did not show a measurable obstacle bypass", flush=True)
                    return 1
            required_travel = math.hypot(x - start_map[0], y - start_map[1])
            if (distance > 0.35 or yaw_error > 0.4 or odom_drift > 0.02
                    or (required_travel > 0.3 and odom_travel < 0.05)):
                return 1
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
