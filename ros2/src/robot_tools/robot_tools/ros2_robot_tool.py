"""Ros2RobotTool: ROS2 implementation of BaseRobotTool.

Publishes RobotCommand to the execution layer (/robot_command) and
reads /odom + /robot_status for pose/status feedback.
"""

from __future__ import annotations

import math
import time
from collections import deque
from threading import Condition, Event
from typing import Any, Dict, Optional

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from robot_interfaces.msg import RobotCommand, RobotStatus

from robot_tools.base_robot_tool import BaseRobotTool


def parse_nav_goal_string(goal: str) -> Optional[tuple[float, float, float]]:
    """Parse a navigation goal "x,y" or "x,y,yaw" into (x, y, yaw).

    Returns None when the goal is empty or malformed.
    """
    parts = [p.strip() for p in (goal or "").split(",") if p.strip()]
    try:
        nums = [float(p) for p in parts]
    except ValueError:
        return None
    if not nums:
        return None
    x = nums[0]
    y = nums[1] if len(nums) > 1 else 0.0
    yaw = nums[2] if len(nums) > 2 else 0.0
    return (x, y, yaw)


class Ros2RobotTool(Node, BaseRobotTool):
    def __init__(
        self,
        node_name: str = "ros2_robot_tool",
        command_topic: str = "/robot_command",
        status_topic: str = "/robot_status",
        odom_topic: str = "/odom",
        navigate_server: str = "/navigate_to_pose",
        scan_topic: str = "/scan",
    ) -> None:
        Node.__init__(self, node_name)
        self._command_topic = command_topic
        self._cmd_pub = self.create_publisher(RobotCommand, command_topic, 10)
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self._status_sub = self.create_subscription(
            RobotStatus, status_topic, self._on_status, 10
        )
        self._nav_client = ActionClient(self, NavigateToPose, navigate_server)
        self._latest_pose: Optional[Dict[str, float]] = None
        self._latest_status: Optional[RobotStatus] = None
        self._latest_scan: Optional[LaserScan] = None
        self._scan_received_at = 0.0
        self._scan_condition = Condition()
        self._scan_history = deque(maxlen=10)
        self._scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self._status_condition = Condition()
        self.get_logger().info(
            f"Ros2RobotTool ready: cmd->{command_topic}, odom->{odom_topic}, "
            f"status->{status_topic}, nav->{navigate_server}"
        )

    # ------------------------------------------------------------------ #

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self._latest_pose = {
            "x": p.x,
            "y": p.y,
            "yaw": yaw,
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        }

    def _on_status(self, msg: RobotStatus) -> None:
        with self._status_condition:
            self._latest_status = msg
            self._status_condition.notify_all()

    def _on_scan(self, msg: LaserScan) -> None:
        with self._scan_condition:
            self._latest_scan = msg
            self._scan_received_at = time.monotonic()
            self._scan_history.append((self._scan_received_at, self.front_range(msg)))
            self._scan_condition.notify_all()

    def wait_for_front_range(self, timeout_sec: float = 3.0, cancel_event=None) -> Dict[str, Any]:
        """Require three consistent scans after allowing the base to settle."""
        started = time.monotonic()
        with self._scan_condition:
            while True:
                if cancel_event is not None and cancel_event.is_set():
                    return {"name": "front_range", "available": False, "distance": None,
                            "cancelled": True}
                samples = [value for stamp, value in self._scan_history if stamp > started + 0.5]
                recent = samples[-3:]
                if len(recent) == 3 and all(value is not None for value in recent):
                    scan = getattr(self, "_latest_scan", None)
                    blind_margin = scan.range_min + 0.05 if scan is not None else 0.0
                    if min(recent) > blind_margin and max(recent) - min(recent) <= 0.03:
                        return {"name": "front_range", "available": True,
                                "distance": sorted(recent)[1], "frame_id": "lidar_link"}
                remaining = started + timeout_sec - time.monotonic()
                if remaining <= 0:
                    return {"name": "front_range", "available": False, "distance": None}
                self._scan_condition.wait(min(remaining, 0.1) if cancel_event is not None else remaining)

    @staticmethod
    def front_range(scan: LaserScan) -> Optional[float]:
        """Closest valid return within ten degrees of the robot's forward axis."""
        if scan.header.frame_id != "lidar_link" or scan.angle_increment <= 0:
            return None
        ranges = [
            value for index, value in enumerate(scan.ranges)
            if abs(scan.angle_min + index * scan.angle_increment) <= math.radians(10)
            and math.isfinite(value)
            and scan.range_min <= value <= scan.range_max
        ]
        if ranges:
            return min(ranges)
        # +inf is the LaserScan representation of no return within sensor
        # range. It provides clearance, unlike NaN or a missing message.
        forward = [value for index, value in enumerate(scan.ranges)
                   if abs(scan.angle_min + index * scan.angle_increment) <= math.radians(10)]
        if forward and all(value == math.inf for value in forward) and math.isfinite(scan.range_max):
            return float(scan.range_max)
        return None

    def status_stamp(self) -> Optional[tuple[int, int]]:
        """Return a snapshot token for waiting on a later motion status."""
        with self._status_condition:
            stamp = self._latest_status.stamp if self._latest_status else None
            return (stamp.sec, stamp.nanosec) if stamp is not None else None

    def wait_for_motion_complete(
        self,
        after_stamp: Optional[tuple[int, int]],
        timeout_sec: float,
        cancel_event=None,
    ) -> bool:
        """Wait for a move/rotate command to report a terminal status."""
        deadline = time.monotonic() + max(0.0, timeout_sec)
        with self._status_condition:
            while True:
                if cancel_event is not None and cancel_event.is_set():
                    return False
                status = self._latest_status
                if status is not None:
                    stamp = (status.stamp.sec, status.stamp.nanosec)
                    is_new = after_stamp is None or stamp > after_stamp
                    if is_new and status.action in ("move", "rotate"):
                        if status.state == "error":
                            return False
                        if status.state in ("done", "stopped"):
                            return True
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._status_condition.wait(min(remaining, 0.1) if cancel_event is not None else remaining)

    def _publish(self, action: str, **fields: Any) -> bool:
        msg = RobotCommand()
        msg.action = action
        msg.linear_x = float(fields.get("linear_x", 0.0))
        msg.linear_y = float(fields.get("linear_y", 0.0))
        msg.angular_z = float(fields.get("angular_z", 0.0))
        msg.duration = float(fields.get("duration", 0.0))
        msg.goal = str(fields.get("goal", ""))
        msg.source = str(fields.get("source", ""))
        self._cmd_pub.publish(msg)
        return True

    # ------------------------------------------------------------------ #

    def move(
        self,
        linear_x: float = 0.0,
        linear_y: float = 0.0,
        angular_z: float = 0.0,
        duration: float = 0.0,
    ) -> bool:
        return self._publish(
            "move", linear_x=linear_x, linear_y=linear_y, angular_z=angular_z, duration=duration
        )

    def stop(self) -> bool:
        return self._publish("stop")

    def rotate(self, angular_z: float, duration: float = 0.0) -> bool:
        return self._publish("rotate", angular_z=angular_z, duration=duration)

    def navigate(self, goal: str, **kwargs: Any) -> bool:
        """Navigate to a map pose and return whether Nav2 reached it.

        ``timeout`` bounds the whole action (goal acceptance and execution).
        Set ``wait=False`` only when the caller deliberately wants fire-and-forget.
        """
        pose = self._parse_nav_goal(goal)
        if pose is None:
            return False
        if not self._nav_client.server_is_ready():
            self.get_logger().warn("Nav2 NavigateToPose server not available")
            return False
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        wait = bool(kwargs.get("wait", True))
        timeout = max(0.0, float(kwargs.get("timeout", 120.0)))
        cancel_event = kwargs.get("cancel_event")
        command_guard = kwargs.get("command_guard")

        def cancelled() -> bool:
            return cancel_event is not None and cancel_event.is_set()

        if cancelled():
            return False
        if command_guard is None:
            goal_future = self._nav_client.send_goal_async(goal_msg)
        else:
            with command_guard:
                if cancelled():
                    return False
                goal_future = self._nav_client.send_goal_async(goal_msg)
        if not wait:
            goal_future.add_done_callback(self._on_nav_goal_response)
            return True

        done = Event()
        timed_out = Event()
        outcome = {"success": False, "goal_handle": None}

        def on_result(result_future) -> None:
            try:
                result = result_future.result()
                outcome["success"] = result.status == GoalStatus.STATUS_SUCCEEDED
                self.get_logger().info(f"Nav2 navigation result: status={result.status}")
            except Exception as exc:
                self.get_logger().error(f"Nav2 result failed: {exc}")
            finally:
                done.set()

        def on_goal_response(goal_future) -> None:
            try:
                handle = goal_future.result()
                if not handle.accepted:
                    self.get_logger().warn("Nav2 goal rejected")
                    done.set()
                    return
                outcome["goal_handle"] = handle
                if timed_out.is_set():
                    handle.cancel_goal_async()
                    done.set()
                    return
                handle.get_result_async().add_done_callback(on_result)
            except Exception as exc:
                self.get_logger().error(f"Nav2 goal response failed: {exc}")
                done.set()

        goal_future.add_done_callback(on_goal_response)
        deadline = time.monotonic() + timeout
        while not done.is_set():
            if cancelled():
                timed_out.set()
                handle = outcome["goal_handle"]
                if handle is not None:
                    handle.cancel_goal_async()
                return False
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            done.wait(min(remaining, 0.1) if cancel_event is not None else remaining)
        if not done.is_set():
            timed_out.set()
            handle = outcome["goal_handle"]
            if handle is not None:
                handle.cancel_goal_async()
            self.get_logger().warn(f"Nav2 navigation timed out after {timeout:.1f}s")
            return False
        return bool(outcome["success"])

    def _parse_nav_goal(self, goal: str) -> Optional[PoseStamped]:
        parsed = parse_nav_goal_string(goal)
        if parsed is None:
            self.get_logger().error(f"invalid navigate goal: {goal!r}")
            return None
        x, y, yaw = parsed
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _on_nav_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal rejected")
            return
        self.get_logger().info("Nav2 goal accepted (fire-and-forget)")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        result = future.result()
        self.get_logger().info(f"Nav2 navigation result: status={result.status}")

    def get_pose(self) -> Optional[Dict[str, float]]:
        return self._latest_pose

    def get_sensor(self, name: str) -> Dict[str, Any]:
        if name == "front_range":
            scan = self._latest_scan
            age = time.monotonic() - self._scan_received_at
            distance = self.front_range(scan) if scan is not None and age <= 1.0 else None
            return {
                "name": name, "available": distance is not None,
                "distance": distance, "age_sec": age,
                "frame_id": "lidar_link",
            }
        # Reserved interface: real sensor topics (camera /scan) land in the
        # vision & navigation milestones. Return honest availability info.
        return {
            "name": name,
            "available": False,
            "note": "sensor integration planned for vision (P2) / navigation (P1) milestones",
        }

    def execute_action(self, action: Dict[str, Any]) -> bool:
        if hasattr(action, "to_command_dict"):
            action = action.to_command_dict()
        if action.get("action", "") == "navigate":
            return self.navigate(action.get("goal", ""))
        return self._publish(
            action.get("action", ""),
            linear_x=action.get("linear_x", 0.0),
            linear_y=action.get("linear_y", 0.0),
            angular_z=action.get("angular_z", 0.0),
            duration=action.get("duration", 0.0),
            goal=action.get("goal", ""),
            source=action.get("source", ""),
        )

    def latest_status(self) -> Optional[RobotStatus]:
        return self._latest_status
