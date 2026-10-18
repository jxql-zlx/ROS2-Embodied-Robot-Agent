"""Ros2RobotTool: ROS2 implementation of BaseRobotTool.

Publishes RobotCommand to the execution layer (/robot_command) and
reads /odom + /robot_status for pose/status feedback.
"""

from __future__ import annotations

import math
import time
import uuid
from collections import OrderedDict, deque
from threading import Condition, Event, Lock
from typing import Any, Dict, Optional

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from robot_interfaces.msg import RobotCommand, RobotStatus

from robot_tools.base_robot_tool import BaseRobotTool

ROS_CLOCK_FORWARD_JUMP_THRESHOLD_SEC = 1.0


def parse_nav_goal_string(goal: str) -> Optional[tuple[float, float, float]]:
    """Parse a navigation goal "x,y" or "x,y,yaw" into (x, y, yaw).

    Returns None when the goal is empty or malformed.
    """
    parts = [p.strip() for p in (goal or "").split(",")]
    if any(not part for part in parts):
        return None
    try:
        nums = [float(p) for p in parts]
    except ValueError:
        return None
    if not nums or len(nums) > 3 or not all(math.isfinite(value) for value in nums):
        return None
    x = nums[0]
    y = nums[1] if len(nums) > 1 else 0.0
    yaw = nums[2] if len(nums) > 2 else 0.0
    return (x, y, yaw)


class Ros2RobotTool(Node, BaseRobotTool):
    SCAN_MAX_SOURCE_AGE_SEC = 1.0
    SCAN_MAX_FUTURE_SEC = 0.1
    SCAN_CLOCK_ROLLBACK_TOLERANCE_SEC = 0.01
    STATUS_HISTORY_LIMIT = 128
    STATUS_HISTORY_TTL_SEC = 60.0
    STATUS_MAX_SOURCE_AGE_SEC = 5.0
    STATUS_MAX_RECEIPT_AGE_SEC = 5.0
    STATUS_CLOCK_ROLLBACK_TOLERANCE_SEC = 0.01

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
        self._latest_command_id: Optional[str] = None
        self._last_navigation_goal_id: Optional[str] = None
        self._latest_scan: Optional[LaserScan] = None
        self._scan_received_at = 0.0
        self._latest_scan_stamp_ns: Optional[int] = None
        self._last_scan_stamp_ns: Optional[int] = None
        self._last_scan_ros_now_ns: Optional[int] = None
        self._scan_condition = Condition()
        self._scan_history = deque(maxlen=10)
        self._scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self._status_condition = Condition()
        self._status_history = OrderedDict()
        self._status_clock_epoch = 0
        self._last_status_ros_now_ns: Optional[int] = None
        self._latest_status_received_at: Optional[float] = None
        self._clock_jump_handler = self.get_clock().create_jump_callback(
            JumpThreshold(
                # Normal simulation clock updates exceed 10 ms. Only invalidate
                # cached ROS-time data for a significant forward discontinuity.
                min_forward=Duration(seconds=ROS_CLOCK_FORWARD_JUMP_THRESHOLD_SEC),
                min_backward=Duration(nanoseconds=-10_000_000),
                on_clock_change=True,
            ),
            post_callback=self._on_ros_clock_jump,
        )
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
        command_id = str(getattr(msg, "command_id", "") or "").strip()
        now = time.monotonic()
        ros_now_ns = self._ros_time_ns()
        with self._status_condition:
            self._sync_status_clock_epoch_locked(ros_now_ns)
            source_stamp_ns = self._status_time_ns(msg)
            if (ros_now_ns <= 0 or source_stamp_ns is None or source_stamp_ns <= 0
                    or source_stamp_ns - ros_now_ns > self.SCAN_MAX_FUTURE_SEC * 1_000_000_000
                    or ros_now_ns - source_stamp_ns > self.STATUS_MAX_SOURCE_AGE_SEC * 1_000_000_000):
                # Missing or old-epoch source time cannot establish a waiter baseline.
                self._status_condition.notify_all()
                return
            # Preserve the latest global snapshot for the existing status API,
            # while keeping a separate bounded receipt per command for waits.
            previous = self._status_history.get(command_id) if command_id else None
            if command_id:
                terminal = {"done", "stopped", "error"}
                if previous is None or previous[0].state not in terminal:
                    old_stamp = self._status_time_ns(previous[0]) if previous else None
                    new_stamp = self._status_time_ns(msg)
                    if old_stamp is None or new_stamp is None or new_stamp >= old_stamp:
                        self._status_history[command_id] = (msg, now)
                        self._status_history.move_to_end(command_id)
                self._prune_status_history(now)
            current_stamp = self._status_time_ns(self._latest_status) if self._latest_status else None
            incoming_stamp = self._status_time_ns(msg)
            if (self._latest_status is None or current_stamp is None or incoming_stamp is None
                    or incoming_stamp >= current_stamp):
                self._latest_status = msg
                self._latest_status_received_at = now
            self._status_condition.notify_all()

    @staticmethod
    def _status_time_ns(status: Optional[RobotStatus]) -> Optional[int]:
        if status is None:
            return None
        stamp = getattr(status, "stamp", None)
        if stamp is None:
            return None
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _prune_status_history(self, now: float) -> None:
        cutoff = now - self.STATUS_HISTORY_TTL_SEC
        for command_id, (_status, received_at) in list(self._status_history.items()):
            if received_at >= cutoff:
                break
            del self._status_history[command_id]
        while len(self._status_history) > self.STATUS_HISTORY_LIMIT:
            self._status_history.popitem(last=False)

    def _ros_time_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _sync_status_clock_epoch_locked(self, ros_now_ns: int) -> None:
        previous = self._last_status_ros_now_ns
        tolerance_ns = self.STATUS_CLOCK_ROLLBACK_TOLERANCE_SEC * 1_000_000_000
        if previous is not None and ros_now_ns + tolerance_ns < previous:
            self._status_history.clear()
            self._latest_status = None
            self._latest_status_received_at = None
            self._status_clock_epoch += 1
        self._last_status_ros_now_ns = ros_now_ns

    def _on_ros_clock_jump(self, _time_jump) -> None:
        """Drop cached sensor evidence whenever the ROS time domain jumps."""
        with self._status_condition:
            self._status_history.clear()
            self._latest_status = None
            self._latest_status_received_at = None
            self._last_status_ros_now_ns = None
            self._status_clock_epoch += 1
            self._status_condition.notify_all()
        with self._scan_condition:
            self._scan_history.clear()
            self._latest_scan = None
            self._latest_scan_stamp_ns = None
            self._last_scan_stamp_ns = None
            self._last_scan_ros_now_ns = None
            self._scan_received_at = 0.0
            self._scan_condition.notify_all()

    def _status_is_fresh(self, status: RobotStatus, received_at: float,
                         now_ros_ns: int, now_monotonic: float) -> bool:
        stamp_ns = self._status_time_ns(status)
        if stamp_ns is None or stamp_ns <= 0 or now_ros_ns <= 0:
            return False
        source_age = (now_ros_ns - stamp_ns) / 1e9
        receipt_age = now_monotonic - received_at
        return (
            -self.SCAN_MAX_FUTURE_SEC <= source_age <= self.STATUS_MAX_SOURCE_AGE_SEC
            and 0.0 <= receipt_age <= self.STATUS_MAX_RECEIPT_AGE_SEC
        )

    def _on_scan(self, msg: LaserScan) -> None:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp_ns <= 0:
            # Without a source timestamp, repeated delivery cannot be
            # distinguished from a new measurement and must not confirm range.
            return
        now_ros_ns = self._ros_time_ns()
        if now_ros_ns <= 0:
            return
        with self._scan_condition:
            clock_reset = (
                self._last_scan_ros_now_ns is not None
                and now_ros_ns + self.SCAN_CLOCK_ROLLBACK_TOLERANCE_SEC * 1_000_000_000
                < self._last_scan_ros_now_ns
            )
            if clock_reset:
                self._scan_history.clear()
                self._latest_scan = None
                self._latest_scan_stamp_ns = None
                self._last_scan_stamp_ns = None
            elif self._last_scan_stamp_ns is not None and stamp_ns < self._last_scan_stamp_ns:
                # A single late/out-of-order sample is not evidence of a clock
                # reset; discard it without changing the current scan snapshot.
                self._last_scan_ros_now_ns = now_ros_ns
                return
            self._last_scan_ros_now_ns = now_ros_ns
            if self._last_scan_stamp_ns == stamp_ns:
                return
            source_age_ns = now_ros_ns - stamp_ns
            if (source_age_ns > self.SCAN_MAX_SOURCE_AGE_SEC * 1_000_000_000
                    or source_age_ns < -self.SCAN_MAX_FUTURE_SEC * 1_000_000_000):
                return
            self._last_scan_stamp_ns = stamp_ns
            self._latest_scan = msg
            self._latest_scan_stamp_ns = stamp_ns
            self._scan_received_at = time.monotonic()
            self._scan_history.append((self._scan_received_at, self.front_range(msg), stamp_ns))
            self._scan_condition.notify_all()

    def wait_for_front_range(self, timeout_sec: float = 3.0, cancel_event=None) -> Dict[str, Any]:
        """Require three consistent scans after allowing the base to settle."""
        started = time.monotonic()
        with self._scan_condition:
            while True:
                if cancel_event is not None and cancel_event.is_set():
                    return {"name": "front_range", "available": False, "distance": None,
                            "cancelled": True}
                samples = [sample for sample in self._scan_history if sample[0] > started + 0.5]
                recent = samples[-3:]
                now_ros_ns = self._ros_time_ns()
                now_monotonic = time.monotonic()
                source_ages = [
                    (now_ros_ns - sample[2]) / 1e9
                    for sample in recent if len(sample) > 2
                ]
                source_fresh = (len(source_ages) == 3 and all(
                    -self.SCAN_MAX_FUTURE_SEC <= age <= self.SCAN_MAX_SOURCE_AGE_SEC
                    for age in source_ages
                ))
                receipt_fresh = (len(recent) == 3 and all(
                    0.0 <= now_monotonic - sample[0] <= self.SCAN_MAX_SOURCE_AGE_SEC
                    for sample in recent
                ))
                if (len(recent) == 3 and all(sample[1] is not None for sample in recent)
                        and source_fresh and receipt_fresh):
                    scan = getattr(self, "_latest_scan", None)
                    blind_margin = scan.range_min + 0.05 if scan is not None else 0.0
                    values = [sample[1] for sample in recent]
                    if min(values) > blind_margin and max(values) - min(values) <= 0.03:
                        return {"name": "front_range", "available": True,
                                "distance": sorted(values)[1], "frame_id": "lidar_link",
                                "source_age_sec": max(source_ages)}
                remaining = started + timeout_sec - now_monotonic
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
        ros_now_ns = self._ros_time_ns()
        with self._status_condition:
            self._sync_status_clock_epoch_locked(ros_now_ns)
            now_monotonic = time.monotonic()
            is_fresh = (
                self._latest_status is not None
                and self._latest_status_received_at is not None
                and self._status_is_fresh(
                    self._latest_status, self._latest_status_received_at,
                    ros_now_ns, now_monotonic,
                )
            )
            stamp = self._latest_status.stamp if is_fresh else None
            return (stamp.sec, stamp.nanosec) if stamp is not None else None

    @property
    def latest_command_id(self) -> Optional[str]:
        return self._latest_command_id

    def wait_for_motion_complete(
        self,
        after_stamp: Optional[tuple[int, int]],
        timeout_sec: float,
        cancel_event=None,
        command_id: Optional[str] = None,
    ) -> bool:
        """Wait for a move/rotate command to report a terminal status."""
        if not command_id:
            return False
        deadline = time.monotonic() + max(0.0, timeout_sec)
        with self._status_condition:
            self._sync_status_clock_epoch_locked(self._ros_time_ns())
            wait_epoch = self._status_clock_epoch
            while True:
                if cancel_event is not None and cancel_event.is_set():
                    return False
                self._sync_status_clock_epoch_locked(self._ros_time_ns())
                if self._status_clock_epoch != wait_epoch:
                    return False
                now_monotonic = time.monotonic()
                self._prune_status_history(now_monotonic)
                item = self._status_history.get(command_id)
                status = item[0] if item else None
                if status is not None:
                    stamp = (status.stamp.sec, status.stamp.nanosec)
                    is_new = after_stamp is None or stamp > after_stamp
                    if (is_new and status.action in ("move", "rotate")
                            and self._status_is_fresh(
                                status, item[1], self._ros_time_ns(), now_monotonic
                            )):
                        if status.state == "error":
                            return False
                        if status.state in ("done", "stopped"):
                            return True
                remaining = deadline - now_monotonic
                if remaining <= 0:
                    return False
                self._status_condition.wait(min(remaining, 0.1) if cancel_event is not None else remaining)

    def wait_for_stop_confirmation(
        self,
        after_stamp: Optional[tuple[int, int]],
        timeout_sec: float = 2.0,
        cancel_event=None,
        command_id: Optional[str] = None,
    ) -> bool:
        """Wait for the control layer to confirm this stop command."""
        if not command_id:
            return False
        deadline = time.monotonic() + max(0.0, timeout_sec)
        with self._status_condition:
            self._sync_status_clock_epoch_locked(self._ros_time_ns())
            wait_epoch = self._status_clock_epoch
            while True:
                if cancel_event is not None and cancel_event.is_set():
                    return False
                self._sync_status_clock_epoch_locked(self._ros_time_ns())
                if self._status_clock_epoch != wait_epoch:
                    return False
                now_monotonic = time.monotonic()
                self._prune_status_history(now_monotonic)
                item = self._status_history.get(command_id)
                status = item[0] if item else None
                if status is not None:
                    stamp = (status.stamp.sec, status.stamp.nanosec)
                    is_new = after_stamp is None or stamp > after_stamp
                    if (is_new and status.action == "stop"
                            and self._status_is_fresh(
                                status, item[1], self._ros_time_ns(), now_monotonic
                            )):
                        return status.state == "stopped"
                remaining = deadline - now_monotonic
                if remaining <= 0:
                    return False
                self._status_condition.wait(
                    min(remaining, 0.1) if cancel_event is not None else remaining
                )

    def _publish(self, action: str, **fields: Any) -> bool:
        msg = RobotCommand()
        command_id = str(fields.get("command_id") or uuid.uuid4().hex)
        msg.action = action
        msg.linear_x = float(fields.get("linear_x", 0.0))
        msg.linear_y = float(fields.get("linear_y", 0.0))
        msg.angular_z = float(fields.get("angular_z", 0.0))
        msg.duration = float(fields.get("duration", 0.0))
        msg.goal = str(fields.get("goal", ""))
        msg.source = str(fields.get("source", ""))
        msg.command_id = command_id
        self._latest_command_id = command_id
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
        self._last_navigation_goal_id = None
        wait = bool(kwargs.get("wait", True))
        timeout = max(0.0, float(kwargs.get("timeout", 120.0)))
        cancel_event = kwargs.get("cancel_event")
        command_guard = kwargs.get("command_guard")
        cancel_timeout = max(0.1, float(kwargs.get("cancel_timeout", 5.0)))
        navigation_state_callback = kwargs.get("navigation_state_callback")

        def set_navigation_active(active: bool) -> None:
            if navigation_state_callback is not None:
                navigation_state_callback(active)

        def cancelled() -> bool:
            return cancel_event is not None and cancel_event.is_set()

        if cancelled():
            return False
        set_navigation_active(True)
        try:
            if command_guard is None:
                if cancelled():
                    set_navigation_active(False)
                    return False
                goal_future = self._nav_client.send_goal_async(goal_msg)
            else:
                with command_guard:
                    if cancelled():
                        goal_future = None
                    else:
                        goal_future = self._nav_client.send_goal_async(goal_msg)
                if goal_future is None:
                    set_navigation_active(False)
                    return False
        except Exception as exc:
            set_navigation_active(False)
            self.get_logger().error(f"Nav2 goal submission failed: {exc}")
            return False

        if not wait:
            def on_fire_and_forget_response(future) -> None:
                try:
                    handle = future.result()
                    if handle is None or not handle.accepted:
                        set_navigation_active(False)
                        self.get_logger().warn("Nav2 goal rejected")
                        return
                    result_future = handle.get_result_async()

                    def on_fire_and_forget_result(done_future) -> None:
                        set_navigation_active(False)
                        self._on_nav_result(done_future)

                    result_future.add_done_callback(on_fire_and_forget_result)
                except Exception as exc:
                    set_navigation_active(False)
                    self.get_logger().error(f"Nav2 goal response failed: {exc}")

            goal_future.add_done_callback(on_fire_and_forget_response)
            return True

        goal_response_done = Event()
        result_done = Event()
        timed_out = Event()
        state_lock = Lock()
        outcome = {
            "success": False,
            "goal_handle": None,
            "accepted": None,
            "cancel_sent": False,
            "cancel_future": None,
        }

        def request_cancel(handle) -> None:
            with state_lock:
                if outcome["cancel_sent"]:
                    return
                outcome["cancel_sent"] = True
            try:
                cancel_future = handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().error(f"Nav2 cancel request failed: {exc}")
                return
            with state_lock:
                outcome["cancel_future"] = cancel_future

            def on_cancel_response(future) -> None:
                try:
                    future.result()
                    self.get_logger().info("Nav2 cancel response received")
                except Exception as exc:
                    self.get_logger().error(f"Nav2 cancel response failed: {exc}")

            cancel_future.add_done_callback(on_cancel_response)

        def on_result(result_future) -> None:
            terminal = False
            try:
                result = result_future.result()
                status = result.status
                terminal = status in (
                    GoalStatus.STATUS_SUCCEEDED,
                    GoalStatus.STATUS_CANCELED,
                    GoalStatus.STATUS_ABORTED,
                )
                outcome["success"] = status == GoalStatus.STATUS_SUCCEEDED
                if terminal:
                    self.get_logger().info(f"Nav2 navigation result: status={status}")
                else:
                    self.get_logger().error(f"Nav2 returned non-terminal result status={status}")
            except Exception as exc:
                self.get_logger().error(f"Nav2 result failed: {exc}")
            finally:
                if terminal:
                    set_navigation_active(False)
                result_done.set()

        def on_goal_response(goal_future) -> None:
            try:
                handle = goal_future.result()
                with state_lock:
                    outcome["accepted"] = None if handle is None else bool(handle.accepted)
                    outcome["goal_handle"] = handle
                goal_info = getattr(handle, "goal_id", None)
                goal_uuid = getattr(goal_info, "uuid", None)
                if goal_uuid is not None and len(goal_uuid):
                    self._last_navigation_goal_id = bytes(goal_uuid).hex()
                goal_response_done.set()
                if handle is None:
                    self.get_logger().error(
                        "Nav2 goal response was empty; keeping navigation ownership locked"
                    )
                    result_done.set()
                    return
                if not handle.accepted:
                    self.get_logger().warn("Nav2 goal rejected")
                    set_navigation_active(False)
                    result_done.set()
                    return
                if timed_out.is_set():
                    request_cancel(handle)
                handle.get_result_async().add_done_callback(on_result)
            except Exception as exc:
                self.get_logger().error(f"Nav2 goal response failed: {exc}")
                goal_response_done.set()
                result_done.set()

        goal_future.add_done_callback(on_goal_response)
        deadline = time.monotonic() + timeout
        cancel_deadline = None
        while not result_done.is_set():
            now = time.monotonic()
            if cancel_deadline is None and (cancelled() or now >= deadline):
                timed_out.set()
                cancel_deadline = now + cancel_timeout
                with state_lock:
                    handle = outcome["goal_handle"]
                if handle is not None:
                    request_cancel(handle)
                reason = "cancelled" if cancelled() else f"timed out after {timeout:.1f}s"
                self.get_logger().warn(f"Nav2 navigation {reason}; waiting for terminal result")
            if cancel_deadline is not None and now >= cancel_deadline and not result_done.is_set():
                self.get_logger().error(
                    "Nav2 cancellation is unconfirmed; keeping navigation ownership locked"
                )
                return False
            if goal_response_done.is_set() and outcome["accepted"] is False:
                break
            result_done.wait(0.05 if cancel_event is not None or cancel_deadline is not None else
                             max(0.01, min(0.1, deadline - now)))
        if not result_done.is_set():
            return False
        return bool(outcome["success"] and not cancelled() and cancel_deadline is None)

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

    @property
    def last_navigation_goal_id(self) -> Optional[str]:
        return self._last_navigation_goal_id

    def get_sensor(self, name: str) -> Dict[str, Any]:
        if name == "front_range":
            with self._scan_condition:
                scan = self._latest_scan
                received_at = self._scan_received_at
                source_stamp_ns = self._latest_scan_stamp_ns
                age = time.monotonic() - received_at
                now_ros_ns = self._ros_time_ns()
                source_age = ((now_ros_ns - source_stamp_ns) / 1e9
                              if source_stamp_ns is not None and now_ros_ns > 0 else None)
                distance = self.front_range(scan) if (
                    scan is not None and age <= self.SCAN_MAX_SOURCE_AGE_SEC
                    and source_age is not None
                    and -self.SCAN_MAX_FUTURE_SEC <= source_age <= self.SCAN_MAX_SOURCE_AGE_SEC
                ) else None
            return {
                "name": name, "available": distance is not None,
                "distance": distance, "age_sec": age,
                "source_age_sec": source_age,
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
            command_id=action.get("command_id") or uuid.uuid4().hex,
        )

    def latest_status(self) -> Optional[RobotStatus]:
        with self._status_condition:
            status = self._latest_status
            received_at = self._latest_status_received_at
            if status is None or received_at is None:
                return None
            if not self._status_is_fresh(
                    status, received_at, self._ros_time_ns(), time.monotonic()):
                return None
            return status
