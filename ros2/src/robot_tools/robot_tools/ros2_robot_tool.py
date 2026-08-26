"""Ros2RobotTool: ROS2 implementation of BaseRobotTool.

Publishes RobotCommand to the execution layer (/robot_command) and
reads /odom + /robot_status for pose/status feedback.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
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
        self._latest_status = msg

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
        pose = self._parse_nav_goal(goal)
        if pose is None:
            return False
        if not self._nav_client.server_is_ready():
            self.get_logger().warn("Nav2 NavigateToPose server not available")
            return False
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_nav_goal_response)
        return True

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
        self.get_logger().info("Nav2 goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        result = future.result()
        self.get_logger().info(f"Nav2 navigation result: status={result.status}")

    def get_pose(self) -> Optional[Dict[str, float]]:
        return self._latest_pose

    def get_sensor(self, name: str) -> Dict[str, Any]:
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
