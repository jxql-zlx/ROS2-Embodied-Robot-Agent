#!/usr/bin/env python3
"""Verify a continuous, fresh odometry window after a task reports completion."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry

from robot_agent.navigation_validation import validate_stop_samples


class StopWindow(Node):
    def __init__(self):
        super().__init__("verify_stop_window", parameter_overrides=[
            Parameter("use_sim_time", value=True),
        ])
        self.samples = []
        self.last_stamp_ns = None
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

    def _on_odom(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        twist = message.twist.twist
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        speed = max(abs(twist.linear.x), abs(twist.linear.y), abs(twist.angular.z))
        stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        stamp_ns = stamp[0] * 1_000_000_000 + stamp[1]
        finite = all(math.isfinite(value) for value in (
            position.x, position.y, yaw, speed,
        ))
        increasing = stamp_ns > 0 and (
            self.last_stamp_ns is None or stamp_ns > self.last_stamp_ns
        )
        valid = finite and increasing
        self.samples.append((
            time.monotonic(), (position.x, position.y), speed, stamp, yaw, valid,
        ))
        if len(self.samples) > 2000:
            self.samples.pop(0)
        if valid:
            self.last_stamp_ns = stamp_ns


def main() -> int:
    rclpy.init()
    node = StopWindow()
    started = time.monotonic()
    deadline = started + 1.0
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        validate_stop_samples(
            node.samples,
            min_samples=10,
            max_receive_gap=0.25,
            max_speed=0.02,
            max_drift=0.02,
            max_yaw_drift=0.03,
        )
    except Exception as exc:
        print(f"FAIL: continuous stop window not verified: {exc}")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(f"PASS: continuous stop window; samples={len(node.samples)}; duration=1.0s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
