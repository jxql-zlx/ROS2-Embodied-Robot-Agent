#!/usr/bin/env python3
"""Wait until a topic publishes at least one message (with timeout).

Usage:
  python3 wait_topic.py /odom nav_msgs/msg/Odometry --timeout 90
"""

from __future__ import annotations

import argparse
import importlib

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("topic")
    parser.add_argument("type", help="e.g. nav_msgs/msg/Odometry")
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--sensor-qos", action="store_true", help="use best-effort sensor QoS")
    args = parser.parse_args()

    package, _, rest = args.type.partition("/")  # "nav_msgs" + "/msg/Odometry"
    cls = rest.split("/")[-1]
    module = importlib.import_module(f"{package}.msg")
    message_class = getattr(module, cls)

    rclpy.init()
    node = Node("wait_topic")
    received: list = []
    qos = qos_profile_sensor_data if args.sensor_qos else 10
    node.create_subscription(message_class, args.topic, lambda _m: received.append(True), qos)

    deadline = node.get_clock().now() + rclpy.duration.Duration(seconds=args.timeout)
    while node.get_clock().now() < deadline and not received:
        rclpy.spin_once(node, timeout_sec=0.2)

    node.destroy_node()
    rclpy.shutdown()
    if received:
        print(f"OK: {args.topic} is publishing")
        return 0
    print(f"TIMEOUT: no message on {args.topic} within {args.timeout}s")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
