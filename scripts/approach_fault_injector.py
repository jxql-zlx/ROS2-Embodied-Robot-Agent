#!/usr/bin/env python3
"""Inject one sensor outage after the demo's first motion command."""

import argparse
import os
import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_interfaces.msg import RobotCommand
from sensor_msgs.msg import LaserScan


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("vision_dropout", "range_dropout"))
    parser.add_argument("vision_pid", type=int)
    args = parser.parse_args()
    rclpy.init()
    node = Node("approach_fault_injector")
    faulted = False
    publisher = node.create_publisher(LaserScan, "/approach_test/scan", qos_profile_sensor_data)

    def on_scan(msg):
        if not faulted:
            publisher.publish(msg)

    def on_command(msg):
        nonlocal faulted
        if faulted or msg.action not in ("move", "rotate"):
            return
        faulted = True
        if args.mode == "vision_dropout":
            # The shell supplies the process group it created for this demo.
            os.killpg(args.vision_pid, signal.SIGINT)
        node.get_logger().info(f"injected {args.mode} after first motion")

    node.create_subscription(LaserScan, "/scan", on_scan, qos_profile_sensor_data)
    node.create_subscription(RobotCommand, "/robot_command", on_command, 10)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
