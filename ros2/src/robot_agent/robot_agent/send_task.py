"""CLI: send a natural-language task to robot_agent.

Examples:
  ros2 run robot_agent send_task "让机器人向前移动"
  ros2 run robot_agent send_task "让机器人向前移动" --service
"""

from __future__ import annotations

import argparse
import math
import time
import uuid

import rclpy
from rclpy.node import Node
from robot_interfaces.srv import CancelTask, TaskExecute
from std_msgs.msg import String

CANCEL_CALL_TIMEOUT_SEC = 13.0  # server budget: initial stop (2s) + Nav2 (8s) + final stop (2s)


def publish_topic_task_once(node: Node, publisher, task: str, timeout_sec: float = 5.0) -> bool:
    """Wait briefly for the task subscriber, then enqueue exactly one message."""
    deadline = time.monotonic() + timeout_sec
    while publisher.get_subscription_count() == 0:
        if time.monotonic() >= deadline:
            return False
        rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
    msg = String()
    msg.data = task
    publisher.publish(msg)
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description="Send a natural-language task to robot_agent")
    parser.add_argument("task", help="task text, e.g. 让机器人向前移动")
    parser.add_argument(
        "--service", action="store_true", help="use the /task_execute service instead of topic"
    )
    parser.add_argument("--timeout", type=float, default=10.0, help="service call timeout (s)")
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        parser.error("--timeout must be finite and greater than zero")

    rclpy.init()
    node = Node("send_task")
    try:
        if args.service:
            client = node.create_client(TaskExecute, "/task_execute")
            if not client.wait_for_service(timeout_sec=5.0):
                print("ERROR: /task_execute service not available")
                return 1
            request = TaskExecute.Request()
            request.task = args.task
            request.task_id = uuid.uuid4().hex
            print(f"task_id    : {request.task_id}")
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
            if not future.done() or future.result() is None:
                print(f"ERROR: task {request.task_id} timed out; requesting cancellation")
                cancel_client = node.create_client(CancelTask, "/task_cancel")
                if not cancel_client.wait_for_service(timeout_sec=3.0):
                    print("ERROR: cancellation service unavailable; task may still be running")
                    return 1
                cancel_request = CancelTask.Request()
                cancel_request.task_id = request.task_id
                cancel_future = cancel_client.call_async(cancel_request)
                rclpy.spin_until_future_complete(
                    node, cancel_future, timeout_sec=CANCEL_CALL_TIMEOUT_SEC
                )
                if not cancel_future.done() or cancel_future.result() is None:
                    print("ERROR: cancellation result unknown; task may still be running")
                else:
                    cancel_result = cancel_future.result()
                    print(f"cancellation : {cancel_result.success} ({cancel_result.response})")
                return 1
            result = future.result()
            print(f"success    : {result.success}")
            print(f"response   : {result.response}")
            print(f"action_json: {result.action_json}")
            return 0 if result.success else 2

        publisher = node.create_publisher(String, "/task_input", 10)
        if not publish_topic_task_once(node, publisher, args.task):
            print("ERROR: no /task_input subscriber became available")
            return 1
        print(f"published task once to /task_input (completion is not acknowledged): {args.task}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
