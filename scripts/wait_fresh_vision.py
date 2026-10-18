#!/usr/bin/env python3
"""Wait for a detection message backed by a verified, fresh camera timestamp."""

from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main() -> int:
    rclpy.init()
    node = Node("wait_fresh_vision")
    verified_at = None
    last_status = "no messages received"
    max_age_sec = 2.5

    def on_message(message: String) -> None:
        nonlocal last_status, verified_at
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                raise TypeError("scene must be a JSON object")
            observed_at = float(data.get("observed_at"))
            source_age = float(data.get("source_age_sec"))
            source_stamp = int(data.get("source_stamp_ns"))
        except (TypeError, ValueError, OverflowError, json.JSONDecodeError):
            last_status = "invalid JSON or missing source metadata"
            return
        now_wall = time.time()
        age = source_age + (now_wall - observed_at)
        if math.isfinite(source_age):
            last_status = (f"status={data.get('source_time_status')}, "
                           f"source_age={source_age:.3f}, combined_age={age:.3f}, "
                           f"source_stamp_ns={source_stamp}")
        else:
            last_status = f"status={data.get('source_time_status')}, invalid source age"
        if (data.get("source_time_status") == "verified" and source_stamp > 0
                and math.isfinite(observed_at) and math.isfinite(source_age)
                and observed_at > 0 and observed_at <= now_wall
                and source_age >= -0.1 and -0.1 <= age <= max_age_sec):
            verified_at = time.monotonic()

    node.create_subscription(String, "/vision/detections", on_message, 10)
    deadline = time.monotonic() + 30.0
    try:
        while rclpy.ok() and time.monotonic() < deadline and verified_at is None:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if verified_at is None:
        print(f"FAIL: no fresh detection with a verified camera source timestamp ({last_status})")
        return 1
    print("PASS: fresh camera source timestamp verified")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
