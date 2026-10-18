#!/usr/bin/env python3
"""ROS2 vision node: camera frame -> YOLO -> /vision/detections (JSON Scene).

Usage (WSL, ROS env sourced):
  python3 vision/ros2_vision_node.py
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # repo root

import cv2  # noqa: E402
import rclpy  # noqa: E402
from rclpy.clock import JumpThreshold  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from vision.yolo_detector import YOLODetector  # noqa: E402
from vision.vision_schema import Scene  # noqa: E402

ROS_CLOCK_FORWARD_JUMP_THRESHOLD_SEC = 1.0


class VisionNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_node")
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("detections_topic", "/vision/detections")
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("conf", 0.35)
        self.declare_parameter("throttle_hz", 2.0)
        self.declare_parameter("max_source_age_sec", 2.5)
        self.declare_parameter("future_source_tolerance_sec", 0.1)

        self._detector = YOLODetector(
            model_path=self.get_parameter("model_path").value,
            conf=float(self.get_parameter("conf").value),
        )
        self._pub = self.create_publisher(
            String, self.get_parameter("detections_topic").value, 10
        )
        self.create_subscription(
            Image, self.get_parameter("camera_topic").value, self._on_image, 10
        )
        self._last_publish = 0.0
        self._interval = 1.0 / float(self.get_parameter("throttle_hz").value)
        self._max_source_age_sec = max(
            0.0, float(self.get_parameter("max_source_age_sec").value)
        )
        self._future_source_tolerance_sec = max(
            0.0, float(self.get_parameter("future_source_tolerance_sec").value)
        )
        self._clock_epoch_warmup = True
        self._clock_warmup_stamp_ns = None
        self._last_source_stamp_ns = None
        self._clock_jump_handler = self.get_clock().create_jump_callback(
            JumpThreshold(
                # A normal Gazebo clock tick is larger than 10 ms. Treating
                # each tick as a jump prevents the two-frame warmup from ever
                # completing. Keep the threshold above normal update periods.
                min_forward=Duration(seconds=ROS_CLOCK_FORWARD_JUMP_THRESHOLD_SEC),
                min_backward=Duration(nanoseconds=-10_000_000),
                on_clock_change=True,
            ),
            post_callback=self._on_ros_clock_jump,
        )
        self.get_logger().info(
            f"vision_node ready: backend={self._detector.backend_name}, "
            f"model={self.get_parameter('model_path').value}"
        )

    def _on_image(self, msg: Image) -> None:
        now_monotonic = time.monotonic()
        if now_monotonic - self._last_publish < self._interval:
            return
        self._last_publish = now_monotonic
        observed_at = time.time()
        try:
            source_metadata = self._source_time_metadata(
                msg.header.stamp,
                self.get_clock().now().nanoseconds,
                self._max_source_age_sec,
                self._future_source_tolerance_sec,
            )
            if source_metadata["source_time_status"] == "verified":
                source_stamp_ns = source_metadata["source_stamp_ns"]
                if self._clock_epoch_warmup:
                    if self._clock_warmup_stamp_ns is None:
                        self._clock_warmup_stamp_ns = source_stamp_ns
                        source_metadata["source_time_status"] = "clock_epoch_warmup"
                    elif source_stamp_ns <= self._clock_warmup_stamp_ns:
                        source_metadata["source_time_status"] = "out_of_order"
                    else:
                        self._clock_epoch_warmup = False
                        self._last_source_stamp_ns = source_stamp_ns
                elif (self._last_source_stamp_ns is not None
                      and source_stamp_ns <= self._last_source_stamp_ns):
                    source_metadata["source_time_status"] = "out_of_order"
                else:
                    self._last_source_stamp_ns = source_stamp_ns
            if source_metadata["source_time_status"] == "verified":
                frame = self._to_bgr(msg)
                scene = self._detector.detect(frame)
            else:
                # Publish an explicit degraded scene so downstream waiters can
                # distinguish an untrusted frame from a verified empty scene.
                scene = Scene(source="yolo")
            scene.observed_at = observed_at
            payload = scene.to_dict()
            payload.update(source_metadata)
            text = json.dumps(payload, ensure_ascii=False)
            self._pub.publish(String(data=text))
            self.get_logger().info(
                f"detected {len(scene.objects)} object(s): "
                f"{[(o.name, o.confidence) for o in scene.objects]}"
            )
        except Exception as exc:  # keep the node alive
            self.get_logger().error(f"vision error: {exc}")

    @staticmethod
    def _source_time_metadata(stamp, now_ros_ns: int, max_age_sec: float,
                              future_tolerance_sec: float) -> dict:
        """Describe whether Image.header.stamp can verify this frame's age."""
        try:
            source_seconds = int(stamp.sec)
            source_nanoseconds = int(stamp.nanosec)
        except (AttributeError, TypeError, ValueError):
            source_seconds = source_nanoseconds = -1
        if source_seconds < 0 or not 0 <= source_nanoseconds < 1_000_000_000:
            source_stamp_ns = 0
        else:
            source_stamp_ns = source_seconds * 1_000_000_000 + source_nanoseconds
        if source_stamp_ns <= 0:
            return {"source_time_status": "unknown", "source_stamp_ns": None,
                    "source_age_sec": None}
        if now_ros_ns <= 0:
            return {"source_time_status": "unknown", "source_stamp_ns": source_stamp_ns,
                    "source_age_sec": None}
        age_sec = (now_ros_ns - source_stamp_ns) / 1e9
        if not math.isfinite(age_sec):
            status = "unknown"
        elif age_sec > max_age_sec:
            status = "stale"
        elif age_sec < -future_tolerance_sec:
            status = "future"
        else:
            status = "verified"
        return {"source_time_status": status, "source_stamp_ns": source_stamp_ns,
                "source_age_sec": age_sec}

    def _on_ros_clock_jump(self, _time_jump) -> None:
        """Require increasing camera stamps before trusting a changed ROS epoch."""
        self._clock_epoch_warmup = True
        self._clock_warmup_stamp_ns = None
        self._last_source_stamp_ns = None

    @staticmethod
    def _to_bgr(msg: Image) -> "cv2.Mat":
        from vision.image_utils import ros_image_to_bgr

        return ros_image_to_bgr(msg.data, msg.height, msg.width, msg.encoding)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
