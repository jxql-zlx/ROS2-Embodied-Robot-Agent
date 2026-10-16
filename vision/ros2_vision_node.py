#!/usr/bin/env python3
"""ROS2 vision node: camera frame -> YOLO -> /vision/detections (JSON Scene).

Usage (WSL, ROS env sourced):
  python3 vision/ros2_vision_node.py
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # repo root

import cv2  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from vision.yolo_detector import YOLODetector  # noqa: E402


class VisionNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_node")
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("detections_topic", "/vision/detections")
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("conf", 0.35)
        self.declare_parameter("throttle_hz", 2.0)

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
        self.get_logger().info(
            f"vision_node ready: backend={self._detector.backend_name}, "
            f"model={self.get_parameter('model_path').value}"
        )

    def _on_image(self, msg: Image) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_publish < self._interval:
            return
        self._last_publish = now
        try:
            observed_at = time.time()
            frame = self._to_bgr(msg)
            scene = self._detector.detect(frame)
            scene.observed_at = observed_at
            text = scene.to_json()
            self._pub.publish(String(data=text))
            self.get_logger().info(
                f"detected {len(scene.objects)} object(s): "
                f"{[(o.name, o.confidence) for o in scene.objects]}"
            )
        except Exception as exc:  # keep the node alive
            self.get_logger().error(f"vision error: {exc}")

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
