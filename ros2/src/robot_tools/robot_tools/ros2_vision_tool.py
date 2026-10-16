"""Ros2VisionTool: agent-facing vision query backed by /vision/detections.

The agent reads perception results over ROS2 (a std_msgs/String carrying the
Scene JSON emitted by ros2_vision_node). This tool never imports YOLO or the
camera driver directly, keeping perception decoupled from planning.
"""

from __future__ import annotations

import json
import time
from typing import Any, Dict, Optional

from rclpy.node import Node
from std_msgs.msg import String


class Ros2VisionTool(Node):
    def __init__(
        self,
        node_name: str = "ros2_vision_tool",
        detections_topic: str = "/vision/detections",
        max_age_sec: float = 2.5,
    ) -> None:
        Node.__init__(self, node_name)
        self._detections_topic = detections_topic
        self._max_age_sec = max(0.0, float(max_age_sec))
        self._latest: Optional[Dict[str, Any]] = None
        self._sub = self.create_subscription(String, detections_topic, self._on_scene, 10)
        self.get_logger().info(f"Ros2VisionTool ready: detections->{detections_topic}")

    def _on_scene(self, msg: String) -> None:
        try:
            self._latest = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warn("ignoring malformed /vision/detections message")

    def get_scene(self) -> Optional[Dict[str, Any]]:
        """Return the latest raw Scene dict (or None before the first message)."""
        if self._latest is None:
            return None
        fresh, age = self._freshness(self._latest, time.time(), self._max_age_sec)
        return {
            **self._latest,
            "stale": not fresh,
            "age_sec": round(age, 3) if age is not None else None,
        }

    def get_objects(self, min_confidence: float = 0.0) -> Dict[str, Any]:
        """Return every object in the latest scene above ``min_confidence``."""
        return self._query_latest("", min_confidence)

    def find_object(self, object_name: str, min_confidence: float = 0.0) -> Dict[str, Any]:
        """Return objects whose name contains ``object_name`` (case-insensitive)."""
        return self._query_latest(object_name, min_confidence)

    def _query_latest(self, object_name: str, min_confidence: float) -> Dict[str, Any]:
        fresh, age = self._freshness(self._latest, time.time(), self._max_age_sec)
        if not fresh:
            return {
                "found": False,
                "objects": [],
                "stale": True,
                "age_sec": round(age, 3) if age is not None else None,
            }
        result = self._find(self._latest, object_name, min_confidence)
        result.update({"stale": False, "age_sec": round(age, 3)})
        return result

    @staticmethod
    def _freshness(
        scene: Optional[Dict[str, Any]], now: float, max_age_sec: float
    ) -> tuple[bool, Optional[float]]:
        """Treat missing, invalid, future, or expired timestamps as stale."""
        try:
            observed_at = float((scene or {}).get("observed_at", 0.0))
        except (TypeError, ValueError):
            return False, None
        if observed_at <= 0 or observed_at > now:
            return False, None
        age = now - observed_at
        return age <= max_age_sec, age

    @staticmethod
    def _find(
        scene: Optional[Dict[str, Any]], object_name: str, min_confidence: float = 0.0
    ) -> Dict[str, Any]:
        """Pure query helper so the matching logic is unit-testable without a node."""
        width = float((scene or {}).get("image_width", 0) or 0)
        key = (object_name or "").strip().lower()
        matches = []
        for obj in (scene or {}).get("objects", []):
            name = str(obj.get("name", ""))
            confidence = float(obj.get("confidence", 0.0))
            if confidence < min_confidence:
                continue
            if key and key not in name.lower():
                continue
            center = obj.get("center") or [None, None]
            cx, cy = center[0], center[1]
            matches.append(
                {
                    "name": name,
                    "confidence": round(confidence, 4),
                    "position": {"x": cx, "y": cy},
                    "center_x": cx,
                    "center_y": cy,
                    "direction": Ros2VisionTool._direction(cx, width),
                    "distance": None,
                }
            )
        return {"found": bool(matches), "objects": matches}

    @staticmethod
    def _direction(center_x: Any, image_width: float) -> str:
        """Bucket a horizontal pixel position into left / center / right."""
        if center_x is None or not image_width:
            return "unknown"
        if center_x < image_width / 3.0:
            return "left"
        if center_x > 2.0 * image_width / 3.0:
            return "right"
        return "center"
