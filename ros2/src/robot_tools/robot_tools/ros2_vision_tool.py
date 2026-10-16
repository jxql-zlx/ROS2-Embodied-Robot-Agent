"""Ros2VisionTool: agent-facing vision query backed by /vision/detections.

The agent reads perception results over ROS2 (a std_msgs/String carrying the
Scene JSON emitted by ros2_vision_node). This tool never imports YOLO or the
camera driver directly, keeping perception decoupled from planning.
"""

from __future__ import annotations

import json
import time
from threading import Condition
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
        self._scene_condition = Condition()
        self._sub = self.create_subscription(String, detections_topic, self._on_scene, 10)
        self.get_logger().info(f"Ros2VisionTool ready: detections->{detections_topic}")

    def _on_scene(self, msg: String) -> None:
        try:
            scene = json.loads(msg.data)
            if not isinstance(scene, dict) or not isinstance(scene.get("objects", []), list):
                raise TypeError("scene must be an object with an objects list")
            with self._scene_condition:
                self._latest = scene
                self._scene_condition.notify_all()
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warn("ignoring malformed /vision/detections message")

    def wait_for_new_scene(
        self, after_observed_at: float, timeout_sec: float = 5.0, cancel_event=None
    ) -> Optional[Dict[str, Any]]:
        """Wait for a scene produced from a newer camera frame."""
        deadline = time.monotonic() + max(0.0, timeout_sec)
        with self._scene_condition:
            while True:
                if cancel_event is not None and cancel_event.is_set():
                    return None
                scene = self._latest
                try:
                    observed_at = float((scene or {}).get("observed_at", 0.0))
                except (TypeError, ValueError):
                    observed_at = 0.0
                if observed_at > after_observed_at:
                    return dict(scene)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._scene_condition.wait(min(remaining, 0.1) if cancel_event is not None else remaining)

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
        with self._scene_condition:
            scene = self._latest
        return self.query_scene(scene, object_name, min_confidence)

    def query_scene(
        self, scene: Optional[Dict[str, Any]], object_name: str = "", min_confidence: float = 0.0
    ) -> Dict[str, Any]:
        """Query one scene snapshot, including its matching freshness metadata."""
        fresh, age = self._freshness(scene, time.time(), self._max_age_sec)
        if not fresh:
            return {
                "found": False,
                "objects": [],
                "stale": True,
                "age_sec": round(age, 3) if age is not None else None,
            }
        result = self._find(scene, object_name, min_confidence)
        result.update({
            "stale": False,
            "age_sec": round(age, 3),
            "observed_at": float(scene["observed_at"]),
        })
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
