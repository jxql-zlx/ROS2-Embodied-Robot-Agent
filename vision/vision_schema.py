"""Structured detection schema shared by all vision backends (YOLO / VLM future)."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class Detection:
    """One detected object."""

    name: str                 # e.g. "cup"
    confidence: float         # 0..1
    bbox: List[float]         # [x1, y1, x2, y2] in pixels
    center: Optional[List[float]] = None  # [cx, cy] in pixels


@dataclass
class Scene:
    """Structured description of what the robot sees."""

    image_width: int = 0
    image_height: int = 0
    objects: List[Detection] = field(default_factory=list)
    source: str = ""          # backend name, e.g. "yolo"
    observed_at: float = 0.0   # Unix time when processing of this frame began

    def find(self, name: str) -> List[Detection]:
        """Return detections whose name contains `name` (case-insensitive)."""
        key = name.lower()
        return [d for d in self.objects if key in d.name.lower()]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "image_width": self.image_width,
            "image_height": self.image_height,
            "source": self.source,
            "observed_at": self.observed_at,
            "objects": [
                {"name": d.name, "confidence": d.confidence, "bbox": d.bbox, "center": d.center}
                for d in self.objects
            ],
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False)
