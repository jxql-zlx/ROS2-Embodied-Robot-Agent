"""Source-clock freshness checks for the ROS image callback."""

import json
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from vision.ros2_vision_node import VisionNode
from vision.vision_schema import Scene


class TestVisionSourceTime(unittest.TestCase):
    @staticmethod
    def _stamp(seconds, nanoseconds=0):
        return SimpleNamespace(sec=seconds, nanosec=nanoseconds)

    def test_source_timestamp_must_be_present_and_ros_clock_running(self):
        self.assertEqual(
            VisionNode._source_time_metadata(self._stamp(0), 1_000_000_000, 2.5, 0.1)[
                "source_time_status"
            ],
            "unknown",
        )
        self.assertEqual(
            VisionNode._source_time_metadata(self._stamp(1), 0, 2.5, 0.1)[
                "source_time_status"
            ],
            "unknown",
        )

    def test_old_future_and_recent_source_frames_are_classified(self):
        old = VisionNode._source_time_metadata(self._stamp(1), 4_000_000_000, 2.5, 0.1)
        future = VisionNode._source_time_metadata(self._stamp(5), 4_000_000_000, 2.5, 0.1)
        recent = VisionNode._source_time_metadata(
            self._stamp(3, 900_000_000), 4_000_000_000, 2.5, 0.1
        )
        self.assertEqual(old["source_time_status"], "stale")
        self.assertEqual(future["source_time_status"], "future")
        self.assertEqual(recent["source_time_status"], "verified")
        self.assertAlmostEqual(recent["source_age_sec"], 0.1)
        self.assertEqual(recent["source_stamp_ns"], 3_900_000_000)

    def _callback_node(self, stamp, ros_now_ns):
        node = object.__new__(VisionNode)
        node._last_publish = 0.0
        node._interval = 0.0
        node._max_source_age_sec = 2.5
        node._future_source_tolerance_sec = 0.1
        node._clock_epoch_warmup = False
        node._clock_warmup_stamp_ns = None
        node._last_source_stamp_ns = None
        node._detector = MagicMock()
        node._detector.detect.return_value = Scene(source="yolo", objects=[])
        node._pub = MagicMock()
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=ros_now_ns))
        node.get_logger = lambda: SimpleNamespace(
            info=lambda *_: None, error=lambda *_: None
        )
        msg = SimpleNamespace(header=SimpleNamespace(stamp=stamp), data=b"", height=1,
                              width=1, encoding="bgr8")
        return node, msg

    @patch("vision.ros2_vision_node.time.monotonic", return_value=1.0)
    @patch("vision.ros2_vision_node.time.time", return_value=100.0)
    @patch.object(VisionNode, "_to_bgr", return_value=object())
    def test_stale_image_is_published_degraded_without_running_detector(
        self, _to_bgr, _wall_time, _mono
    ):
        node, msg = self._callback_node(self._stamp(1), 4_000_000_000)
        node._on_image(msg)
        node._detector.detect.assert_not_called()
        published = json.loads(node._pub.publish.call_args.args[0].data)
        self.assertEqual(published["source_time_status"], "stale")
        self.assertEqual(published["objects"], [])
        self.assertEqual(published["observed_at"], 100.0)

    @patch("vision.ros2_vision_node.time.monotonic", return_value=1.0)
    @patch("vision.ros2_vision_node.time.time", return_value=100.0)
    @patch.object(VisionNode, "_to_bgr", return_value=object())
    def test_verified_image_carries_source_and_wall_clock_metadata(
        self, _to_bgr, _wall_time, _mono
    ):
        node, msg = self._callback_node(self._stamp(3), 4_000_000_000)
        node._on_image(msg)
        node._detector.detect.assert_called_once()
        published = json.loads(node._pub.publish.call_args.args[0].data)
        self.assertEqual(published["source_time_status"], "verified")
        self.assertEqual(published["source_stamp_ns"], 3_000_000_000)
        self.assertEqual(published["source_age_sec"], 1.0)
        self.assertEqual(published["observed_at"], 100.0)

    @patch("vision.ros2_vision_node.time.monotonic", side_effect=[1.0, 2.0])
    @patch("vision.ros2_vision_node.time.time", side_effect=[100.0, 101.0])
    @patch.object(VisionNode, "_to_bgr", return_value=object())
    def test_clock_epoch_requires_two_increasing_source_frames(
        self, _to_bgr, _wall_time, _mono
    ):
        node, first = self._callback_node(self._stamp(3), 4_000_000_000)
        node._clock_epoch_warmup = True
        node._on_image(first)
        first_payload = json.loads(node._pub.publish.call_args.args[0].data)
        self.assertEqual(first_payload["source_time_status"], "clock_epoch_warmup")
        node._detector.detect.assert_not_called()

        second = SimpleNamespace(
            header=SimpleNamespace(stamp=self._stamp(3, 500_000_000)), data=b"",
            height=1, width=1, encoding="bgr8",
        )
        node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=4_500_000_000)
        )
        node._on_image(second)
        second_payload = json.loads(node._pub.publish.call_args.args[0].data)
        self.assertEqual(second_payload["source_time_status"], "verified")
        node._detector.detect.assert_called_once()

    def test_clock_jump_restarts_camera_stamp_warmup(self):
        node, _msg = self._callback_node(self._stamp(3), 4_000_000_000)
        node._clock_epoch_warmup = False
        node._clock_warmup_stamp_ns = 3_000_000_000
        node._last_source_stamp_ns = 3_000_000_000
        node._on_ros_clock_jump(None)
        self.assertTrue(node._clock_epoch_warmup)
        self.assertIsNone(node._clock_warmup_stamp_ns)
        self.assertIsNone(node._last_source_stamp_ns)


if __name__ == "__main__":
    unittest.main()
