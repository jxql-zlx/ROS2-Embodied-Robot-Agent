"""Fresh-scene wait tests without a running ROS graph."""

import json
import unittest
from threading import Condition

from std_msgs.msg import String

from robot_tools.ros2_vision_tool import Ros2VisionTool


class TestFreshSceneWait(unittest.TestCase):
    def setUp(self):
        self.tool = object.__new__(Ros2VisionTool)
        self.tool._scene_condition = Condition()
        self.tool._latest = None

    def test_wait_returns_only_a_scene_with_newer_timestamp(self):
        self.tool._latest = {"observed_at": 10.0}
        msg = String()
        msg.data = json.dumps({"observed_at": 11.0, "objects": []})
        self.tool._on_scene(msg)
        self.assertEqual(self.tool.wait_for_new_scene(10.0, timeout_sec=0.01)["observed_at"], 11.0)

    def test_wait_times_out_without_new_scene(self):
        self.tool._latest = {"observed_at": 10.0}
        self.assertIsNone(self.tool.wait_for_new_scene(10.0, timeout_sec=0.001))

    def test_frame_from_during_motion_does_not_verify_completed_motion(self):
        self.tool._latest = {"observed_at": 11.0}
        self.assertIsNone(self.tool.wait_for_new_scene(12.0, timeout_sec=0.001))


if __name__ == "__main__":
    unittest.main()
