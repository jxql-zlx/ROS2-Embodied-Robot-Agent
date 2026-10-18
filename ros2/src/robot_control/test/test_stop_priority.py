"""Regression for the execution layer's immediate stop state transition."""

import unittest
from unittest.mock import Mock

import rclpy

from robot_control.robot_control_node import RobotControlNode


class TestRobotControlStop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def test_stop_clears_active_velocity_and_reports_stopped(self):
        node = RobotControlNode()
        try:
            node._handle_command("move", 0.4, 0.0, 0.0, 10.0, "", "move-1")
            self.assertEqual(node._last_command_id, "move-1")
            node._status_pub = Mock()
            node._publish_status("running", "moving", 10.0)
            self.assertEqual(node._status_pub.publish.call_args.args[0].command_id, "move-1")
            success, message = node._handle_command("stop", 0.0, 0.0, 0.0, 0.0, "")
            self.assertTrue(success)
            self.assertEqual(message, "stopped")
            self.assertIsNone(node._active)
            self.assertEqual(node._remaining, 0.0)
            self.assertEqual(node._last_action, "stop")
        finally:
            node.destroy_node()


if __name__ == "__main__":
    unittest.main()
