import unittest
from unittest.mock import patch

from robot_agent.send_task import publish_topic_task_once


class _Publisher:
    def __init__(self, counts):
        self.counts = iter(counts)
        self.messages = []

    def get_subscription_count(self):
        return next(self.counts, 0)

    def publish(self, message):
        self.messages.append(message)


class TestSendTask(unittest.TestCase):
    def test_topic_delivery_publishes_once_after_subscriber_matches(self):
        publisher = _Publisher([0, 1])
        with patch("robot_agent.send_task.rclpy.spin_once") as spin_once:
            self.assertTrue(publish_topic_task_once(object(), publisher, "go", timeout_sec=1.0))
        self.assertEqual(len(publisher.messages), 1)
        self.assertEqual(publisher.messages[0].data, "go")
        spin_once.assert_called_once()

    def test_topic_delivery_does_not_publish_without_subscriber(self):
        publisher = _Publisher([0])
        with patch("robot_agent.send_task.rclpy.spin_once"):
            self.assertFalse(publish_topic_task_once(object(), publisher, "go", timeout_sec=0.0))
        self.assertEqual(publisher.messages, [])


if __name__ == "__main__":
    unittest.main()
