import io
import unittest
from contextlib import redirect_stdout
from types import SimpleNamespace
from unittest.mock import patch

from robot_interfaces.srv import CancelTask, TaskExecute
from robot_agent.send_task import main, publish_topic_task_once


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

    def test_service_future_exception_requests_cancel_and_reports_unknown_state(self):
        class _Future:
            def done(self):
                return True

            def result(self):
                raise RuntimeError("service transport failed")

        class _Client:
            def __init__(self, future):
                self.future = future
                self.request = None

            def wait_for_service(self, timeout_sec):
                return True

            def call_async(self, request):
                self.request = request
                return self.future

        task_client = _Client(_Future())
        cancel_response = SimpleNamespace(success=False, response="停止状态未确认")
        cancel_future = SimpleNamespace(done=lambda: True, result=lambda: cancel_response)
        cancel_client = _Client(cancel_future)

        class _Node:
            def create_client(self, service_type, _name):
                return task_client if service_type is TaskExecute else cancel_client

            def destroy_node(self):
                pass

        output = io.StringIO()
        with (
            patch("sys.argv", ["send_task", "go", "--service"]),
            patch("robot_agent.send_task.Node", return_value=_Node()),
            patch("robot_agent.send_task.rclpy.init"),
            patch("robot_agent.send_task.rclpy.shutdown"),
            patch("robot_agent.send_task.rclpy.spin_until_future_complete"),
            redirect_stdout(output),
        ):
            exit_code = main()

        self.assertEqual(exit_code, 1)
        self.assertEqual(cancel_client.request.task_id, task_client.request.task_id)
        self.assertIn("state unknown (service result failed: service transport failed)", output.getvalue())
        self.assertIn("success=False (停止状态未确认)", output.getvalue())

    def test_cancel_future_exception_is_reported_as_unknown(self):
        class _Future:
            def __init__(self, failure=None, response=None):
                self.failure = failure
                self.response = response

            def done(self):
                return True

            def result(self):
                if self.failure:
                    raise self.failure
                return self.response

        class _Client:
            def __init__(self, future):
                self.future = future

            def wait_for_service(self, timeout_sec):
                return True

            def call_async(self, _request):
                return self.future

        task_client = _Client(_Future(failure=RuntimeError("task transport failed")))
        cancel_client = _Client(_Future(failure=RuntimeError("cancel transport failed")))

        class _Node:
            def create_client(self, service_type, _name):
                return task_client if service_type is TaskExecute else cancel_client

            def destroy_node(self):
                pass

        output = io.StringIO()
        with (
            patch("sys.argv", ["send_task", "go", "--service"]),
            patch("robot_agent.send_task.Node", return_value=_Node()),
            patch("robot_agent.send_task.rclpy.init"),
            patch("robot_agent.send_task.rclpy.shutdown"),
            patch("robot_agent.send_task.rclpy.spin_until_future_complete"),
            redirect_stdout(output),
        ):
            exit_code = main()

        self.assertEqual(exit_code, 1)
        self.assertIn("ERROR: cancellation result unknown (cancel transport failed)", output.getvalue())


if __name__ == "__main__":
    unittest.main()
