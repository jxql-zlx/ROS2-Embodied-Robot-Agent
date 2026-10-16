"""ROS service tests for concurrent task cancellation and stop handling."""

import threading
import time
import unittest

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_interfaces.srv import CancelTask, TaskExecute

from robot_agent.action_plan import ActionPlan
from robot_agent.robot_agent_node import RobotAgentNode


class _BlockingLLM:
    def __init__(self):
        self.entered = threading.Event()
        self.release = threading.Event()
        self._calls = 0

    def generate_action(self, _task):
        self._calls += 1
        if self._calls == 1:
            self.entered.set()
            self.release.wait(5.0)
        return ActionPlan("move", linear_x=0.3, duration=1.0)


class _Robot:
    def __init__(self):
        self.actions = []
        self._lock = threading.Lock()

    def execute_action(self, action):
        with self._lock:
            self.actions.append(dict(action))
        return True

    def stop(self):
        with self._lock:
            self.actions.append({"action": "stop"})
        return True


class _Vision:
    pass


class TestAgentCancellation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.llm = _BlockingLLM()
        self.robot = _Robot()
        self.agent = RobotAgentNode(llm=self.llm, tool=self.robot, vision_tool=_Vision())
        self.client_node = Node("agent_cancellation_test_client")
        self.task_client = self.client_node.create_client(TaskExecute, "/task_execute")
        self.cancel_client = self.client_node.create_client(CancelTask, "/task_cancel")
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(self.agent)
        self.executor.add_node(self.client_node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        self.assertTrue(self.task_client.wait_for_service(timeout_sec=3.0))
        self.assertTrue(self.cancel_client.wait_for_service(timeout_sec=3.0))

    def tearDown(self):
        self.llm.release.set()
        self.executor.shutdown(timeout_sec=2.0)
        self.spin_thread.join(timeout=2.0)
        self.client_node.destroy_node()
        self.agent.destroy_node()

    @staticmethod
    def _wait(future, timeout=2.0):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.result() if future.done() else None

    def test_cancel_service_runs_while_task_is_blocked_and_next_task_can_run(self):
        request = TaskExecute.Request()
        request.task = "前进"
        active_future = self.task_client.call_async(request)
        self.assertTrue(self.llm.entered.wait(2.0), "task never entered its blocking LLM call")

        busy_request = TaskExecute.Request()
        busy_request.task = "后退"
        busy_response = self._wait(self.task_client.call_async(busy_request))
        self.assertIsNotNone(busy_response)
        self.assertFalse(busy_response.success)
        self.assertIn("已有任务", busy_response.response)

        cancel_response = self._wait(self.cancel_client.call_async(CancelTask.Request()))
        self.assertIsNotNone(cancel_response, "cancel callback was blocked behind the active task")
        self.assertTrue(cancel_response.success)
        self.assertIn("已取消", cancel_response.response)

        next_request = TaskExecute.Request()
        next_request.task = "前进"
        next_response = self._wait(self.task_client.call_async(next_request))
        self.assertIsNotNone(next_response)
        self.assertTrue(next_response.success)
        self.assertEqual([action["action"] for action in self.robot.actions], ["stop", "move"])

        self.llm.release.set()
        active_response = self._wait(active_future)
        self.assertIsNotNone(active_response)
        self.assertFalse(active_response.success)
        self.assertIn("任务已取消", active_response.response)
        self.assertEqual([action["action"] for action in self.robot.actions], ["stop", "move"])

    def test_stop_task_service_cancels_an_active_task(self):
        request = TaskExecute.Request()
        request.task = "前进"
        active_future = self.task_client.call_async(request)
        self.assertTrue(self.llm.entered.wait(2.0))

        stop_request = TaskExecute.Request()
        stop_request.task = "停止"
        stop_response = self._wait(self.task_client.call_async(stop_request))
        self.assertIsNotNone(stop_response, "stop task callback was blocked behind active task")
        self.assertTrue(stop_response.success)
        self.assertIn("已取消", stop_response.response)
        self.assertEqual(self.robot.actions, [{"action": "stop"}])

        self.llm.release.set()
        active_response = self._wait(active_future)
        self.assertIsNotNone(active_response)
        self.assertFalse(active_response.success)
        self.assertIn("任务已取消", active_response.response)

    def test_stop_language_uses_the_stop_path_without_calling_llm(self):
        result = self.agent.handle_task("停止")
        self.assertTrue(result[0])
        self.assertEqual(self.robot.actions, [{"action": "stop"}])
        self.assertEqual(self.llm._calls, 0)


if __name__ == "__main__":
    unittest.main()
