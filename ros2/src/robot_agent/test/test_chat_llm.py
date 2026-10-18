"""Tests for the DeepSeek/OpenAI-compatible chat provider (HTTP mocked)."""

import os
import unittest
from unittest import mock

from robot_agent.llm import ChatCompletionsLLM, LLMError


class _FakeResponse:
    def __init__(self, payload):
        self._payload = payload

    def raise_for_status(self):
        return None

    def json(self):
        return self._payload


class TestChatCompletionsLLM(unittest.TestCase):
    def setUp(self):
        self._previous = os.environ.get("DEEPSEEK_API_KEY")
        os.environ["DEEPSEEK_API_KEY"] = "test-key"

    def tearDown(self):
        if self._previous is None:
            os.environ.pop("DEEPSEEK_API_KEY", None)
        else:
            os.environ["DEEPSEEK_API_KEY"] = self._previous

    @mock.patch("requests.post")
    def test_vision_action_is_parsed_from_chat_response(self, post):
        post.return_value = _FakeResponse(
            {"choices": [{"message": {"content": '{"action":"vision","goal":"cup"}'}}]}
        )
        llm = ChatCompletionsLLM()
        plan = llm.generate_action("帮我找杯子")
        self.assertEqual(plan.action, "vision")
        self.assertEqual(plan.goal, "cup")

    def test_missing_api_key_raises_clean_error(self):
        os.environ.pop("DEEPSEEK_API_KEY", None)
        llm = ChatCompletionsLLM()
        with self.assertRaises(LLMError):
            llm.generate_action("场景中有什么物体？")

    @mock.patch("requests.post")
    def test_plan_motion_is_parsed_from_chat_response(self, post):
        post.return_value = _FakeResponse(
            {"choices": [{"message": {"content": '{"action":"rotate","angular_z":0.5,"duration":1.05}'}}]}
        )
        llm = ChatCompletionsLLM()
        vision = {"found": True, "objects": [{"direction": "left"}]}
        plan = llm.plan_motion("找杯子", vision)
        self.assertEqual(plan.action, "rotate")
        self.assertGreater(plan.angular_z, 0)

    @mock.patch("requests.post")
    def test_plan_goal_is_parsed_from_chat_response(self, post):
        post.return_value = _FakeResponse(
            {
                "choices": [
                    {
                        "message": {
                            "content": '{"goal":"find cup","tasks":[{"id":1,"tool":"vision","action":"find_object","target":"cup"}]}'
                        }
                    }
                ]
            }
        )
        llm = ChatCompletionsLLM()
        plan = llm.plan_goal("找到杯子然后移动过去")
        self.assertEqual(plan.goal, "find cup")
        self.assertEqual(len(plan.tasks), 1)
        self.assertEqual(plan.tasks[0].tool, "vision")

    @mock.patch("requests.post")
    def test_plan_goal_rejects_unsafe_model_plan(self, post):
        post.return_value = _FakeResponse({"choices": [{"message": {"content":
            '{"goal":"move","tasks":[{"id":1,"tool":"robot","action":"move",'
            '"params":{"duration":0}}]}'
        }}]})
        with self.assertRaises(LLMError):
            ChatCompletionsLLM().plan_goal("move")

    @mock.patch("requests.post", side_effect=TimeoutError("mock timeout"))
    def test_plan_goal_http_timeout_is_reported_without_fallback(self, _post):
        with self.assertRaisesRegex(LLMError, "LLM planning failed"):
            ChatCompletionsLLM().plan_goal("move")


if __name__ == "__main__":
    unittest.main()
