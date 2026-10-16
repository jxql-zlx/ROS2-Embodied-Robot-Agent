"""BaseLLM + providers: NL task -> ActionPlan.

Providers:
  - mock     : deterministic rule-based parser (default, works offline)
  - deepseek : DeepSeek chat completions (OpenAI-compatible)
  - openai   : any OpenAI-compatible chat completions endpoint
"""

from __future__ import annotations

import os
import re
import json
from abc import ABC, abstractmethod
from typing import Any, Dict, Optional

from robot_agent.action_plan import ActionPlan
from robot_agent.planner.task_schema import Plan, Task
from robot_agent.prompt import MOTION_PROMPT, PLAN_PROMPT, SYSTEM_PROMPT, parse_action_json


class LLMError(Exception):
    """Raised when the LLM cannot produce a valid action plan."""


class BaseLLM(ABC):
    """Unified LLM interface (swap provider without touching the agent)."""

    @abstractmethod
    def generate_action(self, task: str) -> ActionPlan:
        """Parse a natural-language task into a structured ActionPlan."""

    @abstractmethod
    def plan_motion(self, task: str, vision: Dict[str, Any]) -> ActionPlan:
        """Given a vision result, produce a motion ActionPlan (move/rotate/stop)."""

    @abstractmethod
    def plan_goal(self, goal: str) -> Plan:
        """Decompose a high-level goal into a Plan of Tasks."""


# ---------------------------------------------------------------------------
# Mock provider: deterministic rule-based parser (offline, no API key)
# ---------------------------------------------------------------------------

_STOP_WORDS = ("停止", "停下", "刹车", "停一下", "别动", "halt", "stop", "brake", "hold on")
_ROTATE_LEFT = ("左转", "左拐", "向左", "左旋", "turn left", "rotate left", "spin left")
_ROTATE_RIGHT = ("右转", "右拐", "向右", "右旋", "turn right", "rotate right", "spin right")
_BACKWARD = ("后退", "向后", "倒车", "backward", "back up", "reverse", "move back")
_FORWARD = ("前进", "向前", "直行", "往前走", "forward", "go straight", "move forward", "ahead")
_NAVIGATE = ("导航", "前往", "去", "到", "navigate", "go to", "travel to")
_VISION_FIND = ("寻找", "找到", "找", "find", "search", "locate")
_VISION_SCENE = ("有什么", "看到什么", "看见什么", "场景", "观察", "看看", "看到", "what", "see", "scene", "detect", "describe")

# Minimal Chinese -> English COCO object-name mapping for the offline mock.
_OBJECT_ZH2EN = {
    "杯子": "cup", "水杯": "cup", "瓶子": "bottle",
    "行人": "person", "人": "person",
    "公交车": "bus", "公共汽车": "bus", "汽车": "car", "车": "car",
    "椅子": "chair", "桌子": "dining table",
    "猫": "cat", "狗": "dog", "手机": "cell phone", "笔记本电脑": "laptop",
}

DEFAULT_SPEED = 0.3
DEFAULT_ANGULAR = 0.5
DEFAULT_MOVE_DURATION = 2.0
DEFAULT_ROTATE_DURATION = 3.14  # ~90 deg at 0.5 rad/s
_APPROACH_ROTATE_DURATION = 30.0 * 3.141592653589793 / 180.0 / DEFAULT_ANGULAR  # ~30 deg
_APPROACH_MOVE_DURATION = 0.5 / DEFAULT_SPEED  # ~0.5 m

# Common Chinese number words -> digits (enough for demo commands like 半米/九十度)
_CHINESE_NUM = {
    "零": "0", "一": "1", "二": "2", "两": "2", "三": "3", "四": "4",
    "五": "5", "六": "6", "七": "7", "八": "8", "九": "9", "十": "10", "半": "0.5",
}


def _first_number(text: str) -> Optional[float]:
    for zh, num in _CHINESE_NUM.items():
        text = text.replace(zh, num)
    match = re.search(r"-?\d+(?:\.\d+)?", text)
    return float(match.group()) if match else None


def _contains_any(text: str, words: tuple) -> bool:
    return any(word in text for word in words)


class MockLLM(BaseLLM):
    """Deterministic Chinese/English NL parser so the demo runs without API keys."""

    def generate_action(self, task: str) -> ActionPlan:
        task = (task or "").strip()
        t = task.lower()

        if _contains_any(t, _STOP_WORDS):
            return ActionPlan("stop", response="好的，机器人已停止。")

        if _contains_any(t, _VISION_FIND):
            target = self._extract_vision_target(task)
            return ActionPlan("approach", goal=target, response=f"正在寻找并靠近：{target}。")
        if _contains_any(t, _VISION_SCENE):
            return ActionPlan("vision", goal="", response="正在查看当前场景。")

        if _contains_any(t, _ROTATE_LEFT):
            return ActionPlan(
                "rotate",
                angular_z=+DEFAULT_ANGULAR,
                duration=self._rotate_duration(t),
                response="好的，机器人向左旋转。",
            )
        if _contains_any(t, _ROTATE_RIGHT):
            return ActionPlan(
                "rotate",
                angular_z=-DEFAULT_ANGULAR,
                duration=self._rotate_duration(t),
                response="好的，机器人向右旋转。",
            )

        if _contains_any(t, _NAVIGATE) and not _contains_any(t, _FORWARD + _BACKWARD):
            goal = self._extract_goal(task)
            return ActionPlan(
                "navigate",
                goal=goal,
                response=f"好的，准备导航到 {goal}（Navigation2 集成后可用）。",
            )

        if _contains_any(t, _BACKWARD):
            return ActionPlan(
                "move",
                linear_x=-DEFAULT_SPEED,
                duration=self._move_duration(t),
                response="好的，机器人向后移动。",
            )
        if _contains_any(t, _FORWARD):
            return ActionPlan(
                "move",
                linear_x=+DEFAULT_SPEED,
                duration=self._move_duration(t),
                response="好的，机器人向前移动。",
            )

        raise LLMError(
            f"无法理解任务（mock 模式）：{task!r}。"
            "可用示例：让机器人向前移动 / 后退半米 / 左转90度 / 停止"
        )

    def _move_duration(self, text: str) -> float:
        number = _first_number(text)
        if number is None:
            return DEFAULT_MOVE_DURATION
        if "秒" in text or text.endswith("s"):
            return number
        return number / DEFAULT_SPEED  # treat as meters

    def _rotate_duration(self, text: str) -> float:
        number = _first_number(text)
        if number is None:
            return DEFAULT_ROTATE_DURATION
        if "度" in text or "deg" in text:
            return abs(number) * 3.141592653589793 / 180.0 / DEFAULT_ANGULAR
        return number  # seconds

    def _extract_goal(self, task: str) -> str:
        for marker in ("导航到", "前往", "去", "到", "go to", "navigate to"):
            idx = task.lower().find(marker)
            if idx != -1:
                goal = task[idx + len(marker) :].strip(" 到。.!！?？")
                if goal:
                    return goal
        return task.strip()

    def _extract_vision_target(self, task: str) -> str:
        t = task.lower()
        for marker in ("find", "search", "locate", "寻找", "找到", "找"):
            idx = t.find(marker)
            if idx != -1:
                target = task[idx + len(marker) :]
                target = target.strip(" 的了一个些，。！？!? ")
                break
        else:
            target = task.strip()
        target = re.sub(r"^(一个|个|一下|一只|一条|a |an |the )", "", target)
        return self._translate_object(target.strip())

    def _translate_object(self, text: str) -> str:
        for zh, en in _OBJECT_ZH2EN.items():
            if zh in text:
                return en
        return text

    def plan_motion(self, task: str, vision: Dict[str, Any]) -> ActionPlan:
        objects = vision.get("objects", [])
        if not objects:
            return ActionPlan("stop", response="未发现目标。")
        direction = objects[0].get("direction", "unknown")
        if direction == "left":
            return ActionPlan(
                "rotate",
                angular_z=+DEFAULT_ANGULAR,
                duration=_APPROACH_ROTATE_DURATION,
                response="目标在左侧，向左旋转。",
            )
        if direction == "right":
            return ActionPlan(
                "rotate",
                angular_z=-DEFAULT_ANGULAR,
                duration=_APPROACH_ROTATE_DURATION,
                response="目标在右侧，向右旋转。",
            )
        if direction == "center":
            return ActionPlan(
                "move",
                linear_x=+DEFAULT_SPEED,
                duration=_APPROACH_MOVE_DURATION,
                response="目标在正前方，向前移动。",
            )
        return ActionPlan("stop", response="无法判断目标方向。")

    def plan_goal(self, goal: str) -> Plan:
        tasks = []
        text = goal.lower()
        nid = 0
        if any(w in text for w in ("找", "find", "search", "观察", "看到", "detect")):
            target = self._extract_vision_target(goal)
            nid += 1
            tasks.append(
                Task(
                    id=nid,
                    tool="vision",
                    action="find_object",
                    target=target,
                    name=f"find {target}",
                    importance=90,
                    urgency=80,
                    complexity=60,
                    dependency=90,
                )
            )
        if any(w in text for w in ("移动", "过去", "靠近", "前进", "move", "approach", "go")):
            nid += 1
            tasks.append(
                Task(
                    id=nid,
                    tool="robot",
                    action="move",
                    target="",
                    name="move to target",
                    importance=85,
                    urgency=75,
                    complexity=50,
                    dependency=0,
                    depends_on=[tasks[0].id] if tasks else [],
                )
            )
        if not tasks:
            nid += 1
            tasks.append(
                Task(
                    id=nid,
                    tool="robot",
                    action="infer",
                    target=goal,
                    name=goal,
                    importance=70,
                    urgency=60,
                    complexity=40,
                    dependency=0,
                )
            )
        return Plan(goal=goal, tasks=tasks)


# ---------------------------------------------------------------------------
# OpenAI-compatible chat completions (DeepSeek / OpenAI / compatible)
# ---------------------------------------------------------------------------


class ChatCompletionsLLM(BaseLLM):
    def __init__(
        self,
        model: str = "deepseek-chat",
        base_url: str = "https://api.deepseek.com/v1",
        api_key_env: str = "DEEPSEEK_API_KEY",
        temperature: float = 0.0,
        timeout: float = 20.0,
    ) -> None:
        try:
            import requests  # lazy: only needed for this provider
        except ImportError as exc:  # pragma: no cover
            raise LLMError("requests not installed; cannot use chat completions provider") from exc
        self._requests = requests
        self.model = model
        self.base_url = base_url.rstrip("/")
        self.api_key_env = api_key_env
        self.api_key = os.environ.get(api_key_env, "")
        self.temperature = temperature
        self.timeout = timeout

    def generate_action(self, task: str) -> ActionPlan:
        if not self.api_key:
            raise LLMError(f"API key environment variable {self.api_key_env} is not set")
        try:
            resp = self._requests.post(
                f"{self.base_url}/chat/completions",
                headers={
                    "Authorization": f"Bearer {self.api_key}",
                    "Content-Type": "application/json",
                },
                json={
                    "model": self.model,
                    "messages": [
                        {"role": "system", "content": SYSTEM_PROMPT},
                        {"role": "user", "content": f"用户指令：{task}"},
                    ],
                    "temperature": self.temperature,
                    "max_tokens": 256,
                },
                timeout=self.timeout,
            )
            resp.raise_for_status()
            content = resp.json()["choices"][0]["message"]["content"]
            data = parse_action_json(content)
            plan = ActionPlan.from_dict(data)
            plan.response = f"已生成动作：{plan.action}"
            return plan
        except LLMError:
            raise
        except Exception as exc:
            raise LLMError(f"LLM call failed: {exc}") from exc

    def plan_motion(self, task: str, vision: Dict[str, Any]) -> ActionPlan:
        if not self.api_key:
            raise LLMError(f"API key environment variable {self.api_key_env} is not set")
        try:
            resp = self._requests.post(
                f"{self.base_url}/chat/completions",
                headers={
                    "Authorization": f"Bearer {self.api_key}",
                    "Content-Type": "application/json",
                },
                json={
                    "model": self.model,
                    "messages": [
                        {"role": "system", "content": MOTION_PROMPT},
                        {
                            "role": "user",
                            "content": f"任务：{task}\n视觉结果：{json.dumps(vision, ensure_ascii=False)}",
                        },
                    ],
                    "temperature": self.temperature,
                    "max_tokens": 256,
                },
                timeout=self.timeout,
            )
            resp.raise_for_status()
            content = resp.json()["choices"][0]["message"]["content"]
            data = parse_action_json(content)
            plan = ActionPlan.from_dict(data)
            plan.response = f"已生成动作：{plan.action}"
            return plan
        except LLMError:
            raise
        except Exception as exc:
            raise LLMError(f"LLM motion planning failed: {exc}") from exc

    def plan_goal(self, goal: str) -> Plan:
        if not self.api_key:
            raise LLMError(f"API key environment variable {self.api_key_env} is not set")
        try:
            resp = self._requests.post(
                f"{self.base_url}/chat/completions",
                headers={
                    "Authorization": f"Bearer {self.api_key}",
                    "Content-Type": "application/json",
                },
                json={
                    "model": self.model,
                    "messages": [
                        {"role": "system", "content": PLAN_PROMPT},
                        {"role": "user", "content": goal},
                    ],
                    "temperature": self.temperature,
                    "max_tokens": 512,
                },
                timeout=self.timeout,
            )
            resp.raise_for_status()
            content = resp.json()["choices"][0]["message"]["content"]
            data = parse_action_json(content)
            return Plan.from_dict(data)
        except LLMError:
            raise
        except Exception as exc:
            raise LLMError(f"LLM planning failed: {exc}") from exc


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------


def load_llm(config: Any) -> BaseLLM:
    """Build an LLM from a config dict or a YAML file path."""
    if isinstance(config, str):
        try:
            import yaml
        except ImportError as exc:  # pragma: no cover
            raise LLMError("PyYAML not installed; cannot read LLM config") from exc
        with open(config, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f) or {}
    if not isinstance(config, dict):
        config = {}

    provider = str(config.get("provider", "mock")).lower()
    if provider in ("deepseek", "openai"):
        return ChatCompletionsLLM(
            model=str(config.get("model", "deepseek-chat")),
            base_url=str(config.get("base_url", "https://api.deepseek.com/v1")),
            api_key_env=str(config.get("api_key_env", "DEEPSEEK_API_KEY")),
            temperature=float(config.get("temperature", 0.0)),
            timeout=float(config.get("timeout", 20.0)),
        )
    if provider == "mock":
        return MockLLM()
    raise LLMError(f"unknown LLM provider: {provider}")
