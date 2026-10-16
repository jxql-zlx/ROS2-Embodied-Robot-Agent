"""Prompt template + robust JSON extraction for LLM action generation."""

from __future__ import annotations

import json
import re
from typing import Any, Dict


SYSTEM_PROMPT = (
    "你是一个具身机器人 Agent 的任务规划器。把用户的中文/英文自然语言指令转成一条结构化动作，"
    "只输出一个 JSON 对象，不要输出任何其他文字或解释。\n"
    "允许的 action 取值：\n"
    '- "move": 平移。linear_x>0 前进，linear_x<0 后退；可带 angular_z 做弧线；'
    "duration 秒数，0 表示持续执行直到收到 stop。\n"
    '- "rotate": 原地旋转。angular_z>0 左转，<0 右转（rad/s）；duration 秒数。\n'
    '- "stop": 立即停止。其余字段为 0。\n'
    '- "navigate": 导航到目标点/地点（未来 Navigation2）。把目标写入 goal 字段。\n'
    '- "vision": 视觉查询。goal 是要找的物体名（英文 COCO 类别，如 "cup"）；goal 为空表示查看整个场景。\n'
    '- "approach": 找物并靠近。goal 是要找的物体名（英文 COCO 类别）；先视觉检测，再根据目标方向移动。\n'
    "输出 JSON 格式（字段名必须完全一致，值为数字/字符串，不要注释）：\n"
    '{"action":"move","linear_x":0.3,"linear_y":0.0,"angular_z":0.0,"duration":2.0,'
    '"goal":"","params":{}}\n'
    "示例：\n"
    '用户："让机器人向前移动" -> {"action":"move","linear_x":0.3,"linear_y":0.0,'
    '"angular_z":0.0,"duration":2.0,"goal":"","params":{}}\n'
    '用户："后退半米" -> {"action":"move","linear_x":-0.3,"linear_y":0.0,'
    '"angular_z":0.0,"duration":1.67,"goal":"","params":{}}\n'
    '用户："左转90度" -> {"action":"rotate","linear_x":0.0,"linear_y":0.0,'
    '"angular_z":0.5,"duration":3.14,"goal":"","params":{}}\n'
    '用户："停止" -> {"action":"stop","linear_x":0.0,"linear_y":0.0,"angular_z":0.0,'
    '"duration":0.0,"goal":"","params":{}}\n'
    '用户："去厨房" -> {"action":"navigate","linear_x":0.0,"linear_y":0.0,'
    '"angular_z":0.0,"duration":0.0,"goal":"kitchen","params":{}}\n'
    '用户："帮我找杯子" -> {"action":"approach","linear_x":0.0,"linear_y":0.0,'
    '"angular_z":0.0,"duration":0.0,"goal":"cup","params":{}}\n'
    '用户："场景中有什么物体？" -> {"action":"vision","linear_x":0.0,"linear_y":0.0,'
    '"angular_z":0.0,"duration":0.0,"goal":"","params":{}}\n'
    "默认速度：linear_x 用 0.3 m/s，angular_z 用 0.5 rad/s。"
    "如果指令给出距离（米）或角度（度），换算成持续时间；如果给了明确速度，按给定速度执行。"
)


MOTION_PROMPT = (
    "你是具身机器人 Agent 的运动规划器。根据视觉检测结果，输出让机器人靠近目标的运动动作，"
    "只输出一个 JSON 对象，不要输出其他文字。\n"
    "规则：目标在图像左侧 -> rotate（angular_z>0 左转）；右侧 -> rotate（angular_z<0 右转）；"
    "中间 -> move（linear_x>0 前进）。若没有目标，输出 stop。\n"
    "默认速度 linear_x=0.3 m/s，angular_z=0.5 rad/s；角度/距离换算成 duration 秒数。\n"
    '输出格式：{"action":"move|rotate|stop","linear_x":0.0,"linear_y":0.0,'
    '"angular_z":0.0,"duration":0.0,"goal":"","params":{}}\n'
)


PLAN_PROMPT = (
    "你是具身机器人 Agent 的高层规划器。把用户的复杂目标拆解成有序的任务列表。\n"
    "只输出一个 JSON 对象，不要输出其他文字。\n"
    "可用 tool：vision（视觉）、robot（运动）、nav（导航，暂未实现）。\n"
    "vision 可用 action：find_object（target=物体英文名）、observe。\n"
    "robot 可用 action：move、rotate、stop。\n"
    "每个任务字段：id、tool、action、target、name、importance、urgency、complexity、"
    "dependency（0-100 优先级分数）和 depends_on（前置任务 id 数组）。\n"
    '输出格式：{"goal":"...","tasks":[{"id":1,"tool":"vision","action":"find_object",'
    '"target":"cup","name":"find cup","importance":90,"urgency":80,"complexity":60,'
    '"dependency":0,"depends_on":[]}]}\n'
)


def parse_action_json(text: str) -> Dict[str, Any]:
    """Extract the first JSON object from LLM output (tolerates markdown fences)."""
    text = text.strip()
    fence = re.search(r"```(?:json)?\s*(.*?)```", text, re.DOTALL)
    if fence:
        text = fence.group(1).strip()
    start = text.find("{")
    end = text.rfind("}")
    if start == -1 or end == -1 or end < start:
        raise ValueError(f"no JSON object found in LLM output: {text[:200]!r}")
    return json.loads(text[start : end + 1])
