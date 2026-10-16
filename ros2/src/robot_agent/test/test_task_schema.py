"""Unit tests for Task / Plan schemas."""

import unittest

from robot_agent.planner.task_schema import Plan, Task


class TestTaskSchema(unittest.TestCase):
    def test_task_roundtrip(self):
        task = Task(
            id=1,
            tool="vision",
            action="find_object",
            target="cup",
            name="find cup",
            importance=90,
            urgency=80,
            complexity=60,
            dependency=50,
            params={"conf": 0.5},
            depends_on=[3],
        )
        data = task.to_dict()
        self.assertEqual(Task.from_dict(data), task)

    def test_plan_roundtrip(self):
        plan = Plan(
            goal="find cup",
            tasks=[Task(id=1, tool="vision", action="find_object", target="cup")],
        )
        data = plan.to_dict()
        restored = Plan.from_dict(data)
        self.assertEqual(restored.goal, "find cup")
        self.assertEqual(len(restored.tasks), 1)
        self.assertEqual(restored.tasks[0].target, "cup")


if __name__ == "__main__":
    unittest.main()
