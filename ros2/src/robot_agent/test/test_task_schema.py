"""Unit tests for Task / Plan schemas."""

import unittest
import math

from robot_agent.planner.task_schema import Plan, PlanValidationError, Task, validate_plan


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

    def test_plan_rejects_invalid_graph_and_actions(self):
        base = {"goal": "find cup", "tasks": [
            {"id": 1, "tool": "vision", "action": "find_object", "target": "cup"},
            {"id": 2, "tool": "robot", "action": "approach", "target": "cup", "depends_on": [1]},
        ]}
        cases = (
            {**base, "tasks": [base["tasks"][0], {**base["tasks"][1], "id": 1}]},
            {**base, "tasks": [base["tasks"][0], {**base["tasks"][1], "depends_on": [3]}]},
            {**base, "tasks": [{**base["tasks"][0], "depends_on": [2]}, base["tasks"][1]]},
            {**base, "tasks": [{**base["tasks"][0], "action": "teleport"}, base["tasks"][1]]},
            {**base, "tasks": [{**base["tasks"][0], "target": ""}, base["tasks"][1]]},
        )
        for payload in cases:
            with self.subTest(payload=payload):
                with self.assertRaises(PlanValidationError):
                    Plan.from_dict(payload)

    def test_plan_rejects_nonfinite_scores_and_motion(self):
        for field, value in (("importance", math.nan), ("params", {"duration": math.inf})):
            payload = {"goal": "move", "tasks": [{
                "id": 1,
                "tool": "robot",
                "action": "move",
                field: value,
            }]}
            with self.subTest(field=field):
                with self.assertRaises(PlanValidationError):
                    Plan.from_dict(payload)

    def test_task_from_dict_does_not_coerce_wrong_types(self):
        with self.assertRaises(PlanValidationError):
            Task.from_dict({"id": "1", "tool": "vision", "action": "find_object"})

    def test_action_params_reject_unknown_fields_and_invalid_numbers(self):
        cases = (
            Task(id=1, tool="nav", action="navigate", target="1,2", params={"retries": 2}),
            Task(id=1, tool="nav", action="navigate", target="1,2", params={"timeout": True}),
            Task(id=1, tool="nav", action="navigate", target="1,2", params={"timeout": 0}),
            Task(id=1, tool="robot", action="move", params={"linear_x": "0.3"}),
            Task(id=1, tool="robot", action="move", params={"linear_x": 1.1}),
            Task(id=1, tool="robot", action="move", params={"duration": -1}),
            Task(id=1, tool="robot", action="stop", params={"duration": 1}),
        )
        for task in cases:
            with self.subTest(task=task), self.assertRaises(PlanValidationError):
                validate_plan(Plan(goal="validate task", tasks=[task]))


if __name__ == "__main__":
    unittest.main()
