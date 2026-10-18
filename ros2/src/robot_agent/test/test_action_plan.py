"""Unit tests for ActionPlan validation/normalization."""

import unittest
import math

from robot_agent.action_plan import ActionPlan


class TestActionPlan(unittest.TestCase):
    def test_from_dict_valid(self):
        plan = ActionPlan.from_dict({"action": "move", "linear_x": 0.3, "duration": 2.0})
        self.assertEqual(plan.action, "move")
        self.assertAlmostEqual(plan.linear_x, 0.3)
        self.assertEqual(plan.duration, 2.0)

    def test_from_dict_rejects_unknown_action(self):
        with self.assertRaises(ValueError):
            ActionPlan.from_dict({"action": "fly"})

    def test_to_command_dict_excludes_response(self):
        plan = ActionPlan("move", linear_x=0.3, duration=2.0, response="ok")
        command = plan.to_command_dict()
        self.assertNotIn("response", command)
        self.assertEqual(command["action"], "move")
        self.assertAlmostEqual(command["linear_x"], 0.3)

    def test_action_lowercased(self):
        plan = ActionPlan.from_dict({"action": "STOP"})
        self.assertEqual(plan.action, "stop")

    def test_vision_action_is_allowed(self):
        plan = ActionPlan.from_dict({"action": "vision", "goal": "cup"})
        self.assertEqual(plan.action, "vision")
        self.assertEqual(plan.goal, "cup")

    def test_approach_action_is_allowed(self):
        plan = ActionPlan.from_dict({"action": "approach", "goal": "cup"})
        self.assertEqual(plan.action, "approach")
        self.assertEqual(plan.goal, "cup")

    def test_from_dict_rejects_wrong_numeric_types_and_unknown_fields(self):
        for payload in (
            {"action": "move", "linear_x": True, "duration": 1},
            {"action": "move", "linear_x": "0.3", "duration": 1},
            {"action": "move", "linear_x": math.nan, "duration": 1},
            {"action": "navigate", "goal": "1,2", "unexpected": 1},
            {"action": "navigate", "goal": "1,2", "params": {"retry": 2}},
            {"action": "navigate", "goal": "1,2", "params": {"timeout": 0}},
        ):
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                ActionPlan.from_dict(payload)

    def test_direct_action_plan_construction_uses_the_same_validation(self):
        for kwargs in (
            {"linear_x": True, "duration": 1.0},
            {"linear_x": 0.3, "duration": math.inf},
            {"linear_x": 0.3, "duration": 31.0},
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                ActionPlan("move", **kwargs)

    def test_command_serialization_revalidates_mutated_plan(self):
        plan = ActionPlan("move", linear_x=0.3, duration=1.0)
        plan.linear_x = math.inf
        with self.assertRaises(ValueError):
            plan.to_command_dict()


if __name__ == "__main__":
    unittest.main()
