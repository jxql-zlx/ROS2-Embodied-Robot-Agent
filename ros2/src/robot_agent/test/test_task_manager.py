"""Task dependency scheduling and plan ID remapping tests."""

import unittest

from robot_agent.planner.task_schema import Plan, Task
from robot_agent.task_manager.task_manager import TaskManager
from robot_agent.task_manager.task_state import TaskStatus


class TestTaskManager(unittest.TestCase):
    def test_dependent_task_waits_for_prerequisite(self):
        manager = TaskManager()
        manager.add_plan(
            Plan(
                goal="observe then move",
                tasks=[
                    Task(id=1, tool="vision", action="observe", importance=10),
                    Task(id=2, tool="robot", action="move", importance=100, depends_on=[1]),
                ],
            )
        )

        self.assertEqual(manager.next_task().id, 1)
        manager.mark_completed(1)
        self.assertEqual(manager.next_task().id, 2)

    def test_adding_plan_remaps_colliding_ids_and_dependencies(self):
        manager = TaskManager()
        manager.add_plan(Plan(goal="first", tasks=[Task(id=1, tool="vision", action="observe")]))
        manager.add_plan(
            Plan(
                goal="recovery",
                tasks=[
                    Task(id=1, tool="robot", action="rotate"),
                    Task(id=2, tool="vision", action="observe", depends_on=[1]),
                ],
            )
        )

        tasks = manager.tasks()
        self.assertEqual([task.id for task in tasks], [1, 2, 3])
        self.assertEqual(tasks[2].depends_on, [2])

    def test_failed_prerequisite_does_not_release_dependent_task(self):
        manager = TaskManager()
        manager.add_plan(
            Plan(
                goal="observe then move",
                tasks=[
                    Task(id=1, tool="vision", action="observe"),
                    Task(id=2, tool="robot", action="move", depends_on=[1]),
                ],
            )
        )
        manager.set_status(1, TaskStatus.FAILED)
        self.assertIsNone(manager.next_task())


if __name__ == "__main__":
    unittest.main()
