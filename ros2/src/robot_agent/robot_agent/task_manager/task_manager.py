"""TaskManager: queue lifecycle + scheduling + status tracking."""

from __future__ import annotations

from copy import deepcopy
from typing import Dict, List, Optional

from robot_agent.planner.task_schema import Plan, Task, validate_plan
from robot_agent.task_manager.scheduler import Scheduler
from robot_agent.task_manager.task_state import TaskStatus


class TaskManager:
    def __init__(self, scheduler: Optional[Scheduler] = None) -> None:
        self._tasks: Dict[int, Task] = {}
        self._scheduler = scheduler or Scheduler()

    def add_task(self, task: Task) -> None:
        self._tasks[task.id] = task

    def add_plan(self, plan: Plan) -> None:
        validate_plan(plan)
        # ID remapping is local to this manager; don't mutate planner-owned plans
        # that may be returned again during a retry or a later run.
        plan = deepcopy(plan)
        used_ids = set(self._tasks)
        next_id = max(used_ids, default=0) + 1
        remap: Dict[int, int] = {}
        for task in plan.tasks:
            if task.id in used_ids or task.id in remap.values():
                while next_id in used_ids or next_id in remap.values():
                    next_id += 1
                remap[task.id] = next_id
                next_id += 1
            else:
                remap[task.id] = task.id
        for task in plan.tasks:
            task.id = remap[task.id]
            task.depends_on = [remap.get(dep_id, dep_id) for dep_id in task.depends_on]
            self.add_task(task)

    def next_task(self) -> Optional[Task]:
        return self._scheduler.next_task(list(self._tasks.values()))

    def set_status(self, task_id: int, status: TaskStatus) -> None:
        if task_id in self._tasks:
            self._tasks[task_id].status = status.value

    def mark_completed(self, task_id: int) -> None:
        self.set_status(task_id, TaskStatus.COMPLETED)

    def mark_failed(self, task_id: int) -> None:
        self.set_status(task_id, TaskStatus.FAILED)

    def status_counts(self) -> Dict[str, int]:
        counts = {s.value: 0 for s in TaskStatus}
        for task in self._tasks.values():
            counts[task.status] = counts.get(task.status, 0) + 1
        return counts

    def tasks(self) -> List[Task]:
        return list(self._tasks.values())

    def clear(self) -> None:
        """Discard all state from a completed, failed, or cancelled run."""
        self._tasks.clear()
