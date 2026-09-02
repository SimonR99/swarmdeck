"""Deterministic mission scheduling primitives for multiple robots.

This module intentionally contains no LLM and executes no commands. A planner
may propose this structure, but the mission executive validates dependencies,
approval, and per-robot serialization before a future FleetTools implementation
dispatches a wave.
"""

from __future__ import annotations

from typing import Any, Dict, List, Literal
from uuid import uuid4

from pydantic import BaseModel, Field, model_validator


TaskStatus = Literal[
    "pending", "ready", "running", "succeeded", "failed", "blocked", "cancelled"
]
MissionAction = Literal[
    "doctor", "deploy", "drive", "navigate", "cancel", "stop", "body"
]


class MissionTask(BaseModel):
    task_id: str = Field(default_factory=lambda: f"task_{uuid4().hex}")
    robot_id: str
    action: MissionAction
    parameters: Dict[str, Any] = Field(default_factory=dict)
    depends_on: List[str] = Field(default_factory=list)
    requires_approval: bool = True
    approved: bool = False
    status: TaskStatus = "pending"


class MissionPlan(BaseModel):
    mission_id: str = Field(default_factory=lambda: f"mission_{uuid4().hex}")
    objective: str
    tasks: List[MissionTask]

    @model_validator(mode="after")
    def validate_graph(self) -> "MissionPlan":
        ids = [task.task_id for task in self.tasks]
        if len(ids) != len(set(ids)):
            raise ValueError("mission task IDs must be unique")
        known = set(ids)
        for task in self.tasks:
            missing = set(task.depends_on) - known
            if missing:
                raise ValueError(
                    f"task {task.task_id} has unknown dependencies: {sorted(missing)}"
                )
            if task.task_id in task.depends_on:
                raise ValueError(f"task {task.task_id} cannot depend on itself")

        visiting: set[str] = set()
        visited: set[str] = set()
        dependencies = {task.task_id: task.depends_on for task in self.tasks}

        def visit(task_id: str) -> None:
            if task_id in visiting:
                raise ValueError("mission task dependencies contain a cycle")
            if task_id in visited:
                return
            visiting.add(task_id)
            for dependency in dependencies[task_id]:
                visit(dependency)
            visiting.remove(task_id)
            visited.add(task_id)

        for task_id in ids:
            visit(task_id)
        return self


class MissionExecutive:
    """Select safe parallel waves without dispatching them."""

    @staticmethod
    def ready_wave(plan: MissionPlan) -> List[MissionTask]:
        by_id = {task.task_id: task for task in plan.tasks}
        candidates: list[MissionTask] = []
        for task in plan.tasks:
            if task.status not in {"pending", "ready"}:
                continue
            dependencies = [by_id[task_id] for task_id in task.depends_on]
            if any(
                dependency.status in {"failed", "blocked", "cancelled"}
                for dependency in dependencies
            ):
                task.status = "blocked"
                continue
            if not all(dependency.status == "succeeded" for dependency in dependencies):
                continue
            if task.requires_approval and not task.approved:
                continue
            candidates.append(task)

        # A wave can operate multiple robots concurrently, but never dispatches
        # two tasks to the same robot at once. Emergency stop wins for a robot.
        candidates.sort(key=lambda task: 0 if task.action == "stop" else 1)
        wave: list[MissionTask] = []
        occupied_robots: set[str] = set()
        for task in candidates:
            if task.robot_id in occupied_robots:
                continue
            task.status = "ready"
            occupied_robots.add(task.robot_id)
            wave.append(task)
        return wave

    @staticmethod
    def mark_result(task: MissionTask, *, succeeded: bool) -> None:
        if task.status not in {"ready", "running"}:
            raise ValueError("only a ready or running task can receive a result")
        task.status = "succeeded" if succeeded else "failed"
