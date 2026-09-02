"""Cortex Skills & Slash Commands Registry.

Provides structured shortcuts and autonomous workflows for fleet operations and codebase tasks.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional


@dataclass
class CortexSkill:
    command: str
    name: str
    description: str
    usage: str
    examples: List[str]
    category: str


BUILTIN_SKILLS: List[CortexSkill] = [
    CortexSkill(
        command="/see",
        name="Visual Inspection",
        description="Inspect what a robot is currently seeing through its camera, including detections and proposals",
        usage="/see [@robot_id]",
        examples=["/see @aslan_0", "/see @botman_0"],
        category="Perception",
    ),
    CortexSkill(
        command="/drive",
        name="Velocity Drive",
        description="Drive a robot with linear and angular velocity for a specified duration",
        usage="/drive [@robot_id] <linear_m_s> <angular_rad_s> [duration_sec]",
        examples=["/drive @aslan_0 0.3 0.0 2.0", "/drive @botman_0 0.0 0.5 1.5"],
        category="Motion",
    ),
    CortexSkill(
        command="/nav",
        name="Waypoint Navigation",
        description="Dispatch a global A* navigation goal to coordinate (x, y)",
        usage="/nav [@robot_id] <x> <y> [yaw]",
        examples=["/nav @aslan_0 1.5 2.0", "/nav @botman_0 -1.0 0.5 1.57"],
        category="Navigation",
    ),
    CortexSkill(
        command="/stop",
        name="Emergency Stop",
        description="Immediately stop all motion and cancel active goals for a robot or the entire fleet",
        usage="/stop [@robot_id | all]",
        examples=["/stop all", "/stop @aslan_0"],
        category="Safety",
    ),
    CortexSkill(
        command="/status",
        name="Fleet Diagnostics",
        description="Check online state, camera frames, video publication, and robot services",
        usage="/status",
        examples=["/status"],
        category="Diagnostics",
    ),
    CortexSkill(
        command="/doctor",
        name="Robot Doctor",
        description="Run evidence-based telemetry, camera, RTSP, SSH, and container checks",
        usage="/doctor [@robot_id | all]",
        examples=["/doctor all", "/doctor @tars_0"],
        category="Diagnostics",
    ),
    CortexSkill(
        command="/code",
        name="Codebase Modification",
        description="Instruct Cortex to inspect, edit, refactor, or test SwarmDeck files",
        usage="/code <task_description>",
        examples=["/code add a battery warning icon to the topbar", "/code verify server endpoints with pytest"],
        category="Development",
    ),
    CortexSkill(
        command="/deploy",
        name="Deploy / Restart Robot",
        description="Build and deploy/restart robot hardware profiles using make deploy ROBOT=...",
        usage="/deploy [spot | aslan | botman | tars | all]",
        examples=["/deploy spot", "/deploy aslan", "/deploy all"],
        category="Deployment",
    ),
    CortexSkill(
        command="/restart",
        name="Restart Robot Profile",
        description="Restart software stack and reset adapters for a robot",
        usage="/restart [spot | aslan | botman | tars | all]",
        examples=["/restart spot", "/restart aslan"],
        category="Deployment",
    ),
]


def get_skills_dict() -> List[Dict[str, Any]]:
    return [
        {
            "command": s.command,
            "name": s.name,
            "description": s.description,
            "usage": s.usage,
            "examples": s.examples,
            "category": s.category,
        }
        for s in BUILTIN_SKILLS
    ]
