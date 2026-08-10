"""Adapter registry: identity, capabilities, liveness, command routing.

The backend never knows what a robot runs underneath — only what its adapter
declared at `hello`.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any

from ..bus import bus, stamps

OFFLINE_AFTER_S = 4.0


@dataclass
class Robot:
    robot_id: str
    robot_type: str = "unknown"
    adapter: str = ""
    ros: str = ""
    coordinate_frame: str = "local"
    capabilities: list[str] = field(default_factory=list)
    footprint_radius: float = 0.3

    pose: dict[str, float] = field(default_factory=lambda: {"x": 0.0, "y": 0.0, "yaw": 0.0})
    battery: float | None = None
    mode: str = "idle"
    nav_status: str = "idle"
    goal: dict[str, float] | None = None
    planned_path: list[dict[str, float]] = field(default_factory=list)

    last_seen: float = field(default_factory=time.monotonic)
    last_attended: float = field(default_factory=time.monotonic)

    @property
    def online(self) -> bool:
        return (time.monotonic() - self.last_seen) < OFFLINE_AFTER_S

    @property
    def unattended_s(self) -> float:
        return time.monotonic() - self.last_attended

    def to_state(self) -> dict[str, Any]:
        return {
            "type": "robot_state",
            "robot_id": self.robot_id,
            "robot_type": self.robot_type,
            "pose": self.pose,
            "battery": self.battery,
            "mode": self.mode,
            "nav_status": self.nav_status,
            "goal": self.goal,
            "planned_path": self.planned_path,
            "capabilities": self.capabilities,
            # Forwarded to the GUI because the fleet is mixed: an AgileX Bunker
            # is 0.64 m circumscribed and a Scout Mini 0.42 m, and an operator
            # judging whether a robot fits through a gap needs it drawn at the
            # size it actually is. The adapter declares this at `hello`.
            "footprint_radius": self.footprint_radius,
            "unattended_s": round(self.unattended_s, 2),
            "online": self.online,
            **stamps(),
        }


class Registry:
    def __init__(self) -> None:
        self.robots: dict[str, Robot] = {}
        self._sinks: dict[str, Any] = {}  # robot_id -> adapter websocket

    def hello(self, msg: dict[str, Any], sink: Any) -> Robot:
        rid = msg["robot_id"]
        r = self.robots.get(rid) or Robot(robot_id=rid)
        r.robot_type = msg.get("robot_type", "unknown")
        r.adapter = msg.get("adapter", "")
        r.ros = msg.get("ros", "")
        r.coordinate_frame = (
            "merged" if msg.get("coordinate_frame") == "merged" else "local"
        )
        r.capabilities = list(msg.get("capabilities", []))
        r.footprint_radius = float(msg.get("footprint_radius", 0.3))
        r.last_seen = time.monotonic()
        self.robots[rid] = r
        self._sinks[rid] = sink
        return r

    def update_state(self, msg: dict[str, Any]) -> Robot | None:
        r = self.robots.get(msg.get("robot_id", ""))
        if not r:
            return None
        r.last_seen = time.monotonic()
        if "pose" in msg:
            r.pose = msg["pose"]
        if "battery" in msg:
            r.battery = msg["battery"]
        if "mode" in msg:
            r.mode = msg["mode"]
        if "nav_status" in msg:
            r.nav_status = msg["nav_status"]
        if "goal" in msg:
            r.goal = msg["goal"]
        if "planned_path" in msg:
            r.planned_path = list(msg["planned_path"] or [])[:200]
        return r

    def attend(self, robot_id: str) -> None:
        """Any operator interaction resets the neglect timer."""
        r = self.robots.get(robot_id)
        if r:
            r.last_attended = time.monotonic()

    def can(self, robot_id: str, cap: str) -> bool:
        r = self.robots.get(robot_id)
        return bool(r and cap in r.capabilities)

    def disconnect(self, robot_id: str) -> None:
        self._sinks.pop(robot_id, None)

    async def send(self, robot_id: str, msg: dict[str, Any]) -> bool:
        sink = self._sinks.get(robot_id)
        if sink is None:
            return False
        try:
            await sink.send_json(msg)
            return True
        except Exception:
            self.disconnect(robot_id)
            return False

    def goal_taken(self, goal: dict[str, float], exclude: str, tol: float = 0.5) -> str | None:
        """FR-N3: reject assigning the same goal to two robots."""
        for rid, r in self.robots.items():
            if rid == exclude or not r.goal:
                continue
            if abs(r.goal["x"] - goal["x"]) < tol and abs(r.goal["y"] - goal["y"]) < tol:
                return rid
        return None

    def snapshot(self) -> list[dict[str, Any]]:
        return [r.to_state() for r in self.robots.values()]


registry = Registry()
