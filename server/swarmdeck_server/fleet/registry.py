"""Adapter registry: identity, capabilities, liveness, command routing.

The backend never knows what a robot runs underneath — only what its adapter
declared at `hello`.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any

from ..bus import bus, stamps

OFFLINE_AFTER_S = 4.0


def parse_footprint(value: Any) -> list[list[float]] | None:
    """Accept a finite 2D base-frame polygon from an adapter hello."""
    if not isinstance(value, list) or len(value) < 3:
        return None
    points: list[list[float]] = []
    for point in value:
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            return None
        try:
            x, y = float(point[0]), float(point[1])
        except (TypeError, ValueError):
            return None
        if not math.isfinite(x) or not math.isfinite(y):
            return None
        points.append([x, y])
    return points


@dataclass
class Robot:
    robot_id: str
    robot_type: str = "unknown"
    adapter: str = ""
    ros: str = ""
    # Where the adapter dialled in from. Observed, never configured: the
    # backend is always the listener, so the only truthful source for this is
    # the socket itself.
    peer: str = ""
    coordinate_frame: str = "local"
    capabilities: list[str] = field(default_factory=list)
    footprint_radius: float = 0.3
    # Optional polygon in the robot's reported base_frame, x forward / y left.
    footprint: list[list[float]] | None = None

    pose: dict[str, float] = field(default_factory=lambda: {"x": 0.0, "y": 0.0, "yaw": 0.0})
    battery: float | None = None
    mode: str = "idle"
    nav_status: str = "idle"
    goal: dict[str, float] | None = None
    planned_path: list[dict[str, float]] = field(default_factory=list)
    global_planned_path: list[dict[str, float]] = field(default_factory=list)
    local_planned_path: list[dict[str, float]] = field(default_factory=list)
    network: dict[str, Any] | None = None

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
            # What this robot says it is, and where it said it from. The
            # settings dialog used to ask an operator to type these in, which
            # created a second source of truth that could disagree with the
            # robot. Bringing up a new platform needs to SHOW what it reported,
            # not require it to be declared in advance.
            "adapter": self.adapter,
            "ros": self.ros,
            "peer": self.peer,
            "pose": self.pose,
            "battery": self.battery,
            "mode": self.mode,
            "nav_status": self.nav_status,
            "goal": self.goal,
            "planned_path": self.planned_path,
            "global_planned_path": self.global_planned_path,
            "local_planned_path": self.local_planned_path,
            "network": self.network,
            "capabilities": self.capabilities,
            # Forwarded to the GUI because the fleet is mixed: an AgileX Bunker
            # is 0.64 m circumscribed and a Scout Mini 0.42 m, and an operator
            # judging whether a robot fits through a gap needs it drawn at the
            # size it actually is. The adapter declares this at `hello`.
            "footprint_radius": self.footprint_radius,
            "footprint": self.footprint,
            "unattended_s": round(self.unattended_s, 2),
            "online": self.online,
            **stamps(),
        }


class Registry:
    def __init__(self) -> None:
        self.robots: dict[str, Robot] = {}
        self._sinks: dict[str, Any] = {}  # robot_id -> adapter websocket

    def hello(self, msg: dict[str, Any], sink: Any, peer: str = "") -> Robot:
        rid = msg["robot_id"]
        r = self.robots.get(rid) or Robot(robot_id=rid)
        r.robot_type = msg.get("robot_type", "unknown")
        r.adapter = msg.get("adapter", "")
        r.ros = msg.get("ros", "")
        # Only overwrite on a socket that knows its peer, so a reconnect
        # through a proxy cannot blank a previously good address.
        if peer:
            r.peer = peer
        r.coordinate_frame = (
            "merged" if msg.get("coordinate_frame") == "merged" else "local"
        )
        r.capabilities = list(msg.get("capabilities", []))
        r.footprint_radius = float(msg.get("footprint_radius", 0.3))
        if "footprint" in msg:
            r.footprint = parse_footprint(msg.get("footprint"))
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
        split_paths = (
            "global_planned_path" in msg or "local_planned_path" in msg
        )
        if "global_planned_path" in msg and msg["global_planned_path"]:
            r.global_planned_path = list(msg["global_planned_path"] or [])[:200]
        elif r.nav_status not in ("active", "nav"):
            r.global_planned_path = []
        if "local_planned_path" in msg:
            r.local_planned_path = list(msg["local_planned_path"] or [])[:200]
        if not split_paths and "planned_path" in msg and msg["planned_path"]:
            r.global_planned_path = r.planned_path.copy()
            r.local_planned_path = []
        if "network" in msg:
            r.network = msg["network"] if isinstance(msg["network"], dict) else None
        return r

    def attend(self, robot_id: str) -> None:
        """Any operator interaction resets the neglect timer."""
        r = self.robots.get(robot_id)
        if r:
            r.last_attended = time.monotonic()

    def can(self, robot_id: str, cap: str) -> bool:
        r = self.robots.get(robot_id)
        return bool(r and cap in r.capabilities)

    def has_sink(self, robot_id: str) -> bool:
        """Is there currently a socket commands for this robot would reach?"""
        return robot_id in self._sinks

    def disconnect(self, robot_id: str, sink: Any = None) -> None:
        """Retire a socket, but only if it is still the one commands go to.

        `robot_id` is stable across reconnects (protocol rule 5), so a robot
        whose link drops and comes back has TWO sockets alive for as long as it
        takes the server to notice the first one died — and the old socket's
        cleanup runs last. Popping unconditionally therefore unbinds the NEW
        socket, and the robot goes on reporting state over it while every
        command silently goes nowhere: `send` returns False and the dashboard
        still draws the robot as online, because `last_seen` keeps advancing.
        That includes `stop`.

        Passing the socket makes cleanup idempotent per-connection. `None` keeps
        the old unconditional behaviour for callers that genuinely mean "this
        robot is gone".
        """
        if sink is not None and self._sinks.get(robot_id) is not sink:
            return
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
