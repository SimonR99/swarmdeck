"""Adapter registry: identity, capabilities, liveness, command routing.

The backend never knows what a robot runs underneath — only what its adapter
declared at `hello`.
"""

from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass, field
from typing import Any

from autonomy.live_mapping import display_path, validate_live_mapping
from autonomy.slam_status import peer_status

from ..bus import stamps

OFFLINE_AFTER_S = 4.0
MAX_NAV_FAILURE_REASON_LENGTH = 512


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

    pose: dict[str, float] = field(
        default_factory=lambda: {"x": 0.0, "y": 0.0, "yaw": 0.0}
    )
    exploration_status: str = "idle"
    exploration_reason: str | None = None
    # The waypoint the robot is exploring toward, while it has no known route.
    exploration_goal: dict | None = None
    fleet_exploration_status: str = "unknown"
    exploration_coordination: dict | None = None
    home_pose: dict[str, float] | None = None
    battery: float | None = None
    mode: str = "idle"
    nav_status: str = "idle"
    nav_failure_reason: str | None = None
    navigation_ready: bool | None = None
    goal: dict[str, float] | None = None
    planned_path: list[dict[str, float]] = field(default_factory=list)
    global_planned_path: list[dict[str, float]] = field(default_factory=list)
    local_planned_path: list[dict[str, float]] = field(default_factory=list)
    network: dict[str, Any] | None = None
    live_mapping: dict[str, Any] | None = None
    peer_slam: dict[str, Any] | None = None
    live_mapping_received_at: float = 0.0
    command_generation: int = 0

    last_seen: float = field(default_factory=time.monotonic)
    last_attended: float = field(default_factory=time.monotonic)

    @property
    def online(self) -> bool:
        return (time.monotonic() - self.last_seen) < OFFLINE_AFTER_S

    @property
    def unattended_s(self) -> float:
        return time.monotonic() - self.last_attended

    def to_state(self) -> dict[str, Any]:
        live_mapping = self.live_mapping if self.online else None
        if live_mapping is not None:
            live_mapping = {
                key: value
                for key, value in live_mapping.items()
                if key
                not in {"planned_path", "global_planned_path", "local_planned_path"}
            }
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
            "home_pose": self.home_pose,
            "exploration_status": self.exploration_status,
            "exploration_reason": self.exploration_reason,
            "exploration_goal": self.exploration_goal,
            "peer_slam": self.peer_slam if self.online else None,
            "live_mapping": live_mapping,
            "fleet_exploration_status": (
                self.fleet_exploration_status if self.online else "unknown"
            ),
            "exploration_coordination": (
                self.exploration_coordination if self.online else None
            ),
            "battery": self.battery,
            "mode": self.mode,
            "nav_status": self.nav_status,
            "nav_failure_reason": self.nav_failure_reason,
            "navigation_ready": self.navigation_ready,
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
    def __init__(self, *, epoch_store=None, command_guard=None) -> None:
        self.robots: dict[str, Robot] = {}
        self._sinks: dict[str, Any] = {}  # robot_id -> adapter websocket
        self.epoch_store = epoch_store
        self.command_guard = command_guard

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
        r.live_mapping = None
        r.peer_slam = None
        r.nav_failure_reason = None
        r.navigation_ready = None
        self.robots[rid] = r
        self._sinks[rid] = sink
        return r

    def update_state(self, msg: dict[str, Any]) -> Robot | None:
        r = self.robots.get(msg.get("robot_id", ""))
        if not r:
            return None
        mission = os.environ.get("SWARMDECK_MISSION_ID")
        floor = (
            self.epoch_store().map_epoch(r.robot_id, mission)
            if self.epoch_store and mission
            else None
        )
        if floor is not None:
            from autonomy.map_epochs import robot_run_id

            live = msg.get("live_mapping")
            peer = msg.get("peer_slam")
            authority = live if isinstance(live, dict) else peer
            if (
                not isinstance(authority, dict)
                or authority.get("mission_id") != mission
                or authority.get("robot_map_epoch") != floor
                or authority.get("run_id") != robot_run_id(mission, r.robot_id, floor)
            ):
                # An old socket can remain alive throughout the restart. Do
                # not let any of its goals, paths, home or authority reappear.
                return None
        r.last_seen = time.monotonic()
        try:
            r.peer_slam = (
                peer_status(msg.get("peer_slam"), r.robot_id, mission)
                if mission
                else None
            )
        except (KeyError, TypeError, ValueError, OverflowError):
            r.peer_slam = None
        try:
            r.live_mapping = (
                validate_live_mapping(msg.get("live_mapping"), r.robot_id)
                if mission
                else None
            )
            r.live_mapping_received_at = r.last_seen
        except (KeyError, TypeError, ValueError, OverflowError):
            r.live_mapping = None
        if "pose" in msg:
            r.pose = msg["pose"]
        if r.home_pose is None and floor is not None and r.live_mapping:
            home = r.live_mapping.get("home")
            if home:
                pose = home["T_navigation_home"]
                r.home_pose = {
                    "x": pose[0][3],
                    "y": pose[1][3],
                    "yaw": math.atan2(pose[1][0], pose[0][0]),
                }
        if "battery" in msg:
            r.battery = msg["battery"]
        if "mode" in msg:
            r.mode = msg["mode"]
        if msg.get("exploration_status") in {
            "idle",
            "starting",
            "exploring",
            "waiting",
            "locally_exhausted",
            "complete",
            "blocked",
            "stopped",
        }:
            r.exploration_status = msg["exploration_status"]
            reason = msg.get("exploration_reason")
            r.exploration_reason = (
                reason.strip()[:512] or None if isinstance(reason, str) else None
            )
        if "exploration_goal" in msg:
            goal = msg["exploration_goal"]
            r.exploration_goal = goal if isinstance(goal, dict) else None
        if msg.get("fleet_exploration_status") in {"unknown", "incomplete", "complete"}:
            r.fleet_exploration_status = msg["fleet_exploration_status"]
        coordination = msg.get("exploration_coordination")
        r.exploration_coordination = (
            coordination if isinstance(coordination, dict) else None
        )
        if "nav_status" in msg:
            r.nav_status = msg["nav_status"]
            if r.nav_status == "failed" and isinstance(
                msg.get("nav_failure_reason"), str
            ):
                reason = msg["nav_failure_reason"].strip()
                r.nav_failure_reason = reason[:MAX_NAV_FAILURE_REASON_LENGTH] or None
            else:
                # A new objective, stop, success, or a generic controller
                # state retires the prior planner diagnostic.
                r.nav_failure_reason = None
        elif "nav_failure_reason" in msg:
            if r.nav_status == "failed" and isinstance(
                msg.get("nav_failure_reason"), str
            ):
                reason = msg["nav_failure_reason"].strip()
                r.nav_failure_reason = reason[:MAX_NAV_FAILURE_REASON_LENGTH] or None
            else:
                r.nav_failure_reason = None
        if isinstance(msg.get("navigation_ready"), bool):
            r.navigation_ready = msg["navigation_ready"]
        if "goal" in msg:
            r.goal = msg["goal"]
        is_nav_active = r.nav_status in ("active", "nav") or bool(r.goal)
        split_paths = "global_planned_path" in msg or "local_planned_path" in msg
        if is_nav_active:
            if "global_planned_path" in msg:
                r.global_planned_path = display_path(
                    list(msg["global_planned_path"] or [])
                )
            if "local_planned_path" in msg:
                r.local_planned_path = display_path(
                    list(msg["local_planned_path"] or [])
                )
            if not split_paths and "planned_path" in msg:
                r.global_planned_path = display_path(list(msg["planned_path"] or []))
                r.local_planned_path = []
            r.planned_path = r.local_planned_path or r.global_planned_path
        else:
            r.global_planned_path = []
            r.local_planned_path = []
            r.planned_path = []
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
        robot = self.robots.get(robot_id)
        if robot is not None:
            robot.live_mapping = None
            robot.peer_slam = None

    def remove(self, robot_id: str) -> bool:
        """Remove a robot from the registry."""
        self._sinks.pop(robot_id, None)
        return self.robots.pop(robot_id, None) is not None

    async def send(self, robot_id: str, msg: dict[str, Any]) -> bool:
        moving = (
            msg.get("type") in {"plan_objective", "body_command"}
            or (
                msg.get("type") == "drive" and (msg.get("linear") or msg.get("angular"))
            )
            or (msg.get("type") == "explore" and msg.get("enabled"))
        )
        if moving and self.command_guard and self.command_guard(robot_id):
            return False
        sink = self._sinks.get(robot_id)
        if sink is None:
            return False
        robot = self.robots.get(robot_id)
        generation = robot.command_generation if robot else None
        if moving and robot and robot.live_mapping:
            live = robot.live_mapping
            msg = {
                **msg,
                "mission_id": live["mission_id"],
                "robot_map_epoch": live["robot_map_epoch"],
                "map_run_id": live["run_id"],
            }
        try:
            await sink.send_json(msg)
            if moving and robot and robot.command_generation != generation:
                return False
            return True
        except Exception:
            # The adapter may have reconnected while this send was waiting.
            # Retire only the socket that actually failed; removing the
            # replacement would leave telemetry online while all controls
            # silently route nowhere.
            self.disconnect(robot_id, sink)
            return False


registry = Registry()
