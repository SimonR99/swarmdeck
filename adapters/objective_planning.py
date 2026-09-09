"""Bounded ROS client for MGG navigate and return-home objectives."""

from __future__ import annotations

import math
import re
import threading
import time
from types import SimpleNamespace

from adapters.exploration import planner_path


class MggObjectivePlanning:
    """Request an MGG path, validate its snapshot, then execute all waypoints."""

    def __init__(self, bridge, config):
        from mgg_msgs.srv import PlanObjective
        from adapters.mapping_authority import get_mapping_authority

        self.bridge = bridge
        self.service_type = PlanObjective
        namespace = str(config.get("namespace") or f"/{bridge.id}/mgg").rstrip("/")
        self.client = bridge.node.create_client(
            PlanObjective, f"{namespace}/plan_objective"
        )
        self.authority_reader = get_mapping_authority(bridge)
        self.frame = str(
            config.get("frame") or getattr(bridge, "map_frame", f"{bridge.id}/map_frame")
        ).lstrip("/")
        self.component_id = str(config.get("component_id") or self.frame)
        self.timeout_s = max(0.1, float(config.get("objective_timeout_s", 30.0)))
        self.planar_tolerance_m = max(
            0.0, float(config.get("planar_tolerance_m", 0.10))
        )
        self.max_inclination_rad = max(
            0.0, float(config.get("max_inclination_rad", math.radians(30.0)))
        )
        self.authority_translation_tolerance_m = max(
            0.0, float(config.get("authority_translation_tolerance_m", 0.02))
        )
        self.authority_rotation_tolerance_rad = max(
            0.0, float(config.get("authority_rotation_tolerance_rad", 0.02))
        )
        self.home = config.get("home")
        self._active_lock = threading.Lock()
        self._active_route = None
        self._authority_timer = bridge.node.create_timer(
            max(0.05, float(config.get("authority_check_period_s", 0.1))),
            self._check_active_authority,
        )

    def navigate(self, goal: dict) -> bool:
        return self.plan("navigate", goal)

    def return_home(self, goal: dict | None = None) -> bool:
        # The authority anchor follows optimizer corrections. A caller-supplied
        # pose may have been resolved before the latest correction, so use it
        # only when no fresh authority anchor is available.
        target = self._authority_home() or goal or self.home
        if not isinstance(target, dict):
            self._fail("return-home requires planning.home or an explicit goal")
            return False
        return self.plan("return_home", target)

    def _authority(self):
        authority = self.authority_reader.current()
        if isinstance(authority, dict):
            return authority
        # Compatibility for injected coordinators that predate the shared
        # authority reader. Production coordinators use the same reader.
        exploration = getattr(self.bridge, "exploration", None)
        coordinator = getattr(exploration, "coordinator", None)
        authority = getattr(coordinator, "authority", None)
        if not isinstance(authority, dict):
            return None
        received_at = getattr(coordinator, "received_at", 0.0)
        if received_at and time.monotonic() - received_at > 3.0:
            return None
        return authority

    @staticmethod
    def _mapping_snapshot(authority: dict) -> tuple | None:
        keys = {
            "map_epoch",
            "mapping_graph_revision",
            "geometry_revision",
            "map_source_stamp",
        }
        if not keys.intersection(authority):
            return None
        try:
            stamp = authority["map_source_stamp"]
            epoch = int(authority["map_epoch"])
            revision = int(authority["mapping_graph_revision"])
            geometry = str(authority["geometry_revision"])
            sec, nanosec = int(stamp["sec"]), int(stamp["nanosec"])
        except (KeyError, TypeError, ValueError):
            raise ValueError("map authority has an incomplete snapshot key")
        if (
            epoch < 0
            or revision < 0
            or sec < 0
            or not 0 <= nanosec < 1_000_000_000
            or not re.fullmatch(r"[0-9a-fA-F]{64}", geometry)
        ):
            raise ValueError("map authority has an invalid snapshot key")
        return epoch, revision, geometry.lower(), sec, nanosec

    def _authority_home(self) -> dict | None:
        authority = self._authority()
        if not isinstance(authority, dict):
            return None
        home = authority.get("home")
        if not isinstance(home, dict):
            # Compatibility with authority envelopes emitted before home
            # became an explicit nested landmark contract.
            home = authority
        transform = home.get("T_navigation_home")
        try:
            matrix = [[float(value) for value in row] for row in transform]
            if len(matrix) != 4 or any(len(row) != 4 for row in matrix):
                return None
            values = [value for row in matrix for value in row]
            if not all(math.isfinite(value) for value in values):
                return None
            return {
                "x": matrix[0][3],
                "y": matrix[1][3],
                "z": matrix[2][3],
                "yaw": math.atan2(matrix[1][0], matrix[0][0]),
                "mission_id": authority.get("mission_id", ""),
                "component_id": authority.get("component_id", self.component_id),
                "landmark_id": home.get("keyframe_id", ""),
            }
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _authority_binding(authority: dict | None):
        """Capture only authority changes that can move an executing route."""

        if not isinstance(authority, dict):
            return None
        transform = authority.get("T_component_navigation")
        if transform is None:
            # Compatibility with pre-correction authorities. Such a route has
            # no transform that can be monitored and remains locally scoped.
            return None
        try:
            matrix = tuple(tuple(float(value) for value in row) for row in transform)
            if len(matrix) != 4 or any(len(row) != 4 for row in matrix):
                raise ValueError
            if not all(math.isfinite(value) for row in matrix for value in row):
                raise ValueError
            correction_revision = int(authority.get("correction_revision", 0))
            if correction_revision < 0:
                raise ValueError
        except (TypeError, ValueError):
            raise ValueError("map authority has an invalid navigation transform")
        return (
            str(authority.get("mission_id", "")),
            str(authority.get("component_id", "")),
            correction_revision,
            matrix,
        )

    def _route_authority_changed(self, original, current) -> bool:
        if original is None:
            return False
        try:
            updated = self._authority_binding(current)
        except ValueError:
            return True
        if updated is None or updated[:3] != original[:3]:
            return True
        before, after = original[3], updated[3]
        translation = math.sqrt(
            sum((before[index][3] - after[index][3]) ** 2 for index in range(3))
        )
        # trace(R_before^T R_after) is the Frobenius dot product. Clamp for
        # floating point noise before converting it to the relative angle.
        trace = sum(
            before[row][column] * after[row][column]
            for row in range(3)
            for column in range(3)
        )
        rotation = math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0)))
        # A newer graph/geometry/keyframe snapshot with the same correction
        # revision does not invalidate a route. A transform change over the
        # tolerance is also rejected even if a malformed producer reused its
        # correction revision.
        return (
            translation > self.authority_translation_tolerance_m
            or rotation > self.authority_rotation_tolerance_rad
        )

    def _check_active_authority(self) -> None:
        with self._active_lock:
            active = self._active_route
        if active is None:
            return
        generation, binding = active
        if generation != self.bridge._goal_generation or self.bridge.nav_status != "active":
            with self._active_lock:
                if self._active_route == active:
                    self._active_route = None
            return
        if not self._route_authority_changed(binding, self._authority()):
            return
        with self._active_lock:
            if (
                self._active_route != active
                or generation != self.bridge._goal_generation
            ):
                return
            self._active_route = None
            # Serialize cancellation with replacement-plan setup. Otherwise a
            # correction timer that inspected the old route could cancel the
            # new goal between its own cancel and request dispatch.
            self.bridge.cancel_goal()
            self._fail("MGG objective route canceled after map authority changed")

    def plan(self, objective: str, goal: dict) -> bool:
        kind = {
            "navigate": self.service_type.Request.NAVIGATE,
            "return_home": self.service_type.Request.RETURN_HOME,
        }.get(objective)
        if kind is None:
            raise ValueError(f"unsupported MGG objective {objective!r}")
        try:
            x, y = float(goal["x"]), float(goal["y"])
            z = float(goal.get("z", 0.0))
            yaw = float(goal.get("yaw", 0.0))
            graph_revision = int(goal.get("graph_revision", 0))
            map_revision = int(goal.get("map_revision", 0))
        except (KeyError, TypeError, ValueError) as exc:
            self._fail(f"invalid MGG objective goal: {exc}")
            return False
        if not all(math.isfinite(value) for value in (x, y, z, yaw)):
            self._fail("MGG objective goal contains a nonfinite value")
            return False

        with self._active_lock:
            self._active_route = None
            self.bridge.cancel_goal()
        generation = self.bridge._goal_generation
        if not self.client.wait_for_service(timeout_sec=min(3.0, self.timeout_s)):
            self._fail("MGG objective service is unavailable")
            return False

        request = self.service_type.Request()
        authority = self._authority() or {}
        try:
            mapping_snapshot = self._mapping_snapshot(authority)
            authority_binding = self._authority_binding(authority)
        except ValueError as exc:
            self._fail(str(exc))
            return False
        request.mission_id = str(
            goal.get("mission_id", authority.get("mission_id", ""))
        )
        request.objective = kind
        request.component_id = str(
            goal.get(
                "component_id", authority.get("component_id", self.component_id)
            )
        )
        if graph_revision < 0 or map_revision < 0:
            self._fail("MGG objective revisions cannot be negative")
            return False
        request.graph_revision = graph_revision
        request.map_revision = map_revision
        if mapping_snapshot is not None:
            if not hasattr(request, "map_epoch"):
                self._fail(
                    "installed mgg_msgs lacks indexed mapping snapshot fields"
                )
                return False
            (
                request.map_epoch,
                request.mapping_graph_revision,
                request.geometry_revision,
                request.map_source_stamp.sec,
                request.map_source_stamp.nanosec,
            ) = mapping_snapshot
        request.goal_landmark_id = str(goal.get("landmark_id", ""))
        request.goal.position.x = x
        request.goal.position.y = y
        request.goal.position.z = z
        request.goal.orientation.z = math.sin(yaw / 2.0)
        request.goal.orientation.w = math.cos(yaw / 2.0)
        future = self.client.call_async(request)
        deadline = time.monotonic() + self.timeout_s
        while not future.done() and time.monotonic() < deadline:
            if generation != self.bridge._goal_generation:
                future.cancel()
                return False
            time.sleep(0.02)
        if not future.done():
            future.cancel()
            self._fail("MGG objective request timed out")
            return False
        if generation != self.bridge._goal_generation:
            return False
        try:
            response = future.result()
        except Exception as exc:
            self._fail(f"MGG objective request failed: {exc}")
            return False
        if response.status != self.service_type.Response.SUCCEEDED:
            self._fail(response.reason or f"MGG planning status {response.status}")
            return False
        if response.component_id != request.component_id:
            self._fail("MGG returned a path from another map component")
            return False
        if request.graph_revision and response.graph_revision != request.graph_revision:
            self._fail("MGG returned a different graph revision")
            return False
        if request.map_revision and response.map_revision != request.map_revision:
            self._fail("MGG returned a different map revision")
            return False
        if mapping_snapshot is not None and (
            response.map_epoch != request.map_epoch
            or response.mapping_graph_revision != request.mapping_graph_revision
            or response.geometry_revision.lower() != request.geometry_revision
            or response.map_source_stamp.sec != request.map_source_stamp.sec
            or response.map_source_stamp.nanosec
            != request.map_source_stamp.nanosec
        ):
            self._fail("MGG returned a different indexed map snapshot")
            return False
        if self._route_authority_changed(authority_binding, self._authority()):
            self._fail("map authority changed while MGG planned the objective")
            return False

        stamp = self.bridge.node.get_clock().now().to_msg()
        path = SimpleNamespace(
            header=SimpleNamespace(frame_id=self.frame, stamp=stamp),
            poses=[
                SimpleNamespace(
                    header=SimpleNamespace(frame_id=self.frame), pose=pose
                )
                for pose in response.path
            ],
        )
        try:
            plan = planner_path(
                path,
                self.frame,
                self.planar_tolerance_m,
                self.max_inclination_rad,
            )
        except (AttributeError, TypeError, ValueError) as exc:
            self._fail(f"MGG returned an unsupported path: {exc}")
            return False
        if generation != self.bridge._goal_generation:
            return False
        submitted = bool(self.bridge.follow_path(plan))
        if submitted and authority_binding is not None:
            with self._active_lock:
                self._active_route = (
                    self.bridge._goal_generation,
                    authority_binding,
                )
        return submitted

    def _fail(self, message: str) -> None:
        self.bridge.node.get_logger().warning(f"[{self.bridge.id}] {message}")
        self.bridge.nav_status = "failed"


def configure_objective_planning(bridge):
    config = (bridge.cfg or {}).get("planning") or {}
    backend = str(
        config.get("backend") or (bridge.cfg or {}).get("planning_backend") or ""
    ).lower()
    bridge.objective_planner = (
        MggObjectivePlanning(bridge, config) if backend == "mgg" else None
    )
