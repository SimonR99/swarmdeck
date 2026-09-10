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
            config.get("frame")
            or getattr(bridge, "map_frame", f"{bridge.id}/map_frame")
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
        self.replan_max_attempts = self._bounded_int(
            config.get("authority_replan_max_attempts", 3), 1, 20, 3
        )
        self.replan_deadline_s = self._bounded_float(
            config.get("authority_replan_deadline_s", 15.0), 0.1, 300.0, 15.0
        )
        self.replan_backoff_s = self._bounded_float(
            config.get("authority_replan_backoff_s", 0.25), 0.0, 10.0, 0.25
        )
        self.home = config.get("home")
        self._active_lock = threading.Lock()
        self._active_route = None
        self._home_intent = None
        self._recovery_generation = None
        self._recovery_deadline = None
        self._recovery_thread = None
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
        authority = self._authority()
        authority_home = self._authority_home_from(authority)
        target = authority_home or goal or self.home
        if not isinstance(target, dict):
            generation = self.bridge.cancel_goal()
            self._fail_if_current(
                "return-home requires planning.home or an explicit goal", generation
            )
            return False
        # Recovery is allowed only for an identity obtained from the current
        # authority envelope. Caller/config fallbacks are intentionally local.
        intent = self._authority_home_identity(authority) if authority_home else None
        if authority_home is not None and (intent is None or intent[4] != self.frame):
            generation = self.bridge.cancel_goal()
            self._fail_if_current(
                "return-home authority has an invalid mission, epoch, or frame",
                generation,
            )
            return False
        outcome, message, generation = self._plan_once(
            "return_home", target, authority=authority, home_intent=intent
        )
        if outcome == "submitted":
            return True
        if outcome == "authority_changed" and intent is not None:
            if self._start_recovery(generation, intent, cancel_route=False):
                return True
        if outcome != "superseded":
            self._fail_if_current(message, generation)
        return False

    @staticmethod
    def _bounded_float(value, lower, upper, default):
        try:
            parsed = float(value)
        except (TypeError, ValueError, OverflowError):
            return default
        return min(upper, max(lower, parsed)) if math.isfinite(parsed) else default

    @staticmethod
    def _bounded_int(value, lower, upper, default):
        try:
            parsed = int(value)
        except (TypeError, ValueError, OverflowError):
            return default
        return min(upper, max(lower, parsed))

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
        return self._authority_home_from(self._authority())

    def _authority_home_from(self, authority) -> dict | None:
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
    def _home_identity(goal: dict):
        if not isinstance(goal, dict):
            return None
        identity = tuple(
            str(goal.get(key, ""))
            for key in ("mission_id", "component_id", "landmark_id")
        )
        return identity if all(identity) else None

    @staticmethod
    def _authority_home_identity(authority):
        if not isinstance(authority, dict):
            return None
        home = authority.get("home")
        if not isinstance(home, dict):
            return None
        try:
            mission_id = authority["mission_id"]
            component_id = authority["component_id"]
            landmark_id = home["keyframe_id"]
            map_epoch = authority["map_epoch"]
            navigation_frame = authority["navigation_frame"]
        except KeyError:
            return None
        if (
            any(
                not isinstance(value, str) or not value
                for value in (
                    mission_id,
                    component_id,
                    landmark_id,
                    navigation_frame,
                )
            )
            or type(map_epoch) is not int
            or map_epoch < 0
        ):
            return None
        return (
            mission_id,
            component_id,
            landmark_id,
            map_epoch,
            navigation_frame.lstrip("/"),
        )

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
        generation, binding, home_intent = active
        if (
            generation != self.bridge._goal_generation
            or self.bridge.nav_status != "active"
        ):
            with self._active_lock:
                if self._active_route == active:
                    self._active_route = None
                    self._home_intent = None
            return
        authority = self._authority()
        home_changed = home_intent is not None and (
            self._authority_home_identity(authority) != home_intent
        )
        if not self._route_authority_changed(binding, authority) and not home_changed:
            return
        with self._active_lock:
            if (
                self._active_route != active
                or generation != self.bridge._goal_generation
            ):
                return
            self._active_route = None
        if home_intent is not None and self._authority_matches_home(
            authority, home_intent
        ):
            if self._start_recovery(generation, home_intent, cancel_route=True):
                return
        canceled_generation = self.bridge.cancel_goal_if_current(generation)
        if canceled_generation is not None:
            self._fail_if_current(
                "MGG objective route canceled after map authority changed",
                canceled_generation,
            )

    def plan(self, objective: str, goal: dict) -> bool:
        outcome, message, generation = self._plan_once(objective, goal)
        if outcome != "submitted" and outcome != "superseded":
            self._fail_if_current(message, generation)
        return outcome == "submitted"

    def _plan_once(
        self,
        objective: str,
        goal: dict,
        *,
        authority=None,
        expected_generation: int | None = None,
        home_intent=None,
        overall_deadline: float | None = None,
    ) -> tuple[str, str, int]:
        kind = {
            "navigate": self.service_type.Request.NAVIGATE,
            "return_home": self.service_type.Request.RETURN_HOME,
        }.get(objective)
        if kind is None:
            raise ValueError(f"unsupported MGG objective {objective!r}")

        if expected_generation is None:
            with self._active_lock:
                self._active_route = None
                self._home_intent = home_intent
                self._recovery_generation = None
                self._recovery_deadline = None
            generation = self.bridge.cancel_goal()
            if not self.bridge.set_goal_pending_if_current(generation):
                return "superseded", "", generation
        else:
            generation = expected_generation
            if not self._owns_recovery(generation, home_intent):
                return "superseded", "", generation
        try:
            x, y = float(goal["x"]), float(goal["y"])
            z = float(goal.get("z", 0.0))
            yaw = float(goal.get("yaw", 0.0))
            graph_revision = int(goal.get("graph_revision", 0))
            map_revision = int(goal.get("map_revision", 0))
        except (KeyError, TypeError, ValueError) as exc:
            return "failed", f"invalid MGG objective goal: {exc}", generation
        if not all(math.isfinite(value) for value in (x, y, z, yaw)):
            return "failed", "MGG objective goal contains a nonfinite value", generation
        quiet_deadline = min(
            time.monotonic() + self.timeout_s,
            overall_deadline if overall_deadline is not None else math.inf,
        )
        if not self.bridge.wait_goal_quiet(generation, quiet_deadline):
            if not self._owns_generation(generation, expected_generation, home_intent):
                return "superseded", "", generation
            return (
                "failed",
                "previous navigation goal did not reach a terminal state",
                generation,
            )
        remaining = self._remaining(overall_deadline)
        if remaining <= 0.0:
            return "failed", "MGG return-home recovery deadline expired", generation
        if not self.client.wait_for_service(
            timeout_sec=min(3.0, self.timeout_s, remaining)
        ):
            if not self._owns_generation(generation, expected_generation, home_intent):
                return "superseded", "", generation
            return "failed", "MGG objective service is unavailable", generation
        if not self._owns_generation(generation, expected_generation, home_intent):
            return "superseded", "", generation

        request = self.service_type.Request()
        authority = (
            authority if isinstance(authority, dict) else self._authority() or {}
        )
        try:
            mapping_snapshot = self._mapping_snapshot(authority)
            authority_binding = self._authority_binding(authority)
        except ValueError as exc:
            return "failed", str(exc), generation
        request.mission_id = str(
            goal.get("mission_id", authority.get("mission_id", ""))
        )
        request.objective = kind
        request.component_id = str(
            goal.get("component_id", authority.get("component_id", self.component_id))
        )
        if graph_revision < 0 or map_revision < 0:
            return "failed", "MGG objective revisions cannot be negative", generation
        request.graph_revision = graph_revision
        request.map_revision = map_revision
        if mapping_snapshot is not None:
            if not hasattr(request, "map_epoch"):
                return (
                    "failed",
                    "installed mgg_msgs lacks indexed mapping snapshot fields",
                    generation,
                )
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
        try:
            future = self.client.call_async(request)
        except Exception as exc:
            return "failed", f"MGG objective request failed: {exc}", generation
        deadline = min(
            time.monotonic() + self.timeout_s,
            overall_deadline if overall_deadline is not None else math.inf,
        )
        while not future.done() and time.monotonic() < deadline:
            if not self._owns_generation(generation, expected_generation, home_intent):
                future.cancel()
                return "superseded", "", generation
            time.sleep(0.02)
        if not future.done():
            future.cancel()
            return "failed", "MGG objective request timed out", generation
        if not self._owns_generation(generation, expected_generation, home_intent):
            return "superseded", "", generation
        try:
            response = future.result()
        except Exception as exc:
            return "failed", f"MGG objective request failed: {exc}", generation
        if response.status != self.service_type.Response.SUCCEEDED:
            return (
                "failed",
                response.reason or f"MGG planning status {response.status}",
                generation,
            )
        if response.component_id != request.component_id:
            return (
                "failed",
                "MGG returned a path from another map component",
                generation,
            )
        if request.graph_revision and response.graph_revision != request.graph_revision:
            return "failed", "MGG returned a different graph revision", generation
        if request.map_revision and response.map_revision != request.map_revision:
            return "failed", "MGG returned a different map revision", generation
        if mapping_snapshot is not None and (
            response.map_epoch != request.map_epoch
            or response.mapping_graph_revision != request.mapping_graph_revision
            or response.geometry_revision.lower() != request.geometry_revision
            or response.map_source_stamp.sec != request.map_source_stamp.sec
            or response.map_source_stamp.nanosec != request.map_source_stamp.nanosec
        ):
            return (
                "failed",
                "MGG returned a different indexed map snapshot",
                generation,
            )
        current_authority = self._authority()
        try:
            current_snapshot = self._mapping_snapshot(current_authority or {})
        except ValueError:
            current_snapshot = object()
        if (
            self._route_authority_changed(authority_binding, current_authority)
            or current_snapshot != mapping_snapshot
        ):
            return (
                "authority_changed",
                "map authority changed while MGG planned the objective",
                generation,
            )

        stamp = self.bridge.node.get_clock().now().to_msg()
        path = SimpleNamespace(
            header=SimpleNamespace(frame_id=self.frame, stamp=stamp),
            poses=[
                SimpleNamespace(header=SimpleNamespace(frame_id=self.frame), pose=pose)
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
            return "failed", f"MGG returned an unsupported path: {exc}", generation
        if not self._owns_generation(generation, expected_generation, home_intent):
            return "superseded", "", generation
        if self._remaining(overall_deadline) <= 0.0:
            return "failed", "MGG return-home recovery deadline expired", generation
        pre_submit_rejected = [False]

        def validate_authority_for_dispatch():
            valid = self._authority_matches_plan(
                authority_binding, mapping_snapshot, home_intent
            )
            if not valid:
                pre_submit_rejected[0] = True
            return valid

        submitted_generation = self.bridge.follow_path(
            plan,
            expected_generation=generation,
            not_after=overall_deadline,
            pre_submit=validate_authority_for_dispatch,
        )
        if type(submitted_generation) is not int:
            if pre_submit_rejected[0]:
                return (
                    "authority_changed",
                    "map authority changed before MGG route dispatch",
                    generation,
                )
            if not self._owns_generation(generation, expected_generation, home_intent):
                return "superseded", "", generation
            if submitted_generation is None:
                if self._remaining(overall_deadline) <= 0.0:
                    return (
                        "failed",
                        "MGG return-home recovery deadline expired",
                        generation,
                    )
                return "superseded", "", generation
            return "failed", "MGG route could not be submitted", generation
        with self._active_lock:
            if self.bridge._goal_generation != submitted_generation:
                return "superseded", "", generation
            if expected_generation is not None and (
                self._recovery_generation != generation
                or self._home_intent != home_intent
            ):
                return "superseded", "", generation
            self._recovery_generation = None
            self._recovery_deadline = None
            self._home_intent = home_intent
            if authority_binding is not None:
                self._active_route = (
                    submitted_generation,
                    authority_binding,
                    home_intent,
                )
        return "submitted", "", submitted_generation

    @staticmethod
    def _remaining(deadline: float | None) -> float:
        if deadline is None:
            return math.inf
        return max(0.0, deadline - time.monotonic())

    def _owns_generation(self, generation, expected_generation, home_intent) -> bool:
        if generation != self.bridge._goal_generation:
            return False
        return expected_generation is None or self._owns_recovery(
            generation, home_intent
        )

    def _owns_recovery(self, generation, home_intent) -> bool:
        with self._active_lock:
            return (
                generation == self.bridge._goal_generation
                and self._recovery_generation == generation
                and self._home_intent == home_intent
            )

    def _authority_matches_home(self, authority, home_intent) -> bool:
        home = self._authority_home_from(authority)
        if (
            home is None
            or self._authority_home_identity(authority) != home_intent
            or self._home_identity(home) != home_intent[:3]
            or home_intent[4] != self.frame
        ):
            return False
        try:
            return self._authority_binding(authority) is not None
        except ValueError:
            return False

    def _authority_matches_plan(self, binding, snapshot, home_intent) -> bool:
        authority = self._authority()
        if home_intent is not None and (
            self._authority_home_identity(authority) != home_intent
            or home_intent[4] != self.frame
        ):
            return False
        try:
            if self._route_authority_changed(binding, authority):
                return False
            return self._mapping_snapshot(authority or {}) == snapshot
        except ValueError:
            return False

    def _start_recovery(self, generation, home_intent, *, cancel_route) -> bool:
        if cancel_route:
            generation = self.bridge.cancel_goal_if_current(generation, pending=True)
            if generation is None:
                return False
        elif not self.bridge.set_goal_pending_if_current(generation):
            return False
        deadline = time.monotonic() + self.replan_deadline_s
        with self._active_lock:
            if generation != self.bridge._goal_generation:
                return False
            self._home_intent = home_intent
            self._recovery_generation = generation
            self._recovery_deadline = deadline
            if self._recovery_thread is not None:
                return True
            worker = threading.Thread(
                target=self._recovery_worker,
                name=f"{self.bridge.id}-return-home-replan",
                daemon=True,
            )
            self._recovery_thread = worker
            try:
                worker.start()
            except Exception:
                self._recovery_thread = None
                self._recovery_generation = None
                self._recovery_deadline = None
                self._home_intent = None
                raise
        return True

    def _recovery_worker(self) -> None:
        while True:
            with self._active_lock:
                generation = self._recovery_generation
                home_intent = self._home_intent
                deadline = self._recovery_deadline
                if generation is None or home_intent is None or deadline is None:
                    self._recovery_thread = None
                    return
            try:
                self._recover_home(generation, home_intent, deadline)
            except Exception as exc:
                self._finish_recovery_failure(
                    generation, f"return-home recovery failed: {exc}"
                )

    def _recover_home(self, generation, home_intent, deadline) -> None:
        last_error = "no fresh map authority was available"
        attempts = 0
        if not self.bridge.wait_goal_quiet(generation, deadline):
            if self._owns_recovery(generation, home_intent):
                self._finish_recovery_failure(
                    generation,
                    "return-home cancellation did not settle before deadline",
                )
            else:
                self._clear_recovery(generation, home_intent)
            return
        while attempts < self.replan_max_attempts and self._remaining(deadline) > 0:
            if not self._wait_for_recovery(generation, home_intent, deadline):
                self._clear_recovery(generation, home_intent)
                return
            authority = self._authority()
            identity = self._authority_home_identity(authority)
            if identity is not None and identity != home_intent:
                self._finish_recovery_failure(
                    generation,
                    "return-home authority identity changed during recovery",
                )
                return
            home = self._authority_home_from(authority)
            if home is None:
                last_error = "no fresh return-home authority was available"
                attempts += 1
                continue
            try:
                if self._authority_binding(authority) is None:
                    raise ValueError("map authority has no navigation transform")
            except ValueError as exc:
                last_error = str(exc)
                attempts += 1
                continue
            attempts += 1
            outcome, last_error, _ = self._plan_once(
                "return_home",
                home,
                authority=authority,
                expected_generation=generation,
                home_intent=home_intent,
                overall_deadline=deadline,
            )
            if outcome == "submitted":
                return
            if outcome == "superseded":
                self._clear_recovery(generation, home_intent)
                return
            if not self.bridge.set_goal_pending_if_current(generation):
                self._clear_recovery(generation, home_intent)
                return
        self._finish_recovery_failure(
            generation,
            f"return-home recovery exhausted after {attempts} attempts: {last_error}",
        )

    def _wait_for_recovery(self, generation, home_intent, deadline) -> bool:
        wake_at = min(deadline, time.monotonic() + self.replan_backoff_s)
        while time.monotonic() < wake_at:
            if not self._owns_recovery(generation, home_intent):
                return False
            time.sleep(min(0.02, wake_at - time.monotonic()))
        return self._owns_recovery(generation, home_intent)

    def _finish_recovery_failure(self, generation, message) -> None:
        with self._active_lock:
            if self._recovery_generation != generation or self._home_intent is None:
                return
            self._recovery_generation = None
            self._recovery_deadline = None
            self._home_intent = None
        self._fail_if_current(message, generation)

    def _clear_recovery(self, generation, home_intent) -> None:
        with self._active_lock:
            if (
                self._recovery_generation == generation
                and self._home_intent == home_intent
            ):
                self._recovery_generation = None
                self._recovery_deadline = None
                self._home_intent = None

    def _fail_if_current(self, message: str, generation: int) -> None:
        if self.bridge.set_nav_status_if_current(generation, "failed"):
            self.bridge.node.get_logger().warning(f"[{self.bridge.id}] {message}")


def configure_objective_planning(bridge):
    config = (bridge.cfg or {}).get("planning") or {}
    backend = str(
        config.get("backend") or (bridge.cfg or {}).get("planning_backend") or ""
    ).lower()
    bridge.objective_planner = (
        MggObjectivePlanning(bridge, config) if backend == "mgg" else None
    )
