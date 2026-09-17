"""Bounded ROS client for MGG navigate and return-home objectives."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import math
import re
import threading
import time
from types import SimpleNamespace

from adapters.exploration import is_physical_no_progress_failure, planner_path
from autonomy.live_mapping import navigation_goal, solution_order

MAX_NAV_FAILURE_REASON_LENGTH = 512
FULL_ROUTE_ENDPOINT_TOLERANCE_M = 0.001
GRID_REFINEMENT_DEADLINE_REASON = "grid refinement exceeded its cooperative deadline"
# Planner responses that mean its inputs are momentarily missing, not that the
# goal is infeasible. MGG drops its MOLA map the instant a new keyframe changes
# the authority key and serves again once the rebuilt product is loaded,
# typically one to three seconds later; a rolling route asks for its next
# section exactly then, right after the keyframes of the section it just drove.
PLANNER_INPUT_UNAVAILABLE_REASONS = (
    "odometry or planning map is unavailable",
    "MOLA map snapshot is missing or stale",
    "MOLA map unavailable",
    # The indexed map server fails closed while one refresh of its product is
    # rejected, and lags the authority by one poll after every new revision.
    # Both clear on its next successful refresh; MGG reports them as a stale
    # revision rather than as a blocked route.
    "a different snapshot failed indexed publication",
    "requested snapshot is not current",
    "source stamp does not match indexed snapshot",
)
PLANNER_INPUT_RETRY_S = 0.5


def planner_input_unavailable(reason) -> bool:
    """True when a planner refusal only reports missing input."""

    text = str(reason or "")
    return any(marker in text for marker in PLANNER_INPUT_UNAVAILABLE_REASONS)


# A committed objective is delivered section by section as terrain validates.
# Native MGG guarantees each section makes useful progress along its route, but
# a route that keeps leading sideways or doubling back could otherwise continue
# forever. Every section that fails to bring the closest planned endpoint any
# nearer to the committed goal counts once; this many consecutive such sections
# ends the objective rather than looping.
PREFIX_NO_PROGRESS_LIMIT = 3
PREFIX_PROGRESS_EPSILON_M = 0.05


@dataclass(frozen=True)
class ObjectivePlanClaim:
    """Generation ownership minted before blocking objective planning starts."""

    objective: str
    goal: dict | None
    generation: int


class MggObjectivePlanning:
    """Request an MGG path, validate its snapshot, then execute all waypoints."""

    def __init__(self, bridge, config):
        from mgg_msgs.srv import PlanObjective
        from adapters.mapping_authority import get_mapping_authority, planning_frame

        self.bridge = bridge
        self.service_type = PlanObjective
        namespace = str(config.get("namespace") or f"/{bridge.id}/mgg").rstrip("/")
        self.client = bridge.node.create_client(
            PlanObjective, f"{namespace}/plan_objective"
        )
        try:
            from mgg_msgs.srv import RefineObjectiveRoute
        except ImportError:
            RefineObjectiveRoute = None
        self.refine_service_type = RefineObjectiveRoute
        self.refine_client = (
            bridge.node.create_client(
                RefineObjectiveRoute, f"{namespace}/refine_objective_route"
            )
            if RefineObjectiveRoute is not None
            else None
        )
        try:
            from mgg_msgs.srv import ValidateObjectiveRoute
        except ImportError:
            ValidateObjectiveRoute = None
        self.route_validation_service_type = ValidateObjectiveRoute
        self.route_validation_client = (
            bridge.node.create_client(
                ValidateObjectiveRoute, f"{namespace}/validate_objective_route"
            )
            if ValidateObjectiveRoute is not None
            else None
        )
        self.authority_reader = get_mapping_authority(bridge)
        self.frame = planning_frame(bridge)
        configured_frame = str(config.get("frame") or "").lstrip("/")
        if configured_frame and configured_frame != self.frame:
            raise ValueError(
                "planning.frame must match the configured MGG planning frame"
            )
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
        self.execution_authority_tolerance_m = self._bounded_float(
            config.get("execution_authority_tolerance_m", 0.25),
            0.01,
            5.0,
            0.25,
        )
        self.execution_goal_yaw_tolerance_rad = self._bounded_float(
            config.get("execution_goal_yaw_tolerance_rad", math.pi),
            0.0,
            math.pi,
            math.pi,
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
        self._initial_claim_generation = None
        self._objective_kind = None
        self._objective_goal = None
        self._indexed_map_validated = False
        self._nav_failure_reason = None
        self._nav_failure_generation = None
        self._blocked_retry_count = 0
        self._validation_path = None
        self._validation_future = None
        self._validation_future_token = None
        self._validation_token = 0
        self._validation_started_at = 0.0
        self._validation_next_at = 0.0
        self._home_intent = None
        self._rolling_route_id = None
        self._rolling_partial = False
        self._rolling_global_plan = None
        self._rolling_planned_goal = None
        self._rolling_authority_binding = None
        self._rolling_best_remaining_m = None
        self._rolling_no_progress_sections = 0
        self._continuation_generation = None
        self._continuation_deadline = None
        self._continuation_thread = None
        self._recovery_generation = None
        self._recovery_binding = None
        self._recovery_deadline = None
        self._recovery_mode = None
        self._recovery_thread = None
        self._authority_timer = bridge.node.create_timer(
            max(0.05, float(config.get("authority_check_period_s", 0.1))),
            self._check_active_authority,
        )

    def navigate(self, goal: dict) -> bool:
        return self.plan("navigate", goal)

    def return_home(self, goal: dict | None = None) -> bool:
        claim = self.claim_objective("return_home", goal)
        return self.execute_claimed(claim)

    def _return_home_claimed(self, claim: ObjectivePlanClaim) -> bool:
        # The authority anchor follows optimizer corrections. A caller-supplied
        # pose may have been resolved before the latest correction, so use it
        # only when no fresh authority anchor is available.
        authority = self._authority()
        authority_home = self._authority_home_from(authority)
        target = authority_home or claim.goal or self.home
        if not isinstance(target, dict):
            self._fail_if_current(
                "return-home requires planning.home or an explicit goal",
                claim.generation,
            )
            return False
        # Recovery is allowed only for an identity obtained from the current
        # authority envelope. Caller/config fallbacks are intentionally local.
        intent = self._authority_home_identity(authority) if authority_home else None
        if authority_home is not None and (intent is None or intent[4] != self.frame):
            self._fail_if_current(
                "return-home authority has an invalid mission, epoch, or frame",
                claim.generation,
            )
            return False
        with self._active_lock:
            if (
                self._initial_claim_generation != claim.generation
                or self.bridge._goal_generation != claim.generation
            ):
                return False
            self._objective_goal = deepcopy(target)
            self._home_intent = intent
        outcome, message, generation = self._plan_once(
            "return_home",
            target,
            authority=authority,
            expected_generation=claim.generation,
            home_intent=intent,
        )
        if outcome == "submitted":
            return True
        if outcome == "temporary":
            try:
                binding = self._authority_binding(authority)
            except ValueError:
                binding = None
            if binding is not None and self._start_recovery(
                generation,
                intent,
                cancel_route=False,
                binding=binding,
                mode="planning_budget",
            ):
                return True
        if outcome == "authority_changed" and intent is not None:
            if self._start_recovery(generation, intent, cancel_route=False):
                return True
        if outcome != "superseded":
            self._fail_if_current(message, generation)
        return False

    def claim_objective(
        self, objective: str, goal: dict | None = None
    ) -> ObjectivePlanClaim:
        """Synchronously supersede motion before an RPC is offloaded.

        The WebSocket receive loop calls this before scheduling the blocking
        planner work. Stop, manual drive, reset, and replacement commands can
        then invalidate the generation while the RPC remains in flight.
        """

        if objective not in ("navigate", "return_home"):
            raise ValueError(f"unsupported MGG objective {objective!r}")
        copied_goal = deepcopy(goal) if isinstance(goal, dict) else None
        self._retire_route_validation()
        generation = self.bridge.cancel_goal()
        with self._active_lock:
            self._active_route = None
            self._initial_claim_generation = generation
            self._nav_failure_reason = None
            self._nav_failure_generation = None
            self._blocked_retry_count = 0
            self._validation_path = None
            self._validation_token += 1
            self._objective_kind = objective
            self._objective_goal = deepcopy(copied_goal)
            self._indexed_map_validated = False
            self._home_intent = None
            self._clear_rolling_locked()
            self._recovery_generation = None
            self._recovery_binding = None
            self._recovery_deadline = None
            self._recovery_mode = None
        if not self.bridge.set_goal_pending_if_current(generation):
            self._retire_initial_claim(generation)
            return ObjectivePlanClaim(objective, copied_goal, generation)
        return ObjectivePlanClaim(objective, copied_goal, generation)

    def execute_claimed(self, claim: ObjectivePlanClaim) -> bool:
        """Run blocking planning only while the synchronously claimed generation owns it."""

        if not isinstance(claim, ObjectivePlanClaim):
            raise TypeError("objective plan claim is required")
        if not self._owns_expected(claim.generation, None):
            self._retire_initial_claim(claim.generation)
            return False
        if claim.objective == "return_home":
            result = self._return_home_claimed(claim)
            if not result and claim.generation != self.bridge._goal_generation:
                self._retire_initial_claim(claim.generation)
            return result
        goal = claim.goal if isinstance(claim.goal, dict) else {}
        raw_authority = self._raw_authority()
        authority = (
            self._authority_from_raw(raw_authority)
            if isinstance(raw_authority, dict)
            else None
        )
        try:
            binding = self._authority_binding(authority)
        except ValueError:
            binding = None
        planning_authority = (
            {"authority": authority, "raw_authority": raw_authority}
            if isinstance(authority, dict)
            else {}
        )
        outcome, message, generation = self._plan_once(
            claim.objective,
            goal,
            expected_generation=claim.generation,
            enforce_goal_solution_order=True,
            **planning_authority,
        )
        if outcome == "temporary":
            if binding is not None and self._start_recovery(
                generation,
                None,
                cancel_route=False,
                binding=binding,
                mode="planning_budget",
            ):
                return True
        if outcome == "authority_changed":
            if self._start_recovery(generation, None, cancel_route=False):
                return True
        if outcome != "submitted" and outcome != "superseded":
            self._fail_if_current(message, generation)
        elif outcome == "superseded":
            self._retire_initial_claim(generation)
        return outcome == "submitted"

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

    def _clear_rolling_locked(self) -> None:
        """Retire one native cached-route binding without joining its worker."""

        self._rolling_route_id = None
        self._rolling_partial = False
        self._rolling_global_plan = None
        self._rolling_planned_goal = None
        self._rolling_authority_binding = None
        self._rolling_best_remaining_m = None
        self._rolling_no_progress_sections = 0
        self._continuation_generation = None
        self._continuation_deadline = None

    @staticmethod
    def _remaining_goal_distance(plan, goal) -> float:
        """Planar distance from a section's planned endpoint to the objective."""

        endpoint = plan.poses[-1]
        return math.hypot(endpoint.x - float(goal["x"]), endpoint.y - float(goal["y"]))

    def global_display_plan(self):
        """Return the immutable full graph route owned by the current command."""

        with self._active_lock:
            retained = self._rolling_global_plan
            if retained is None or retained[0] != self.bridge._goal_generation:
                return None
            return retained[1]

    def _authority(self):
        authority = self._raw_authority()
        if isinstance(authority, dict):
            return self._authority_from_raw(authority)
        return None

    def _authority_from_raw(self, authority):
        from adapters.mapping_authority import authority_for_frame

        try:
            return authority_for_frame(authority, self.frame)
        except (KeyError, ValueError):
            # Legacy default-frame authorities can still bind an indexed map
            # or Home landmark without a correction transform. Alternate
            # planning frames always require their explicit qualified pair.
            raw_frame = str(authority.get("navigation_frame", self.frame)).lstrip("/")
            if raw_frame == self.frame and "planning_frame" not in authority:
                return deepcopy(authority)
            return None

    def _raw_authority(self):
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
                "frame_id": str(authority.get("navigation_frame", self.frame)).lstrip(
                    "/"
                ),
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
        # Mission and component identify the route's frame authority. A newer
        # correction revision is metadata; only the relative transform below
        # can establish that the executing route physically moved.
        if updated is None or updated[:2] != original[:2]:
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

    def _execution_authority_change(self, original, current, route_points):
        """Evaluate accepted-route displacement without resetting its baseline."""
        if original is None:
            return False, ""
        try:
            updated = self._authority_binding(current)
        except ValueError:
            return True, "invalid map authority"
        if updated is None:
            return True, "map authority unavailable or stale"
        if updated[:2] != original[:2]:
            return True, "map authority mission or component changed"
        before, after = original[3], updated[3]
        if before == after:
            return False, ""
        translation = math.sqrt(
            sum((before[index][3] - after[index][3]) ** 2 for index in range(3))
        )
        trace = sum(
            before[row][column] * after[row][column]
            for row in range(3)
            for column in range(3)
        )
        rotation = math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0)))
        # Full routes may contain thousands of controller poses. Unchanged
        # orientation makes every point shift equally; only rotations need a
        # scan through the route. Compare coefficients exactly so a small
        # rotation at a distant endpoint cannot be rounded away.
        if all(before[row][:3] == after[row][:3] for row in range(3)):
            max_shift = translation
        else:
            delta = tuple(
                tuple(after[row][column] - before[row][column] for column in range(4))
                for row in range(3)
            )
            max_shift = 0.0
            for point in route_points:
                shift = tuple(
                    sum(row[column] * point[column] for column in range(3)) + row[3]
                    for row in delta
                )
                max_shift = max(max_shift, math.hypot(*shift))
        changed = (
            max_shift > self.execution_authority_tolerance_m
            or rotation > self.authority_rotation_tolerance_rad
        )
        reason = (
            f"translation={translation:.3f}m, rotation={rotation:.3f}rad, "
            f"max_route_shift={max_shift:.3f}m, "
            f"limits={self.execution_authority_tolerance_m:.3f}m/"
            f"{self.authority_rotation_tolerance_rad:.3f}rad"
        )
        return changed, reason

    def _execution_goal_change(self, before, after):
        """Apply controller-scale tolerances to an active point goal."""
        if not isinstance(after, dict):
            return True, "home anchor is unavailable"
        try:
            shift = math.sqrt(
                sum(
                    (float(before.get(axis, 0.0)) - float(after.get(axis, 0.0))) ** 2
                    for axis in ("x", "y", "z")
                )
            )
            yaw_delta = abs(
                math.atan2(
                    math.sin(
                        float(after.get("yaw", 0.0)) - float(before.get("yaw", 0.0))
                    ),
                    math.cos(
                        float(after.get("yaw", 0.0)) - float(before.get("yaw", 0.0))
                    ),
                )
            )
        except (AttributeError, TypeError, ValueError):
            return True, "home anchor is invalid"
        changed = (
            shift > self.execution_authority_tolerance_m
            or yaw_delta > self.execution_goal_yaw_tolerance_rad
        )
        return changed, (
            f"home_anchor_shift={shift:.3f}m, "
            f"home_anchor_yaw_shift={yaw_delta:.3f}rad, "
            f"limits={self.execution_authority_tolerance_m:.3f}m/"
            f"{self.execution_goal_yaw_tolerance_rad:.3f}rad"
        )

    def _check_active_authority(self) -> None:
        with self._active_lock:
            if self._nav_failure_generation != self.bridge._goal_generation:
                self._nav_failure_reason = None
                self._nav_failure_generation = None
            if (
                self._continuation_generation is not None
                and self._continuation_generation != self.bridge._goal_generation
            ):
                self._clear_rolling_locked()
                if (
                    self._initial_claim_generation is None
                    and self._recovery_generation is None
                ):
                    self._objective_kind = None
                    self._objective_goal = None
                    self._home_intent = None
                    self._indexed_map_validated = False
            active = self._active_route
        if active is None:
            return
        (
            generation,
            binding,
            home_intent,
            planned_goal,
            _,
            route_points,
        ) = active
        if generation != self.bridge._goal_generation:
            self._retire_route_validation()
            with self._active_lock:
                if self._active_route == active:
                    self._active_route = None
                    self._home_intent = None
                    self._objective_kind = None
                    self._objective_goal = None
                    self._indexed_map_validated = False
                    self._clear_rolling_locked()
                if self._nav_failure_generation != self.bridge._goal_generation:
                    self._nav_failure_reason = None
                    self._nav_failure_generation = None
            return
        if self.bridge.nav_status == "succeeded":
            self._retire_route_validation()
            with self._active_lock:
                continue_route = bool(
                    self._active_route == active
                    and self._objective_kind in ("navigate", "return_home")
                    and self._rolling_partial
                    and self._rolling_route_id
                )
            if continue_route:
                self._start_route_continuation(active)
                return
            with self._active_lock:
                if self._active_route == active:
                    self._active_route = None
                if generation == self.bridge._goal_generation:
                    self._objective_kind = None
                    self._objective_goal = None
                    self._home_intent = None
                    self._indexed_map_validated = False
                    self._nav_failure_reason = None
                    self._nav_failure_generation = None
                    self._blocked_retry_count = 0
                    self._clear_rolling_locked()
            return
        if self.bridge.nav_status == "failed":
            self._retire_route_validation()
            reason = str(getattr(self.bridge, "_nav_failure_reason", "") or "")
            # Nav2's progress checker is the evidence that execution was
            # physically blocked. Planning failures (including unknown map
            # space) never enter or consume this retry budget.
            if is_physical_no_progress_failure(reason):
                with self._active_lock:
                    if self._active_route != active:
                        return
                    self._blocked_retry_count += 1
                    if self._blocked_retry_count >= 3:
                        self._active_route = None
                        self._home_intent = None
                        self._objective_kind = None
                        self._objective_goal = None
                        self._indexed_map_validated = False
                        self._clear_rolling_locked()
                        return
                    self._active_route = None
                if self._start_recovery(
                    generation, home_intent, cancel_route=False, binding=binding
                ):
                    self.bridge.node.get_logger().warning(
                        f"[{self.bridge.id}] MGG objective replanning after "
                        f"controller made no progress "
                        f"({self._blocked_retry_count}/3)"
                    )
                    return
        if self.bridge.nav_status != "active":
            with self._active_lock:
                if self._active_route == active:
                    self._active_route = None
                    self._home_intent = None
                    self._objective_kind = None
                    self._objective_goal = None
                    self._indexed_map_validated = False
                    self._clear_rolling_locked()
            return
        if self._check_route_validation(active):
            return
        raw_authority = self._raw_authority()
        authority = (
            self._authority_from_raw(raw_authority)
            if isinstance(raw_authority, dict)
            else None
        )
        home_identity_changed = home_intent is not None and (
            self._authority_home_identity(authority) != home_intent
        )
        latest_home = (
            self._authority_home_from(authority) if home_intent is not None else None
        )
        home_pose_changed, home_reason = (
            self._execution_goal_change(planned_goal, latest_home)
            if home_intent is not None
            else (False, "")
        )
        authority_changed, authority_reason = self._execution_authority_change(
            binding, authority, route_points
        )
        if (
            not authority_changed
            and not home_identity_changed
            and not home_pose_changed
        ):
            return
        with self._active_lock:
            if (
                self._active_route != active
                or generation != self.bridge._goal_generation
            ):
                return
            self._active_route = None
        identity_preserved = self._authority_identity_matches(binding, authority)
        # A missed authority heartbeat requires stopping motion, but does not
        # prove that the retained destination became invalid. Wait for fresh
        # authority under the recovery deadline and preserve its identity.
        if raw_authority is None or (
            identity_preserved
            and (
                home_intent is None
                or self._authority_matches_home(authority, home_intent)
            )
        ):
            if self._start_recovery(
                generation, home_intent, cancel_route=True, binding=binding
            ):
                cancel_reason = (
                    authority_reason
                    if authority_changed
                    else (
                        "home landmark identity changed"
                        if home_identity_changed
                        else home_reason
                    )
                )
                self.bridge.node.get_logger().warning(
                    f"[{self.bridge.id}] MGG objective route canceled after "
                    f"map authority changed ({cancel_reason})"
                )
                return
        canceled_generation = self.bridge.cancel_goal_if_current(generation)
        if canceled_generation is not None:
            self._fail_if_current(
                "MGG objective route canceled after map authority changed: "
                + (authority_reason or home_reason or "home landmark identity changed"),
                canceled_generation,
            )

    def plan(self, objective: str, goal: dict) -> bool:
        claim = self.claim_objective(objective, goal)
        return self.execute_claimed(claim)

    def _start_route_continuation(self, active) -> bool:
        """Move a completed local objective chunk into bounded refinement."""

        generation, binding, home_intent, _, _, _ = active
        if not self.bridge.set_goal_pending_if_current(generation):
            return False
        with self._active_lock:
            if (
                self._active_route != active
                or generation != self.bridge._goal_generation
                or self._objective_kind not in ("navigate", "return_home")
                or not self._rolling_partial
                or not self._rolling_route_id
            ):
                return False
            self._active_route = None
            self._continuation_generation = generation
            self._continuation_deadline = time.monotonic() + self.replan_deadline_s
            worker = threading.Thread(
                target=self._continuation_worker,
                name=f"{self.bridge.id}-return-home-refine",
                daemon=True,
            )
            self._continuation_thread = worker
            try:
                worker.start()
            except Exception:
                self._continuation_thread = None
                self._continuation_generation = None
                self._continuation_deadline = None
                raise
        return True

    def _owns_continuation(self, generation, route_id, home_intent) -> bool:
        with self._active_lock:
            return self._owns_continuation_locked(generation, route_id, home_intent)

    def _continuation_worker(self) -> None:
        worker = threading.current_thread()
        try:
            while True:
                with self._active_lock:
                    generation = self._continuation_generation
                    route_id = self._rolling_route_id
                    home_intent = self._home_intent
                    deadline = self._continuation_deadline
                    global_retained = self._rolling_global_plan
                if (
                    generation is None
                    or not route_id
                    or deadline is None
                    or global_retained is None
                ):
                    return
                # The completed chunk was removed from _active_route before
                # this worker started. The global route retains its original
                # authority binding in a separate field below.
                with self._active_lock:
                    binding = self._rolling_authority_binding
                try:
                    outcome, message = self._refine_route_once(
                        generation,
                        route_id,
                        home_intent,
                        binding,
                        global_retained[1],
                        deadline,
                    )
                except Exception as exc:
                    outcome = "failed"
                    message = f"MGG objective refinement failed: {exc}"
                if outcome == "submitted":
                    return
                if outcome == "superseded":
                    with self._active_lock:
                        if self._continuation_generation == generation:
                            self._clear_rolling_locked()
                            self._objective_kind = None
                            self._objective_goal = None
                            self._home_intent = None
                            self._indexed_map_validated = False
                    return
                if outcome == "retry" and self._remaining(deadline) > 0.0:
                    if not self._wait_for_continuation(
                        generation, route_id, home_intent, deadline
                    ):
                        return
                    continue
                if outcome == "replan":
                    with self._active_lock:
                        if not self._owns_continuation_locked(
                            generation, route_id, home_intent
                        ):
                            return
                        self._clear_rolling_locked()
                    if self._start_recovery(
                        generation,
                        home_intent,
                        cancel_route=False,
                        binding=binding,
                    ):
                        self.bridge.node.get_logger().warning(
                            f"[{self.bridge.id}] MGG objective local route replaced: {message}"
                        )
                    return
                if outcome == "retry":
                    message = "MGG objective refinement deadline expired"
                self._fail_if_current(message, generation)
                return
        finally:
            with self._active_lock:
                if self._continuation_thread is worker:
                    self._continuation_thread = None

    def _owns_continuation_locked(self, generation, route_id, home_intent) -> bool:
        return (
            generation == self.bridge._goal_generation
            and self._continuation_generation == generation
            and self._rolling_route_id == route_id
            and self._home_intent == home_intent
            and self._objective_kind in ("navigate", "return_home")
        )

    def _wait_for_continuation(
        self, generation, route_id, home_intent, deadline
    ) -> bool:
        wake_at = min(deadline, time.monotonic() + self.replan_backoff_s)
        while True:
            if not self._owns_continuation(generation, route_id, home_intent):
                return False
            remaining = wake_at - time.monotonic()
            if remaining <= 0.0:
                break
            time.sleep(min(0.02, remaining))
        return self._owns_continuation(generation, route_id, home_intent)

    def _refine_route_once(
        self,
        generation,
        route_id,
        home_intent,
        binding,
        global_plan,
        deadline,
    ) -> tuple[str, str]:
        service, client = self.refine_service_type, self.refine_client
        if service is None or client is None:
            return "failed", "installed mgg_msgs lacks rolling route refinement"
        if not self._owns_continuation(generation, route_id, home_intent):
            return "superseded", ""
        remaining = self._remaining(deadline)
        if remaining <= 0.0:
            return "retry", "MGG objective refinement deadline expired"
        if not client.wait_for_service(timeout_sec=min(3.0, remaining)):
            return "retry", "MGG objective refinement service is unavailable"
        raw_authority = self._raw_authority()
        authority = (
            self._authority_from_raw(raw_authority)
            if isinstance(raw_authority, dict)
            else None
        )
        if authority is None:
            return "retry", "no fresh map authority was available"
        if (
            not self._authority_identity_matches(binding, authority)
            or (
                home_intent is not None
                and not self._authority_matches_home(authority, home_intent)
            )
            or self._route_authority_changed(binding, authority)
        ):
            return "replan", "map authority or objective landmark changed"
        try:
            mapping_snapshot = self._mapping_snapshot(authority)
        except ValueError as exc:
            return "replan", str(exc)
        with self._active_lock:
            indexed_required = self._indexed_map_validated
        if indexed_required and mapping_snapshot is None:
            return "failed", "rolling objective requires indexed map authority"

        request = service.Request()
        request.mission_id = str(authority["mission_id"])
        request.route_id = route_id
        request.component_id = str(authority["component_id"])
        # Zero asks native MGG to bind its current additive graph/map revision.
        request.graph_revision = 0
        request.map_revision = 0
        if mapping_snapshot is not None:
            (
                request.map_epoch,
                request.mapping_graph_revision,
                request.geometry_revision,
                request.map_source_stamp.sec,
                request.map_source_stamp.nanosec,
            ) = mapping_snapshot
        try:
            future = client.call_async(request)
        except Exception as exc:
            return "retry", f"MGG objective refinement request failed: {exc}"
        while not future.done() and time.monotonic() < deadline:
            if not self._owns_continuation(generation, route_id, home_intent):
                future.cancel()
                return "superseded", ""
            time.sleep(0.02)
        if not future.done():
            future.cancel()
            return "retry", "MGG objective refinement request timed out"
        if not self._owns_continuation(generation, route_id, home_intent):
            return "superseded", ""
        try:
            response = future.result()
        except Exception as exc:
            return "retry", f"MGG objective refinement request failed: {exc}"
        if response.status != service.Response.SUCCEEDED:
            reason = response.reason or f"MGG refinement status {response.status}"
            if response.status in {
                getattr(service.Response, "BLOCKED", object()),
                getattr(service.Response, "STALE_REVISION", object()),
            }:
                return "replan", reason
            return "failed", reason
        if response.component_id != request.component_id:
            return "failed", "MGG refined a route from another map component"
        indexed_map_validated = getattr(response, "indexed_map_validated", False)
        if type(indexed_map_validated) is not bool:
            return "failed", "MGG refinement returned invalid indexed-map evidence"
        if indexed_map_validated != indexed_required:
            return "failed", "MGG refinement changed its map evidence source"
        if mapping_snapshot is not None and (
            response.map_epoch != request.map_epoch
            or response.mapping_graph_revision != request.mapping_graph_revision
            or response.geometry_revision.lower() != request.geometry_revision
            or response.map_source_stamp.sec != request.map_source_stamp.sec
            or response.map_source_stamp.nanosec != request.map_source_stamp.nanosec
        ):
            return "replan", "MGG refined against a different indexed map snapshot"
        current_authority = self._authority()
        try:
            current_snapshot = self._mapping_snapshot(current_authority or {})
        except ValueError:
            current_snapshot = None
        if (
            self._route_authority_changed(binding, current_authority)
            or (
                home_intent is not None
                and self._authority_home_identity(current_authority) != home_intent
            )
            or (mapping_snapshot is not None and current_snapshot != mapping_snapshot)
        ):
            return "replan", "map authority changed while MGG refined the objective"

        stamp = self.bridge.node.get_clock().now().to_msg()
        local_path = SimpleNamespace(
            header=SimpleNamespace(frame_id=self.frame, stamp=stamp),
            poses=[
                SimpleNamespace(header=SimpleNamespace(frame_id=self.frame), pose=pose)
                for pose in response.path
            ],
        )
        try:
            plan = planner_path(
                local_path,
                self.frame,
                self.planar_tolerance_m,
                self.max_inclination_rad,
            )
        except (AttributeError, TypeError, ValueError) as exc:
            return "failed", f"MGG returned an unsupported refined path: {exc}"
        partial = getattr(response, "partial", False)
        if type(partial) is not bool:
            return "failed", "MGG returned an invalid partial-route flag"
        # Small accepted corrections do not move the cached graph route's goal.
        # Material corrections are fenced above and require a new route.
        with self._active_lock:
            goal = deepcopy(self._rolling_planned_goal)
        if not isinstance(goal, dict):
            return "failed", "retained objective goal is unavailable"
        if not partial:
            # The exact endpoint is owned by the final section only. An earlier
            # section stops wherever validated terrain ends.
            endpoint_error = self._remaining_goal_distance(plan, goal)
            if endpoint_error > FULL_ROUTE_ENDPOINT_TOLERANCE_M:
                return (
                    "failed",
                    "MGG final objective chunk endpoint differs from its goal by "
                    f"{endpoint_error:.3f}m",
                )
        else:
            # Native MGG shortens a section to the terrain it has validated, so
            # consecutive sections can be short. They must still carry the robot
            # closer to the committed goal, or the objective ends here.
            remaining_m = self._remaining_goal_distance(plan, goal)
            with self._active_lock:
                best = self._rolling_best_remaining_m
                if best is None or remaining_m < best - PREFIX_PROGRESS_EPSILON_M:
                    self._rolling_best_remaining_m = remaining_m
                    self._rolling_no_progress_sections = 0
                else:
                    self._rolling_no_progress_sections += 1
                stalled = self._rolling_no_progress_sections >= PREFIX_NO_PROGRESS_LIMIT
            if stalled:
                return (
                    "failed",
                    "MGG objective route came no closer to its goal across "
                    f"{PREFIX_NO_PROGRESS_LIMIT} consecutive validated sections "
                    f"(still {remaining_m:.3f}m away)",
                )
        pre_submit_rejected = [False]

        def validate_authority_for_dispatch():
            valid = self._authority_matches_plan(
                binding, mapping_snapshot, home_intent, indexed_map_validated
            )
            if not valid:
                pre_submit_rejected[0] = True
            return valid

        submitted_generation = self.bridge.follow_path(
            plan,
            expected_generation=generation,
            not_after=deadline,
            pre_submit=validate_authority_for_dispatch,
        )
        if type(submitted_generation) is not int:
            if pre_submit_rejected[0]:
                return "replan", "map authority changed before refined route dispatch"
            if not self._owns_continuation(generation, route_id, home_intent):
                return "superseded", ""
            return "failed", "MGG refined objective route could not be submitted"
        with self._active_lock:
            if (
                submitted_generation != self.bridge._goal_generation
                or self._continuation_generation != generation
                or self._rolling_route_id != route_id
                or self._home_intent != home_intent
                or self._objective_kind not in ("navigate", "return_home")
            ):
                return "superseded", ""
            self._continuation_generation = None
            self._continuation_deadline = None
            self._rolling_partial = partial
            self._rolling_global_plan = (submitted_generation, global_plan)
            self._rolling_planned_goal = deepcopy(goal)
            self._indexed_map_validated = indexed_map_validated
            route_points = tuple((pose.x, pose.y, pose.z) for pose in global_plan.poses)
            self._active_route = (
                submitted_generation,
                binding,
                home_intent,
                deepcopy(goal),
                indexed_map_validated,
                route_points,
            )
            self._validation_path = tuple(deepcopy(response.path))
            self._validation_token += 1
            self._validation_next_at = 0.0
        return "submitted", ""

    def _plan_once(
        self,
        objective: str,
        goal: dict,
        *,
        authority=None,
        raw_authority=None,
        expected_generation: int | None = None,
        home_intent=None,
        overall_deadline: float | None = None,
        enforce_goal_solution_order: bool = False,
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
                self._nav_failure_reason = None
                self._nav_failure_generation = None
                self._objective_kind = objective
                self._objective_goal = deepcopy(goal)
                self._indexed_map_validated = False
                self._home_intent = home_intent
                self._clear_rolling_locked()
                self._recovery_generation = None
                self._recovery_deadline = None
                self._recovery_mode = None
            generation = self.bridge.cancel_goal()
            if not self.bridge.set_goal_pending_if_current(generation):
                return "superseded", "", generation
        else:
            generation = expected_generation
            if not self._owns_expected(generation, home_intent):
                return "superseded", "", generation
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
        if not isinstance(authority, dict):
            raw_authority = self._raw_authority()
            if isinstance(raw_authority, dict):
                authority = self._authority_from_raw(raw_authority) or {}
            else:
                authority = {}
        identity_error = self._goal_authority_error(
            goal, authority, raw_authority=raw_authority
        )
        if identity_error:
            return "failed", identity_error, generation
        if enforce_goal_solution_order:
            solution_error = self._goal_solution_order_error(goal, authority)
            if solution_error:
                return "failed", solution_error, generation
        try:
            goal = self._goal_in_planning(goal, authority, raw_authority)
            x, y = float(goal["x"]), float(goal["y"])
            z = float(goal.get("z", 0.0))
            yaw = float(goal.get("yaw", 0.0))
            graph_revision = int(goal.get("graph_revision", 0))
            map_revision = int(goal.get("map_revision", 0))
        except (KeyError, TypeError, ValueError) as exc:
            return "failed", f"invalid MGG objective goal: {exc}", generation
        if not all(math.isfinite(value) for value in (x, y, z, yaw)):
            return "failed", "MGG objective goal contains a nonfinite value", generation
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
            reason = response.reason or f"MGG planning status {response.status}"
            blocked = getattr(self.service_type.Response, "BLOCKED", None)
            stale = getattr(self.service_type.Response, "STALE_REVISION", None)
            if (
                blocked is not None
                and response.status == blocked
                and (
                    GRID_REFINEMENT_DEADLINE_REASON in reason
                    or planner_input_unavailable(reason)
                )
            ):
                return "temporary", reason, generation
            if (
                stale is not None
                and response.status == stale
                and planner_input_unavailable(reason)
            ):
                return "temporary", reason, generation
            return (
                "failed",
                reason,
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
        # An older native ABI has no provenance bit. It remains usable as an
        # MGG-native route, but its echoed snapshot must not imply MOLA checks.
        indexed_map_validated = getattr(response, "indexed_map_validated", False)
        if type(indexed_map_validated) is not bool:
            return "failed", "MGG returned invalid indexed-map evidence", generation
        if indexed_map_validated and (
            mapping_snapshot is None
            or response.map_epoch != request.map_epoch
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
        if self._route_authority_changed(authority_binding, current_authority) or (
            indexed_map_validated and current_snapshot != mapping_snapshot
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
        partial = getattr(response, "partial", False)
        if type(partial) is not bool:
            return "terminal", "MGG returned an invalid partial-route flag", generation
        route_id = getattr(response, "route_id", "")
        if not isinstance(route_id, str) or len(route_id) > 512:
            return "terminal", "MGG returned an invalid route token", generation
        if partial and (
            not route_id
            or self.refine_service_type is None
            or self.refine_client is None
        ):
            return (
                "terminal",
                "MGG partial route cannot continue with the installed ABI",
                generation,
            )
        global_poses = getattr(response, "global_path", None)
        if not global_poses:
            global_poses = response.path if not partial else None
        if global_poses is None:
            return "terminal", "MGG partial route has no global route", generation
        global_path = SimpleNamespace(
            header=SimpleNamespace(frame_id=self.frame, stamp=stamp),
            poses=[
                SimpleNamespace(header=SimpleNamespace(frame_id=self.frame), pose=pose)
                for pose in global_poses
            ],
        )
        try:
            # The graph route is display and continuation topology. Only the
            # controller-local path above must already satisfy every refined
            # ground segment.
            global_plan = planner_path(global_path, self.frame, math.inf, math.pi)
        except (AttributeError, TypeError, ValueError) as exc:
            return (
                "failed",
                f"MGG returned an unsupported global route: {exc}",
                generation,
            )
        endpoint = global_plan.poses[-1] if partial else plan.poses[-1]
        endpoint_error = math.hypot(endpoint.x - x, endpoint.y - y)
        if endpoint_error > FULL_ROUTE_ENDPOINT_TOLERANCE_M:
            route_kind = "global" if partial else "full"
            return (
                "terminal",
                f"MGG {route_kind} route endpoint differs from the requested objective "
                f"by {endpoint_error:.3f}m "
                f"(limit {FULL_ROUTE_ENDPOINT_TOLERANCE_M:.3f}m)",
                generation,
            )
        if not self._owns_generation(generation, expected_generation, home_intent):
            return "superseded", "", generation
        if self._remaining(overall_deadline) <= 0.0:
            return "failed", "MGG return-home recovery deadline expired", generation
        pre_submit_rejected = [False]

        def validate_authority_for_dispatch():
            valid = self._authority_matches_plan(
                authority_binding,
                mapping_snapshot,
                home_intent,
                indexed_map_validated,
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
                self._initial_claim_generation != generation
                and (
                    self._recovery_generation != generation
                    or self._home_intent != home_intent
                )
            ):
                return "superseded", "", generation
            self._initial_claim_generation = None
            self._recovery_generation = None
            self._recovery_deadline = None
            self._recovery_mode = None
            self._home_intent = home_intent
            self._indexed_map_validated = indexed_map_validated
            self._rolling_route_id = route_id if partial else None
            self._rolling_partial = partial
            self._rolling_global_plan = (submitted_generation, global_plan)
            self._rolling_planned_goal = deepcopy(goal)
            self._rolling_authority_binding = authority_binding
            # The first section establishes how close the objective has come.
            self._rolling_best_remaining_m = (
                self._remaining_goal_distance(plan, goal) if partial else None
            )
            self._rolling_no_progress_sections = 0
            self._continuation_generation = None
            self._continuation_deadline = None
            self._active_route = (
                submitted_generation,
                authority_binding,
                home_intent,
                deepcopy(goal),
                indexed_map_validated,
                tuple((pose.x, pose.y, pose.z) for pose in global_plan.poses)
                + ((x, y, z),),
            )
            self._validation_path = tuple(deepcopy(response.path))
            self._validation_token += 1
            self._validation_next_at = 0.0
        return "submitted", "", submitted_generation

    def _check_route_validation(self, active) -> bool:
        """Cancel only when the native map finds a known near-route hazard."""

        client = self.route_validation_client
        service = self.route_validation_service_type
        if client is None or service is None:
            return False
        now = time.monotonic()
        future = self._validation_future
        if future is not None:
            if not future.done():
                if now - self._validation_started_at >= 0.8:
                    self._retire_route_validation()
                    self._validation_next_at = now + 1.0
                return False
            self._validation_future = None
            token = self._validation_future_token
            self._validation_future_token = None
            try:
                response = future.result()
            except Exception:
                self._validation_next_at = now + 1.0
                return False
            with self._active_lock:
                current = self._active_route
                current_token = self._validation_token
            if token != current_token or current != active:
                return False
            if getattr(response, "status", None) != service.Response.INVALID:
                self._validation_next_at = now + 1.0
                return False
            generation, binding, home_intent, _, _, _ = active
            authority = self._authority()
            if (
                generation != self.bridge._goal_generation
                or self.bridge.nav_status != "active"
                or binding is None
                or self._route_authority_changed(binding, authority)
            ):
                return False
            with self._active_lock:
                if (
                    self._active_route != active
                    or self._validation_token != token
                    or generation != self.bridge._goal_generation
                    or self.bridge.nav_status != "active"
                ):
                    return False
                self._active_route = None
                self._validation_token += 1
            reason = str(getattr(response, "reason", "") or "known route hazard")[
                :MAX_NAV_FAILURE_REASON_LENGTH
            ]
            if self._start_recovery(
                generation, home_intent, cancel_route=True, binding=binding
            ):
                self.bridge.node.get_logger().warning(
                    f"[{self.bridge.id}] MGG route invalidated by current map: {reason}"
                )
                return True
            return False
        if now < self._validation_next_at or not client.service_is_ready():
            return False
        generation, binding, _, _, _, _ = active
        with self._active_lock:
            path = self._validation_path
            token = self._validation_token
        if (
            generation != self.bridge._goal_generation
            or binding is None
            or not path
            or len(path) > 2048
        ):
            self._validation_next_at = now + 1.0
            return False
        request = service.Request()
        request.mission_id = binding[0]
        request.component_id = binding[1]
        request.frame_id = self.frame
        request.path = list(deepcopy(path))
        request.lookahead_m = 3.0
        try:
            future = client.call_async(request)
        except Exception:
            self._validation_next_at = now + 1.0
            return False
        self._validation_future = future
        self._validation_future_token = token
        self._validation_started_at = now
        self._validation_next_at = now + 1.0
        return False

    def _retire_route_validation(self) -> None:
        future = self._validation_future
        self._validation_future = None
        self._validation_future_token = None
        if future is None or future.done():
            return
        future.cancel()
        client = self.route_validation_client
        remove = getattr(client, "remove_pending_request", None)
        if callable(remove):
            try:
                remove(future)
            except Exception:
                pass

    @staticmethod
    def _remaining(deadline: float | None) -> float:
        if deadline is None:
            return math.inf
        return max(0.0, deadline - time.monotonic())

    def _owns_generation(self, generation, expected_generation, home_intent) -> bool:
        if generation != self.bridge._goal_generation:
            return False
        return expected_generation is None or self._owns_expected(
            generation, home_intent
        )

    def _owns_expected(self, generation, home_intent) -> bool:
        with self._active_lock:
            return generation == self.bridge._goal_generation and (
                self._initial_claim_generation == generation
                or (
                    self._recovery_generation == generation
                    and self._home_intent == home_intent
                )
            )

    def _retire_initial_claim(self, generation) -> None:
        with self._active_lock:
            if self._initial_claim_generation != generation:
                return
            self._initial_claim_generation = None
            if self._active_route is None and self._recovery_generation is None:
                self._objective_kind = None
                self._objective_goal = None
                self._home_intent = None
                self._indexed_map_validated = False
                self._clear_rolling_locked()

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

    def _authority_identity_matches(self, binding, authority) -> bool:
        if binding is None:
            return True
        try:
            current = self._authority_binding(authority)
        except ValueError:
            return False
        return current is not None and current[:2] == binding[:2]

    def _goals_materially_differ(self, before, after) -> bool:
        try:
            translation = math.sqrt(
                sum(
                    (float(before.get(axis, 0.0)) - float(after.get(axis, 0.0))) ** 2
                    for axis in ("x", "y", "z")
                )
            )
            yaw_delta = math.atan2(
                math.sin(float(after.get("yaw", 0.0)) - float(before.get("yaw", 0.0))),
                math.cos(float(after.get("yaw", 0.0)) - float(before.get("yaw", 0.0))),
            )
        except (AttributeError, TypeError, ValueError):
            return True
        return (
            translation > self.authority_translation_tolerance_m
            or abs(yaw_delta) > self.authority_rotation_tolerance_rad
        )

    def _goal_authority_error(
        self, goal, authority, *, raw_authority=None
    ) -> str | None:
        """Reject explicit frame identities that disagree with fresh authority."""

        component_goal = goal.get("component_goal")
        source_frame = str(
            goal.get("frame_id") or getattr(self.bridge, "map_frame", self.frame)
        ).lstrip("/")
        raw_frame = str((raw_authority or {}).get("navigation_frame", "")).lstrip("/")
        if not raw_frame and raw_authority is None:
            raw_frame = str(authority.get("navigation_frame", "")).lstrip("/")
        if component_goal is not None:
            identities = (
                goal.get("mission_id"),
                goal.get("component_id"),
                goal.get("frame_id"),
                authority.get("mission_id"),
                authority.get("component_id"),
                raw_frame,
            )
            if not isinstance(component_goal, dict) or any(
                not isinstance(value, str) or not value for value in identities
            ):
                return (
                    "component-frame objective requires explicit current frame "
                    "authority"
                )
        goal_mission = str(goal.get("mission_id", ""))
        goal_component = str(goal.get("component_id", ""))
        authority_mission = str(authority.get("mission_id", ""))
        authority_component = str(authority.get("component_id", ""))
        if goal_mission and authority_mission and goal_mission != authority_mission:
            return "MGG objective mission differs from current map authority"
        if (
            goal_component
            and authority_component
            and goal_component != authority_component
        ):
            return "MGG objective component differs from current map authority"
        authority_frame = str(authority.get("navigation_frame", "")).lstrip("/")
        if authority_frame and authority_frame != self.frame:
            return "MGG objective frame differs from current navigation authority"
        if component_goal is not None and source_frame != raw_frame:
            return "MGG objective frame differs from current navigation authority"
        if source_frame not in {self.frame, raw_frame}:
            return "MGG objective frame differs from current navigation authority"
        return None

    @staticmethod
    def _goal_solution_order_error(goal, authority) -> str | None:
        """Fence a newly admitted component click to its displayed frame revision."""

        if not isinstance(goal.get("component_goal"), dict):
            return None
        expected = goal.get("solution_order")
        if expected is None:
            return "component-frame objective has no frame revision token"
        try:
            expected_order = solution_order(expected)
            current_order = solution_order(authority["solution_order"])
        except (KeyError, TypeError, ValueError):
            return "component-frame objective has no valid frame revision authority"
        if current_order != expected_order:
            return "component-frame objective uses a stale frame revision"
        return None

    @staticmethod
    def _component_goal_in_navigation(goal, authority):
        """Resolve an immutable component goal through fresh shared authority."""

        component_goal = goal.get("component_goal")
        if not isinstance(component_goal, dict):
            return deepcopy(goal)
        transform = authority.get("T_component_navigation")
        if transform is None:
            raise ValueError("component-frame objective has no navigation transform")
        resolved = deepcopy(goal)
        resolved.update(navigation_goal(component_goal, transform))
        resolved["frame_id"] = str(authority.get("navigation_frame", "")).lstrip("/")
        return resolved

    def _goal_in_planning(self, goal, authority, raw_authority=None):
        """Resolve one exact UI/component goal into the selected planner frame."""

        if isinstance(goal.get("component_goal"), dict):
            return self._component_goal_in_navigation(goal, authority)
        source_frame = str(
            goal.get("frame_id") or getattr(self.bridge, "map_frame", self.frame)
        ).lstrip("/")
        resolved = deepcopy(goal)
        if source_frame != self.frame:
            if not isinstance(raw_authority, dict):
                raise ValueError("MGG objective has no fresh source-frame authority")
            raw_frame = str(raw_authority.get("navigation_frame", "")).lstrip("/")
            if source_frame != raw_frame:
                raise ValueError("MGG objective has an unsupported source frame")
            source_transform = raw_authority.get("T_component_navigation")
            target_transform = authority.get("T_component_navigation")
            if source_transform is None or target_transform is None:
                raise ValueError("MGG objective has no qualified frame transform")
            # Convert navigation -> component, then use the same qualified
            # component -> planning inverse as component-frame clicks.
            from autonomy.contracts import validate_se3
            import numpy as np

            inverse_source = np.linalg.inv(
                np.asarray(validate_se3(source_transform), dtype=float)
            ).tolist()
            component = navigation_goal(goal, inverse_source)
            resolved.update(navigation_goal(component, target_transform))
        resolved["frame_id"] = self.frame
        return resolved

    def decorate_state(self, state: dict) -> dict:
        """Keep an owned objective visible across controller/planner handoffs.

        The bridge owns controller status. Session serialization calls this
        after reading bridge state so correction recovery cannot erase the
        retained destination while its replacement full route is planned.
        """

        decorated = dict(state)
        with self._active_lock:
            active = self._active_route
            recovery_generation = self._recovery_generation
            continuation_generation = self._continuation_generation
            objective = self._objective_kind
            goal = deepcopy(self._objective_goal)
            home_intent = self._home_intent
            indexed_map_validated = self._indexed_map_validated
            generation = self.bridge._goal_generation
            initial_claim_generation = self._initial_claim_generation
            rolling_partial = self._rolling_partial
            if self._nav_failure_generation != generation:
                self._nav_failure_reason = None
                self._nav_failure_generation = None
            failure_reason = self._nav_failure_reason
            owned_sequence = bool(
                objective
                and (
                    (active is not None and active[0] == generation)
                    or recovery_generation == generation
                    or continuation_generation == generation
                    or initial_claim_generation == generation
                )
            )
        if owned_sequence:
            raw_authority = self._raw_authority()
            authority = (
                self._authority_from_raw(raw_authority)
                if isinstance(raw_authority, dict)
                else None
            )
            if active is not None:
                goal = {**active[3], "frame_id": self.frame}
            elif (
                objective == "return_home"
                and self._authority_home_identity(authority) == home_intent
            ):
                goal = self._authority_home_from(authority)
            elif isinstance(goal, dict):
                if not self._goal_authority_error(
                    goal, authority or {}, raw_authority=raw_authority
                ):
                    try:
                        goal = self._goal_in_planning(
                            goal, authority or {}, raw_authority
                        )
                    except ValueError:
                        goal = None
                else:
                    goal = None
        # A controller failure/cancellation is terminal. Only hide the
        # controller's ordinary active/idle handoff states while replanning
        # remains owned.
        terminal_failure = decorated.get("nav_status") in {
            "failed",
            "cancelled",
            "estop",
        }
        final_success = bool(
            active is not None
            and decorated.get("nav_status") == "succeeded"
            and not rolling_partial
        )
        if owned_sequence and not terminal_failure and not final_success:
            decorated["nav_status"] = "active"
            if goal is not None:
                decorated["goal"] = goal
            decorated["objective_continuation"] = {
                "objective": objective,
                "evidence_source": (
                    "mola_indexed" if indexed_map_validated else "mgg_native"
                ),
                "phase": (
                    "following_local"
                    if active is not None and rolling_partial
                    else "following_final" if active is not None else "planning"
                ),
            }
        if decorated.get("nav_status") == "failed":
            # Native planner failures are retained by the planner, while
            # controller failures arrive directly from the bridge telemetry.
            # Keep the planner's bounded reason authoritative when both exist.
            reason = failure_reason or decorated.get("nav_failure_reason")
            if reason:
                decorated["nav_failure_reason"] = str(reason)[
                    :MAX_NAV_FAILURE_REASON_LENGTH
                ]
        else:
            decorated.pop("nav_failure_reason", None)
        return decorated

    def _authority_matches_plan(
        self, binding, snapshot, home_intent, indexed_map_validated=False
    ) -> bool:
        authority = self._authority()
        if home_intent is not None and (
            self._authority_home_identity(authority) != home_intent
            or home_intent[4] != self.frame
        ):
            return False
        try:
            if self._route_authority_changed(binding, authority):
                return False
            return (
                not indexed_map_validated
                or self._mapping_snapshot(authority or {}) == snapshot
            )
        except ValueError:
            return False

    def _start_recovery(
        self,
        generation,
        home_intent,
        *,
        cancel_route,
        binding=None,
        mode="general",
    ) -> bool:
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
            self._initial_claim_generation = None
            self._home_intent = home_intent
            self._clear_rolling_locked()
            self._recovery_generation = generation
            self._recovery_binding = binding
            self._recovery_deadline = deadline
            self._recovery_mode = mode
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
                self._recovery_mode = None
                self._home_intent = None
                raise
        return True

    def _recovery_worker(self) -> None:
        while True:
            with self._active_lock:
                generation = self._recovery_generation
                home_intent = self._home_intent
                objective = self._objective_kind
                deadline = self._recovery_deadline
                binding = self._recovery_binding
                mode = self._recovery_mode
                if generation is None or objective is None or deadline is None:
                    self._recovery_thread = None
                    return
            try:
                self._recover_objective(
                    generation, objective, home_intent, deadline, binding, mode
                )
            except Exception as exc:
                self._finish_recovery_failure(
                    generation, f"MGG objective recovery failed: {exc}"
                )

    def _recover_objective(
        self,
        generation,
        objective,
        home_intent,
        deadline,
        binding=None,
        mode="general",
    ) -> None:
        last_error = "no fresh map authority was available"
        last_outcome = None
        attempts = 0
        if not self.bridge.wait_goal_quiet(generation, deadline):
            if self._owns_recovery(generation, home_intent):
                self._finish_recovery_failure(
                    generation,
                    "MGG objective cancellation did not settle before deadline",
                )
            else:
                self._clear_recovery(generation, home_intent)
            return
        while attempts < self.replan_max_attempts and self._remaining(deadline) > 0:
            if not self._wait_for_recovery(generation, home_intent, deadline):
                self._clear_recovery(generation, home_intent)
                return
            raw_authority = self._raw_authority()
            authority = (
                self._authority_from_raw(raw_authority)
                if isinstance(raw_authority, dict)
                else None
            )
            if raw_authority is None and (
                binding is not None or home_intent is not None
            ):
                # Unavailable input is not a failed planning attempt. Keep
                # motion stopped and wait, bounded by the original deadline.
                last_error = "no fresh map authority was available"
                time.sleep(min(0.05, self._remaining(deadline)))
                continue
            if binding is not None and not self._authority_identity_matches(
                binding, authority
            ):
                self._finish_recovery_failure(
                    generation,
                    "map authority mission or component changed during recovery",
                )
                return
            try:
                if objective == "return_home" and home_intent is not None:
                    if self._authority_binding(authority) is None:
                        raise ValueError("map authority has no navigation transform")
                    identity = self._authority_home_identity(authority)
                    if identity is not None and identity != home_intent:
                        self._finish_recovery_failure(
                            generation,
                            "return-home authority identity changed during recovery",
                        )
                        return
                    target = self._authority_home_from(authority)
                    if target is None:
                        raise ValueError("no fresh return-home authority was available")
                else:
                    with self._active_lock:
                        original = deepcopy(self._objective_goal)
                    if not isinstance(original, dict):
                        raise ValueError("retained Navigate objective is unavailable")
                    identity_error = self._goal_authority_error(
                        original,
                        authority or {},
                        raw_authority=raw_authority,
                    )
                    if identity_error:
                        raise ValueError(identity_error)
                    target = self._goal_in_planning(
                        original, authority or {}, raw_authority
                    )
            except ValueError as exc:
                last_error = str(exc)
                attempts += 1
                continue
            attempts += 1
            outcome, last_error, _ = self._plan_once(
                objective,
                target,
                authority=authority,
                expected_generation=generation,
                home_intent=home_intent,
                overall_deadline=deadline,
            )
            last_outcome = outcome
            if outcome == "temporary" and planner_input_unavailable(last_error):
                # Missing planner input is not a failed planning attempt. Keep
                # motion stopped and ask again, bounded by the recovery deadline
                # rather than by the attempt budget.
                attempts -= 1
                time.sleep(min(PLANNER_INPUT_RETRY_S, self._remaining(deadline)))
            if outcome == "submitted":
                return
            if outcome == "superseded":
                self._clear_recovery(generation, home_intent)
                return
            if outcome == "terminal":
                self._finish_recovery_failure(generation, last_error)
                return
            if mode == "planning_budget" and outcome not in {
                "temporary",
                "authority_changed",
            }:
                self._finish_recovery_failure(generation, last_error)
                return
            if not self.bridge.set_goal_pending_if_current(generation):
                self._clear_recovery(generation, home_intent)
                return
        if last_outcome == "temporary" and planner_input_unavailable(last_error):
            message = (
                "MGG planning map stayed unavailable for "
                f"{self.replan_deadline_s:.0f} s: {last_error}"
            )
        elif last_outcome == "temporary":
            message = (
                f"MGG planning budget remained exhausted after {attempts} retries: "
                f"{last_error}"
            )
        else:
            message = (
                f"MGG objective recovery exhausted after {attempts} attempts: "
                f"{last_error}"
            )
        self._finish_recovery_failure(generation, message)

    def _wait_for_recovery(self, generation, home_intent, deadline) -> bool:
        wake_at = min(deadline, time.monotonic() + self.replan_backoff_s)
        while time.monotonic() < wake_at:
            if not self._owns_recovery(generation, home_intent):
                return False
            time.sleep(min(0.02, wake_at - time.monotonic()))
        return self._owns_recovery(generation, home_intent)

    def _finish_recovery_failure(self, generation, message) -> None:
        with self._active_lock:
            if self._recovery_generation != generation or self._objective_kind is None:
                return
            self._recovery_generation = None
            self._recovery_deadline = None
            self._recovery_mode = None
            self._initial_claim_generation = None
            self._home_intent = None
            self._objective_kind = None
            self._objective_goal = None
            self._indexed_map_validated = False
            self._clear_rolling_locked()
        self._fail_if_current(message, generation)

    def _clear_recovery(self, generation, home_intent) -> None:
        with self._active_lock:
            if (
                self._recovery_generation == generation
                and self._home_intent == home_intent
            ):
                self._recovery_generation = None
                self._recovery_deadline = None
                self._recovery_mode = None
                self._initial_claim_generation = None
                self._home_intent = None
                self._objective_kind = None
                self._objective_goal = None
                self._indexed_map_validated = False
                self._clear_rolling_locked()

    def _fail_if_current(self, message: str, generation: int) -> None:
        if self.bridge.set_nav_status_if_current(generation, "failed"):
            with self._active_lock:
                if generation == self.bridge._goal_generation:
                    reason = str(message).strip()[:MAX_NAV_FAILURE_REASON_LENGTH]
                    self._nav_failure_reason = reason or None
                    self._nav_failure_generation = generation
                    self._active_route = None
                    self._initial_claim_generation = None
                    self._recovery_generation = None
                    self._recovery_deadline = None
                    self._recovery_mode = None
                    self._home_intent = None
                    self._objective_kind = None
                    self._objective_goal = None
                    self._indexed_map_validated = False
                    self._clear_rolling_locked()
            self.bridge.node.get_logger().warning(f"[{self.bridge.id}] {message}")


def configure_objective_planning(bridge):
    config = (bridge.cfg or {}).get("planning") or {}
    backend = str(
        config.get("backend") or (bridge.cfg or {}).get("planning_backend") or ""
    ).lower()
    bridge.objective_planner = (
        MggObjectivePlanning(bridge, config) if backend == "mgg" else None
    )
