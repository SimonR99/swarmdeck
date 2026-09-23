"""Bounded MGG client for whole-route navigate and return-home objectives."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import math
import threading
import time
from types import SimpleNamespace

from adapters.exploration import is_physical_no_progress_failure, planner_path
from autonomy.live_mapping import navigation_goal

MAX_NAV_FAILURE_REASON_LENGTH = 512
FULL_ROUTE_ENDPOINT_TOLERANCE_M = 0.001


@dataclass(frozen=True)
class ObjectivePlanClaim:
    """Generation ownership minted before blocking objective planning starts."""

    objective: str
    goal: dict | None
    generation: int
    # Explore toward a navigate goal MGG has no known route to, instead of
    # failing it (adapters.goal_exploration).
    explore_if_unknown: bool = False


class MggObjectivePlanning:
    """Plan one complete MGG route and give it to the full-path controller."""

    def __init__(self, bridge, config):
        from adapters.mapping_authority import get_mapping_authority, planning_frame
        from mgg_msgs.srv import PlanObjective

        self.bridge = bridge
        self.service_type = PlanObjective
        namespace = str(config.get("namespace") or f"/{bridge.id}/mgg").rstrip("/")
        self.client = bridge.node.create_client(
            PlanObjective, f"{namespace}/plan_objective"
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
        self.authority_replan_max_attempts = self._bounded_int(
            config.get("authority_replan_max_attempts", 3), 1, 20, 3
        )
        self.authority_replan_backoff_s = self._bounded_float(
            config.get("authority_replan_backoff_s", 0.25), 0.0, 10.0, 0.25
        )
        self.controller_replan_max_attempts = self._bounded_int(
            config.get("controller_replan_max_attempts", 2), 0, 20, 2
        )
        self.controller_replan_deadline_s = self._bounded_float(
            config.get("controller_replan_deadline_s", 15.0), 0.1, 300.0, 15.0
        )
        self.controller_replan_backoff_s = self._bounded_float(
            config.get("controller_replan_backoff_s", 0.25), 0.0, 10.0, 0.25
        )
        self.home = config.get("home")
        self._lock = threading.RLock()
        self._objective_kind = None
        self._objective_goal = None
        self._active_route = None
        self._planning_generation = None
        self._initial_claim_generation = None
        self._nav_failure_reason = None
        self._nav_failure_generation = None
        self._controller_attempts = 0
        self._controller_deadline = 0.0
        self._controller_due = 0.0
        self._recovery_thread = None
        self._recovery_generation = None
        self._authority_timer = bridge.node.create_timer(
            max(0.05, float(config.get("authority_check_period_s", 0.1))), self.tick
        )

    @staticmethod
    def _bounded_float(value, lower, upper, default):
        try:
            value = float(value)
        except (TypeError, ValueError, OverflowError):
            return default
        return min(upper, max(lower, value)) if math.isfinite(value) else default

    @staticmethod
    def _bounded_int(value, lower, upper, default):
        try:
            value = int(value)
        except (TypeError, ValueError, OverflowError):
            return default
        return min(upper, max(lower, value))

    def _raw_authority(self):
        current = getattr(self.authority_reader, "current", None)
        value = current() if callable(current) else None
        if isinstance(value, dict):
            return value
        return None

    def _authority(self):
        from adapters.mapping_authority import authority_for_frame

        raw = self._raw_authority()
        if not isinstance(raw, dict):
            return None
        try:
            return authority_for_frame(raw, self.frame)
        except (KeyError, ValueError):
            raw_frame = str(raw.get("navigation_frame", self.frame)).lstrip("/")
            return deepcopy(raw) if raw_frame == self.frame else None

    def _authority_home(self, authority):
        if not isinstance(authority, dict):
            return None
        home = authority.get("home")
        if not isinstance(home, dict):
            home = authority
        transform = home.get("T_navigation_home")
        try:
            matrix = [[float(x) for x in row] for row in transform]
            values = [x for row in matrix for x in row]
            if len(matrix) != 4 or any(len(row) != 4 for row in matrix):
                return None
            if not all(math.isfinite(x) for x in values):
                return None
            return {
                "x": matrix[0][3],
                "y": matrix[1][3],
                "z": matrix[2][3],
                "yaw": math.atan2(matrix[1][0], matrix[0][0]),
                "frame_id": str(authority.get("navigation_frame", self.frame)).lstrip(
                    "/"
                ),
            }
        except (TypeError, ValueError):
            return None

    def _goal_in_frame(self, goal, authority):
        if not isinstance(goal, dict):
            raise ValueError("objective goal is required")
        resolved = deepcopy(goal)
        component_goal = resolved.get("component_goal")
        if isinstance(component_goal, dict):
            transform = (authority or {}).get("T_component_navigation")
            if transform is None:
                raise ValueError("objective has no navigation transform")
            resolved.update(navigation_goal(component_goal, transform))
        if "z" not in resolved:
            # A 2-D goal has no absolute floor selection. Seed ground projection
            # from the robot's current altitude, not the odometry origin.
            current = self.bridge.map_pose()
            if not isinstance(current, dict) or "z" not in current:
                raise ValueError("objective has no current navigation altitude")
            height = current["z"]
            if self.frame != self.bridge.navigation_frame.lstrip("/"):
                import numpy as np
                from adapters.mapping_authority import authority_for_frame

                source = authority_for_frame(
                    self._raw_authority(), self.bridge.navigation_frame
                )
                transform = np.linalg.solve(
                    np.asarray(authority["T_component_navigation"]),
                    np.asarray(source["T_component_navigation"]),
                )
                position = transform @ np.array(
                    [current["x"], current["y"], current["z"], 1.0]
                )
                height = position[2]
            resolved["z"] = height
        values = [resolved.get(key, 0.0) for key in ("x", "y", "z", "yaw")]
        try:
            values = [float(value) for value in values]
        except (TypeError, ValueError):
            raise ValueError("objective goal contains invalid coordinates") from None
        if not all(math.isfinite(value) for value in values):
            raise ValueError("objective goal contains nonfinite coordinates")
        resolved.update(dict(zip(("x", "y", "z", "yaw"), values)))
        resolved["frame_id"] = self.frame
        return resolved

    @staticmethod
    def _snapshot(authority):
        authority = authority or {}
        stamp = authority.get("map_source_stamp") or {}
        try:
            epoch = int(authority.get("map_epoch", 0))
            revision = int(authority.get("mapping_graph_revision", 0))
            sec = int(stamp.get("sec", 0))
            nanosec = int(stamp.get("nanosec", 0))
        except (AttributeError, TypeError, ValueError):
            raise ValueError("map authority has an invalid snapshot") from None
        geometry = str(authority.get("geometry_revision", ""))
        if epoch < 0 or revision < 0 or sec < 0 or not 0 <= nanosec < 1_000_000_000:
            raise ValueError("map authority has an invalid snapshot")
        return epoch, revision, geometry, sec, nanosec

    def _request(self, objective, goal, authority):
        request = self.service_type.Request()
        request.mission_id = str((authority or {}).get("mission_id", ""))
        request.component_id = str(
            (authority or {}).get("component_id", self.component_id)
        )
        request.objective = {
            "navigate": self.service_type.Request.NAVIGATE,
            "return_home": self.service_type.Request.RETURN_HOME,
        }[objective]
        epoch, revision, geometry, sec, nanosec = self._snapshot(authority)
        request.map_epoch = epoch
        request.mapping_graph_revision = revision
        request.geometry_revision = geometry
        request.map_source_stamp.sec = sec
        request.map_source_stamp.nanosec = nanosec
        request.goal.position.x = goal["x"]
        request.goal.position.y = goal["y"]
        request.goal.position.z = goal.get("z", 0.0)
        request.goal.orientation.x = 0.0
        request.goal.orientation.y = 0.0
        request.goal.orientation.z = math.sin(goal.get("yaw", 0.0) / 2.0)
        request.goal.orientation.w = math.cos(goal.get("yaw", 0.0) / 2.0)
        return request

    @staticmethod
    def _identity_matches(request, response):
        for field in (
            "component_id",
            "map_epoch",
            "mapping_graph_revision",
            "geometry_revision",
        ):
            if getattr(response, field, None) != getattr(request, field, None):
                return False
        expected = request.map_source_stamp
        actual = getattr(response, "map_source_stamp", None)
        return (
            actual is not None
            and int(getattr(actual, "sec", -1)) == int(expected.sec)
            and int(getattr(actual, "nanosec", -1)) == int(expected.nanosec)
        )

    def _call_once(self, objective, goal, generation, deadline=None):
        authority = self._authority() or {}
        request = self._request(objective, goal, authority)
        if not self._owns(generation):
            return "superseded", "", None
        if not self.client.wait_for_service(
            timeout_sec=min(self.timeout_s, max(0.0, self._remaining(deadline)))
        ):
            return "failed", "MGG objective service is unavailable", None
        try:
            future = self.client.call_async(request)
        except Exception as exc:
            return "failed", f"MGG objective request failed: {exc}", None
        end = min(
            time.monotonic() + self.timeout_s,
            deadline if deadline is not None else math.inf,
        )
        while not future.done() and time.monotonic() < end:
            if not self._owns(generation):
                future.cancel()
                return "superseded", "", None
            time.sleep(0.01)
        if not future.done():
            future.cancel()
            return "failed", "MGG objective request timed out", None
        if not self._owns(generation):
            return "superseded", "", None
        try:
            response = future.result()
        except Exception as exc:
            return "failed", f"MGG objective request failed: {exc}", None
        stale = getattr(self.service_type.Response, "STALE_REVISION", object())
        if response.status == stale or not self._identity_matches(request, response):
            return "stale", "MGG returned a stale map identity", None
        if response.status != self.service_type.Response.SUCCEEDED:
            reason = str(
                getattr(response, "reason", "") or "MGG objective was rejected"
            )
            return "failed", reason, None
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
                path, self.frame, self.planar_tolerance_m, self.max_inclination_rad
            )
        except (AttributeError, TypeError, ValueError) as exc:
            return "failed", f"MGG returned an unsupported path: {exc}", None
        endpoint = plan.poses[-1]
        distance = math.hypot(endpoint.x - goal["x"], endpoint.y - goal["y"])
        if distance > FULL_ROUTE_ENDPOINT_TOLERANCE_M:
            return "failed", "MGG whole route endpoint differs from objective", None
        return "ready", "", plan

    def _owns(self, generation):
        # None: a route probe, which drives nothing and is owned by no goal.
        return generation is None or generation == self.bridge._goal_generation

    @staticmethod
    def _remaining(deadline):
        return math.inf if deadline is None else max(0.0, deadline - time.monotonic())

    def claim_objective(self, objective, goal=None, explore_if_unknown=False):
        if objective not in ("navigate", "return_home"):
            raise ValueError(f"unsupported MGG objective {objective!r}")
        copied = deepcopy(goal) if isinstance(goal, dict) else None
        generation = self.bridge.cancel_goal()
        with self._lock:
            self._objective_kind = objective
            self._objective_goal = copied
            self._active_route = None
            self._initial_claim_generation = generation
            self._planning_generation = generation
            self._nav_failure_reason = None
            self._nav_failure_generation = None
            self._controller_attempts = 0
        self.bridge.set_goal_pending_if_current(generation)
        return ObjectivePlanClaim(
            objective,
            copied,
            generation,
            explore_if_unknown=bool(explore_if_unknown) and objective == "navigate",
        )

    def execute_claimed(self, claim):
        if not isinstance(claim, ObjectivePlanClaim):
            raise TypeError("objective plan claim is required")
        if not self._owns(claim.generation):
            return False
        goal = claim.goal
        authority = self._authority() or {}
        if claim.objective == "return_home":
            goal = self._authority_home(authority) or goal or self.home
        try:
            goal = self._goal_in_frame(goal, authority)
        except ValueError as exc:
            self._fail_if_current(str(exc), claim.generation)
            return False
        deadline = time.monotonic() + self.timeout_s
        for attempt in range(self.authority_replan_max_attempts + 1):
            if not self._owns(claim.generation):
                return False
            outcome, reason, plan = self._call_once(
                claim.objective, goal, claim.generation, deadline
            )
            if outcome == "stale":
                authority = self._authority() or {}
                if claim.objective == "return_home":
                    goal = self._authority_home(authority) or goal
                try:
                    goal = self._goal_in_frame(goal, authority)
                except ValueError as exc:
                    self._fail_if_current(str(exc), claim.generation)
                    return False
                if attempt < self.authority_replan_max_attempts:
                    time.sleep(
                        min(
                            self.authority_replan_backoff_s,
                            self._remaining(deadline),
                        )
                    )
                    continue
                reason = "MGG objective map identity stayed stale"
                outcome = "failed"
            if outcome != "ready":
                if outcome != "superseded" and not self._explore_instead(
                    claim, goal, reason
                ):
                    self._fail_if_current(reason, claim.generation)
                return False
            try:
                accepted = self.bridge.follow_path(
                    plan, expected_generation=claim.generation
                )
            except Exception as exc:
                self._fail_if_current(f"path execution failed: {exc}", claim.generation)
                return False
            if type(accepted) is not int or accepted != self.bridge._goal_generation:
                return False
            with self._lock:
                self._initial_claim_generation = None
                self._planning_generation = None
                self._objective_goal = deepcopy(goal)
                self._active_route = (accepted, plan)
            return True
        return False

    def plan(self, objective, goal):
        claim = self.claim_objective(objective, goal)
        return self.execute_claimed(claim)

    def navigate(self, goal):
        return self.plan("navigate", goal)

    def return_home(self, goal=None):
        return self.plan("return_home", goal)

    def global_display_plan(self):
        with self._lock:
            active = self._active_route
            if active is None or active[0] != self.bridge._goal_generation:
                return None
            return active[1]

    def _start_controller_recovery(self, generation):
        with self._lock:
            if self._recovery_thread is not None or not self._owns(generation):
                return
            self._recovery_generation = generation
            self._controller_deadline = (
                time.monotonic() + self.controller_replan_deadline_s
            )
            worker = threading.Thread(
                target=self._controller_recovery_worker,
                args=(generation,),
                daemon=True,
            )
            self._recovery_thread = worker
            worker.start()

    def _controller_recovery_worker(self, generation):
        try:
            while self._owns(generation):
                with self._lock:
                    if self._controller_attempts >= self.controller_replan_max_attempts:
                        self._fail_if_current(
                            "controller recovery exhausted", generation
                        )
                        return
                    deadline = self._controller_deadline
                    objective = self._objective_kind
                    goal = deepcopy(self._objective_goal)
                if self._remaining(deadline) <= 0.0:
                    self._fail_if_current("controller recovery timed out", generation)
                    return
                time.sleep(
                    min(self.controller_replan_backoff_s, self._remaining(deadline))
                )
                with self._lock:
                    self._controller_attempts += 1
                authority = self._authority() or {}
                if objective == "return_home":
                    goal = self._authority_home(authority) or goal
                try:
                    goal = self._goal_in_frame(goal, authority)
                except ValueError as exc:
                    self._fail_if_current(str(exc), generation)
                    return
                outcome, reason, plan = self._call_once(
                    objective, goal, generation, deadline
                )
                if outcome == "stale":
                    continue
                if outcome != "ready":
                    self._fail_if_current(reason, generation)
                    return
                accepted = self.bridge.follow_path(plan, expected_generation=generation)
                if (
                    type(accepted) is not int
                    or accepted != self.bridge._goal_generation
                ):
                    return
                with self._lock:
                    self._objective_goal = deepcopy(goal)
                    self._active_route = (accepted, plan)
                return
        finally:
            with self._lock:
                self._recovery_thread = None
                self._recovery_generation = None

    def _check_controller_result(self, now=None):
        with self._lock:
            active = self._active_route
        if active is None or active[0] != self.bridge._goal_generation:
            return
        if self.bridge.nav_status == "succeeded":
            with self._lock:
                self._active_route = active
                self._nav_failure_reason = None
                self._nav_failure_generation = None
            return
        if self.bridge.nav_status != "failed":
            return
        reason = getattr(self.bridge, "_nav_failure_reason", None)
        if not is_physical_no_progress_failure(reason):
            self._fail_if_current(str(reason or "controller failed"), active[0])
            return
        with self._lock:
            if self._controller_attempts >= self.controller_replan_max_attempts:
                exhausted = True
            else:
                exhausted = False
        if exhausted:
            self._fail_if_current(
                str(reason or "controller made no progress"), active[0]
            )
        else:
            self._start_controller_recovery(active[0])

    def tick(self):
        self._check_controller_result()

    def decorate_state(self, state):
        decorated = dict(state)
        with self._lock:
            active = self._active_route
            objective = self._objective_kind
            goal = deepcopy(self._objective_goal)
            generation = self.bridge._goal_generation
            failure = (
                self._nav_failure_reason
                if self._nav_failure_generation == generation
                else None
            )
            planning = self._planning_generation == generation
            recovering = self._recovery_generation == generation
        active_owned = active is not None and active[0] == generation
        owned = objective is not None and (planning or recovering or active_owned)
        if owned and decorated.get("nav_status") not in {
            "failed",
            "cancelled",
            "estop",
            "succeeded",
        }:
            decorated["nav_status"] = "active"
            if goal is not None:
                decorated["goal"] = goal
        if decorated.get("nav_status") == "failed":
            reason = failure or decorated.get("nav_failure_reason")
            if reason:
                decorated["nav_failure_reason"] = str(reason)[
                    :MAX_NAV_FAILURE_REASON_LENGTH
                ]
        else:
            decorated.pop("nav_failure_reason", None)
        return decorated

    def _explore_instead(self, claim, goal, reason):
        """Hand a navigate goal with no known route to goal exploration."""
        from adapters.goal_exploration import is_no_known_route

        explorer = getattr(self.bridge, "goal_exploration", None)
        if (
            not claim.explore_if_unknown
            or explorer is None
            or not is_no_known_route(reason)
            or not self._owns(claim.generation)
        ):
            return False
        with self._lock:
            self._planning_generation = None
            self._initial_claim_generation = None
        return explorer.begin(claim.goal, goal)

    def _fail_if_current(self, message, generation):
        if self.bridge.set_nav_status_if_current(generation, "failed"):
            with self._lock:
                self._nav_failure_reason = str(message).strip()[
                    :MAX_NAV_FAILURE_REASON_LENGTH
                ]
                self._nav_failure_generation = generation
                self._active_route = None
                self._planning_generation = None
                self._initial_claim_generation = None
            logger = self.bridge.node.get_logger()
            logger.warning(f"[{self.bridge.id}] {message}")


def configure_objective_planning(bridge):
    config = (bridge.cfg or {}).get("planning") or {}
    backend = str(
        config.get("backend") or (bridge.cfg or {}).get("planning_backend") or ""
    ).lower()
    bridge.objective_planner = (
        MggObjectivePlanning(bridge, config) if backend == "mgg" else None
    )
    bridge.goal_exploration = None
    exploration = getattr(bridge, "exploration", None)
    if bridge.objective_planner is not None and exploration is not None:
        from adapters.goal_exploration import GoalExploration

        try:
            bridge.goal_exploration = GoalExploration(
                bridge, bridge.objective_planner, exploration, config
            )
        except ImportError as exc:
            # An mgg_msgs older than set_exploration_target: goals without a
            # known route fail as before.
            bridge.node.get_logger().warning(
                f"[{bridge.id}] explore-to-goal unavailable: {exc}"
            )
