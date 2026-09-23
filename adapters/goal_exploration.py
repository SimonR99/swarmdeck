"""Explore toward a waypoint that has no known route yet, then go there.

A navigate objective sent with ``explore_if_unknown`` that MGG refuses for
lack of a known route (the goal is not linked to the explored map, or there
is no mapped ground under it) is handed here instead of failing. This sets
the goal as MGG's exploration target, which biases exploration toward it
softly, and runs ordinary exploration. Every few seconds it asks MGG, without
driving, whether a route exists yet; once one does, exploration stops and the
goal is navigated to like any other. Exploration running out first fails the
goal with that reason; any operator command ends it quietly.
"""

from __future__ import annotations

from contextlib import nullcontext
from copy import deepcopy
import threading
import time

# MGG refusals that mean "the explored map does not reach the goal yet", as
# opposed to a planner, map or robot problem that exploring would not fix.
NO_KNOWN_ROUTE_PHRASES = (
    "goal cannot be linked",
    "no mapped ground under the goal",
    "no route over the global graph reaches the goal",
)


def is_no_known_route(reason) -> bool:
    text = str(reason or "").lower()
    return any(phrase in text for phrase in NO_KNOWN_ROUTE_PHRASES)


class GoalExploration:
    def __init__(self, bridge, planner, exploration, config):
        from mgg_msgs.srv import PlannerSetExplorationTarget

        self.bridge = bridge
        self.planner = planner
        self.exploration = exploration
        self.service_type = PlannerSetExplorationTarget
        namespace = str(config.get("namespace") or f"/{bridge.id}/mgg").rstrip("/")
        self.client = bridge.node.create_client(
            PlannerSetExplorationTarget, f"{namespace}/set_exploration_target"
        )
        self.probe_period_s = max(
            0.5, float(config.get("explore_to_goal_probe_period_s", 5.0))
        )
        self._lock = threading.RLock()
        self.active = False
        # The goal as the operator sent it, and in the planning frame.
        self.requested_goal = None
        self.goal = None
        self._run = 0
        self._next_probe = 0.0
        self._probe = None
        self._route_ready_run = None
        self.timer = bridge.node.create_timer(0.5, self.tick)

    @property
    def display_goal(self):
        with self._lock:
            return deepcopy(self.requested_goal) if self.active else None

    def begin(self, requested_goal, goal) -> bool:
        """Take over a navigate objective refused for lack of a known route.

        `goal` is in the planning frame. Returns False, leaving the objective
        to fail as usual, when exploration cannot start.
        """
        if self.exploration is None or not self.client.service_is_ready():
            return False
        with self._lock:
            self._run += 1
            run = self._run
            self.active = True
            self.requested_goal = deepcopy(requested_goal)
            self.goal = deepcopy(goal)
            self._next_probe = time.monotonic() + self.probe_period_s
            self._route_ready_run = None
        self._send_target(goal)
        self.exploration.start()
        if not self.exploration.active:
            self._end(run)
            return False
        self._log("no known route to the goal yet; exploring toward it")
        return True

    def cancel(self):
        """An operator command superseded the goal: stop quietly."""
        with self._lock:
            run = self._run if self.active else None
        if run is not None:
            self._end(run)

    def tick(self):
        with self._lock:
            if not self.active:
                return
            run = self._run
            ready = self._route_ready_run == run
            probing = self._probe is not None and self._probe.is_alive()
            due = time.monotonic() >= self._next_probe
            goal = deepcopy(self.goal)
            requested = deepcopy(self.requested_goal)
        if ready:
            self._navigate(run, requested)
            return
        if not self.exploration.active:
            status = getattr(self.exploration, "status", "stopped")
            why = getattr(self.exploration, "reason", None)
            self._end(run)
            if status in ("complete", "locally_exhausted", "blocked"):
                detail = f": {why}" if why else ""
                self.planner._fail_if_current(
                    "no known route to the goal, and exploring toward it "
                    f"ended ({status}{detail})",
                    self.bridge._goal_generation,
                )
            return
        if probing or not due:
            return
        with self._lock:
            self._next_probe = time.monotonic() + self.probe_period_s
            self._probe = threading.Thread(
                target=self._probe_route, args=(run, goal), daemon=True
            )
            self._probe.start()

    def _probe_route(self, run, goal):
        # Not tied to a goal generation: exploration's own paths advance it.
        outcome, _reason, _plan = self.planner._call_once("navigate", goal, None)
        with self._lock:
            if outcome == "ready" and self.active and self._run == run:
                self._route_ready_run = run

    def _navigate(self, run, requested):
        with self._lock:
            if not self.active or self._run != run:
                return
        self._end(run)
        self.exploration.stop()
        self._log("a route to the goal is known now; navigating to it")
        with getattr(self.bridge, "_goal_lock", nullcontext()):
            claim = self.planner.claim_objective("navigate", requested)
        threading.Thread(
            target=self.planner.execute_claimed, args=(claim,), daemon=True
        ).start()

    def _end(self, run):
        with self._lock:
            if self._run != run or not self.active:
                return
            self.active = False
            self.goal = None
            self.requested_goal = None
            self._route_ready_run = None
        self._send_target(None)

    def _send_target(self, goal):
        request = self.service_type.Request()
        request.active = goal is not None
        if goal is not None:
            request.target.x = float(goal["x"])
            request.target.y = float(goal["y"])
            request.target.z = float(goal.get("z", 0.0))
        try:
            self.client.call_async(request)
        except Exception as exc:
            self._log(f"exploration target request failed: {exc}")

    def _log(self, text):
        self.bridge.node.get_logger().info(f"[{self.bridge.id}] {text}")
