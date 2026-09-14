"""Receipt-relative exploration leases in verified localization components.

Transport neighbors may belong to unrelated coordinate frames. Such neighbors
are visible for liveness, but their targets must never enter spatial arbitration.
The server has no role in deciding or renewing these leases.
"""

from __future__ import annotations
from dataclasses import dataclass
import math
import time
from typing import Callable

from .contracts import KeyframeId


@dataclass(frozen=True)
class Intention:
    robot_id: str
    session_id: str
    sequence: int
    component_id: str
    target: tuple[float, float, float]
    radius_m: float
    cost: float
    lease_s: float
    active: bool = True

    def __post_init__(self):
        KeyframeId(self.robot_id, self.session_id, self.sequence)
        if not self.component_id or len(self.target) != 3:
            raise ValueError("An intention requires a component and XYZ target")
        if not all(
            math.isfinite(v)
            for v in (*self.target, self.radius_m, self.cost, self.lease_s)
        ):
            raise ValueError("Nonfinite intention")
        if self.radius_m <= 0 or self.cost < 0 or not 0 < self.lease_s <= 10:
            raise ValueError("Invalid intention limits")


class LeaseArbiter:
    def __init__(
        self,
        robot_id: str,
        session_id: str,
        participants: set[str],
        *,
        clock: Callable[[], float] = time.monotonic,
        settle_s: float = 0.5,
    ):
        KeyframeId(robot_id, session_id, 0)
        if robot_id not in participants or not 0 <= settle_s <= 5:
            raise ValueError("Invalid local participant or arbitration interval")
        self.robot_id, self.session_id = robot_id, session_id
        self.participants, self.clock, self.settle_s = (
            set(participants),
            clock,
            settle_s,
        )
        self.component_id = None
        self.heads, self.leases = {}, {}
        self.sequence = 0
        self.local = None
        self.proposed_at = 0.0

    def set_component(self, component_id: str | None):
        if component_id != self.component_id:
            self.component_id = component_id
            self.local = None
            self.leases.clear()
            return True
        return False

    def receive(self, claim: Intention) -> bool:
        if (
            claim.session_id != self.session_id
            or claim.robot_id not in self.participants
        ):
            return False
        if claim.sequence <= self.heads.get(claim.robot_id, -1):
            return False
        self.heads[claim.robot_id] = claim.sequence
        if claim.active and claim.component_id == self.component_id:
            self.leases[claim.robot_id] = (claim, self.clock() + claim.lease_s)
        else:
            self.leases.pop(claim.robot_id, None)
        return True

    def propose(self, target, *, radius_m=2.0, cost=0.0, lease_s=3.0):
        if self.component_id is None:
            raise ValueError("No verified local coordinate frame")
        self.sequence += 1
        claim = Intention(
            self.robot_id,
            self.session_id,
            self.sequence,
            self.component_id,
            tuple(target),
            radius_m,
            cost,
            lease_s,
        )
        if (
            self.local is None
            or self.local.target != claim.target
            or self.leases.get(self.robot_id, (None, 0))[1] <= self.clock()
        ):
            self.proposed_at = self.clock()
        self.local = claim
        self.receive(claim)
        return claim

    def release(self):
        if self.local is None:
            return None
        self.sequence += 1
        old = self.local
        claim = Intention(
            self.robot_id,
            self.session_id,
            self.sequence,
            old.component_id,
            old.target,
            old.radius_m,
            old.cost,
            old.lease_s,
            False,
        )
        self.receive(claim)
        self.local = None
        return claim

    def decision_with_winner(self):
        """Return one atomic arbitration result and its winning robot, if any."""
        if self.local is None:
            return "unassigned", None
        now = self.clock()
        alive = {robot: pair for robot, pair in self.leases.items() if pair[1] > now}
        self.leases = alive
        if self.robot_id not in alive:
            return "expired", None
        if now - self.proposed_at < self.settle_s:
            return "pending", None
        contenders = [
            claim
            for claim, _ in alive.values()
            if claim.component_id == self.component_id
            and math.dist(claim.target, self.local.target)
            < claim.radius_m + self.local.radius_m
        ]
        winner = min(contenders, key=lambda claim: (claim.cost, claim.robot_id))
        return (
            ("granted", winner.robot_id)
            if winner.robot_id == self.robot_id
            else ("conflict", winner.robot_id)
        )

    def decision(self):
        return self.decision_with_winner()[0]


@dataclass(frozen=True)
class ExplorationReport:
    robot_id: str
    state: str
    active_assignments: int
    coverage_met: bool


class CompletionTracker:
    """A fresh empty local frontier set is evidence, not fleet completion."""

    def __init__(self, participants: set[str], *, clock=time.monotonic, max_age_s=5.0):
        self.participants, self.clock, self.max_age_s = (
            set(participants),
            clock,
            max_age_s,
        )
        self.reports = {}

    def receive(self, report: ExplorationReport):
        if report.robot_id not in self.participants:
            return False
        if report.state not in {
            "exploring",
            "waiting_for_map",
            "blocked",
            "locally_exhausted",
        }:
            raise ValueError("Invalid exploration report")
        if report.active_assignments < 0:
            raise ValueError("Invalid assignment count")
        self.reports[report.robot_id] = (report, self.clock())
        return True

    def state(self):
        now = self.clock()
        reports = [self.reports.get(robot) for robot in self.participants]
        if not reports or any(
            pair is None or now - pair[1] > self.max_age_s for pair in reports
        ):
            return "unknown"
        if all(
            pair[0].state == "locally_exhausted"
            and pair[0].active_assignments == 0
            and pair[0].coverage_met
            for pair in reports
        ):
            return "complete"
        return "incomplete"
