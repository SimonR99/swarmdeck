"""Operator review of map detections: propose, accept, ignore, merge.

The detector proposes; the operator disposes. Raw detections keep flowing into
`_detections` as live camera tracks, and this module is the separate question of
what has earned a permanent place on the shared map.

Three things make it more than a queue of yes/no prompts:

**An object already on the map does not ask again.** A sighting landing within
`same_radius` of a confirmed entity of its class is folded in silently. That is
the common case by a wide margin — a robot parked in front of a duck produces a
sighting every frame — and prompting for each would make the queue useless.

**Uncertainty is what gets escalated.** A sighting between `same_radius` and
`ask_radius` is the genuinely ambiguous one: probably the object we know about,
possibly its neighbour. Those arrive as a proposal with the merge already
suggested, so the operator answers "same or different" rather than re-deriving
it from coordinates.

**Position is the mean of accepted evidence, not the latest frame.** A single
monocular range estimate is worth little; twenty of them from two robots at
different angles put the marker where the object is. Running sums keep this O(1)
and mean the centroid covers every accepted observation, not a recent window.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Literal

# Defaults, overridable from operator settings. Chosen against the object sizes
# this fleet looks for: a rubber duck is ~0.3 m across, so two sightings 0.5 m
# apart are the same duck seen from two angles, and 1.5 m apart are two ducks.
DEFAULT_SAME_RADIUS_M = 0.5
DEFAULT_ASK_RADIUS_M = 1.5
DEFAULT_IGNORE_RADIUS_M = 1.0

# Pending proposals are capped so a mislabelling detector cannot bury the
# operator. The weakest are dropped, never the strongest: a flood of junk must
# not push out the one real finding in it.
MAX_PENDING = 32

# Samples retained per object for display only. The centroid uses running sums,
# so trimming this changes what you can inspect, never where the marker sits.
MAX_SAMPLES = 24

Outcome = Literal["folded", "proposed", "updated", "suppressed"]


@dataclass
class Sample:
    robot_id: str
    x: float
    y: float
    score: float
    t: float

    def as_dict(self) -> dict[str, Any]:
        return {
            "robot_id": self.robot_id,
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "score": round(self.score, 4),
            "t": self.t,
        }


@dataclass
class Accumulator:
    """Running centroid over every accepted observation."""

    sum_x: float = 0.0
    sum_y: float = 0.0
    count: int = 0
    best_score: float = 0.0
    robots: set[str] = field(default_factory=set)
    samples: list[Sample] = field(default_factory=list)
    first_seen: float = 0.0
    last_seen: float = 0.0

    def add(self, sample: Sample) -> None:
        if not self.count:
            self.first_seen = sample.t
        self.sum_x += sample.x
        self.sum_y += sample.y
        self.count += 1
        self.best_score = max(self.best_score, sample.score)
        self.robots.add(sample.robot_id)
        self.samples.append(sample)
        if len(self.samples) > MAX_SAMPLES:
            del self.samples[0]
        self.last_seen = sample.t

    def absorb(self, other: "Accumulator") -> None:
        self.sum_x += other.sum_x
        self.sum_y += other.sum_y
        self.count += other.count
        self.best_score = max(self.best_score, other.best_score)
        self.robots |= other.robots
        self.samples = (self.samples + other.samples)[-MAX_SAMPLES:]
        self.first_seen = min(self.first_seen or other.first_seen, other.first_seen)
        self.last_seen = max(self.last_seen, other.last_seen)

    @property
    def x(self) -> float:
        return self.sum_x / self.count if self.count else 0.0

    @property
    def y(self) -> float:
        return self.sum_y / self.count if self.count else 0.0


@dataclass
class Entity:
    """An object the operator has confirmed onto the map."""

    id: str
    cls: str
    acc: Accumulator

    def as_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "class": self.cls,
            "position": {"x": round(self.acc.x, 3), "y": round(self.acc.y, 3)},
            "observations": self.acc.count,
            "best_score": round(self.acc.best_score, 4),
            "robot_ids": sorted(self.acc.robots),
            "first_seen": self.acc.first_seen,
            "last_seen": self.acc.last_seen,
            "samples": [s.as_dict() for s in self.acc.samples],
        }


@dataclass
class Proposal:
    """A sighting awaiting an operator decision."""

    id: str
    cls: str
    acc: Accumulator
    suggested_entity_id: str | None = None
    suggested_distance: float | None = None

    def as_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "class": self.cls,
            "position": {"x": round(self.acc.x, 3), "y": round(self.acc.y, 3)},
            "observations": self.acc.count,
            "best_score": round(self.acc.best_score, 4),
            "robot_ids": sorted(self.acc.robots),
            "first_seen": self.acc.first_seen,
            "last_seen": self.acc.last_seen,
            "suggested_entity_id": self.suggested_entity_id,
            "suggested_distance": (
                round(self.suggested_distance, 3)
                if self.suggested_distance is not None
                else None
            ),
        }


@dataclass
class IgnoreZone:
    cls: str
    x: float
    y: float
    radius: float

    def contains(self, cls: str, x: float, y: float) -> bool:
        return cls == self.cls and math.hypot(x - self.x, y - self.y) <= self.radius


class ReviewStore:
    def __init__(
        self,
        same_radius: float = DEFAULT_SAME_RADIUS_M,
        ask_radius: float = DEFAULT_ASK_RADIUS_M,
        ignore_radius: float = DEFAULT_IGNORE_RADIUS_M,
    ) -> None:
        self.same_radius = same_radius
        self.ask_radius = ask_radius
        self.ignore_radius = ignore_radius
        self.entities: dict[str, Entity] = {}
        self.proposals: dict[str, Proposal] = {}
        self.ignored: list[IgnoreZone] = []
        self._next_id = 1

    # ------------------------------------------------------------- internals

    def _mint(self, prefix: str) -> str:
        value = f"{prefix}_{self._next_id}"
        self._next_id += 1
        return value

    def _nearest_entity(self, cls: str, x: float, y: float) -> tuple[Entity | None, float]:
        best: Entity | None = None
        best_d = math.inf
        for entity in self.entities.values():
            if entity.cls != cls:
                continue
            d = math.hypot(entity.acc.x - x, entity.acc.y - y)
            if d < best_d:
                best, best_d = entity, d
        return best, best_d

    def _nearest_proposal(self, cls: str, x: float, y: float) -> tuple[Proposal | None, float]:
        best: Proposal | None = None
        best_d = math.inf
        for proposal in self.proposals.values():
            if proposal.cls != cls:
                continue
            d = math.hypot(proposal.acc.x - x, proposal.acc.y - y)
            if d < best_d:
                best, best_d = proposal, d
        return best, best_d

    def _trim_pending(self) -> None:
        if len(self.proposals) <= MAX_PENDING:
            return
        # Weakest evidence first, oldest breaking the tie.
        ordered = sorted(
            self.proposals.values(),
            key=lambda p: (p.acc.best_score, p.acc.count, -p.acc.last_seen),
        )
        for proposal in ordered[: len(self.proposals) - MAX_PENDING]:
            self.proposals.pop(proposal.id, None)

    # ---------------------------------------------------------------- ingest

    def observe(
        self,
        robot_id: str,
        cls: str,
        x: float,
        y: float,
        score: float,
        now: float | None = None,
    ) -> tuple[Outcome, Entity | Proposal | None]:
        """Route one located sighting to the map, the queue, or the bin."""
        now = time.time() if now is None else now
        sample = Sample(robot_id, float(x), float(y), float(score), now)

        for zone in self.ignored:
            if zone.contains(cls, x, y):
                return "suppressed", None

        entity, distance = self._nearest_entity(cls, x, y)
        if entity is not None and distance <= self.same_radius:
            entity.acc.add(sample)
            return "folded", entity

        proposal, pdist = self._nearest_proposal(cls, x, y)
        if proposal is not None and pdist <= self.same_radius:
            # Repeat sightings strengthen one pending item rather than filling
            # the queue with near-duplicates of the same question.
            proposal.acc.add(sample)
            if entity is not None and distance <= self.ask_radius:
                proposal.suggested_entity_id = entity.id
                proposal.suggested_distance = distance
            return "updated", proposal

        fresh = Proposal(id=self._mint("prop"), cls=cls, acc=Accumulator())
        fresh.acc.add(sample)
        if entity is not None and distance <= self.ask_radius:
            fresh.suggested_entity_id = entity.id
            fresh.suggested_distance = distance
        self.proposals[fresh.id] = fresh
        self._trim_pending()
        return "proposed", fresh

    # -------------------------------------------------------------- decisions

    def accept(self, proposal_id: str) -> Entity | None:
        proposal = self.proposals.pop(proposal_id, None)
        if proposal is None:
            return None
        entity = Entity(id=self._mint("ent"), cls=proposal.cls, acc=proposal.acc)
        self.entities[entity.id] = entity
        return entity

    def merge(self, proposal_id: str, entity_id: str) -> Entity | None:
        proposal = self.proposals.get(proposal_id)
        entity = self.entities.get(entity_id)
        if proposal is None or entity is None or proposal.cls != entity.cls:
            return None
        self.proposals.pop(proposal_id, None)
        entity.acc.absorb(proposal.acc)
        return entity

    def ignore(self, proposal_id: str) -> bool:
        """Drop it and stop asking about that spot.

        The zone is what makes ignore stick. Without it the next frame proposes
        the same object again, and "ignore" degrades into "dismiss", which the
        operator has to keep doing forever.
        """
        proposal = self.proposals.pop(proposal_id, None)
        if proposal is None:
            return False
        self.ignored.append(
            IgnoreZone(proposal.cls, proposal.acc.x, proposal.acc.y, self.ignore_radius)
        )
        return True

    def forget(self, entity_id: str) -> bool:
        """Remove a confirmed entity, e.g. one accepted by mistake."""
        return self.entities.pop(entity_id, None) is not None

    def clear_ignored(self) -> int:
        count = len(self.ignored)
        self.ignored.clear()
        return count

    def reset(self) -> None:
        self.entities.clear()
        self.proposals.clear()
        self.ignored.clear()

    def drop_classes(self, allowed: set[str] | None) -> None:
        """Forget everything whose class the operator has switched off."""
        if allowed is None:
            return
        for store in (self.entities, self.proposals):
            for key in [k for k, v in store.items() if v.cls not in allowed]:
                store.pop(key, None)
        self.ignored = [z for z in self.ignored if z.cls in allowed]

    # --------------------------------------------------------------- reading

    def snapshot(self) -> dict[str, Any]:
        return {
            "entities": [e.as_dict() for e in self.entities.values()],
            "proposals": [p.as_dict() for p in self.proposals.values()],
            "ignored": len(self.ignored),
            "radii": {
                "same": self.same_radius,
                "ask": self.ask_radius,
                "ignore": self.ignore_radius,
            },
        }
