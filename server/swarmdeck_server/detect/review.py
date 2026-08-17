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

**But only a moved viewpoint counts as new evidence.** Frames are not
independent measurements. A robot parked in front of a duck emits one every
frame, all carrying that pose's depth bias, and folding each in equally let a
stationary robot own the average: measured on a live run, one robot idling
overnight contributed 176k samples and dragged a marker from 0.17 m of error out
to 0.39 m. Averaging is supposed to buy accuracy, so a sighting only moves the
centroid when the observer has actually changed where it is looking from.
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

# How far an observer must move before its next sighting of the same object
# counts as a fresh viewpoint. Below this it is the same measurement again and
# only refreshes `last_seen` and `best_score`.
MIN_VIEWPOINT_MOVE_M = 0.25

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
    #: Every frame this object was seen in, including ones too close to the
    #: previous viewpoint to move the centroid. Reported, never averaged.
    sightings: int = 0
    best_score: float = 0.0
    robots: set[str] = field(default_factory=set)
    samples: list[Sample] = field(default_factory=list)
    #: Where each robot was standing when it last moved this centroid.
    viewpoints: dict[str, tuple[float, float]] = field(default_factory=dict)
    first_seen: float = 0.0
    last_seen: float = 0.0

    def add(self, sample: Sample, observer: tuple[float, float] | None = None) -> bool:
        """Record a sighting. Returns whether it moved the centroid."""
        if not self.sightings:
            self.first_seen = sample.t
        self.sightings += 1
        self.best_score = max(self.best_score, sample.score)
        self.robots.add(sample.robot_id)
        self.last_seen = sample.t

        if observer is not None:
            previous = self.viewpoints.get(sample.robot_id)
            if previous is not None and math.hypot(
                observer[0] - previous[0], observer[1] - previous[1]
            ) < MIN_VIEWPOINT_MOVE_M:
                return False
            self.viewpoints[sample.robot_id] = observer

        self.sum_x += sample.x
        self.sum_y += sample.y
        self.count += 1
        self.samples.append(sample)
        if len(self.samples) > MAX_SAMPLES:
            del self.samples[0]
        return True

    def absorb(self, other: "Accumulator") -> None:
        self.sum_x += other.sum_x
        self.sum_y += other.sum_y
        self.count += other.count
        self.sightings += other.sightings
        self.best_score = max(self.best_score, other.best_score)
        self.robots |= other.robots
        self.samples = (self.samples + other.samples)[-MAX_SAMPLES:]
        self.viewpoints.update(other.viewpoints)
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
            # Viewpoints that moved the centroid, and every frame it appeared
            # in. The first is the number that means anything.
            "observations": self.acc.count,
            "sightings": self.acc.sightings,
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
            "sightings": self.acc.sightings,
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


def _acc_to_dict(acc: Accumulator) -> dict[str, Any]:
    return {
        "sum_x": acc.sum_x,
        "sum_y": acc.sum_y,
        "count": acc.count,
        "sightings": acc.sightings,
        "best_score": acc.best_score,
        "robots": sorted(acc.robots),
        "viewpoints": {k: [v[0], v[1]] for k, v in acc.viewpoints.items()},
        "samples": [s.as_dict() for s in acc.samples],
        "first_seen": acc.first_seen,
        "last_seen": acc.last_seen,
    }


def _acc_from_dict(raw: dict[str, Any]) -> Accumulator:
    acc = Accumulator()
    try:
        acc.sum_x = float(raw.get("sum_x", 0.0))
        acc.sum_y = float(raw.get("sum_y", 0.0))
        acc.count = int(raw.get("count", 0))
        acc.sightings = int(raw.get("sightings", acc.count))
        acc.best_score = float(raw.get("best_score", 0.0))
        acc.first_seen = float(raw.get("first_seen", 0.0))
        acc.last_seen = float(raw.get("last_seen", 0.0))
    except (TypeError, ValueError):
        return Accumulator()
    acc.robots = {str(r) for r in (raw.get("robots") or [])}
    for robot, point in (raw.get("viewpoints") or {}).items():
        try:
            acc.viewpoints[str(robot)] = (float(point[0]), float(point[1]))
        except (TypeError, ValueError, IndexError):
            continue
    for item in (raw.get("samples") or [])[-MAX_SAMPLES:]:
        try:
            acc.samples.append(Sample(
                str(item["robot_id"]), float(item["x"]), float(item["y"]),
                float(item.get("score", 0.0)), float(item.get("t", 0.0)),
            ))
        except (KeyError, TypeError, ValueError):
            continue
    # A record claiming evidence it cannot show is not usable: the centroid
    # divides by `count`, so a zero count with non-zero sums would read as the
    # origin. Drop back to empty and let it be re-observed.
    if acc.count <= 0:
        return Accumulator()
    return acc


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
        observer: tuple[float, float] | None = None,
    ) -> tuple[Outcome, Entity | Proposal | None]:
        """Route one located sighting to the map, the queue, or the bin.

        `observer` is where the reporting robot was standing. Passing it gates
        the centroid on viewpoint change; omitting it averages every frame,
        which is what let a parked robot own an object's position.
        """
        now = time.time() if now is None else now
        sample = Sample(robot_id, float(x), float(y), float(score), now)

        for zone in self.ignored:
            if zone.contains(cls, x, y):
                return "suppressed", None

        entity, distance = self._nearest_entity(cls, x, y)
        if entity is not None and distance <= self.same_radius:
            entity.acc.add(sample, observer)
            return "folded", entity

        proposal, pdist = self._nearest_proposal(cls, x, y)
        if proposal is not None and pdist <= self.same_radius:
            # Repeat sightings strengthen one pending item rather than filling
            # the queue with near-duplicates of the same question.
            proposal.acc.add(sample, observer)
            if entity is not None and distance <= self.ask_radius:
                proposal.suggested_entity_id = entity.id
                proposal.suggested_distance = distance
            return "updated", proposal

        fresh = Proposal(id=self._mint("prop"), cls=cls, acc=Accumulator())
        fresh.acc.add(sample, observer)
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
        """Remove a confirmed entity, e.g. one accepted by mistake.

        Deliberately NOT the same as ignoring it. A robot still looking at the
        object will propose it again, which is correct: deleting says "that is
        not on my map", not "never mention this again". `ignore` is the second
        one and writes a suppression zone.
        """
        return self.entities.pop(entity_id, None) is not None

    def forget_all(self) -> int:
        count = len(self.entities)
        self.entities.clear()
        return count

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

    # ----------------------------------------------------------- persistence

    def to_dict(self) -> dict[str, Any]:
        """Everything needed to resume, including the centroid's running sums.

        The sums matter more than they look: reloading only the averaged
        position would restart every object at weight one, so the first sighting
        after a restart would yank a well-measured marker halfway to itself.
        `viewpoints` is persisted for the same reason — without it a robot that
        has not moved would be treated as a fresh vantage point.
        """
        return {
            "version": 1,
            "next_id": self._next_id,
            "entities": [
                {"id": e.id, "class": e.cls, **_acc_to_dict(e.acc)}
                for e in self.entities.values()
            ],
            "proposals": [
                {
                    "id": p.id,
                    "class": p.cls,
                    "suggested_entity_id": p.suggested_entity_id,
                    "suggested_distance": p.suggested_distance,
                    **_acc_to_dict(p.acc),
                }
                for p in self.proposals.values()
            ],
            "ignored": [
                {"class": z.cls, "x": z.x, "y": z.y, "radius": z.radius}
                for z in self.ignored
            ],
        }

    def load_dict(self, raw: Any) -> None:
        """Replace state from `to_dict` output. Malformed input loses nothing
        that was not already lost -- an unreadable file must not stop the
        backend coming up."""
        if not isinstance(raw, dict):
            return
        self.reset()
        # A record with no surviving evidence is dropped rather than restored:
        # the centroid divides by `count`, so a zero-count entity would come
        # back as a phantom object sitting at the origin of the map.
        for item in raw.get("entities") or []:
            if not isinstance(item, dict) or not item.get("id"):
                continue
            acc = _acc_from_dict(item)
            if not acc.count:
                continue
            self.entities[str(item["id"])] = Entity(
                id=str(item["id"]),
                cls=str(item.get("class", "object")),
                acc=acc,
            )
        for item in raw.get("proposals") or []:
            if not isinstance(item, dict) or not item.get("id"):
                continue
            acc = _acc_from_dict(item)
            if not acc.count:
                continue
            distance = item.get("suggested_distance")
            self.proposals[str(item["id"])] = Proposal(
                id=str(item["id"]),
                cls=str(item.get("class", "object")),
                acc=acc,
                suggested_entity_id=item.get("suggested_entity_id"),
                suggested_distance=float(distance) if distance is not None else None,
            )
        for item in raw.get("ignored") or []:
            if not isinstance(item, dict):
                continue
            try:
                self.ignored.append(IgnoreZone(
                    str(item.get("class", "object")),
                    float(item["x"]), float(item["y"]),
                    float(item.get("radius", self.ignore_radius)),
                ))
            except (KeyError, TypeError, ValueError):
                continue
        # Never reuse an id: a stale dashboard holding an old proposal id must
        # not be able to answer a different object.
        highest = 0
        for key in list(self.entities) + list(self.proposals):
            tail = key.rsplit("_", 1)[-1]
            if tail.isdigit():
                highest = max(highest, int(tail))
        try:
            self._next_id = max(int(raw.get("next_id", 1)), highest + 1)
        except (TypeError, ValueError):
            self._next_id = highest + 1

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
