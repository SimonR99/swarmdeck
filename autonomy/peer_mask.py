"""Capture-time peer-body masking for normalized lidar captures.

A fleet that starts grouped scans its own neighbours. Those returns are real
at the instant they are captured, but they describe a robot that drives away,
so retaining them writes another robot's chassis into the keyframe, into the
Swarm-SLAM descriptor and into every MOLA product derived from it. Later
free-space rays may clear the occupancy eventually; nothing clears the stored
geometry. The only place the contamination can be removed for good is at
capture time, before the keyframe is retained.

This module is the numpy core of that filter and holds no ROS dependency, so
the geometry can be tested without a running graph.

Frames. Everything here works in ONE frame: the capturing robot's base frame
at the capture stamp, which is the frame the bridge has already rotated the
scan into using capture-time TF. A peer's box is supplied as the 4x4
``T_base_peer`` placing that peer's base frame in the capturing robot's base
frame AT THAT SAME STAMP. Composing a peer pose sampled at another time, or
the latest pose, rotates a moving body against stale geometry and masks the
wrong volume; the caller owns that timestamp join and this module assumes it
was done.

Evidence. Masked points are REMOVED endpoints, never endpoints converted into
free space. A dropped return contributes no occupied cell and no free ray, so
the volume a peer occupies becomes unobserved rather than empty. That is
deliberate: the peer really is there at the capture stamp, and carving a free
ray through it would be a second, worse falsehood. Removing endpoints does not
weaken the ``RayEvidence`` certificate, which describes return semantics,
deskewing and origin association for the capture as a whole; the retained
endpoints still come from the one capture and its single stored sensor origin.
"""

from __future__ import annotations

from dataclasses import dataclass
from threading import Lock

import numpy as np

# Sampled poses older than this are never a timestamp join, whatever tolerance
# a caller asks for. It bounds the damage from a misconfigured tolerance.
MAX_PEER_POSE_TOLERANCE_S = 1.0

# How far back a pose history keeps samples. The ARGoS lidar is 10 Hz while
# poses arrive per 100 Hz simulation tick, and a scan may carry a stamp up to
# one second behind the exchange that delivered it, so the join has to reach
# back further than a single scan period.
PEER_POSE_HISTORY_S = 4.0


@dataclass(frozen=True)
class PeerBody:
    """One platform's masking volume, as a box in its own base frame.

    The box is centred on base_link in x and y, spans the chassis rectangle,
    and runs vertically from the floor (``base_height`` below base_link) to
    ``top_height`` above it. ``top_height`` has to clear the tallest thing
    bolted to the robot rather than the deck alone: a mapping lidar on a mast
    is the part of a peer that another peer's rings see first.

    These are nominal platform dimensions, not a calibration.
    """

    length: float
    width: float
    base_height: float
    top_height: float

    def __post_init__(self) -> None:
        for name in ("length", "width", "base_height", "top_height"):
            value = float(getattr(self, name))
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"peer body {name} must be finite and positive")
            object.__setattr__(self, name, value)


# The simulated fleet's platforms, mirroring ROBOT_PROFILES in
# swarmdeck_ros/src/swarmdeck_sim/scenario/spawn_fleet.py. The peer container
# mounts autonomy/ and deploy/autonomy/ only, so it cannot import the
# simulation package; autonomy/tests/test_peer_mask.py imports both tables and
# fails if they ever drift apart.
#
# top_height is max(deck_top, lidar_z) per platform: on all three the mapping
# lidar stands above the deck.
PEER_BODY_PROFILES: dict[str, PeerBody] = {
    "bunker": PeerBody(length=1.023, width=0.778, base_height=0.200, top_height=0.520),
    "scout_mini": PeerBody(
        length=0.612, width=0.580, base_height=0.1225, top_height=0.330
    ),
    "spot": PeerBody(length=1.100, width=0.500, base_height=0.500, top_height=0.470),
}


def peer_body(platform: str) -> PeerBody:
    if platform not in PEER_BODY_PROFILES:
        raise ValueError(
            f"unknown peer platform {platform!r}; "
            f"available: {sorted(PEER_BODY_PROFILES)}"
        )
    return PEER_BODY_PROFILES[platform]


class PeerPoseHistory:
    """Bounded timestamped poses for one robot in a shared reference frame.

    Samples are kept so a capture can be joined to the pose that was true at
    its own stamp. Nothing here ever returns "the latest pose": a lookup that
    finds no sample close enough to the requested stamp returns None, and the
    caller is expected to leave that peer unmasked rather than guess.
    """

    def __init__(self, horizon_s: float = PEER_POSE_HISTORY_S) -> None:
        self._horizon_ns = int(float(horizon_s) * 1e9)
        self._samples: list[tuple[int, np.ndarray]] = []
        # The bridge currently adds poses and joins captures on the same
        # single-threaded sensor executor; the lock keeps the history correct
        # if either side ever moves to another thread.
        self._lock = Lock()

    def __len__(self) -> int:
        with self._lock:
            return len(self._samples)

    def add(self, stamp_ns: int, T_reference_base) -> None:
        matrix = np.asarray(T_reference_base, dtype=np.float64)
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise ValueError("peer pose must be a finite 4x4 transform")
        stamp = int(stamp_ns)
        matrix = matrix.copy()
        matrix.setflags(write=False)
        with self._lock:
            self._samples.append((stamp, matrix))
            # Samples usually arrive in order; a simulator restart or an out
            # of order delivery must not leave the list unsorted for the join.
            if len(self._samples) > 1 and stamp < self._samples[-2][0]:
                self._samples.sort(key=lambda sample: sample[0])
            newest = self._samples[-1][0]
            cutoff = newest - self._horizon_ns
            first = 0
            while first < len(self._samples) and self._samples[first][0] < cutoff:
                first += 1
            if first:
                del self._samples[:first]

    def sample_at(self, stamp_ns: int, tolerance_ns: int) -> np.ndarray | None:
        """The pose nearest ``stamp_ns``, or None when none is close enough."""

        with self._lock:
            samples = list(self._samples)
        if not samples:
            return None
        limit = min(int(tolerance_ns), int(MAX_PEER_POSE_TOLERANCE_S * 1e9))
        if limit < 0:
            return None
        target = int(stamp_ns)
        best: np.ndarray | None = None
        best_gap = limit + 1
        for sample_stamp, matrix in samples:
            gap = abs(sample_stamp - target)
            if gap <= limit and gap < best_gap:
                best, best_gap = matrix, gap
        return best


def points_inside_body(points_base, T_base_peer, body: PeerBody, margin_m: float):
    """Boolean mask of points falling inside one peer's inflated body box.

    ``points_base`` is (N, 3) in the capturing robot's base frame and
    ``T_base_peer`` places the peer's base frame in that same frame, both at
    the capture stamp.
    """

    points = np.asarray(points_base, dtype=np.float64).reshape(-1, 3)
    transform = np.asarray(T_base_peer, dtype=np.float64)
    if transform.shape != (4, 4):
        raise ValueError("T_base_peer must be a 4x4 transform")
    margin = float(margin_m)
    if not np.isfinite(margin) or margin < 0.0:
        raise ValueError("peer mask margin must be finite and nonnegative")
    if not len(points):
        return np.zeros(0, dtype=bool)
    # Into the peer's own frame, where the box is axis aligned.
    local = (points - transform[:3, 3]) @ transform[:3, :3]
    return (
        (np.abs(local[:, 0]) <= body.length / 2.0 + margin)
        & (np.abs(local[:, 1]) <= body.width / 2.0 + margin)
        & (local[:, 2] >= -(body.base_height + margin))
        & (local[:, 2] <= body.top_height + margin)
    )


def mask_peer_bodies(points_base, peers, margin_m: float):
    """Drop every point inside a peer body.

    ``peers`` is an ordered mapping of peer name to ``(T_base_peer, PeerBody)``
    at the capture stamp. Returns ``(kept_points, dropped_by_peer)``, where the
    per-peer counts are disjoint and therefore sum to the number of points
    removed, so a diagnostics total never double counts a point that two
    overlapping boxes both claim.
    """

    points = np.asarray(points_base, dtype=np.float64).reshape(-1, 3)
    keep = np.ones(len(points), dtype=bool)
    dropped: dict[str, int] = {}
    for name, (transform, body) in peers.items():
        if not keep.any():
            dropped[name] = 0
            continue
        inside = points_inside_body(points, transform, body, margin_m)
        claimed = inside & keep
        dropped[name] = int(np.count_nonzero(claimed))
        keep &= ~claimed
    return points[keep], dropped


class PeerBodyMask:
    """The whole capture-time peer mask: poses, the join, and its counters.

    The bridge owns one of these and does nothing but feed it timestamped
    poses and hand it each capture. Keeping the accounting here rather than in
    the node means the stale-pose policy can be exercised without a ROS
    runtime, and it is the policy rather than the box test that decides
    whether real geometry survives.
    """

    def __init__(self, robot, bodies, tolerance_ns, margin_m):
        self.robot = str(robot)
        self.bodies = dict(bodies)
        if self.robot not in self.bodies:
            raise ValueError("the capturing robot needs a body profile too")
        self.tolerance_ns = int(tolerance_ns)
        if self.tolerance_ns <= 0:
            raise ValueError("peer pose tolerance must be positive")
        self.margin_m = float(margin_m)
        if not np.isfinite(self.margin_m) or self.margin_m < 0.0:
            raise ValueError("peer mask margin must be finite and nonnegative")
        self._history = {name: PeerPoseHistory() for name in self.bodies}
        self._lock = Lock()
        self.points_dropped = 0
        self.dropped_by_peer: dict[str, int] = {}
        self.peers_skipped_stale = 0
        self.captures_without_self_pose = 0
        self.pose_rejections = 0

    def add_pose(self, robot, stamp_ns, T_reference_base) -> bool:
        """Record one sample. Returns False when it could not be used."""

        history = self._history.get(robot)
        if history is None:
            with self._lock:
                self.pose_rejections += 1
            return False
        try:
            history.add(stamp_ns, T_reference_base)
        except ValueError:
            with self._lock:
                self.pose_rejections += 1
            return False
        return True

    def reject_pose(self) -> None:
        """Count a sample the caller discarded, such as one in another frame."""

        with self._lock:
            self.pose_rejections += 1

    def apply(self, points_base, stamp_ns):
        """Drop returns inside another robot at this capture's stamp.

        `points_base` is already in the capturing robot's base frame at
        `stamp_ns`. A peer without a pose close enough to that stamp is left
        unmasked and counted: keeping a few of its returns is a smaller error
        than deleting real geometry at a pose we cannot vouch for.
        """

        own = self._history[self.robot].sample_at(stamp_ns, self.tolerance_ns)
        if own is None:
            # Without our own pose at this stamp no peer can be placed in this
            # capture's frame, so the whole capture goes through untouched.
            with self._lock:
                self.captures_without_self_pose += 1
                self.peers_skipped_stale += max(len(self.bodies) - 1, 0)
            return points_base
        peers, stale = {}, 0
        for name, body in self.bodies.items():
            if name == self.robot:
                continue
            sample = self._history[name].sample_at(stamp_ns, self.tolerance_ns)
            if sample is None:
                stale += 1
                continue
            peers[name] = (relative_transform(own, sample), body)
        kept, dropped = mask_peer_bodies(points_base, peers, self.margin_m)
        with self._lock:
            self.peers_skipped_stale += stale
            for name, count in dropped.items():
                if count:
                    self.dropped_by_peer[name] = (
                        self.dropped_by_peer.get(name, 0) + count
                    )
                    self.points_dropped += count
        return kept

    def counters(self) -> dict:
        """A snapshot of the diagnostics, safe to read from another thread."""

        with self._lock:
            return {
                "peer_body_mask_points_dropped": self.points_dropped,
                "peer_body_mask_points_dropped_by_peer": dict(
                    sorted(self.dropped_by_peer.items())
                ),
                "peer_body_mask_peers_skipped_stale": self.peers_skipped_stale,
                "peer_body_mask_captures_without_self_pose": (
                    self.captures_without_self_pose
                ),
                "peer_body_mask_pose_rejections": self.pose_rejections,
            }


def idle_mask_counters() -> dict:
    """The same counter shape a disabled mask reports, freshly built.

    Diagnostics keep one set of keys whether or not the mask runs, so a status
    file never leaves a reader guessing whether zero means off or means clean.
    """

    return {
        "peer_body_mask_points_dropped": 0,
        "peer_body_mask_points_dropped_by_peer": {},
        "peer_body_mask_peers_skipped_stale": 0,
        "peer_body_mask_captures_without_self_pose": 0,
        "peer_body_mask_pose_rejections": 0,
    }


def relative_transform(T_reference_base, T_reference_peer):
    """``T_base_peer`` from two poses sampled in one shared reference frame.

    Both arguments must be sampled at the same instant. Inverting the rigid
    transform directly keeps this exact for a rotation matrix rather than
    inheriting a general inverse's conditioning.
    """

    base = np.asarray(T_reference_base, dtype=np.float64)
    peer = np.asarray(T_reference_peer, dtype=np.float64)
    if base.shape != (4, 4) or peer.shape != (4, 4):
        raise ValueError("poses must be 4x4 transforms")
    rotation = base[:3, :3]
    inverse = np.eye(4)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ base[:3, 3]
    return inverse @ peer
