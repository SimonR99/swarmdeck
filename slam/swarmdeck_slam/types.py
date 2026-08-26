"""Core domain types for the collaborative pose graph.

This module is the contract every other module in the package builds against. It
imports numpy and nothing else -- deliberately. The solver is swappable behind
:class:`PoseGraphOptimizer`, and keeping its types free of ``gtsam`` means the
descriptor, verification, and rendering layers can be tested without a solver
installed at all.

Frame convention
----------------
**Every transform in this package is named ``T_a_b`` and maps points expressed in
frame ``b`` into frame ``a``.** So ``T_world_base @ p_base == p_world``, and
composition reads left-to-right with the inner frames cancelling::

    T_world_base = T_world_map @ T_map_odom @ T_odom_base

This matches ``small_gicp``'s ``result.T_target_source`` (verified: it maps source
points into the target frame). Sign and direction errors in relative transforms
are the single most common defect in pose-graph code and they fail *silently* --
the graph still optimizes, it just converges to a mirrored map. Name every local
variable in this form and the compiler in your head does the checking.

The frame chain
---------------
``world`` is the fleet frame. Each robot has its own ``map`` frame, which is where
its own SLAM solution lives, and ``T_world_map`` is what the optimizer estimates
for it. ``odom`` is the robot's continuous, non-jumping odometry frame; the
optimizer never touches it, which is what lets the controller keep running
through a loop closure that moves the whole trajectory.

What actually arrives, and why the odom name is a lie
------------------------------------------------------
``Keyframe.t_odom_base`` -- and the wire field of the same name in
``swarmdeck_protocol`` -- does **not** carry ``T_odom_base``. Both adapters look
up ``map_frame -> base_frame`` and send that (``adapter_ros2.pose7``, whose own
docstring says ``T_map_base``), so what the graph receives is the robot's own
SLAM-corrected pose in its own ``map`` frame.

The name is kept because changing it is a wire-protocol change across both
adapters, the protocol package, and every recorded capture in
``sessions/captures``; the correction is written down here instead. Two
consequences follow, and both are load-bearing:

* **That frame is not continuous.** A SLAM node moves its map frame every time
  it re-optimizes. Anything that samples the frame at one instant is sampling a
  moving target -- which is why ``GtsamPoseGraph._t_world_map`` fits
  ``T_world_map`` over a robot's whole trajectory rather than reading it off
  the newest keyframe, and why doing the latter published a 6.5 m error into
  the operator's fleet view until 2026-08-25.
* **It can change frames mid-session.** ``pose7`` falls back to an odometry
  pose when the TF lookup fails, so a single robot's keyframe stream can switch
  frames between one keyframe and the next. The relative transform across that
  switch is meaningless, and it becomes an ``ODOMETRY`` edge, which GNC is
  structurally forbidden from rejecting (see ``graph.py``). ``CollaborativeBackend``
  guards this by down-weighting implausibly large hops rather than trusting them.

Relative transforms between keyframes are unaffected by which of the two frames
is in use, as long as both endpoints agree -- which is why the pose graph works
at all despite the above, and why the guard targets the hop that *straddles* a
change rather than the frame choice itself.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing import Final, Iterable, Iterator, NamedTuple, Protocol

import numpy as np

# gtsam's Pose3 tangent space is ROTATION FIRST: [rx, ry, rz, tx, ty, tz].
# Information and covariance matrices in this package use that same ordering so
# they can be handed to the solver untouched. Getting this backwards produces a
# graph that optimizes happily and trusts rotation where it meant to trust
# translation, which looks like mysterious yaw drift rather than like a bug.
TANGENT_ORDER: Final = ("rx", "ry", "rz", "tx", "ty", "tz")

_ROBOT_INDEX_BITS: Final = 16
_SEQ_BITS: Final = 48
_MAX_ROBOTS: Final = 1 << _ROBOT_INDEX_BITS
_MAX_SEQ: Final = 1 << _SEQ_BITS


class KeyframeId(NamedTuple):
    """Globally unique keyframe identity: which robot, and its sequence number."""

    robot_id: str
    seq: int

    def __str__(self) -> str:  # pragma: no cover - trivial
        return f"{self.robot_id}#{self.seq}"


class KeyRegistry:
    """Bijection between :class:`KeyframeId` and the integer keys a solver wants.

    Solvers index variables by integer. Robot ids are strings, so something has
    to assign each one a stable small index; doing it in one place means the
    mapping cannot disagree between the module that adds a factor and the module
    that reads the result back out.

    Indices are assigned on first sight and never reused, so keys stay stable for
    the lifetime of a session even as robots join and drop.
    """

    def __init__(self) -> None:
        self._index: dict[str, int] = {}
        self._robots: list[str] = []

    def robot_index(self, robot_id: str) -> int:
        try:
            return self._index[robot_id]
        except KeyError:
            pass
        if len(self._robots) >= _MAX_ROBOTS:
            raise ValueError(f"cannot track more than {_MAX_ROBOTS} robots")
        index = len(self._robots)
        self._index[robot_id] = index
        self._robots.append(robot_id)
        return index

    def key(self, keyframe_id: KeyframeId) -> int:
        if not 0 <= keyframe_id.seq < _MAX_SEQ:
            raise ValueError(f"seq {keyframe_id.seq} out of range for {_SEQ_BITS} bits")
        return (self.robot_index(keyframe_id.robot_id) << _SEQ_BITS) | keyframe_id.seq

    def unkey(self, key: int) -> KeyframeId:
        index = key >> _SEQ_BITS
        if index >= len(self._robots):
            raise KeyError(f"key {key} refers to unknown robot index {index}")
        return KeyframeId(self._robots[index], key & (_MAX_SEQ - 1))

    @property
    def robots(self) -> tuple[str, ...]:
        return tuple(self._robots)


# --------------------------------------------------------------------------- #
# SE(3) helpers
#
# Poses are 4x4 float64 homogeneous matrices throughout. They compose with plain
# `@`, invert exactly, and print readably -- which matters more than the marginal
# speed of a quaternion representation at the scale this graph runs at.
# --------------------------------------------------------------------------- #

def se3_identity() -> np.ndarray:
    return np.eye(4, dtype=np.float64)


def se3_from_quat_xyz(pose7: Iterable[float]) -> np.ndarray:
    """Build ``T`` from ``[x, y, z, qx, qy, qz, qw]`` (ROS order, scalar last)."""
    values = np.asarray(list(pose7), dtype=np.float64)
    if values.shape != (7,):
        raise ValueError(f"expected 7 values [x,y,z,qx,qy,qz,qw], got {values.shape}")
    translation, quat = values[:3], values[3:]
    norm = float(np.linalg.norm(quat))
    if norm < 1e-9:
        raise ValueError("quaternion has zero norm")
    x, y, z, w = quat / norm

    matrix = se3_identity()
    matrix[:3, :3] = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    matrix[:3, 3] = translation
    return matrix


def quat_xyz_from_se3(matrix: np.ndarray) -> np.ndarray:
    """Inverse of :func:`se3_from_quat_xyz`. Returns ``[x,y,z,qx,qy,qz,qw]``.

    Uses the branch-on-largest-diagonal form rather than the naive trace formula,
    which loses precision and can take a square root of a negative near 180 deg.
    """
    rotation = np.asarray(matrix, dtype=np.float64)[:3, :3]
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / s
        x = 0.25 * s
        y = (rotation[0, 1] + rotation[1, 0]) / s
        z = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / s
        x = (rotation[0, 1] + rotation[1, 0]) / s
        y = 0.25 * s
        z = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / s
        x = (rotation[0, 2] + rotation[2, 0]) / s
        y = (rotation[1, 2] + rotation[2, 1]) / s
        z = 0.25 * s
    quat = np.array([x, y, z, w], dtype=np.float64)
    return np.concatenate([matrix[:3, 3], quat / np.linalg.norm(quat)])


def se3_inverse(matrix: np.ndarray) -> np.ndarray:
    """Exact inverse using the orthogonality of R -- never ``np.linalg.inv``."""
    rotation = matrix[:3, :3]
    out = se3_identity()
    out[:3, :3] = rotation.T
    out[:3, 3] = -rotation.T @ matrix[:3, 3]
    return out


def se3_relative(t_a_from: np.ndarray, t_a_to: np.ndarray) -> np.ndarray:
    """``T_from_to`` given two poses expressed in a common frame ``a``."""
    return se3_inverse(t_a_from) @ t_a_to


def transform_points(matrix: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Apply ``T_a_b`` to ``points`` given in frame ``b``; returns frame ``a``.

    Keeps float32 input in float32: keyframe clouds are float32 and promoting
    them to float64 for a render doubles peak memory for no accuracy that
    survives the occupancy quantization anyway.
    """
    pts = np.asarray(points)
    rotation = matrix[:3, :3].astype(pts.dtype, copy=False)
    translation = matrix[:3, 3].astype(pts.dtype, copy=False)
    return pts @ rotation.T + translation


def se3_kabsch(source_points: np.ndarray, target_points: np.ndarray) -> np.ndarray:
    """Best-fit rigid ``T_target_source`` between two corresponding point sets.

    Kabsch's algorithm via SVD of the cross-covariance: the closed-form
    least-squares rotation and translation carrying ``source_points`` onto
    ``target_points``. Scale is fixed at 1 and never fitted -- lidar SLAM is
    metric, so a scale error is a real error in the map and a fit that could
    absorb it would report a perfect frame for a map 5% too big.

    The reflection correction keeps ``det(R) == +1`` even when the SVD alone
    would hand back a mirror, which is what makes this exact (no NaN, no
    divide-by-zero) on rank-deficient input -- a robot that has only ever
    driven a straight corridor is collinear, and that is a real trajectory
    shape rather than a degenerate one. Note that a collinear fit leaves
    rotation about the travel axis unconstrained: the result is *a* best fit,
    not a unique one, so callers that care should check the residual.

    Raises ``ValueError`` for fewer than 2 points, where there is nothing to
    determine a rotation from.
    """
    source = np.asarray(source_points, dtype=np.float64)
    target = np.asarray(target_points, dtype=np.float64)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError(
            f"need two matching [n, 3] point sets, got {source.shape} and {target.shape}"
        )
    if source.shape[0] < 2:
        raise ValueError(f"rigid fit needs at least 2 points, got {source.shape[0]}")

    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    covariance = (source - source_mean).T @ (target - target_mean)
    u, _, vt = np.linalg.svd(covariance)
    det = float(np.linalg.det(vt.T @ u.T))
    correction = np.diag([1.0, 1.0, 1.0 if det >= 0.0 else -1.0])
    rotation = vt.T @ correction @ u.T

    out = se3_identity()
    out[:3, :3] = rotation
    out[:3, 3] = target_mean - rotation @ source_mean
    return out


def se3_distance(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    """``(translation_m, rotation_rad)`` between two poses. For gating and tests."""
    delta = se3_relative(a, b)
    translation = float(np.linalg.norm(delta[:3, 3]))
    cos_theta = np.clip((float(np.trace(delta[:3, :3])) - 1.0) / 2.0, -1.0, 1.0)
    return translation, float(np.arccos(cos_theta))


# --------------------------------------------------------------------------- #
# Graph elements
# --------------------------------------------------------------------------- #

@dataclass(slots=True)
class Keyframe:
    """One pose-graph node: a cloud, where odometry thought it was, and a descriptor."""

    id: KeyframeId
    stamp: float
    #: 4x4. Named for the odom frame, but production adapters send the robot's
    #: own SLAM ``T_map_base`` here -- see this module's docstring, which
    #: explains why the name stays and what depends on the difference.
    t_odom_base: np.ndarray
    points: np.ndarray  # float32 [n, 3] in the base frame at capture
    descriptor: np.ndarray | None = None  # uint8 [rings, sectors]
    descriptor_kind: str = ""

    def __post_init__(self) -> None:
        if self.t_odom_base.shape != (4, 4):
            raise ValueError(f"t_odom_base must be 4x4, got {self.t_odom_base.shape}")
        if self.points.ndim != 2 or self.points.shape[1] != 3:
            raise ValueError(f"points must be [n, 3], got {self.points.shape}")


class EdgeKind(enum.Enum):
    """Why an edge exists. Drives which edges a robust kernel is allowed to reject.

    Odometry and anchor edges are structural -- rejecting them disconnects the
    graph. Only loop closures are treated as outlier candidates, because only
    loop closures can be wrong in the way robust estimation exists to handle: a
    confident match to the wrong place.
    """

    ODOMETRY = "odometry"
    INTRA_LOOP = "intra_loop"
    INTER_LOOP = "inter_loop"
    ANCHOR_PRIOR = "anchor_prior"

    @property
    def is_loop_closure(self) -> bool:
        return self in (EdgeKind.INTRA_LOOP, EdgeKind.INTER_LOOP)


@dataclass(slots=True)
class Edge:
    """A relative-pose constraint ``T_src_dst`` with its information matrix."""

    kind: EdgeKind
    src: KeyframeId
    dst: KeyframeId
    t_src_dst: np.ndarray  # 4x4
    information: np.ndarray  # 6x6, TANGENT_ORDER
    fitness: float = 0.0  # registration quality in [0, 1]; 0 for structural edges
    inlier_ratio: float = 0.0

    def __post_init__(self) -> None:
        if self.t_src_dst.shape != (4, 4):
            raise ValueError(f"t_src_dst must be 4x4, got {self.t_src_dst.shape}")
        if self.information.shape != (6, 6):
            raise ValueError(f"information must be 6x6, got {self.information.shape}")
        if self.src == self.dst:
            raise ValueError(f"edge cannot connect {self.src} to itself")

    @property
    def is_inter_robot(self) -> bool:
        return self.src.robot_id != self.dst.robot_id


@dataclass(frozen=True, slots=True)
class Component:
    """A set of robots joined by verified inter-robot closures.

    Robots in different components have **no known relative transform**. They are
    never placed in a common frame on assumption -- an unmerged map is a correct
    statement of ignorance, whereas an overlaid one is a confident lie that the
    operator cannot see through.
    """

    component_id: int
    robots: frozenset[str]
    anchor: KeyframeId

    def __contains__(self, robot_id: object) -> bool:
        return robot_id in self.robots


@dataclass(slots=True)
class OptimizedGraph:
    """The solver's output: where every keyframe actually is."""

    poses: dict[KeyframeId, np.ndarray] = field(default_factory=dict)
    t_world_map: dict[str, np.ndarray] = field(default_factory=dict)
    components: list[Component] = field(default_factory=list)
    rejected_edges: list[Edge] = field(default_factory=list)
    iterations: int = 0
    final_error: float = 0.0

    def component_of(self, robot_id: str) -> Component | None:
        return next((c for c in self.components if robot_id in c.robots), None)

    def share_frame(self, a: str, b: str) -> bool:
        """Whether two robots have a verified relative transform.

        The gate for using another robot's map. False means plan around unknown
        space, not around an assumed identity transform.
        """
        component = self.component_of(a)
        return component is not None and b in component.robots

    def keyframes_of(self, robot_id: str) -> Iterator[tuple[KeyframeId, np.ndarray]]:
        for keyframe_id, pose in self.poses.items():
            if keyframe_id.robot_id == robot_id:
                yield keyframe_id, pose


class PoseGraphOptimizer(Protocol):
    """What the rest of the package needs from a solver.

    Narrow on purpose: the solver owns optimization and nothing else. It does not
    decide which candidates to verify, does not hold clouds, and does not render.
    That keeps the gtsam dependency -- the one component pinned to Python 3.12 --
    behind an interface small enough to reimplement if that pin ever becomes a
    problem.
    """

    def add_keyframe(self, keyframe: Keyframe) -> None: ...

    def add_edge(self, edge: Edge) -> None: ...

    def optimize(self) -> OptimizedGraph: ...
