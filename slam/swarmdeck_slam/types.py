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

* **It does not survive a reboot.** A restarted SLAM node starts a brand new
  map frame at wherever the robot happens to be standing, with no relation to
  the one before it, and the producer's ``seq`` restarts at zero alongside it.
  This is not a degraded measurement to down-weight, it is two different
  quantities sharing a name, so it is handled by identity rather than by a
  gate: see :class:`TrajectoryId`. Everything that composes one pose against
  another keys on the trajectory; everything that names a physical machine
  keys on the robot.
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


class TrajectoryId(NamedTuple):
    """One CONTINUOUS trajectory: which robot, and which of its runs.

    ``robot_id`` used to do two jobs at once, and a robot reboot splits them
    apart. As *identity* -- which physical machine this is -- it survives a
    reboot, and everything that answers to the operator (``Edge.is_inter_robot``,
    the ``robot:<id>`` map scope, who is on the fleet view) must keep using it.
    As *continuity* -- which unbroken stretch of driving this keyframe belongs
    to -- it does not survive a reboot at all, and everything that composes one
    pose against another must use this type instead:

    * the odometry chain, which fabricates an edge between consecutive
      keyframes. Across a reboot the two endpoints are in different map frames
      and the "measurement" between them is of nothing;
    * union-find components and anchor choice, since two segments of one robot
      start with **no known relative transform** and saying otherwise is the
      confident lie this package exists to refuse;
    * PCM grouping, whose consistency check bridges two of a robot's keyframes
      through that robot's own odometry -- meaningless across a discontinuity.

    ``session`` is a boot id the producer mints once at startup (see
    ``adapters/keyframe_producer.mint_session``). The empty string means "this
    packet predates the field", which is one trajectory per robot -- the
    behaviour of every recorded capture in ``sessions/captures``, preserved
    exactly.
    """

    robot_id: str
    session: str = ""

    def __str__(self) -> str:
        return self.robot_id if not self.session else f"{self.robot_id}@{self.session}"

    @classmethod
    def parse(cls, text: str) -> "TrajectoryId":
        """Inverse of :meth:`__str__`, for HTTP arguments and CLI flags.

        Splits on the FIRST ``@``: robot ids never contain one, and the
        protocol restricts session characters to ``[A-Za-z0-9._-]``, so the
        round trip is exact.
        """
        robot_id, _, session = text.partition("@")
        if not robot_id:
            raise ValueError(f"trajectory id {text!r} has no robot id")
        return cls(robot_id, session)


class KeyframeId(NamedTuple):
    """Globally unique keyframe identity: which trajectory, and its sequence number.

    ``session`` is last and defaults to ``""`` so ``KeyframeId("alpha", 3)``
    keeps working unchanged -- both for the tests that write it and for every
    packet decoded from a capture recorded before sessions existed, which is
    what makes those two compare equal instead of splitting into two nodes.
    """

    robot_id: str
    seq: int
    session: str = ""

    @property
    def trajectory(self) -> TrajectoryId:
        return TrajectoryId(self.robot_id, self.session)

    def __str__(self) -> str:  # pragma: no cover - trivial
        return f"{self.trajectory}#{self.seq}"


class KeyRegistry:
    """Bijection between :class:`KeyframeId` and the integer keys a solver wants.

    Solvers index variables by integer. Trajectory ids are pairs of strings, so
    something has to assign each one a stable small index; doing it in one place
    means the mapping cannot disagree between the module that adds a factor and
    the module that reads the result back out.

    The high bits index the TRAJECTORY, not the robot. That is what makes
    ``(robot_id, session, seq)`` collide-proof: a rebooted robot re-emits
    ``seq`` 0 for a different stretch of driving, and keying on the robot alone
    hands the solver one variable for two different poses.

    Indices are assigned on first sight and never reused, so keys stay stable for
    the lifetime of a run even as robots join, drop, and rejoin.
    """

    def __init__(self) -> None:
        self._index: dict[TrajectoryId, int] = {}
        self._trajectories: list[TrajectoryId] = []

    def trajectory_index(self, trajectory: TrajectoryId) -> int:
        try:
            return self._index[trajectory]
        except KeyError:
            pass
        if len(self._trajectories) >= _MAX_ROBOTS:
            raise ValueError(f"cannot track more than {_MAX_ROBOTS} trajectories")
        index = len(self._trajectories)
        self._index[trajectory] = index
        self._trajectories.append(trajectory)
        return index

    def key(self, keyframe_id: KeyframeId) -> int:
        if not 0 <= keyframe_id.seq < _MAX_SEQ:
            raise ValueError(f"seq {keyframe_id.seq} out of range for {_SEQ_BITS} bits")
        return (
            self.trajectory_index(keyframe_id.trajectory) << _SEQ_BITS
        ) | keyframe_id.seq

    def unkey(self, key: int) -> KeyframeId:
        index = key >> _SEQ_BITS
        if index >= len(self._trajectories):
            raise KeyError(f"key {key} refers to unknown trajectory index {index}")
        trajectory = self._trajectories[index]
        return KeyframeId(trajectory.robot_id, key & (_MAX_SEQ - 1), trajectory.session)

    @property
    def trajectories(self) -> tuple[TrajectoryId, ...]:
        return tuple(self._trajectories)

    @property
    def robots(self) -> tuple[str, ...]:
        """Distinct robots seen, in first-sight order. A robot that has restarted
        appears once here and once per session in :attr:`trajectories`."""
        seen: dict[str, None] = {}
        for trajectory in self._trajectories:
            seen.setdefault(trajectory.robot_id, None)
        return tuple(seen)


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


def se3_medoid(
    candidates: Iterable[np.ndarray],
    *,
    translation_scale_m: float = 1.0,
    rotation_scale_rad: float = 1.0,
) -> np.ndarray:
    """Return the observed rigid transform most central to all candidates.

    Unlike averaging matrices, a medoid is always one physically valid input
    transform. The scales make metres and radians comparable and let a
    majority of consistent surveyed starts reject one bad survey without
    deforming any trajectory.
    """
    values = list(candidates)
    if not values:
        raise ValueError("at least one SE(3) candidate is required")
    if translation_scale_m <= 0.0 or rotation_scale_rad <= 0.0:
        raise ValueError("SE(3) medoid scales must be positive")
    return min(
        values,
        key=lambda candidate: sum(
            translation / translation_scale_m + rotation / rotation_scale_rad
            for translation, rotation in (
                se3_distance(candidate, other) for other in values
            )
        ),
    )


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
    #: Optional physical height calibration carried by current producers.
    #: ``ground_z`` is the floor plane in the base frame at capture; the two
    #: limits are metres above that plane. Older captures leave these unset.
    ground_z: float | None = None
    min_height: float | None = None
    max_height: float | None = None
    lidar_height: float | None = None

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

    #: ``INTRA``/``INTER`` here mean *robot*, not *trajectory*, and are a
    #: reporting label: they answer "was this collaboration?". Which closures
    #: PCM has to cross-check is a different question, answered by
    #: :attr:`Edge.is_inter_trajectory` -- a robot rejoining its own pre-reboot
    #: segment is ``INTRA_LOOP`` (it is not collaboration) and still goes
    #: through PCM (there is no known transform to merge on).

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
        """Whether this closure is COLLABORATION -- two different machines.

        Deliberately keyed on ``robot_id`` and not on the trajectory: a robot
        closing a loop against its own pre-reboot segment is a real and
        valuable closure, but it is not two robots meeting, and counting it as
        one would inflate ``inter_robot_closures`` on the operator's fleet view
        every time a robot rebooted mid-run.
        """
        return self.src.robot_id != self.dst.robot_id

    @property
    def is_inter_trajectory(self) -> bool:
        """Whether this closure spans two trajectories with no shared frame.

        The question PCM asks. True for every inter-robot closure, and *also*
        for a same-robot closure across a restart -- which is the point: the
        robot's two segments start in unrelated map frames, so re-merging them
        needs the same corroboration as merging two strangers, and nothing in
        the pre-session design provided it.
        """
        return self.src.trajectory != self.dst.trajectory


@dataclass(frozen=True, slots=True)
class Component:
    """A set of TRAJECTORIES joined by verified closures between them.

    Trajectories in different components have **no known relative transform**.
    They are never placed in a common frame on assumption -- an unmerged map is
    a correct statement of ignorance, whereas an overlaid one is a confident lie
    that the operator cannot see through.

    The membership that matters is per trajectory, not per robot: a robot's
    pre- and post-reboot segments are two independent frames, and until enough
    corroborated closures tie them together they belong in two components. Once
    they do merge, the same robot appears in one component through both.

    :attr:`robots` is the projection onto physical machines, kept because that
    is what every consumer downstream of the solver actually asks for -- who is
    on the merged map, which ``robot:<id>`` grids to publish, which peers to
    list. It is derived, and it is lossy on purpose.

    ``trajectories`` is defaulted and derived from ``robots`` when omitted, so
    ``Component(0, frozenset({"alpha"}), anchor)`` still means what it always
    meant: one robot, one trajectory, no session declared.
    """

    component_id: int
    robots: frozenset[str]
    anchor: KeyframeId
    trajectories: frozenset[TrajectoryId] = frozenset()
    keyframe_ids: frozenset[KeyframeId] = frozenset()

    def __post_init__(self) -> None:
        if not self.trajectories:
            object.__setattr__(
                self,
                "trajectories",
                frozenset(TrajectoryId(robot_id) for robot_id in self.robots),
            )

    def __contains__(self, item: object) -> bool:
        """Membership by robot id (``str``) or by :class:`TrajectoryId`.

        The two differ for a robot whose segments have not re-merged: the robot
        is in both components, each of its trajectories in only one.
        """
        if isinstance(item, TrajectoryId):
            return item in self.trajectories
        return item in self.robots

    def trajectories_of(self, robot_id: str) -> frozenset[TrajectoryId]:
        return frozenset(t for t in self.trajectories if t.robot_id == robot_id)


@dataclass(slots=True)
class OptimizedGraph:
    """The solver's output: where every keyframe actually is."""

    poses: dict[KeyframeId, np.ndarray] = field(default_factory=dict)
    t_world_map: dict[str, np.ndarray] = field(default_factory=dict)
    """Per ROBOT: the transform carrying its CURRENT map frame into world.

    A convenience projection of :attr:`t_world_trajectory` onto the robot's
    newest trajectory, because that is the frame a live robot is publishing in
    right now and the only one an ``origins`` entry or a navigation goal can
    mean. For a robot that has never restarted the two are identical, which is
    every capture recorded before sessions existed.

    Never compose across a restart with this. A robot's older segments sit in
    their own frames and asking this dict for them silently returns the wrong
    one; ask :attr:`t_world_trajectory` instead.
    """

    components: list[Component] = field(default_factory=list)
    rejected_edges: list[Edge] = field(default_factory=list)
    iterations: int = 0
    final_error: float = 0.0

    t_world_trajectory: dict[TrajectoryId, np.ndarray] = field(default_factory=dict)
    """Per TRAJECTORY: the authoritative version of the quantity above.

    One rigid transform per continuous stretch of driving, each fitted over
    only that stretch. Fitting one frame across a reboot would average two
    unrelated gauges and land the robot between them -- a specific wrong
    answer, published straight to the operator, of exactly the kind
    ``_t_world_map``'s docstring already documents costing 6.5 m once.
    """

    def component_of(self, robot_id: str) -> Component | None:
        matching = [c for c in self.components if robot_id in c.robots]
        if not matching:
            return None
        return max(
            matching,
            key=lambda c: (
                len([k for k in c.keyframe_ids if k.robot_id == robot_id])
                if c.keyframe_ids
                else 1
            ),
        )

    def component_of_trajectory(self, trajectory: TrajectoryId) -> Component | None:
        matching = [c for c in self.components if trajectory in c.trajectories]
        if not matching:
            return None
        return max(
            matching,
            key=lambda c: (
                len([k for k in c.keyframe_ids if k.trajectory == trajectory])
                if c.keyframe_ids
                else 1
            ),
        )

    def share_frame(self, a: str, b: str) -> bool:
        """Whether two robots have a verified relative transform.

        The gate for using another robot's map. False means plan around unknown
        space, not around an assumed identity transform.

        True if *some* trajectory of ``a`` shares a component with *some*
        trajectory of ``b``, which is the right answer to "can I use their map"
        and the wrong one to "can I compose these two specific poses" -- for
        that, see :meth:`share_frame_trajectory`.
        """
        return any(a in c.robots and b in c.robots for c in self.components)

    def share_frame_trajectory(self, a: TrajectoryId, b: TrajectoryId) -> bool:
        """Whether two trajectories are in one component, so their poses compose.

        Distinct from :meth:`share_frame` exactly where it matters: two
        segments of one robot pass the robot-level test trivially (same robot!)
        while having no verified transform between them at all.
        """
        component = self.component_of_trajectory(a)
        return component is not None and b in component.trajectories

    def keyframes_of(self, robot_id: str) -> Iterator[tuple[KeyframeId, np.ndarray]]:
        for keyframe_id, pose in self.poses.items():
            if keyframe_id.robot_id == robot_id:
                yield keyframe_id, pose

    def keyframes_of_trajectory(
        self, trajectory: TrajectoryId
    ) -> Iterator[tuple[KeyframeId, np.ndarray]]:
        for keyframe_id, pose in self.poses.items():
            if keyframe_id.trajectory == trajectory:
                yield keyframe_id, pose

    def trajectories(self) -> list[TrajectoryId]:
        """Every trajectory with a solved pose, sorted."""
        return sorted({keyframe_id.trajectory for keyframe_id in self.poses})


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
