"""Online collaborative pose-graph back-end.

This is the process that Phase 2 of the mapping plan exists to run: adapters
stream keyframe blobs, this module turns them into a joint trajectory, and
occupancy is *rendered* from the optimized poses. There is no grid registration
anywhere in this path.

The optimizer is CPU-heavy and must never run inside the server's asyncio loop
-- gtsam is also pinned to Python 3.12 / numpy 1.26, which the server process
is not. So this module is imported only by the SLAM service, never by FastAPI.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Iterable

import numpy as np

from swarmdeck_protocol import KeyframePacket
from swarmdeck_slam.descriptors import (
    DEFAULT_MAX_RANGE,
    DEFAULT_RINGS,
    DEFAULT_SECTORS,
    DESCRIPTOR_KIND,
    PlaceCandidate,
    ScanContextIndex,
    scan_context_descriptor,
)
from swarmdeck_slam.graph import GtsamPoseGraph, _MIN_FIT_KEYFRAMES, _frame_residual
from swarmdeck_slam.odom_free import (
    OdomFreeConfig,
    PreparedCloud,
    RegistrationHypothesis,
    prepare_cloud,
    register_clouds,
)
from swarmdeck_slam.reconstruction import (
    FragmentMatchConfig,
    ReconstructionFrame,
    TemporalConfig,
    build_temporal_fragments,
    filter_inter_robot_connections,
    find_fragment_connections,
    find_intra_fragment_loops,
    optimize_keyframe_poses,
    place_fragments,
)
from swarmdeck_slam.render import RenderConfig, RenderedGrid, render_all
from swarmdeck_slam.types import (
    Component,
    Edge,
    EdgeKind,
    Keyframe,
    KeyframeId,
    OptimizedGraph,
    TrajectoryId,
    se3_distance,
    se3_from_quat_xyz,
    se3_identity,
    se3_inverse,
    se3_kabsch,
    se3_relative,
)
from swarmdeck_slam.verify import VerifyConfig, verify_candidate

# Same order of magnitude as the integration-test odometry weight. A tighter
# matrix starves loop closures of leverage; a looser one lets odometry drift
# freely even after a good closure. Calibrate against real bags, not this
# number, once they exist (Phase 6).
ODOM_INFORMATION = np.eye(6, dtype=np.float64) * 400.0

# Production verification keeps GICP's conditioned Hessian.
#
# This was isotropic (I6 * 400 * fitness) on the strength of the synthetic
# fixture, where it measured better ATE. Real captured data disagrees. Measured
# on sessions/captures/3d-run-01 (two robots, 239 keyframes, Gazebo ground
# truth), optimized yaw error RMSE:
#
#                     robot_0   robot_1
#   isotropic          9.22      1.14      (front end: 3.65 / 1.13)
#   hessian            7.38      0.74
#
# Better on both robots, and hessian is the only setting where the pose graph
# beats its own front end (robot_1 1.13 -> 0.74) rather than merely surviving.
# Joint ATE agrees: 0.6837 -> 0.6604 m translation, 6.69 -> 5.30 deg rotation.
#
# The reason the fixture preferred isotropic is visible in that table too: the
# fixture is planar and non-repetitive, so its matches are never the degenerate
# corridor-slide case the conditioned Hessian exists to describe, and throwing
# the matrix away costs nothing there. On a real building it costs the
# optimizer the one signal that says which direction a match does not constrain.
#
# Degeneracy gates run on the Hessian either way, so this changes what a
# surviving edge CLAIMS, not which edges survive.
PRODUCTION_VERIFY = VerifyConfig(information="hessian")


def se2_of(matrix: np.ndarray) -> tuple[float, float, float]:
    """Planar ``(x, y, yaw)`` of a 4x4 ``T_a_b``. Ground robots live here."""
    return (
        float(matrix[0, 3]),
        float(matrix[1, 3]),
        float(math.atan2(matrix[1, 0], matrix[0, 0])),
    )


def keyframe_from_packet(
    packet: KeyframePacket, session: str | None = None
) -> Keyframe:
    """Lift a wire packet into a pose-graph node.

    The descriptor is computed here when the adapter omitted one: adapters are
    allowed to send clouds without Scan Context (the wire format says so), and
    the back-end is the one copy of the descriptor that the tests already
    cover. Computing it is ~1 ms and is not why this process is separate.

    ``session`` overrides the packet's own, which is how a session is
    reconstructed for a capture recorded before the field existed -- see
    :class:`LegacySegmenter`. ``None`` means "trust the packet".
    """
    descriptor = packet.descriptor.data if packet.descriptor is not None else None
    kind = packet.descriptor.kind if packet.descriptor is not None else ""
    if descriptor is None:
        descriptor = scan_context_descriptor(packet.points)
        kind = DESCRIPTOR_KIND
    return Keyframe(
        id=KeyframeId(
            packet.robot_id,
            packet.seq,
            packet.session if session is None else session,
        ),
        stamp=packet.stamp,
        t_odom_base=se3_from_quat_xyz(packet.t_odom_base),
        points=np.asarray(packet.points, dtype=np.float32),
        descriptor=descriptor,
        descriptor_kind=kind,
        ground_z=packet.ground_z,
        min_height=packet.min_height,
        max_height=packet.max_height,
        lidar_height=packet.lidar_height,
    )


#: Session id given to the second and later segments a :class:`LegacySegmenter`
#: finds. The FIRST segment keeps ``""``, so a capture with no restart in it
#: decodes to exactly the ``KeyframeId``s it decoded to before sessions
#: existed -- which is what makes every existing capture, test, and stored
#: reference replay unchanged.
LEGACY_SESSION_PREFIX = "restart-"


@dataclass(slots=True)
class _LegacySegment:
    session: str
    min_seq: int
    #: ``seq -> stamp`` for this segment. The stamp is what separates a
    #: retransmitted packet from a restart that reused the same ``seq``.
    seen: dict[int, float]
    restarts: int


class LegacySegmenter:
    """Recover trajectory boundaries for packets that carry no session.

    Every blob in ``sessions/captures`` predates the session field, and one of
    them (``hw-run-02``) contains three real reboots. Replaying it without
    segmentation reproduces the live bug rather than the fix: the post-reboot
    ``seq`` values collide with the pre-reboot ones, ``ingest_keyframe`` drops
    them as duplicates and returns False with no error, and whatever survives
    is chained to the old segment by an odometry edge spanning two unrelated
    map frames.

    The only signal available offline is ``seq`` itself, and the useful
    property is that a producer's ``seq`` is strictly increasing from 0 within
    one run. So a new segment starts when the incoming ``seq``:

    * is at or below the LOWEST ``seq`` this segment has seen -- a counter that
      has gone back to the beginning; or
    * has already been seen in this segment WITH A DIFFERENT STAMP -- a
      ``seq`` cannot repeat inside one run, so a genuinely different keyframe
      wearing one is proof of a new run.

    The stamp qualifier is what keeps a retransmitted packet from being read as
    a reboot. The same blob delivered twice (an HTTP retry, a replayed capture)
    carries the same ``seq`` and the same stamp, and it must stay what it
    always was: a duplicate, dropped by ``ingest_keyframe``. Only a repeat that
    is a different keyframe means the counter restarted.

    Not simply "``seq`` <= the last one seen", which is the tempting rule and
    is wrong on real data: ``hw-run-02`` contains a single late-arriving
    aslan_0 packet (``seq`` 109 delivered after 120, a queue reorder, not a
    reboot), and that rule splits its trajectory in three where two is the
    truth. The rules above are indifferent to arrival order, which is the
    property that matters -- the service's bounded queue is explicitly allowed
    to reorder.

    Two blind spots, stated because both are real:

    * a restart whose first keyframes are all lost, so its lowest surviving
      ``seq`` is above the previous segment's lowest AND was itself never
      delivered before the restart, looks like a continuation;
    * a packet reordered so late that its ``seq`` lands below everything seen
      so far looks like a restart, stranding the segment's first few keyframes
      in a segment of their own.

    Every real boundary in ``sessions/captures/hw-run-02`` also has a stamp
    strictly newer than anything in the segment before it, and requiring that
    would close the second blind spot. It is deliberately NOT required: a robot
    with no RTC boots at the same fake timestamp every time, so the check would
    trade a visible, harmless over-split for an invisible under-split -- and an
    under-split is the original bug, silently dropping keyframes to a ``seq``
    collision. The over-split shows up as an extra row in ``/status``, which an
    operator can see and can re-merge by selection.

    None of this runs against a modern stream. A packet that carries a session
    never reaches this class.
    """

    def __init__(self) -> None:
        self._segments: dict[str, _LegacySegment] = {}

    def session_for(self, robot_id: str, seq: int, stamp: float) -> str:
        segment = self._segments.get(robot_id)
        if segment is None:
            self._segments[robot_id] = _LegacySegment("", seq, {seq: stamp}, 0)
            return ""
        known = segment.seen.get(seq)
        if known is not None and known == stamp:
            return segment.session  # the same keyframe again, not a new run
        if seq <= segment.min_seq or known is not None:
            segment.restarts += 1
            segment.session = f"{LEGACY_SESSION_PREFIX}{segment.restarts}"
            segment.min_seq = seq
            segment.seen = {seq: stamp}
            return segment.session
        segment.seen[seq] = stamp
        return segment.session

    @property
    def restarts(self) -> int:
        """How many segment boundaries have been detected across the fleet."""
        return sum(segment.restarts for segment in self._segments.values())


@dataclass(frozen=True, slots=True)
class TrajectorySummary:
    """One selectable trajectory, as the operator sees it in ``/status``."""

    trajectory_id: TrajectoryId
    keyframes: int
    first_seq: int
    last_seq: int
    first_stamp: float
    last_stamp: float
    #: Which component the solver put it in, or None when it is excluded and so
    #: was never in the graph to be placed.
    component_id: int | None
    included: bool

    @property
    def robot_id(self) -> str:
        return self.trajectory_id.robot_id

    @property
    def session(self) -> str:
        return self.trajectory_id.session

    def to_dict(self) -> dict:
        return {
            "id": str(self.trajectory_id),
            "robot_id": self.robot_id,
            "session": self.session,
            "keyframes": self.keyframes,
            "first_seq": self.first_seq,
            "last_seq": self.last_seq,
            "first_stamp": self.first_stamp,
            "last_stamp": self.last_stamp,
            "component": self.component_id,
            "included": self.included,
        }


@dataclass(slots=True)
class BackendSnapshot:
    """One published view of the graph, ready to ship to the SwarmDeck server."""

    optimized: OptimizedGraph
    grids: dict[int, RenderedGrid]
    #: One grid per robot, from the same optimized poses. Lets the operator see
    #: a robot that merged with nobody, which the merged map deliberately omits.
    robot_grids: dict[str, RenderedGrid]
    keyframe_counts: dict[str, int]
    accepted_closures: int
    inter_robot_closures: int
    stamp: float
    #: One grid per trajectory, and ONLY for robots that have more than one --
    #: see :func:`~swarmdeck_slam.render.render_per_trajectory`. Empty for a
    #: fleet where nothing restarted, which is the ordinary case.
    trajectory_grids: dict[TrajectoryId, RenderedGrid] = field(default_factory=dict)
    #: Every trajectory the back-end holds, INCLUDING the excluded ones. An
    #: excluded segment is still stored and still listed; that is what makes
    #: excluding it reversible.
    trajectories: list[TrajectorySummary] = field(default_factory=list)


@dataclass
class CollaborativeBackend:
    """Incremental ingest + batched optimize/render.

    ``ingest`` is cheap relative to ``optimize``: it appends a node, an
    odometry edge, and any verified loop closures. ``optimize_and_render`` is
    the slow call and is owned by the service worker, never by the HTTP
    thread that accepted the blob.
    """

    verify: VerifyConfig = field(default_factory=lambda: PRODUCTION_VERIFY)
    render: RenderConfig = field(default_factory=RenderConfig)
    descriptor_k: int = 3
    temporal_window: int = 5
    min_points: int = 50
    odom_information_scale: float = 1.0
    """Multiplies ODOM_INFORMATION on every odometry edge. 1.0 is today's value.

    ODOM_INFORMATION is 400 in all six DoF, constant, whatever the robot was
    doing. That is a claim that one keyframe hop is known to the same precision
    while grinding against a wall as while driving clean -- and a differential
    drive that is wedged keeps turning its wheels, so precisely when the number
    is most wrong it is asserted most confidently. Its own comment says to
    calibrate it against real data rather than trust it.

    The cost is paid at loop closure: robot_0 accumulates -15 deg of yaw drift,
    and when it finally revisits a place the closure must undo all of it against
    a chain of edges each insisting the drift never happened. The optimizer
    compromises the only way it can, by smearing the correction along the
    trajectory -- which is how a graph turns a 3.65 deg front-end error into 9 deg.
    """

    allow_inter_robot: bool = True
    """Set False to refuse every inter-robot closure, leaving each robot's own
    loop closures intact.

    A diagnostic, not a deployment setting: with it off the fleet cannot merge
    at all, which defeats the point of the system. It exists because "the graph
    degrades robot_0's yaw" has two very different causes -- its own loop
    closures, or the ones tying it to robot_1 -- and the two are indistinguishable
    from the outside. Turning this off isolates them: if the damage survives, it
    is a single-robot bug that a two-robot run merely revealed.
    """

    descriptor_ratio: float = 0.0
    """Reject an ambiguous place match: 0 disables, typical values 0.75-0.9.

    Lowe's ratio test, aimed at the failure the spatial gate could not reach.
    A merge that starts wrong starts wrong at BOOTSTRAP, before any common frame
    exists to measure against, so no estimate-based gate can see it. What is
    visible even then is the descriptor itself: in a distinctive place the best
    match beats the runner-up clearly, and in a hall of identical corridors it
    does not.

    "Runner-up" means a candidate from the SAME robot that is far away in that
    robot's own trajectory (``ambiguity_radius_m``). That restriction is the
    whole trick. Two adjacent keyframes matching equally well is not ambiguity,
    it is one place seen twice, and rejecting it would throw away exactly the
    closures we want. But one robot cannot be in two places at once -- so when
    two of its keyframes a corridor apart both explain our scan equally well,
    the scene repeats and the match is a coin flip.
    """

    ambiguity_radius_m: float = 5.0
    """How far apart two of one robot's keyframes must be, in its own odometry,
    before they count as competing PLACES rather than one place seen twice.
    Comfortably beyond keyframe spacing (0.5 m gated) and inside the building's
    bay period, so ordinary neighbours never trip the ratio test."""

    max_plausible_speed_mps: float = 5.0
    """Implied speed between consecutive keyframes above which the hop is
    treated as a frame discontinuity rather than as robot motion.

    An ODOMETRY edge is a GNC known-inlier: neither PCM nor GNC is allowed to
    reject it, because rejecting one would disconnect the graph. That makes a
    wrong odometry edge the only input to this system with no defense behind
    it, and there is a real way to produce one. What arrives in
    ``t_odom_base`` is the robot's own SLAM map pose (see ``types.py``), and
    the adapter falls back to an odom-frame pose when its TF lookup fails, so
    a stream can switch frames -- or absorb a local SLAM re-optimization --
    between one keyframe and the next. The hop that straddles that is not a
    measurement of anything.

    Speed, not distance, because distance alone cannot tell a frame jump from
    a long hop. The service's bounded queue drops blobs under load, and a
    dropped keyframe legitimately stretches the next hop to several keyframe
    spacings -- but it stretches the elapsed time by exactly as much, so the
    implied speed is unchanged. A frame jump is instantaneous by definition
    and shows up as a speed nothing on this fleet can reach.

    5 m/s is over 3x the fastest hop measured across both robots of
    ``sessions/captures/3d-run-01`` (2.30 m in 1.50 s = 1.5 m/s) and well
    above the platforms' own top speeds (Scout Mini ~1.5 m/s, Spot ~1.6 m/s),
    while a frame jump lands one to two orders of magnitude above it.
    """

    max_plausible_yaw_rate: float = math.radians(90.0)
    """Rotation counterpart to :attr:`max_plausible_speed_mps`, in rad/s. An
    order of magnitude above the 8 deg/s the producer already gates keyframe
    capture on (``keyframe_producer.DEFAULT_MAX_YAW_RATE``), so a robot that
    turned hard between two accepted keyframes is never mistaken for a frame
    that rotated."""

    min_hop_m: float = 1.0
    """Floor below which a hop is never flagged, whatever speed it implies.

    Stamps come off a ROS message header and are not guaranteed sane; a
    near-zero or backwards ``dt`` would otherwise turn every ordinary
    half-metre step into an infinite-speed "jump". Well above the producer's
    0.5 m capture gate, so a normal hop can never be flagged on a bad clock
    alone, and far below any real discontinuity."""

    fallback_hop_m: float = 5.0
    """Absolute distance gate used when the stamps cannot give a usable ``dt``
    (non-monotonic clock, replayed capture, duplicate stamps). Distance is the
    weaker test -- it is the one that cannot distinguish a queue-drop-stretched
    hop from a jump -- so it is the fallback rather than the rule."""

    implausible_hop_information_scale: float = 1e-4
    """What an implausible hop's information is multiplied by.

    Down-weighted, never dropped. Dropping the edge would split that robot's
    chain into two subgraphs with no constraint between them, and a component
    is anchored once -- so the far side would be gauge-free and the solver
    would place it arbitrarily. That turns a bad edge into a broken graph,
    which is worse. Keeping it at ~1e-4 of normal weight leaves the graph
    connected while letting the loop closures around it decide where the far
    side actually goes.
    """

    pcm_confidence: float = 0.99
    min_pcm_clique_size: int = 2
    gnc_weight_threshold: float = 0.5
    """PCM/GNC knobs, forwarded to :class:`~swarmdeck_slam.graph.GtsamPoseGraph`.

    Defaults repeat that class's own, which is where each one is documented.
    They are surfaced here because the graph was previously constructed with
    no arguments at all, which left the fleet's entire outlier-rejection
    policy unreachable from the service, from ``tools/replay.py``, and from
    any test that wanted to sweep it against a recorded run -- the one place
    those thresholds can actually be calibrated.
    """

    legacy_session_split: bool = True
    """Recover trajectory boundaries for packets that declare no session.

    ON by default, and that is a deliberate behaviour change on old data.
    Leaving it off would preserve today's behaviour on a capture with a reboot
    in it, and today's behaviour there is the bug: ``ingest_keyframe`` silently
    drops every post-reboot keyframe whose ``seq`` collides with a pre-reboot
    one, and chains whatever survives across two unrelated map frames. There is
    nothing worth preserving in that, and no reading of the recorded bytes
    under which the old answer is the better one.

    It is safe to leave on because it cannot fire on a modern stream: a packet
    that carries a session is never inspected, and within one session ``seq``
    cannot go backwards. It only ever looks at packets that predate the field.
    Set False to reproduce a pre-trajectory replay exactly -- for comparing
    against an old recorded result, which is the one case where the bug is what
    you are trying to measure.

    See :class:`LegacySegmenter` for the detection rule and its blind spot.
    """

    registration_prior: str = "none"
    """Where to seed GICP with the current estimate of both keyframes, rather
    than with place recognition's yaw and a zero translation.

    ``"none"`` restores the yaw-only seed everywhere. ``"intra"`` (the
    default) seeds only same-robot closures. ``"all"`` seeds inter-robot
    closures too, once the robots share a frame.

    ``verify_candidate``'s zero-translation seed silently caps loop closure at
    roughly ``max_correspondence_distance`` (1.0 m) of true separation: beyond
    that no point pair is within range and GICP returns near-identity. On
    ``sessions/captures/3d-run-01`` the recovered transform error tracked the
    ground-truth separation almost exactly in every band past 2 m.

    Measured (``tools/replay.py --ablate prior-none prior-intra prior-all``)::

                                     none      intra       all
        cross-robot rel pose [m]    1.9313    1.2465    1.2449
        cross-robot rel pose [deg]   8.443     3.580     3.646
        joint ATE [m]               0.6604    0.6503    0.6501
        robot_0 ATE [m]             0.5325    0.5097    0.5114
        robot_0 RPE-5 [deg]           5.92      3.56      3.56
        closures (total / inter)   259/ 91   284/ 91   264/ 71
        t_world_map err r0 [m]      0.2342    0.3146    0.3001

    DEFAULTED TO "none" ON REAL DATA, reversing the choice below. The table
    above is from 3d-run-01, a Gazebo run. On ``sessions/captures/hw-run-01``
    (real hardware) seeding same-robot closures COSTS long-range loop closures,
    which are the ones that matter -- aslan_0's closures at a sequence gap of
    25 or more fell 90 -> 39, and its longest closure 103 -> 60 gaps.

    The mechanism is the feedback loop named below, but it bites precisely
    where the seed looked safest. A closure spanning a full tour is drifted by
    exactly the amount that closure exists to correct, so seeding GICP at the
    current estimate starts it far from the answer and it fails. Short-gap
    closures, where the estimate is good, survive -- which is why the damage
    shows up only in the long-gap tail and is invisible in a closure COUNT.
    "Same-robot seeding cannot bias what odometry already pins" was wrong: the
    odometry chain pins neighbours, and it is the accumulated drift BETWEEN
    distant keyframes that a loop closure has to overrule.

    Keep the option -- the cross-robot gain it showed on 3d-run-01 is real --
    but it needs a gap-aware rule before it can be a default, seeding only
    where the estimate has not had room to drift.

    ``"intra"`` was chosen because it captures the entire gain -- the two robots land
    35% closer in translation and 58% closer in rotation *relative to each
    other* -- while ``"all"`` adds nothing and costs 20 inter-robot closures
    (91 -> 71). Those closures are what PCM cross-checks against, so trading
    them for no accuracy is a bad deal, and an inter-robot edge seeded from
    the current inter-robot estimate is the one case where the seed could
    plausibly bias the quantity it is supposed to measure. Same-robot seeding
    cannot: within one robot's own frame the transform between two of its
    keyframes is already pinned by its odometry chain.

    READ THE LAST ROW WITH CARE, and do not tune against it. ``t_world_map``
    error gets *worse* while every direct measure of the trajectory gets
    better, and the direct measures are the ones to believe.
    ``t_world_map`` is a single rigid frame fitted to a whole trajectory, so
    when better closures change that trajectory's SHAPE the best-fit frame
    moves too -- and its distance from truth's own best-fit frame can grow
    even as every pose in it gets closer to ground truth. The cross-robot
    relative pose above never routes through that summary, which is exactly
    why it was the measurement that settled this.
    """

    registration_mode: str = "graph"
    """Registration solver mode: 'graph' (conventional pose graph) or 'odom_free'
    (odometry-free geometric reconstruction)."""

    temporal_config: TemporalConfig = field(default_factory=TemporalConfig)
    match_config: FragmentMatchConfig = field(default_factory=FragmentMatchConfig)
    odom_free_config: OdomFreeConfig = field(default_factory=OdomFreeConfig)

    def __post_init__(self) -> None:
        self._prepared_clouds: dict[KeyframeId, PreparedCloud] = {}
        self._pair_cache: dict[tuple[KeyframeId, KeyframeId], list[RegistrationHypothesis]] = {}
        self._graph = GtsamPoseGraph(
            pcm_confidence=self.pcm_confidence,
            min_pcm_clique_size=self.min_pcm_clique_size,
            gnc_weight_threshold=self.gnc_weight_threshold,
        )
        self._index = ScanContextIndex(
            rings=DEFAULT_RINGS,
            sectors=DEFAULT_SECTORS,
            temporal_window=self.temporal_window,
        )
        self._keyframes: dict[KeyframeId, Keyframe] = {}
        #: Every edge ever built, kept here and not only inside the solver so
        #: the graph can be rebuilt over a different SUBSET of trajectories
        #: without re-ingesting anything. That is what makes exclusion
        #: reversible: nothing is thrown away, only left out.
        self._edges: list[Edge] = []
        #: Newest keyframe per TRAJECTORY. Per robot, this chained a
        #: post-reboot keyframe to a pre-reboot one and called the difference
        #: between two unrelated map frames an odometry measurement -- an edge
        #: GNC is structurally forbidden from rejecting.
        self._last_of: dict[TrajectoryId, KeyframeId] = {}
        #: Replaced wholesale, never mutated in place, so a reader can take a
        #: local reference and be sure the set it filters keyframes by is the
        #: same one it filters edges by. The selection arrives on the HTTP
        #: thread while the worker thread is ingesting.
        self._excluded: frozenset[TrajectoryId] = frozenset()
        #: Set when the selection changes; consumed by the worker thread at
        #: the top of the next optimize. Rebuilding the solver where the
        #: selection is CHANGED would do it on the HTTP thread, concurrently
        #: with an ingest that is halfway through adding a keyframe and its
        #: edge -- which is how a graph ends up holding an edge whose endpoint
        #: it never received.
        self._graph_stale = False
        #: Guards insertion into ``_keyframes`` against the snapshot that
        #: ``trajectory_summaries`` takes for ``/status``. Microseconds, and
        #: nothing slow is ever done while holding it.
        self._keyframes_lock = threading.Lock()
        self._segmenter = LegacySegmenter()
        self.ambiguous_matches = 0
        self.implausible_hops = 0
        self.primed_verifications = 0
        self._last_solved: OptimizedGraph | None = None
        self._accepted = 0
        self._inter_robot = 0
        self._dirty = False
        self._new_since_optimize = 0

    def __len__(self) -> int:
        return len(self._keyframes)

    @property
    def dirty(self) -> bool:
        return self._dirty

    @property
    def new_since_optimize(self) -> int:
        return self._new_since_optimize

    def ingest_packet(self, packet: KeyframePacket) -> bool:
        """Add one decoded keyframe. Returns False if it was ignored."""
        session = None
        if self.legacy_session_split and not packet.session:
            session = self._segmenter.session_for(
                packet.robot_id, packet.seq, packet.stamp
            )
        return self.ingest_keyframe(keyframe_from_packet(packet, session))

    def ingest_keyframe(self, keyframe: Keyframe) -> bool:
        """Add one node. Duplicate ``(robot_id, session, seq)`` triples are dropped.

        The triple, not the pair. ``seq`` restarts at zero every time the
        producer process does, so keying on ``(robot_id, seq)`` made a robot's
        first N post-reboot keyframes indistinguishable from its first N
        pre-reboot ones -- and this method dropped them here, returning False
        with no error recorded anywhere.
        """
        if keyframe.id in self._keyframes:
            return False
        if keyframe.points.shape[0] < self.min_points:
            return False

        # One read of the selection for the whole call: taking it twice could
        # add the keyframe under one answer and skip its edges under the other,
        # leaving the solver a variable no factor touches.
        excluded = self._excluded
        previous_id = self._last_of.get(keyframe.id.trajectory)
        if keyframe.id.trajectory not in excluded:
            self._graph.add_keyframe(keyframe)
        with self._keyframes_lock:
            self._keyframes[keyframe.id] = keyframe

        # ``.get``, not ``[]``: /reset swaps the whole keyframe store out from
        # under the worker thread, so a keyframe already halfway through this
        # method can find its predecessor gone. Losing one odometry edge across
        # a reset is nothing; killing the worker thread with a KeyError takes
        # the map down until the process restarts.
        previous = None if previous_id is None else self._keyframes.get(previous_id)
        if previous is not None:
            t_src_dst = se3_relative(previous.t_odom_base, keyframe.t_odom_base)
            information = ODOM_INFORMATION * self.odom_information_scale
            if self._implausible_hop(t_src_dst, keyframe.stamp - previous.stamp):
                information = information * self.implausible_hop_information_scale
                self.implausible_hops += 1
            self._add_edge(
                excluded,
                Edge(
                    kind=EdgeKind.ODOMETRY,
                    src=previous.id,
                    dst=keyframe.id,
                    t_src_dst=t_src_dst,
                    information=information,
                ),
            )

        descriptor = keyframe.descriptor
        if descriptor is None:
            descriptor = scan_context_descriptor(keyframe.points)
            keyframe.descriptor = descriptor
            keyframe.descriptor_kind = DESCRIPTOR_KIND

        candidates = self._index.query(
            descriptor, k=self.descriptor_k, query_id=keyframe.id
        )
        if self._ambiguous(candidates):
            self.ambiguous_matches += 1
            candidates = []
        for candidate in candidates:
            target = self._keyframes.get(candidate.keyframe_id)
            if target is None:
                continue
            if (
                not self.allow_inter_robot
                and target.id.robot_id != keyframe.id.robot_id
            ):
                continue
            # An excluded trajectory is not in the graph, so an edge to it
            # would reference a keyframe the solver has never been given.
            # Skipping the verification also saves the GICP run.
            if target.id.trajectory in excluded:
                continue
            prior = self._registration_prior(keyframe, target)
            self.primed_verifications += int(prior is not None)
            edge = verify_candidate(
                source=keyframe,
                target=target,
                yaw_prior=candidate.yaw,
                config=self.verify,
                t_target_source_prior=prior,
            )
            if edge is None:
                continue
            self._add_edge(excluded, edge)
            self._accepted += 1
            self._inter_robot += int(edge.is_inter_robot)

        self._index.add(keyframe.id, descriptor)
        self._last_of[keyframe.id.trajectory] = keyframe.id
        self._dirty = True
        self._new_since_optimize += 1
        return True

    def _add_edge(self, excluded: frozenset[TrajectoryId], edge: Edge) -> None:
        """Record an edge, and hand it to the solver unless it is excluded."""
        self._edges.append(edge)
        if self._edge_included(edge, excluded):
            self._graph.add_edge(edge)

    def _edge_included(
        self, edge: Edge, excluded: frozenset[TrajectoryId] | None = None
    ) -> bool:
        excluded = self._excluded if excluded is None else excluded
        return (
            edge.src.trajectory not in excluded and edge.dst.trajectory not in excluded
        )

    def _registration_prior(
        self, source: Keyframe, target: Keyframe
    ) -> np.ndarray | None:
        """``T_target_source`` from the last solved graph, or None.

        None is returned whenever the estimate cannot actually place the two
        keyframes in one frame, because a seed built from unrelated frames is
        worse than no seed: it is a specific, confident wrong answer that GICP
        will happily converge near. The cases:

        * no solve yet -- nothing to seed from;
        * ``target`` was not in the last solve;
        * the two trajectories are in different components, meaning no verified
          relative transform exists between them. This is the one that
          matters. Using ``t_world_map`` across a component boundary would
          compose two independent gauge choices and call the result a pose.

        ``target`` is read from the solved poses directly. ``source`` is the
        keyframe being ingested and so is not in the graph yet; it is placed
        with its TRAJECTORY's ``t_world_map`` correction composed onto its own
        live pose, which is exactly the use that quantity is documented for.

        ``"intra"`` means same TRAJECTORY, not same robot. Its whole
        justification is that "within one robot's own frame the transform
        between two of its keyframes is already pinned by its odometry chain"
        -- and that chain stops at a reboot. A closure onto the robot's own
        pre-reboot segment has no more free information behind it than a
        closure onto a stranger, so it is seeded only under ``"all"``.
        """
        if self.registration_prior == "none":
            return None
        same_trajectory = source.id.trajectory == target.id.trajectory
        if not same_trajectory and self.registration_prior != "all":
            return None
        solved = self._last_solved
        if solved is None:
            return None
        target_pose = solved.poses.get(target.id)
        if target_pose is None:
            return None
        if not solved.share_frame_trajectory(
            source.id.trajectory, target.id.trajectory
        ):
            return None
        correction = solved.t_world_trajectory.get(source.id.trajectory)
        if correction is None:
            return None
        t_world_source = correction @ source.t_odom_base
        return se3_inverse(target_pose) @ t_world_source

    def _implausible_hop(self, t_src_dst: np.ndarray, dt_s: float) -> bool:
        """Whether a consecutive-keyframe transform is a frame discontinuity
        rather than robot motion. See :attr:`max_plausible_speed_mps`."""
        translation_m, rotation_rad = se3_distance(se3_identity(), t_src_dst)
        if (
            translation_m < self.min_hop_m
            and rotation_rad < self.max_plausible_yaw_rate
        ):
            return False
        if dt_s <= 0.0:
            return translation_m > self.fallback_hop_m
        return (
            translation_m / dt_s > self.max_plausible_speed_mps
            or rotation_rad / dt_s > self.max_plausible_yaw_rate
        )

    def _ambiguous(self, candidates: list[PlaceCandidate]) -> bool:
        """Whether the best match is indistinguishable from a different place.

        Compares the best candidate against the nearest runner-up that is a
        genuinely different location -- same TRAJECTORY, far apart in that
        trajectory's own odometry. Anything else (a different robot, a
        different run of the same robot, or the same run's neighbouring
        keyframe) is not evidence of repetition and is skipped.

        Same trajectory rather than same robot because the separation below is
        a distance between two ``t_odom_base`` translations, and across a
        reboot those are coordinates in two different frames. The subtraction
        would produce a number, and the number would mean nothing -- it would
        pass or fail the radius test on where the robot happened to be standing
        when it came back up.
        """
        if self.descriptor_ratio <= 0.0 or len(candidates) < 2:
            return False
        best = candidates[0]
        best_kf = self._keyframes.get(best.keyframe_id)
        if best_kf is None:
            return False
        for other in candidates[1:]:
            if other.keyframe_id.trajectory != best.keyframe_id.trajectory:
                continue
            other_kf = self._keyframes.get(other.keyframe_id)
            if other_kf is None:
                continue
            separation = float(
                np.linalg.norm(best_kf.t_odom_base[:3, 3] - other_kf.t_odom_base[:3, 3])
            )
            if separation < self.ambiguity_radius_m:
                continue  # one place seen twice, not two places
            # Both distances are "smaller is better", so a ratio near 1 means
            # the runner-up explains the scan just as well as the winner.
            if best.distance >= self.descriptor_ratio * max(other.distance, 1e-9):
                return True
            return False
        return False

    # ------------------------------------------------------------------ #
    # Trajectory selection
    # ------------------------------------------------------------------ #

    def trajectory_ids(self) -> list[TrajectoryId]:
        """Every trajectory held, included or not, sorted."""
        return sorted({kf_id.trajectory for kf_id in self._keyframes})

    def is_included(self, trajectory: TrajectoryId) -> bool:
        return trajectory not in self._excluded

    def set_included(self, trajectory: TrajectoryId, included: bool) -> bool:
        """Include or exclude one trajectory from optimization. True if it changed.

        An excluded trajectory stays STORED -- its keyframes, its clouds, its
        descriptors and every edge that touches it are all still here. It
        simply is not handed to the solver, so it contributes no constraint and
        gets no pose, and therefore renders into no grid. Re-including it puts
        every one of those edges back exactly as it was, which is the whole
        point: an operator can drop a segment, look at the map without it, and
        put it back, without re-ingesting a single blob.
        """
        if included == self.is_included(trajectory):
            return False
        self._excluded = (
            self._excluded - {trajectory} if included else self._excluded | {trajectory}
        )
        self._mark_selection_changed()
        return True

    def include_only(self, trajectories: Iterable[TrajectoryId]) -> None:
        """Include exactly this set and exclude every other trajectory held.

        The "re-optimise a chosen set" entry point: name the segments the map
        should be rebuilt from, then call :meth:`optimize_and_render`.
        Trajectories that were never ingested are ignored rather than an error
        -- the caller is naming a selection, not asserting the contents of the
        session.
        """
        wanted = set(trajectories)
        self._excluded = frozenset(t for t in self.trajectory_ids() if t not in wanted)
        self._mark_selection_changed()

    def _mark_selection_changed(self) -> None:
        """Ask for a rebuild, and for a re-solve, without doing either here.

        Called from whichever thread the operator's request landed on. The
        rebuild itself is the worker's job -- see :attr:`_graph_stale`.
        """
        self._graph_stale = True
        self._dirty = True
        # The last solve described a different selection; seeding registration
        # priors from it would place a keyframe using a component membership
        # that no longer holds.
        self._last_solved = None

    def _rebuild_graph(self) -> None:
        """Re-seed the solver from the included keyframes and edges alone.

        A fresh :class:`GtsamPoseGraph` rather than a mutation of the existing
        one, because that class only ever appends -- it has no remove, on
        purpose, since a solver that can forget a factor is a solver whose
        result depends on call history. Rebuilding from the retained
        keyframes and edges is O(n) appends against a solve that is
        superlinear, and it is exactly reproducible.

        Keyframes go in in ingest order and edges after them, so the rebuilt
        graph is the one that would have existed had the excluded trajectories
        never arrived.

        The stale flag is cleared BEFORE the rebuild, not after: a selection
        that arrives while this is running has to be honoured on the next pass,
        and clearing afterwards would swallow it.
        """
        self._graph_stale = False
        excluded = self._excluded
        self._graph = GtsamPoseGraph(
            pcm_confidence=self.pcm_confidence,
            min_pcm_clique_size=self.min_pcm_clique_size,
            gnc_weight_threshold=self.gnc_weight_threshold,
        )
        for keyframe in list(self._keyframes.values()):
            if keyframe.id.trajectory not in excluded:
                self._graph.add_keyframe(keyframe)
        for edge in list(self._edges):
            if self._edge_included(edge, excluded):
                self._graph.add_edge(edge)

    def trajectory_summaries(
        self, optimized: OptimizedGraph | None = None
    ) -> list[TrajectorySummary]:
        """One row per trajectory held, for the operator's selection list."""
        with self._keyframes_lock:
            keyframes = list(self._keyframes.values())
        by_trajectory: dict[TrajectoryId, list[Keyframe]] = {}
        for keyframe in keyframes:
            by_trajectory.setdefault(keyframe.id.trajectory, []).append(keyframe)

        summaries: list[TrajectorySummary] = []
        for trajectory in sorted(by_trajectory):
            members = by_trajectory[trajectory]
            seqs = [kf.id.seq for kf in members]
            stamps = [kf.stamp for kf in members]
            component = (
                optimized.component_of_trajectory(trajectory)
                if optimized is not None
                else None
            )
            summaries.append(
                TrajectorySummary(
                    trajectory_id=trajectory,
                    keyframes=len(members),
                    first_seq=min(seqs),
                    last_seq=max(seqs),
                    first_stamp=min(stamps),
                    last_stamp=max(stamps),
                    component_id=None if component is None else component.component_id,
                    included=self.is_included(trajectory),
                )
            )
        return summaries

    def _optimize_odom_free(
        self, included: list[Keyframe]
    ) -> tuple[OptimizedGraph, int, int]:
        frames = []
        for idx, kf in enumerate(included):
            cloud = self._prepared_clouds.get(kf.id)
            if cloud is None:
                cloud = prepare_cloud(kf.points, self.odom_free_config)
                self._prepared_clouds[kf.id] = cloud
            frames.append(
                ReconstructionFrame(
                    index=idx,
                    robot_id=kf.id.robot_id,
                    seq=kf.id.seq,
                    stamp=kf.stamp,
                    cloud=cloud,
                    session=kf.id.session,
                )
            )

        def memo_register(
            target: ReconstructionFrame, source: ReconstructionFrame
        ) -> list[RegistrationHypothesis]:
            key = (included[target.index].id, included[source.index].id)
            if key not in self._pair_cache:
                self._pair_cache[key] = register_clouds(
                    target.cloud, source.cloud, self.odom_free_config
                )
            return self._pair_cache[key]

        fragments, boundaries = build_temporal_fragments(
            frames, memo_register, self.temporal_config
        )
        connections, rejected_connections = find_fragment_connections(
            frames, fragments, memo_register, self.match_config
        )
        connections, rejected_inter = filter_inter_robot_connections(
            fragments, connections, self.match_config
        )
        loop_closures = find_intra_fragment_loops(
            frames, fragments, memo_register, self.match_config
        )
        placement = place_fragments(fragments, connections)
        frame_poses = optimize_keyframe_poses(
            fragments, connections, placement, loop_closures
        )

        poses = {included[idx].id: frame_poses[idx] for idx in frame_poses}
        components = []
        frag_by_id = {f.fragment_id: f for f in fragments}
        for comp_idx, comp_frags in enumerate(placement.components):
            comp_kf_ids = []
            for fid in comp_frags:
                if fid in frag_by_id:
                    for frame_idx in frag_by_id[fid].frame_indices:
                        kf_id = included[frame_idx].id
                        if kf_id in poses:
                            comp_kf_ids.append(kf_id)
            if not comp_kf_ids:
                continue
            anchor = min(comp_kf_ids, key=lambda k: k.seq)
            robots = frozenset(k.robot_id for k in comp_kf_ids)
            trajs = frozenset(k.trajectory for k in comp_kf_ids)
            components.append(
                Component(
                    component_id=comp_idx,
                    robots=robots,
                    anchor=anchor,
                    trajectories=trajs,
                    keyframe_ids=frozenset(comp_kf_ids),
                )
            )

        kf_map = {kf.id: kf for kf in included}
        by_traj: dict[TrajectoryId, list[KeyframeId]] = {}
        for k in included:
            if k.id in poses:
                by_traj.setdefault(k.id.trajectory, []).append(k.id)

        t_world_traj: dict[TrajectoryId, np.ndarray] = {}
        for traj, kf_ids in by_traj.items():
            kf_ids.sort(key=lambda k: k.seq)
            snapshot = poses[kf_ids[-1]] @ se3_inverse(
                kf_map[kf_ids[-1]].t_odom_base
            )
            if len(kf_ids) >= _MIN_FIT_KEYFRAMES:
                src = np.array([kf_map[k].t_odom_base[:3, 3] for k in kf_ids])
                tgt = np.array([poses[k][:3, 3] for k in kf_ids])
                try:
                    fitted = se3_kabsch(src, tgt)
                    if _frame_residual(
                        fitted, kf_ids, kf_map, poses
                    ) <= _frame_residual(snapshot, kf_ids, kf_map, poses):
                        t_world_traj[traj] = fitted
                    else:
                        t_world_traj[traj] = snapshot
                except Exception:
                    t_world_traj[traj] = snapshot
            else:
                t_world_traj[traj] = snapshot

        best_t: dict[str, tuple[float, np.ndarray]] = {}
        for traj, tf in t_world_traj.items():
            latest_st = max(kf_map[k].stamp for k in by_traj[traj])
            if traj.robot_id not in best_t or latest_st > best_t[traj.robot_id][0]:
                best_t[traj.robot_id] = (latest_st, tf)

        t_world_map = {rid: tf for rid, (_, tf) in best_t.items()}
        inter_robot = sum(
            1
            for c in connections
            if c.fragment_a.split("@")[0] != c.fragment_b.split("@")[0]
        )
        total_closures = len(connections) + len(loop_closures)
        optimized = OptimizedGraph(
            poses=poses,
            t_world_map=t_world_map,
            t_world_trajectory=t_world_traj,
            components=components,
        )
        return optimized, total_closures, inter_robot

    def optimize_and_render(self) -> BackendSnapshot | None:
        """Run the solver and rasterize occupancy. None if nothing is ingested yet.

        Renders every partition through :func:`render_all`, which poses and
        filters each keyframe once for all the groupings rather than once each.
        """
        if not self._keyframes:
            return None
        with self._keyframes_lock:
            keyframes = list(self._keyframes.values())
        excluded = self._excluded
        included = [
            keyframe for keyframe in keyframes if keyframe.id.trajectory not in excluded
        ]
        if not included:
            return None

        if self.registration_mode == "odom_free":
            optimized, accepted_count, inter_robot_count = self._optimize_odom_free(
                included
            )
        else:
            if self._graph_stale:
                self._rebuild_graph()
            optimized = self._graph.optimize()
            loops = [
                edge
                for edge in list(self._edges)
                if edge.kind.is_loop_closure and self._edge_included(edge, excluded)
            ]
            accepted_count = len(loops)
            inter_robot_count = sum(1 for edge in loops if edge.is_inter_robot)

        self._last_solved = optimized
        grids, robot_grids, trajectory_grids = render_all(
            optimized, included, self.render
        )
        self._dirty = False
        self._new_since_optimize = 0

        return BackendSnapshot(
            optimized=optimized,
            grids=grids,
            robot_grids=robot_grids,
            keyframe_counts=_counts(k.id for k in included),
            accepted_closures=accepted_count,
            inter_robot_closures=inter_robot_count,
            stamp=time.time(),
            trajectory_grids=trajectory_grids,
            trajectories=self.trajectory_summaries(optimized),
        )

    def reset(self) -> None:
        """Forget the session. Config (verify/render) stays."""
        self.__post_init__()


def _counts(keyframe_ids: Iterable[KeyframeId]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for kf_id in keyframe_ids:
        counts[kf_id.robot_id] = counts.get(kf_id.robot_id, 0) + 1
    return counts


def majority_component(snapshot: BackendSnapshot) -> RenderedGrid | None:
    """The component that should occupy the operator's merged map.

    Unmerged robots stay off that map: overlaying two components is a
    confident lie. A component of one robot is not published either -- the
    operator already has that robot's local map, and putting it in the fleet
    view would look like a merge that has not happened.
    """
    multi = [grid for grid in snapshot.grids.values() if len(grid.robots) >= 2]
    if not multi:
        return None
    return max(multi, key=lambda grid: (len(grid.robots), grid.width * grid.height))


def scoped_grids(snapshot: BackendSnapshot) -> list[tuple[str, RenderedGrid]]:
    """Every ``(scope, grid)`` the service publishes for one snapshot.

    The scope names live here, in one place, because two things have to agree
    on them exactly: the service that POSTs each grid, and the ``scopes`` list
    in :func:`snapshot_update` that tells the server which scopes are still
    live so it can drop the rest. If those two ever disagree, the server
    garbage-collects a grid the service just published, or keeps one it never
    will again -- and the second failure is silent.
    """
    grids: list[tuple[str, RenderedGrid]] = [
        (f"robot:{robot_id}", grid)
        for robot_id, grid in sorted(snapshot.robot_grids.items())
    ]
    # A robot's segments stay grouped under robot:<id> -- that is still one
    # machine's coverage and the operator asked for it by machine. The
    # trajectory scopes are ADDED beside it so a single segment can be
    # inspected alone, which is the only way to see whether a robot's two
    # halves actually agree about the building. Present only for robots that
    # have more than one segment; for anyone else it would be a byte-identical
    # copy of their robot: grid under a second name.
    grids.extend(
        (f"trajectory:{trajectory}", grid)
        for trajectory, grid in sorted(snapshot.trajectory_grids.items())
    )
    for component in snapshot.optimized.components:
        grid = snapshot.grids.get(component.component_id)
        if grid is not None:
            grids.append((f"component:{component.component_id}", grid))
    return grids


def _newest_trajectories(snapshot: BackendSnapshot) -> dict[TrajectoryId, float]:
    """``{newest trajectory of each robot: its last stamp}``.

    Newest by wall-clock stamp, matching ``GtsamPoseGraph._current_t_world_map``
    -- the two have to agree, or ``origins`` would report one segment's frame
    against another segment's pose.
    """
    best: dict[str, tuple[float, TrajectoryId]] = {}
    for summary in snapshot.trajectories:
        if not summary.included:
            continue
        rank = (summary.last_stamp, summary.trajectory_id)
        if summary.robot_id not in best or rank > best[summary.robot_id]:
            best[summary.robot_id] = rank
    if best:
        return {trajectory: stamp for stamp, trajectory in best.values()}

    # No summaries: a snapshot assembled by hand (tools, tests) rather than by
    # optimize_and_render. Fall back to the solved poses, where "newest" can
    # only be guessed -- harmlessly, because a snapshot built that way has one
    # trajectory per robot and there is nothing to choose between.
    for trajectory in {kf_id.trajectory for kf_id in snapshot.optimized.poses}:
        rank = (0.0, trajectory)
        if trajectory.robot_id not in best or rank > best[trajectory.robot_id]:
            best[trajectory.robot_id] = rank
    return {trajectory: stamp for stamp, trajectory in best.values()}


def _newest_trajectory_of(
    snapshot: BackendSnapshot, robot_id: str
) -> TrajectoryId | None:
    return next(
        (t for t in _newest_trajectories(snapshot) if t.robot_id == robot_id), None
    )


def snapshot_update(snapshot: BackendSnapshot) -> dict:
    """JSON body for ``POST /api/slam/update``.

    ``in_common_frame`` is true only for robots in a multi-robot component:
    that is the gate the server uses to put a robot on the merged map.

    ``origins`` stays keyed by robot and carries that robot's CURRENT map
    frame, because that is the only frame a live robot is publishing in and the
    only one a goal sent to it can be expressed in. The per-segment frames are
    not summarized here; a segment is inspected through its own scoped grid and
    its row in ``trajectories``.
    """
    majority = majority_component(snapshot)
    majority_robots = majority.robots if majority is not None else frozenset()
    frame = f"component-{majority.component_id}" if majority is not None else ""

    origins: dict[str, dict[str, float | str]] = {}
    common_poses: dict[str, dict[str, float]] = {}
    graphs: dict[str, dict] = {}

    # A robot whose segments have not re-merged appears in more than one
    # component. It gets ONE origins entry, from the component holding its
    # current (newest) trajectory -- the frame t_world_map is fitted to -- so
    # the fleet view cannot show the same robot twice in two places.
    current_component: dict[str, int] = {}
    for trajectory, _ in _newest_trajectories(snapshot).items():
        component = snapshot.optimized.component_of_trajectory(trajectory)
        if component is not None:
            current_component[trajectory.robot_id] = component.component_id

    for component in snapshot.optimized.components:
        for robot_id in sorted(component.robots):
            if (
                current_component.get(robot_id, component.component_id)
                != component.component_id
            ):
                continue
            correction = snapshot.optimized.t_world_map.get(robot_id)
            if correction is None:
                continue
            x, y, yaw = se2_of(correction)
            in_majority = robot_id in majority_robots
            origins[robot_id] = {
                "x": x,
                "y": y,
                "yaw": yaw,
                "frame": (
                    frame if in_majority else f"component-{component.component_id}"
                ),
            }
            # Newest keyframe of the robot's newest trajectory. Not simply
            # the highest seq across the robot: seq restarts at zero, so after
            # a reboot the highest one belongs to the segment that ENDED.
            current = _newest_trajectory_of(snapshot, robot_id)
            latest_id = max(
                (
                    kf_id
                    for kf_id in snapshot.optimized.poses
                    if kf_id.trajectory == current
                ),
                key=lambda kf_id: kf_id.seq,
                default=None,
            )
            if latest_id is not None:
                px, py, pyaw = se2_of(snapshot.optimized.poses[latest_id])
                common_poses[robot_id] = {"x": px, "y": py, "yaw": pyaw}

            peers = sorted(r for r in component.robots if r != robot_id)
            graphs[robot_id] = {
                "keyframes": snapshot.keyframe_counts.get(robot_id, 0),
                "in_common_frame": in_majority,
                "inter_robot": peers,
                "residual": snapshot.optimized.final_error,
            }

    return {
        "components": [
            {
                "id": c.component_id,
                "robots": sorted(c.robots),
                "trajectories": sorted(str(t) for t in c.trajectories),
                "anchor": str(c.anchor),
            }
            for c in snapshot.optimized.components
        ],
        "trajectories": [t.to_dict() for t in snapshot.trajectories],
        # Every scope that is still live. Component ids are positional over
        # sorted union-find roots, so they are NOT stable: when two robots
        # merge, the component count drops and the highest id stops being
        # published -- while the server, which only ever learns about a scope
        # by being handed one, would serve that dead grid forever. Naming the
        # live set on every update lets it drop the rest.
        "scopes": [scope for scope, _grid in scoped_grids(snapshot)],
        "origins": origins,
        "common_poses": common_poses,
        "graphs": graphs,
        "accepted_closures": snapshot.accepted_closures,
        "inter_robot_closures": snapshot.inter_robot_closures,
        "stamp": snapshot.stamp,
    }
