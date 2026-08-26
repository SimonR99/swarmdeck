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
import time
from dataclasses import dataclass, field

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
from swarmdeck_slam.graph import GtsamPoseGraph
from swarmdeck_slam.render import RenderConfig, RenderedGrid, render_all
from swarmdeck_slam.types import (
    Edge,
    EdgeKind,
    Keyframe,
    KeyframeId,
    OptimizedGraph,
    se3_distance,
    se3_from_quat_xyz,
    se3_identity,
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


def keyframe_from_packet(packet: KeyframePacket) -> Keyframe:
    """Lift a wire packet into a pose-graph node.

    The descriptor is computed here when the adapter omitted one: adapters are
    allowed to send clouds without Scan Context (the wire format says so), and
    the back-end is the one copy of the descriptor that the tests already
    cover. Computing it is ~1 ms and is not why this process is separate.
    """
    descriptor = packet.descriptor.data if packet.descriptor is not None else None
    kind = packet.descriptor.kind if packet.descriptor is not None else ""
    if descriptor is None:
        descriptor = scan_context_descriptor(packet.points)
        kind = DESCRIPTOR_KIND
    return Keyframe(
        id=KeyframeId(packet.robot_id, packet.seq),
        stamp=packet.stamp,
        t_odom_base=se3_from_quat_xyz(packet.t_odom_base),
        points=np.asarray(packet.points, dtype=np.float32),
        descriptor=descriptor,
        descriptor_kind=kind,
    )


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

    def __post_init__(self) -> None:
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
        self._last_of: dict[str, KeyframeId] = {}
        self.ambiguous_matches = 0
        self.implausible_hops = 0
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
        return self.ingest_keyframe(keyframe_from_packet(packet))

    def ingest_keyframe(self, keyframe: Keyframe) -> bool:
        """Add one node. Duplicate ``(robot_id, seq)`` pairs are dropped."""
        if keyframe.id in self._keyframes:
            return False
        if keyframe.points.shape[0] < self.min_points:
            return False

        previous_id = self._last_of.get(keyframe.id.robot_id)
        self._graph.add_keyframe(keyframe)
        self._keyframes[keyframe.id] = keyframe

        if previous_id is not None:
            previous = self._keyframes[previous_id]
            t_src_dst = se3_relative(previous.t_odom_base, keyframe.t_odom_base)
            information = ODOM_INFORMATION * self.odom_information_scale
            if self._implausible_hop(t_src_dst, keyframe.stamp - previous.stamp):
                information = information * self.implausible_hop_information_scale
                self.implausible_hops += 1
            self._graph.add_edge(
                Edge(
                    kind=EdgeKind.ODOMETRY,
                    src=previous.id,
                    dst=keyframe.id,
                    t_src_dst=t_src_dst,
                    information=information,
                )
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
            edge = verify_candidate(
                source=keyframe,
                target=target,
                yaw_prior=candidate.yaw,
                config=self.verify,
            )
            if edge is None:
                continue
            self._graph.add_edge(edge)
            self._accepted += 1
            self._inter_robot += int(edge.is_inter_robot)

        self._index.add(keyframe.id, descriptor)
        self._last_of[keyframe.id.robot_id] = keyframe.id
        self._dirty = True
        self._new_since_optimize += 1
        return True

    def _implausible_hop(self, t_src_dst: np.ndarray, dt_s: float) -> bool:
        """Whether a consecutive-keyframe transform is a frame discontinuity
        rather than robot motion. See :attr:`max_plausible_speed_mps`."""
        translation_m, rotation_rad = se3_distance(se3_identity(), t_src_dst)
        if translation_m < self.min_hop_m and rotation_rad < self.max_plausible_yaw_rate:
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
        genuinely different location -- same robot, far apart in that robot's
        own odometry. Anything else (a different robot, or the same robot's
        neighbouring keyframe) is not evidence of repetition and is skipped.
        """
        if self.descriptor_ratio <= 0.0 or len(candidates) < 2:
            return False
        best = candidates[0]
        best_kf = self._keyframes.get(best.keyframe_id)
        if best_kf is None:
            return False
        for other in candidates[1:]:
            if other.keyframe_id.robot_id != best.keyframe_id.robot_id:
                continue
            other_kf = self._keyframes.get(other.keyframe_id)
            if other_kf is None:
                continue
            separation = float(
                np.linalg.norm(
                    best_kf.t_odom_base[:3, 3] - other_kf.t_odom_base[:3, 3]
                )
            )
            if separation < self.ambiguity_radius_m:
                continue  # one place seen twice, not two places
            # Both distances are "smaller is better", so a ratio near 1 means
            # the runner-up explains the scan just as well as the winner.
            if best.distance >= self.descriptor_ratio * max(other.distance, 1e-9):
                return True
            return False
        return False

    def optimize_and_render(self) -> BackendSnapshot | None:
        """Run the solver and rasterize occupancy. None if nothing is ingested yet.

        Renders both partitions through :func:`render_all`, which poses and
        filters each keyframe once for the two groupings rather than twice.
        """
        if not self._keyframes:
            return None
        optimized = self._graph.optimize()
        grids, robot_grids = render_all(optimized, self._keyframes.values(), self.render)
        self._dirty = False
        self._new_since_optimize = 0
        return BackendSnapshot(
            optimized=optimized,
            grids=grids,
            robot_grids=robot_grids,
            keyframe_counts=_counts(self._keyframes),
            accepted_closures=self._accepted,
            inter_robot_closures=self._inter_robot,
            stamp=time.time(),
        )

    def reset(self) -> None:
        """Forget the session. Config (verify/render) stays."""
        self.__post_init__()


def _counts(keyframes: dict[KeyframeId, Keyframe]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for kf_id in keyframes:
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
        (f"robot:{robot_id}", grid) for robot_id, grid in sorted(snapshot.robot_grids.items())
    ]
    for component in snapshot.optimized.components:
        grid = snapshot.grids.get(component.component_id)
        if grid is not None:
            grids.append((f"component:{component.component_id}", grid))
    return grids


def snapshot_update(snapshot: BackendSnapshot) -> dict:
    """JSON body for ``POST /api/slam/update``.

    ``in_common_frame`` is true only for robots in a multi-robot component:
    that is the gate the server uses to put a robot on the merged map.
    """
    majority = majority_component(snapshot)
    majority_robots = majority.robots if majority is not None else frozenset()
    frame = f"component-{majority.component_id}" if majority is not None else ""

    origins: dict[str, dict[str, float | str]] = {}
    common_poses: dict[str, dict[str, float]] = {}
    graphs: dict[str, dict] = {}

    for component in snapshot.optimized.components:
        for robot_id in sorted(component.robots):
            correction = snapshot.optimized.t_world_map.get(robot_id)
            if correction is None:
                continue
            x, y, yaw = se2_of(correction)
            in_majority = robot_id in majority_robots
            origins[robot_id] = {
                "x": x,
                "y": y,
                "yaw": yaw,
                "frame": frame if in_majority else f"component-{component.component_id}",
            }
            latest_id = max(
                (kf_id for kf_id in snapshot.optimized.poses if kf_id.robot_id == robot_id),
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
                "anchor": str(c.anchor),
            }
            for c in snapshot.optimized.components
        ],
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
