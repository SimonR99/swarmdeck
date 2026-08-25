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
    ScanContextIndex,
    scan_context_descriptor,
)
from swarmdeck_slam.graph import GtsamPoseGraph
from swarmdeck_slam.render import RenderConfig, RenderedGrid, render_occupancy
from swarmdeck_slam.types import (
    Edge,
    EdgeKind,
    Keyframe,
    KeyframeId,
    OptimizedGraph,
    se3_from_quat_xyz,
    se3_relative,
)
from swarmdeck_slam.verify import VerifyConfig, verify_candidate

# Same order of magnitude as the integration-test odometry weight. A tighter
# matrix starves loop closures of leverage; a looser one lets odometry drift
# freely even after a good closure. Calibrate against real bags, not this
# number, once they exist (Phase 6).
ODOM_INFORMATION = np.eye(6, dtype=np.float64) * 400.0

# Production verification uses isotropic information: the Hessian weighting is
# the tracked ATE defect, and isotropic is the setting that actually improves
# it. Degeneracy gates still run on the Hessian so a single-wall match cannot
# sneak through just because we threw the matrix away afterwards.
PRODUCTION_VERIFY = VerifyConfig(information="isotropic", isotropic_scale=400.0)


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

    def __post_init__(self) -> None:
        self._graph = GtsamPoseGraph()
        self._index = ScanContextIndex(
            rings=DEFAULT_RINGS,
            sectors=DEFAULT_SECTORS,
            temporal_window=self.temporal_window,
        )
        self._keyframes: dict[KeyframeId, Keyframe] = {}
        self._last_of: dict[str, KeyframeId] = {}
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
            self._graph.add_edge(
                Edge(
                    kind=EdgeKind.ODOMETRY,
                    src=previous.id,
                    dst=keyframe.id,
                    t_src_dst=se3_relative(previous.t_odom_base, keyframe.t_odom_base),
                    information=ODOM_INFORMATION,
                )
            )

        descriptor = keyframe.descriptor
        if descriptor is None:
            descriptor = scan_context_descriptor(keyframe.points)
            keyframe.descriptor = descriptor
            keyframe.descriptor_kind = DESCRIPTOR_KIND

        for candidate in self._index.query(
            descriptor, k=self.descriptor_k, query_id=keyframe.id
        ):
            target = self._keyframes.get(candidate.keyframe_id)
            if target is None:
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

    def optimize_and_render(self) -> BackendSnapshot | None:
        """Run the solver and rasterize occupancy. None if there is nothing new."""
        if not self._keyframes:
            return None
        optimized = self._graph.optimize()
        grids = render_occupancy(optimized, self._keyframes.values(), self.render)
        self._dirty = False
        self._new_since_optimize = 0
        return BackendSnapshot(
            optimized=optimized,
            grids=grids,
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
        "origins": origins,
        "common_poses": common_poses,
        "graphs": graphs,
        "accepted_closures": snapshot.accepted_closures,
        "inter_robot_closures": snapshot.inter_robot_closures,
        "stamp": snapshot.stamp,
    }
