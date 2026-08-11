"""Merged occupancy grid.

Per-robot grids arrive over HTTP, each in its OWN frame (SLAM puts the origin at
wherever that robot started). Transforms come either from config (`static`) or
from grid registration (`auto`). The merged grid is derived, diffed, and emitted
as bounding-box patches (architecture.md §5).
"""

from __future__ import annotations

import asyncio
import base64
import io
import math
import zlib
from typing import Any

import numpy as np

from .grid_meta import GridMeta
from .registration import Registration, register, register_3d, score_transform
from .scan_grid import ScanGridAccumulator

UNKNOWN = -1
FREE = 0
OCCUPIED = 100
# Cell value at or above which a grid is asserting "something is here". Matches
# registration.py, so the merge and the registration agree on what a wall is.
OCCUPIED_MIN = 50

# How a cell is resolved when members disagree about it.
#   majority  a cell is occupied unless strictly more members have OBSERVED it
#             free than occupied; unknown abstains. Clears stale ghosts.
#   occupied  any single occupied observation wins, forever. The pre-vote
#             behaviour, kept because a fleet of two cannot outvote anything and
#             some deployments would rather over-report obstacles.
MERGE_CONFLICT_MODES = ("majority", "occupied")

# Yaw search window around a prior, degrees. The wide window is for a configured
# start pose, which is only approximate; the narrow one is for a robot already
# registered, where the answer is known and only drifts.
PRIOR_WINDOW_DEG = 40.0
LOCKED_WINDOW_DEG = 8.0

# Consecutive ambiguous results before a registered robot is dropped from the
# merged map. One marginal frame is not evidence that a transform accepted
# seconds earlier has gone wrong, and treating it as such made membership
# oscillate at the map upload rate: the narrow search a lock enables can land on
# a slightly different yaw whose translation correlation is a near-tie, which
# unlocked the robot, which widened the next search, which accepted it again.
# The escalation from narrow back to wide search is still immediate — only the
# eviction waits, so a robot that genuinely stops matching still leaves after
# this many uploads rather than being held on a stale transform indefinitely.
REGISTRATION_MISS_LIMIT = 3

# Shared known area, as a fraction of the smaller map, below which the
# independent cslam cross-check has nothing to compare and stays silent.
CHECK_MIN_SUPPORT = 0.35


class MapService:
    def __init__(self, resolution: float = 0.05, size_m: float = 30.0) -> None:
        n = int(size_m / resolution)
        self.meta = GridMeta(resolution, n, n, -size_m / 2, -size_m / 2)
        self.merged = np.full((n, n), UNKNOWN, dtype=np.int8)
        self._prev = self.merged.copy()
        self.robot_grids: dict[str, tuple[GridMeta, np.ndarray]] = {}
        self.robot_revisions: dict[str, int] = {}
        self.transforms: dict[str, tuple[float, float, float]] = {}
        self.transform_priors: dict[str, tuple[float, float, float]] = {}
        self.registrations: dict[str, Registration] = {}
        # Raw height-band proposals and the modality used by the accepted result.
        # A cloud proposal is never applied directly: `_reregister` first scores
        # it against the independent occupancy grids.
        self.cloud_registrations: dict[str, Registration] = {}
        self.registration_sources: dict[str, str] = {}
        self.registration_rejections: dict[str, str] = {}
        # Robots whose transform came from an accepted registration and is still
        # trusted. Deliberately NOT the same thing as `locked_dyaw`: the lock is
        # only a hint that narrows the next yaw search, and one ambiguous frame
        # should widen that search without also evicting the robot from the map.
        # Conflating the two is what made the merged map flicker — see
        # REGISTRATION_MISS_LIMIT.
        self.registered: set[str] = set()
        # Consecutive ambiguous results per robot since its last decisive one.
        self.registration_misses: dict[str, int] = {}
        self.locked_dyaw: dict[str, float] = {}
        # Latest `slam_graph` per robot: keyframe count, inter-robot loop
        # closures, optimisation residual, and whether the collaborative back end
        # considers this robot part of the common frame. Only `cslam` mode reads
        # it; the other modes carry it for display.
        self.slam_graphs: dict[str, dict[str, Any]] = {}
        # cslam mode only: how far the independent grid correlation disagrees
        # with the collaborative back end's alignment, in metres and radians.
        self.cslam_disagreement: dict[str, tuple[float, float, bool]] = {}
        # Newest 3D cloud per robot, in that robot's own map frame, as an
        # (N, 3) float32 array of metres. Optional: a fleet on 2D SLAM never
        # sends one and the 3D view stays empty rather than wrong.
        self.robot_clouds: dict[str, np.ndarray] = {}
        # Per-robot raytraced grid, for robots whose SLAM stack has no
        # OccupancyGrid of its own — see scan_grid.py. Separate from
        # `robot_grids`, which `ingest_scan` feeds into via the same `ingest()`
        # a robot with a native grid uses.
        self._scan_grids: dict[str, ScanGridAccumulator] = {}
        # A ready-made merged grid supplied by a collaborative back end, already
        # in its common frame. When present in `cslam` mode there is nothing to
        # merge: the back end has done it, from its own keyframes at its own
        # optimised poses, and re-deriving it here could only reintroduce the
        # frame mismatch this exists to remove.
        self.global_grid: tuple[GridMeta, np.ndarray] | None = None
        # Each robot's pose as the collaborative back end reports it, already in
        # the common frame the merged map uses.
        self.common_poses: dict[str, dict[str, float]] = {}
        # Which collaborative common frame each robot is currently expressed in.
        # Two robots that have never met report different frames, and merging
        # across them would be meaningless.
        self.cslam_frames: dict[str, str] = {}
        self.merge_mode = "static"
        # See MERGE_CONFLICT_MODES and _remerge.
        self.merge_conflict = "majority"
        self.reference: str | None = None
        self.seq = 0
        # Serialises ingests when they are offloaded off the event loop, so two
        # adapters uploading at once cannot interleave inside the shared grids.
        self._ingest_lock = asyncio.Lock()

        # Precomputed world-cell centre coordinates, for the backward warp.
        cx = self.meta.origin_x + (np.arange(n) + 0.5) * resolution
        cy = self.meta.origin_y + (np.arange(n) + 0.5) * resolution
        self._wx, self._wy = np.meshgrid(cx, cy)

    # ------------------------------------------------------------------ setup

    def set_transform(self, robot_id: str, x: float, y: float, yaw: float) -> None:
        transform = (x, y, yaw)
        self.transforms[robot_id] = transform
        self.transform_priors[robot_id] = transform
        # Config order defines a stable reference. Without configured robots,
        # ingest() still selects the first map that actually arrives.
        if self.reference is None:
            self.reference = robot_id

    def reset_robot(self, robot_id: str | None = None) -> list[str]:
        """Discard accumulated map state for one robot, or for the whole fleet.

        Configured transform priors survive a reset: they describe deployment
        geometry, not accumulated sensor data. A targeted reset is intentionally
        backend-only; the robot's SLAM process keeps running and may repopulate
        its map immediately.
        """
        if robot_id is None:
            reset_ids = sorted(
                set(self.robot_grids)
                | set(self.robot_clouds)
                | set(self._scan_grids)
                | set(self.slam_graphs)
            )
            self.robot_grids.clear()
            self.robot_revisions.clear()
            self.registrations.clear()
            self.cloud_registrations.clear()
            self.registration_sources.clear()
            self.registration_rejections.clear()
            self.registered.clear()
            self.registration_misses.clear()
            self.locked_dyaw.clear()
            self.slam_graphs.clear()
            self.cslam_disagreement.clear()
            self.robot_clouds.clear()
            self._scan_grids.clear()
            self.global_grid = None
            self.common_poses.clear()
            self.cslam_frames.clear()
            self.transforms = dict(self.transform_priors)
            self.reference = next(iter(self.transform_priors), None)
            self._remerge()
            return reset_ids

        existed = any(
            robot_id in collection
            for collection in (
                self.robot_grids,
                self.robot_clouds,
                self._scan_grids,
                self.slam_graphs,
                self.common_poses,
                self.cslam_frames,
            )
        )
        self.robot_grids.pop(robot_id, None)
        self.robot_revisions.pop(robot_id, None)
        self.registrations.pop(robot_id, None)
        self.cloud_registrations.pop(robot_id, None)
        self.registration_sources.pop(robot_id, None)
        self.registration_rejections.pop(robot_id, None)
        self.registered.discard(robot_id)
        self.registration_misses.pop(robot_id, None)
        self.locked_dyaw.pop(robot_id, None)
        self.slam_graphs.pop(robot_id, None)
        self.cslam_disagreement.pop(robot_id, None)
        self.robot_clouds.pop(robot_id, None)
        self._scan_grids.pop(robot_id, None)
        self.common_poses.pop(robot_id, None)
        self.cslam_frames.pop(robot_id, None)

        prior = self.transform_priors.get(robot_id)
        if prior is None:
            self.transforms.pop(robot_id, None)
        else:
            self.transforms[robot_id] = prior

        if self.reference == robot_id and prior is None:
            self.reference = next(iter(self.robot_grids), None)
            # Every automatic registration was relative to the old reference.
            self.registrations.clear()
            self.cloud_registrations.clear()
            self.registration_sources.clear()
            self.registration_rejections.clear()
            self.registered.clear()
            self.registration_misses.clear()
            self.locked_dyaw.clear()

        self._remerge()
        return [robot_id] if existed else []

    async def reset_robot_async(self, robot_id: str | None = None) -> list[str]:
        """Serialise resets against concurrent adapter map uploads."""
        async with self._ingest_lock:
            return await asyncio.to_thread(self.reset_robot, robot_id)

    MODES = ("static", "auto", "cslam")

    def set_mode(self, mode: str) -> None:
        self.merge_mode = mode if mode in self.MODES else "static"

    def set_conflict_mode(self, mode: str) -> None:
        self.merge_conflict = mode if mode in MERGE_CONFLICT_MODES else "majority"

    def reset(self) -> None:
        """Forget every map, keeping the configuration that shapes them.

        Everything *derived from robots* goes: their grids, clouds, pose graphs,
        registrations and the merged result. Everything *chosen by the operator*
        stays — resolution, extent, merge mode, and the configured start poses.
        That split is the whole point: a reset puts the fleet back at the start of
        the same run, and re-reading the config would make it a different one.

        `transforms` is rebuilt from `transform_priors` rather than cleared,
        because in `static` mode the priors ARE the transforms and dropping them
        would silently move every robot to the origin. In `auto` and `cslam` the
        priors are only a search prior, and the estimate that refined them is
        gone with the maps it was computed from — which is correct: it described
        a map that no longer exists.

        `_prev` is deliberately NOT reset alongside `merged`. take_patch() diffs
        the two, so leaving the old grid in `_prev` makes the next patch describe
        exactly the cells that just went back to unknown, and the browser clears
        itself through the same path it draws through. Clearing both would leave
        the GUI showing a map the server no longer has.

        `reference` also stays. It names which robot defines the shared frame,
        which is a property of the fleet and not of any map — and when it came
        from config it is the reference the operator chose. If it was instead
        picked by whichever grid arrived first, keeping it merely keeps the same
        robot in the role across the reset.
        """
        self.merged = np.full_like(self.merged, UNKNOWN)
        self.robot_grids.clear()
        self.robot_revisions.clear()
        self.robot_clouds.clear()
        self.registrations.clear()
        self.registration_rejections.clear()
        self.registered.clear()
        self.registration_misses.clear()
        self.locked_dyaw.clear()
        self.slam_graphs.clear()
        self.cslam_disagreement.clear()
        self.common_poses.clear()
        self.cslam_frames.clear()
        self.global_grid = None
        self.transforms = dict(self.transform_priors)
        # A scan-fed robot has no grid of its own to drop — `_scan_grids` IS its
        # map, accumulated here. Leaving it means the whole pre-reset map is
        # re-ingested by the next scan that arrives, which is exactly the "old
        # map comes straight back" failure the reset ordering exists to prevent.
        # It costs nothing in the simulator, where nothing uses this path, and
        # makes the reset a no-op for every robot mapping from `map_cloud`.
        self._scan_grids.clear()
        # All of these describe registrations against grids that no longer exist.
        self.cloud_registrations.clear()
        self.registration_sources.clear()
        self.registered.clear()
        self.registration_misses.clear()

    def set_cloud(self, robot_id: str, points: np.ndarray) -> None:
        """Store a cloud and refresh cloud-assisted registration when applicable."""
        self.robot_clouds[robot_id] = points
        if self.merge_mode != "auto" or self.reference is None:
            return

        # A new reference cloud changes every pair; a new moving cloud changes
        # only its own pair.  Registration runs under the same ingest lock as map
        # updates (see `set_cloud_async`), so these shared dictionaries cannot be
        # observed half-updated by concurrent uploads.
        targets = (
            [rid for rid in self.robot_clouds if rid != self.reference]
            if robot_id == self.reference
            else [robot_id]
        )
        for rid in targets:
            if rid not in self.robot_grids:
                continue
            self._update_cloud_registration(rid)
            self._reregister(rid)
        self._remerge()

    async def set_cloud_async(self, robot_id: str, points: np.ndarray) -> None:
        """Cloud storage/registration off the event loop, like grid ingestion."""
        async with self._ingest_lock:
            await asyncio.to_thread(self.set_cloud, robot_id, points)

    def merged_cloud(self) -> tuple[np.ndarray, np.ndarray, list[str]]:
        """Every member robot's cloud, rotated into the merged frame.

        Returns points (N, 3) float32, a per-point robot index (N,) uint8, and
        the robot ids those indices refer to. Only robots that are actually in
        the shared frame contribute — the same rule the 2D merge uses, for the
        same reason: drawing an unregistered robot's cloud in the shared frame
        would render a guess as though it were a measurement.
        """
        members = self.global_members()
        chunks: list[np.ndarray] = []
        indices: list[np.ndarray] = []
        names: list[str] = []
        for rid in sorted(self.robot_clouds):
            if rid not in members:
                continue
            points = self.robot_clouds[rid]
            if points.size == 0:
                continue
            tx, ty, yaw = self.transforms.get(rid, (0.0, 0.0, 0.0))
            c, s = math.cos(yaw), math.sin(yaw)
            out = np.empty_like(points)
            # Planar rotation only: the merge frame is SE(2), so z passes through.
            out[:, 0] = tx + points[:, 0] * c - points[:, 1] * s
            out[:, 1] = ty + points[:, 0] * s + points[:, 1] * c
            out[:, 2] = points[:, 2]
            chunks.append(out)
            indices.append(np.full(len(out), len(names), dtype=np.uint8))
            names.append(rid)
        if not chunks:
            return (
                np.zeros((0, 3), dtype=np.float32),
                np.zeros(0, dtype=np.uint8),
                [],
            )
        return np.concatenate(chunks), np.concatenate(indices), names

    def set_common_pose(self, robot_id: str, pose: dict[str, float]) -> None:
        self.common_poses[robot_id] = {
            "x": float(pose.get("x", 0.0)),
            "y": float(pose.get("y", 0.0)),
            "yaw": float(pose.get("yaw", 0.0)),
        }

    def _common_to_world(self) -> tuple[float, float, float]:
        """Where the collaborative back end's common frame sits in the world.

        cslam anchors its common frame at the ORIGIN ROBOT's first keyframe —
        that robot's start pose — not at the world origin. Everything it reports
        is therefore offset by exactly that pose, which measured as an 8.8 m
        error for a robot spawned at x = -9. The configured start pose of the
        reference robot is the same information `merge_mode: static` already
        relies on, so use it to put the common frame back in the world.
        """
        if self.reference is None:
            return (0.0, 0.0, 0.0)
        return self.transform_priors.get(self.reference, (0.0, 0.0, 0.0))

    def common_pose(self, robot_id: str) -> dict[str, float] | None:
        """The back end's own answer for this robot, if it is usable.

        Only in `cslam` mode, only for robots the back end has actually placed
        in the common frame, and only when that frame is the one the merged map
        is drawn in — otherwise this would report a pose from a different
        cluster's frame, which is worse than a transformed one.
        """
        if self.merge_mode != "cslam":
            return None
        if robot_id not in self.global_members():
            return None
        pose = self.common_poses.get(robot_id)
        if pose is None:
            return None
        tx, ty, tyaw = self._common_to_world()
        c, s = math.cos(tyaw), math.sin(tyaw)
        return {
            "x": tx + pose["x"] * c - pose["y"] * s,
            "y": ty + pose["x"] * s + pose["y"] * c,
            "yaw": self._wrap_yaw(pose["yaw"] + tyaw),
        }

    def set_global_grid(self, meta: GridMeta, cells: np.ndarray) -> None:
        """Adopt a collaborative back end's own merged grid wholesale.

        Only meaningful in `cslam` mode. The grid arrives in the back end's
        common frame — the same frame `set_cslam_origin` places robots in — so
        geometry and poses agree by construction. That is the entire fix for the
        11-16 m error: previously the grids came from RTAB-Map and the poses
        from cslam, two independently optimised trajectories that disagreed by
        metres.
        """
        if self.merge_mode != "cslam":
            return
        self.global_grid = (meta, cells.astype(np.int8, copy=True))
        self._remerge()

    def set_cslam_origin(
        self, robot_id: str, x: float, y: float, yaw: float, frame: str
    ) -> None:
        """Adopt the collaborative back end's transform for one robot.

        This is what makes `cslam` mode different in kind from `auto`. In `auto`
        the transform is *re-estimated* from two finished occupancy grids, long
        after both robots have already been wrong; here it falls out of the
        inter-robot loop closures themselves, so one robot's observations
        actually correct another's.

        Only applied in `cslam` mode — in `static` the operator's configured
        priors win, and in `auto` this would silently override the registration
        the operator asked for. `frame` records which cluster the robot belongs
        to: a fleet that has split into two groups that never met has two
        different common frames, and only robots sharing one may be drawn
        together. Robots in a minority cluster are held out rather than being
        placed relative to a frame that means something else.
        """
        self.cslam_frames[robot_id] = frame
        if self.merge_mode != "cslam":
            return
        self.transforms[robot_id] = (x, y, yaw)
        if self.reference is None:
            self.reference = robot_id
        self._remerge()

    def cslam_majority_frame(self) -> str | None:
        """The common frame most of the fleet agrees on, if any."""
        counts: dict[str, int] = {}
        for rid, frame in self.cslam_frames.items():
            if frame and self.in_common_frame(rid):
                counts[frame] = counts.get(frame, 0) + 1
        if not counts:
            return None
        return max(counts.items(), key=lambda kv: kv[1])[0]

    def set_slam_graph(self, robot_id: str, graph: dict[str, Any]) -> None:
        """Record a robot's view of the collaborative pose graph.

        This is what a robot running Swarm-SLAM reports about itself: how many
        keyframes it holds, which other robots it has closed a loop with, the
        optimiser's residual, and whether it has been placed in the common frame
        yet. In `cslam` mode the last of those decides membership of the merged
        map, so a robot that has not yet met anyone is visibly absent rather than
        silently overlaid at a guessed pose.
        """
        self.slam_graphs[robot_id] = graph

    def robot_to_world(self, robot_id: str, pose: dict[str, float]) -> dict[str, float]:
        """Transform a pose from one robot's SLAM frame into the merged frame."""
        tx, ty, yaw = self.transforms.get(robot_id, (0.0, 0.0, 0.0))
        c, s = math.cos(yaw), math.sin(yaw)
        x, y = float(pose["x"]), float(pose["y"])
        result = dict(pose)
        result["x"] = tx + x * c - y * s
        result["y"] = ty + x * s + y * c
        if "yaw" in pose:
            result["yaw"] = self._wrap_yaw(float(pose["yaw"]) + yaw)
        return result

    def world_to_robot(self, robot_id: str, pose: dict[str, float]) -> dict[str, float]:
        """Transform a merged-frame pose into one robot's SLAM/navigation frame."""
        tx, ty, yaw = self.transforms.get(robot_id, (0.0, 0.0, 0.0))
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = float(pose["x"]) - tx, float(pose["y"]) - ty
        result = dict(pose)
        result["x"] = dx * c + dy * s
        result["y"] = -dx * s + dy * c
        if "yaw" in pose:
            result["yaw"] = self._wrap_yaw(float(pose["yaw"]) - yaw)
        return result

    @staticmethod
    def _wrap_yaw(yaw: float) -> float:
        return (yaw + math.pi) % (2 * math.pi) - math.pi

    # ---------------------------------------------------------------- ingest

    def ingest(self, robot_id: str, meta: GridMeta, cells: np.ndarray) -> None:
        self.robot_grids[robot_id] = (meta, cells)
        self.robot_revisions[robot_id] = self.robot_revisions.get(robot_id, 0) + 1
        if self.reference is None:
            self.reference = robot_id
        if self.merge_mode == "auto":
            self._reregister(robot_id)
        elif self.merge_mode == "cslam":
            self._cslam_check(robot_id)
        self._remerge()

    async def ingest_async(self, robot_id: str, meta: GridMeta, cells: np.ndarray) -> None:
        """Ingest without stalling the event loop.

        Registration is hundreds of milliseconds of numpy. Called inline from an
        async request handler it blocks every websocket on the server, so
        telemetry visibly stutters each time a robot uploads a map.
        """
        async with self._ingest_lock:
            await asyncio.to_thread(self.ingest, robot_id, meta, cells)

    def ingest_scan(
        self, robot_id: str, origin_x: float, origin_y: float, points_xy: np.ndarray
    ) -> None:
        """Accumulate one lidar scan into a raytraced grid, then ingest it.

        For a robot with no `OccupancyGrid` publisher of its own: the adapter
        forwards already-registered scan points plus the sensor position
        instead of a finished grid, `ScanGridAccumulator` builds one up scan by
        scan, and it is fed into exactly the same `ingest()` a robot that DOES
        publish its own grid uses — so registration, merging and everything
        downstream neither knows nor cares which kind of robot this is.
        """
        acc = self._scan_grids.get(robot_id)
        if acc is None:
            # Size the window from the configured merged extent rather than the
            # accumulator's own default. They are the same number today only by
            # coincidence — raise `map.size_m` for a larger building and a
            # hardcoded 40 m window would silently drop everything beyond it,
            # with nothing anywhere to say a robot had driven off its own map.
            acc = ScanGridAccumulator(
                origin_x,
                origin_y,
                resolution=self.meta.resolution,
                size_m=self.meta.width * self.meta.resolution,
            )
            self._scan_grids[robot_id] = acc
        acc.integrate(origin_x, origin_y, points_xy)
        self.ingest(robot_id, acc.meta, acc.cells)

    async def ingest_scan_async(
        self, robot_id: str, origin_x: float, origin_y: float, points_xy: np.ndarray
    ) -> None:
        """`ingest_scan`, off the event loop — see `ingest_async`."""
        async with self._ingest_lock:
            await asyncio.to_thread(self.ingest_scan, robot_id, origin_x, origin_y, points_xy)

    # ------------------------------------------------------------ registration

    def _reregister(self, robot_id: str) -> None:
        """Estimate this robot's transform against the reference robot's grid."""
        if robot_id == self.reference:
            self.transforms.setdefault(robot_id, (0.0, 0.0, 0.0))
            return
        ref = self.robot_grids.get(self.reference or "")
        mov = self.robot_grids.get(robot_id)
        if ref is None or mov is None:
            return

        ref_meta, ref_cells = ref
        mov_meta, mov_cells = mov

        # Compose with the reference's own world transform.
        rx, ry, ryaw = self.transforms.get(self.reference or "", (0.0, 0.0, 0.0))

        # Narrow the rotation search when the answer is already approximately
        # known. Once locked this is a refinement, not a fresh global search, and
        # a configured start pose is a legitimate window because a result more
        # than 30 deg from it is rejected below regardless.
        locked = self.locked_dyaw.get(robot_id)
        configured = self.transform_priors.get(robot_id)
        if locked is not None:
            yaw_prior: float | None = locked
            window = LOCKED_WINDOW_DEG
        elif configured is not None:
            yaw_prior = self._wrap_yaw(configured[2] - ryaw)
            window = PRIOR_WINDOW_DEG
        else:
            yaw_prior = None  # unknown start: nothing to narrow the sweep with
            window = PRIOR_WINDOW_DEG

        grid_result = register(
            ref_cells, (ref_meta.resolution, ref_meta.origin_x, ref_meta.origin_y),
            mov_cells, (mov_meta.resolution, mov_meta.origin_x, mov_meta.origin_y),
            yaw_prior=yaw_prior,
            yaw_window_deg=window,
        )
        result = grid_result
        source = "grid"

        # Prefer a geometrically unambiguous cloud proposal only after the 2D
        # grids independently confirm its wall overlap and shared known area.
        # The cloud contributes the rival-translation/yaw tests; the grids
        # contribute score/overlap/support.  Neither can accept a transform alone.
        cloud = self.cloud_registrations.get(robot_id)
        if cloud is None and robot_id in self.robot_clouds:
            self._update_cloud_registration(robot_id)
            cloud = self.cloud_registrations.get(robot_id)
        if cloud is not None:
            score, overlap, support = score_transform(
                ref_cells,
                (ref_meta.resolution, ref_meta.origin_x, ref_meta.origin_y),
                mov_cells,
                (mov_meta.resolution, mov_meta.origin_x, mov_meta.origin_y),
                cloud.dx,
                cloud.dy,
                cloud.dyaw,
            )
            cloud_validated = Registration(
                dx=cloud.dx,
                dy=cloud.dy,
                dyaw=cloud.dyaw,
                score=score,
                overlap=overlap,
                ratio=cloud.ratio,
                yaw_ratio=cloud.yaw_ratio,
                support=support,
            )
            if cloud_validated.confident:
                result = cloud_validated
                source = "pointcloud+grid"

        self.registrations[robot_id] = result
        self.registration_sources[robot_id] = source
        if not result.confident:
            # A locked robot that stops matching has to go back to searching
            # widely, or a bad lock is self-perpetuating.
            self.locked_dyaw.pop(robot_id, None)
            misses = self.registration_misses.get(robot_id, 0) + 1
            self.registration_misses[robot_id] = misses
            if robot_id in self.registered and misses < REGISTRATION_MISS_LIMIT:
                # Hold: keep the last accepted transform and stay in the merged
                # map. The wider search the dropped lock just enabled gets a
                # chance to confirm the robot before it is evicted, which is what
                # stops one marginal frame from blanking the operator's map.
                return
            self.registration_rejections[robot_id] = (
                "ambiguous grid and point-cloud match"
                if cloud is not None
                else "ambiguous occupancy match"
            )
            self.registered.discard(robot_id)
            return
        c, s = math.cos(ryaw), math.sin(ryaw)
        wx = rx + result.dx * c - result.dy * s
        wy = ry + result.dx * s + result.dy * c
        candidate_yaw = self._wrap_yaw(ryaw + result.dyaw)

        # Repetitive rooms can produce a strong but physically impossible FFT
        # peak. When deployment config provides an approximate start pose, use
        # it as a safety prior instead of allowing a map match to teleport or
        # flip a robot. Deployments with genuinely unknown starts omit the prior.
        prior = self.transform_priors.get(robot_id)
        if prior is not None:
            translation_error = math.hypot(wx - prior[0], wy - prior[1])
            yaw_error = abs(self._wrap_yaw(candidate_yaw - prior[2]))
            if translation_error > 2.0 or yaw_error > math.radians(30.0):
                self.registration_rejections[robot_id] = (
                    f"outside configured prior ({translation_error:.2f} m, "
                    f"{math.degrees(yaw_error):.1f} deg)"
                )
                self.transforms[robot_id] = prior
                self.locked_dyaw.pop(robot_id, None)
                # No hysteresis here, unlike the ambiguous case above: this is a
                # confident match that contradicts known deployment geometry, so
                # it is positive evidence against whatever transform was held —
                # not the absence of evidence for it.
                self.registered.discard(robot_id)
                self.registration_misses[robot_id] = 0
                return

        self.registration_rejections.pop(robot_id, None)
        self.transforms[robot_id] = (wx, wy, candidate_yaw)
        self.locked_dyaw[robot_id] = result.dyaw
        self.registered.add(robot_id)
        self.registration_misses[robot_id] = 0

    def _update_cloud_registration(self, robot_id: str) -> None:
        """Estimate ``T_reference_robot`` from corresponding 3D height slices."""
        if robot_id == self.reference or self.reference is None:
            return
        ref_points = self.robot_clouds.get(self.reference)
        mov_points = self.robot_clouds.get(robot_id)
        if ref_points is None or mov_points is None:
            self.cloud_registrations.pop(robot_id, None)
            return

        ref_points = ref_points[np.isfinite(ref_points).all(axis=1)]
        mov_points = mov_points[np.isfinite(mov_points).all(axis=1)]

        # Registration cannot use geometry outside the two local map windows and
        # neither can the final compositor.  Clipping here also prevents one bad
        # sensor outlier (or a malformed upload scale) from expanding the FFT
        # raster to an unbounded allocation.
        ref_meta = self.robot_grids[self.reference][0]
        mov_meta = self.robot_grids[robot_id][0]

        def inside_grid(points: np.ndarray, meta: GridMeta) -> np.ndarray:
            max_x = meta.origin_x + meta.width * meta.resolution
            max_y = meta.origin_y + meta.height * meta.resolution
            return points[
                (points[:, 0] >= meta.origin_x)
                & (points[:, 0] < max_x)
                & (points[:, 1] >= meta.origin_y)
                & (points[:, 1] < max_y)
            ]

        ref_points = inside_grid(ref_points, ref_meta)
        mov_points = inside_grid(mov_points, mov_meta)
        if len(ref_points) < 40 or len(mov_points) < 40:
            self.cloud_registrations.pop(robot_id, None)
            return

        # Both clouds need one common raster extent.  Derive it from their local
        # metric coordinates instead of the merged-map origin: independent SLAM
        # frames can have very different origins even when the rooms overlap.
        xy_lo = np.minimum(
            ref_points[:, :2].min(axis=0), mov_points[:, :2].min(axis=0)
        )
        xy_hi = np.maximum(
            ref_points[:, :2].max(axis=0), mov_points[:, :2].max(axis=0)
        )
        resolution = self.meta.resolution
        origin = np.floor((xy_lo - 0.5) / resolution) * resolution
        size = np.ceil((xy_hi + 0.5 - origin) / resolution).astype(int) + 1

        ryaw = self.transforms.get(self.reference, (0.0, 0.0, 0.0))[2]
        locked = self.locked_dyaw.get(robot_id)
        configured = self.transform_priors.get(robot_id)
        if locked is not None:
            yaw_prior: float | None = locked
            window = LOCKED_WINDOW_DEG
        elif configured is not None:
            yaw_prior = self._wrap_yaw(configured[2] - ryaw)
            window = PRIOR_WINDOW_DEG
        else:
            yaw_prior = None
            window = PRIOR_WINDOW_DEG

        self.cloud_registrations[robot_id] = register_3d(
            ref_points,
            mov_points,
            resolution,
            (int(size[0]), int(size[1]), float(origin[0]), float(origin[1])),
            yaw_prior=yaw_prior,
            yaw_window_deg=window,
        )

    # --------------------------------------------------- collaborative back end

    def in_common_frame(self, robot_id: str) -> bool:
        """Has the collaborative back end placed this robot in the shared frame?

        The reference robot defines the frame, so it is in it by construction.
        Everyone else has to have closed an inter-robot loop, and says so in its
        `slam_graph`. Absent that report the answer is no: an unregistered robot
        overlaid at its configured start pose is exactly the confident-but-wrong
        merge the rejection tests exist to prevent.
        """
        if robot_id == self.reference:
            return True
        return bool(self.slam_graphs.get(robot_id, {}).get("in_common_frame"))

    def _cslam_check(self, robot_id: str) -> None:
        """Score the collaborative alignment against independent evidence.

        In `cslam` mode the transforms are not estimated here — the robots
        already publish poses and grids in one frame, and `mapsvc` is bookkeeping.
        That removes this module's job and leaves it a better one: grid
        correlation is now an *independent* check on the pose graph, using
        evidence (occupied and free cells over the whole map) that the loop
        closures did not use.

        Both grids are already in the common frame, so a correct alignment must
        correlate at approximately identity. Whatever offset the search does find
        is the disagreement, and it is reported rather than applied. Applying it
        would defeat the point: the pose graph, not this, is the estimator.
        """
        if robot_id == self.reference or not self.in_common_frame(robot_id):
            self.cslam_disagreement.pop(robot_id, None)
            return
        ref = self.robot_grids.get(self.reference or "")
        mov = self.robot_grids.get(robot_id)
        if ref is None or mov is None:
            return
        ref_meta, ref_cells = ref
        mov_meta, mov_cells = mov
        result = register(
            ref_cells, (ref_meta.resolution, ref_meta.origin_x, ref_meta.origin_y),
            mov_cells, (mov_meta.resolution, mov_meta.origin_x, mov_meta.origin_y),
            yaw_prior=0.0,
            yaw_window_deg=LOCKED_WINDOW_DEG,
        )
        self.registrations[robot_id] = result
        # Gated on `support` alone, and deliberately not on `confident`.
        #
        # The rejection tests exist to stop a wrong transform being *applied*.
        # Nothing is applied here, so the question is only whether the two maps
        # overlap enough for the comparison to mean anything — which is exactly
        # what `support` measures. `ratio` and `yaw_ratio` ask whether rival
        # hypotheses can be told apart, which is the right question when
        # choosing a transform and the wrong one when measuring a residual
        # against a known prior: a symmetric room makes rotations ambiguous
        # without making the residual unmeasurable.
        #
        # `confident` is still carried through to the operator rather than
        # discarded, because a disagreement drawn from an ambiguous correlation
        # deserves to be labelled as one.
        if result.support >= CHECK_MIN_SUPPORT:
            self.cslam_disagreement[robot_id] = (
                math.hypot(result.dx, result.dy),
                abs(result.dyaw),
                result.confident,
            )
        else:
            self.cslam_disagreement.pop(robot_id, None)

    # --------------------------------------------------------------- merging

    def global_members(self) -> set[str]:
        """Robots whose grids are genuinely in the shared map frame.

        Static mode is an operator assertion that all configured transforms are
        valid.  Auto mode is stricter: a configured start pose is only a search
        prior, not evidence of registration.  A global map exists once at least
        one robot has been accepted against the reference.
        """
        if self.merge_mode == "static":
            # An unconfigured reference may safely define the world frame at
            # identity by construction.  Every additional robot needs an explicit
            # transform; silently defaulting all of them to identity is the exact
            # failure that overlaid independent hardware maps and clouds.
            members = {rid for rid in self.robot_grids if rid in self.transforms}
            if self.reference in self.robot_grids:
                members.add(self.reference)
            return members
        if self.merge_mode == "cslam":
            # Membership is the collaborative back end's call, not a correlation
            # score: a robot is in the map once it has actually closed a loop
            # with the fleet AND is expressed in the same common frame as the
            # majority. A fleet that has split into two groups which never met
            # has two unrelated frames, and overlaying them would place robots
            # confidently in the wrong building.
            majority = self.cslam_majority_frame()
            return {
                rid
                for rid in self.robot_grids
                if self.in_common_frame(rid)
                and (majority is None or self.cslam_frames.get(rid) == majority)
            }
        # `registered`, not the latest result's `confident` flag: a robot that has
        # been accepted keeps its place through a few ambiguous frames, and the
        # merged map it contributes to must not blink out under it meanwhile.
        accepted = {
            rid
            for rid in self.registered
            if rid not in self.registration_rejections
        }
        if not accepted or self.reference is None:
            return set()
        return accepted | {self.reference}

    def _warp(self, meta: GridMeta, cells: np.ndarray,
              tf: tuple[float, float, float]) -> np.ndarray:
        """Backward-warp one robot grid into the merged frame (nearest neighbour).

        For every world cell we compute the source cell, rather than scattering
        source cells forward — that leaves no holes when rotating.
        """
        tx, ty, yaw = tf
        c, s = math.cos(-yaw), math.sin(-yaw)
        dx = self._wx - tx
        dy = self._wy - ty
        rx = dx * c - dy * s
        ry = dx * s + dy * c

        gx = np.floor((rx - meta.origin_x) / meta.resolution).astype(np.int64)
        gy = np.floor((ry - meta.origin_y) / meta.resolution).astype(np.int64)
        valid = (gx >= 0) & (gx < meta.width) & (gy >= 0) & (gy < meta.height)

        out = np.full(self.merged.shape, UNKNOWN, dtype=np.int8)
        out[valid] = cells[gy[valid], gx[valid]]
        return out

    def _remerge(self) -> None:
        """Combine every member's grid, resolving disagreements by vote.

        Unless a collaborative back end has supplied a finished grid, in which
        case that IS the map — see set_global_grid.

        The old rule was `maximum`: occupied beat free unconditionally. That is
        the safe reading of one robot's map, and the wrong reading of four,
        because it makes a stale cell immortal. A robot that once drove past
        another robot records it as a wall; every other robot afterwards drives
        straight through that spot and reports free space; and the ghost outvotes
        all of them forever because it is the larger number.

        `majority` counts only ACTUAL observations — unknown cells abstain, they
        do not vote for free. A cell is occupied unless strictly more members
        have seen it empty than have seen it filled. So a real obstacle one robot
        alone has seen is kept (1 vs 0), and a ghost is erased once two robots
        have driven through it (1 vs 2).

        Ties go to occupied, deliberately: with two robots disagreeing there is
        no majority to be had, and telling an operator a space is clear when one
        robot says otherwise is the worse error. The limitation is real and worth
        stating — on a two-robot fleet a ghost from one of them can never be
        outvoted.
        """
        if self.merge_mode == "cslam" and self.global_grid is not None:
            # Same offset as the poses: the back end's grid is in its common
            # frame, anchored at the reference robot's start pose.
            meta, cells = self.global_grid
            self.merged = self._warp(meta, cells, self._common_to_world())
            return

        members = self.global_members()
        warped_grids = [
            self._warp(meta, cells, self.transforms.get(rid, (0.0, 0.0, 0.0)))
            for rid, (meta, cells) in self.robot_grids.items()
            if rid in members
        ]
        if not warped_grids:
            self.merged = np.full_like(self.merged, UNKNOWN)
            return

        if self.merge_conflict == "occupied":
            out = np.full_like(self.merged, UNKNOWN)
            for warped in warped_grids:
                known = warped != UNKNOWN
                out[known] = np.maximum(out[known], warped[known])
            self.merged = out
            return

        occupied_votes = np.zeros(self.merged.shape, dtype=np.int16)
        free_votes = np.zeros(self.merged.shape, dtype=np.int16)
        for warped in warped_grids:
            known = warped != UNKNOWN
            occupied_votes += (known & (warped >= OCCUPIED_MIN)).astype(np.int16)
            free_votes += (known & (warped < OCCUPIED_MIN)).astype(np.int16)

        out = np.full_like(self.merged, UNKNOWN)
        observed = (occupied_votes + free_votes) > 0
        out[observed] = np.where(
            occupied_votes[observed] >= free_votes[observed], OCCUPIED, FREE
        )
        self.merged = out

    # --------------------------------------------------------------- output

    def take_patch(self) -> dict[str, Any] | None:
        """Bounding box of everything that changed since the last call."""
        diff = self.merged != self._prev
        if not diff.any():
            return None
        ys, xs = np.where(diff)
        y0, y1 = int(ys.min()), int(ys.max()) + 1
        x0, x1 = int(xs.min()), int(xs.max()) + 1
        sub = np.ascontiguousarray(self.merged[y0:y1, x0:x1])
        self._prev = self.merged.copy()
        self.seq += 1
        return {
            "type": "map_patch",
            "seq": self.seq,
            "resolution": self.meta.resolution,
            "origin": {"x": self.meta.origin_x, "y": self.meta.origin_y},
            "x0": x0,
            "y0": y0,
            "w": x1 - x0,
            "h": y1 - y0,
            "data": base64.b64encode(zlib.compress(sub.tobytes())).decode(),
        }

    # Unknown / free / occupied. Must stay in step with the palette in
    # ui/src/lib/stores/mapstore.svelte.ts: the same map reaches the browser as
    # this PNG on connect and as int8 patches afterwards, so a mismatch shows up
    # as a seam after a reload. Unknown is well clear of white because at a
    # glance "explored and empty" and "never seen" are the two states an
    # operator most needs to tell apart.
    UNKNOWN_RGB = (214, 218, 224)
    FREE_RGB = (255, 255, 255)
    OCCUPIED_RGB = (52, 58, 68)

    @classmethod
    def _grid_png(cls, meta: GridMeta, cells: np.ndarray) -> bytes:
        """Render one occupancy grid in the frontend's top-down orientation."""
        from PIL import Image

        img = np.zeros((meta.height, meta.width, 3), dtype=np.uint8)
        img[...] = cls.UNKNOWN_RGB
        img[cells == 0] = cls.FREE_RGB
        img[cells >= 50] = cls.OCCUPIED_RGB
        img = np.flipud(img)
        buf = io.BytesIO()
        Image.fromarray(img).save(buf, format="PNG", optimize=True)
        return buf.getvalue()

    def as_png(self) -> bytes:
        """Full grid for GET /api/map — browser-cacheable, survives reload."""
        return self._grid_png(self.meta, self.merged)

    def local_info(self, robot_id: str) -> dict[str, Any] | None:
        grid = self.robot_grids.get(robot_id)
        if grid is None:
            return None
        meta, _ = grid
        return meta.as_dict(self.robot_revisions.get(robot_id, 0))

    def local_png(self, robot_id: str) -> bytes | None:
        grid = self.robot_grids.get(robot_id)
        if grid is None:
            return None
        meta, cells = grid
        return self._grid_png(meta, cells)

    def status(self) -> dict[str, Any]:
        members = self.global_members()
        return {
            "mode": self.merge_mode,
            "merge_conflict": self.merge_conflict,
            "reference": self.reference,
            "transforms": {
                k: {"x": round(v[0], 3), "y": round(v[1], 3), "yaw": round(v[2], 4)}
                for k, v in self.transforms.items()
            },
            "registrations": {
                k: {
                    "score": round(r.score, 4),
                    "overlap": r.overlap,
                    "ratio": round(r.ratio, 3),
                    "yaw_ratio": round(r.yaw_ratio, 3),
                    "support": round(r.support, 3),
                    "confident": r.confident,
                    # Whether this robot is in the merged map right now, which
                    # during a hold is deliberately not the same as whether its
                    # newest result was confident. `misses` is what shows the
                    # difference: non-zero means the map is being held together
                    # on an older transform.
                    "accepted": k in members,
                    "misses": self.registration_misses.get(k, 0),
                    "rejection": self.registration_rejections.get(k),
                    "dyaw_deg": round(math.degrees(r.dyaw), 2),
                    "locked": k in self.locked_dyaw,
                    "source": self.registration_sources.get(k, "grid"),
                }
                for k, r in self.registrations.items()
            },
            "global_members": sorted(members),
            "view_by_robot": {
                rid: "global" if rid in members else "local"
                for rid in self.robot_grids
            },
            "slam_graphs": self.slam_graphs,
            # cslam mode only. Grid correlation no longer produces the transform,
            # so what it reports is how far it disagrees with the pose graph —
            # an independent check, using evidence the loop closures did not use.
            "cslam_disagreement": {
                rid: {
                    "metres": round(d, 3),
                    "degrees": round(math.degrees(a), 2),
                    # False means the correlation could not separate rival
                    # hypotheses — a repetitive building, not necessarily a bad
                    # alignment. Read the number as indicative, not as a verdict.
                    "confident": confident,
                }
                for rid, (d, a, confident) in self.cslam_disagreement.items()
            },
        }


map_service = MapService()
