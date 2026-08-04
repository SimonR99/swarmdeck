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
from dataclasses import dataclass
from typing import Any

import numpy as np

from .registration import Registration, register

UNKNOWN = -1

# Yaw search window around a prior, degrees. The wide window is for a configured
# start pose, which is only approximate; the narrow one is for a robot already
# registered, where the answer is known and only drifts.
PRIOR_WINDOW_DEG = 40.0
LOCKED_WINDOW_DEG = 8.0

# Shared known area, as a fraction of the smaller map, below which the
# independent cslam cross-check has nothing to compare and stays silent.
CHECK_MIN_SUPPORT = 0.35


@dataclass
class GridMeta:
    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float

    def as_dict(self, seq: int = 0) -> dict[str, Any]:
        return {
            "resolution": self.resolution,
            "width": self.width,
            "height": self.height,
            "origin": {"x": self.origin_x, "y": self.origin_y},
            "seq": seq,
        }


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
        self.registration_rejections: dict[str, str] = {}
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
        self.merge_mode = "static"
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

    MODES = ("static", "auto", "cslam")

    def set_mode(self, mode: str) -> None:
        self.merge_mode = mode if mode in self.MODES else "static"

    def set_cloud(self, robot_id: str, points: np.ndarray) -> None:
        """Store one robot's 3D cloud, already voxel-downsampled by the adapter."""
        self.robot_clouds[robot_id] = points

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

        result = register(
            ref_cells, (ref_meta.resolution, ref_meta.origin_x, ref_meta.origin_y),
            mov_cells, (mov_meta.resolution, mov_meta.origin_x, mov_meta.origin_y),
            yaw_prior=yaw_prior,
            yaw_window_deg=window,
        )
        self.registrations[robot_id] = result
        if not result.confident:
            self.registration_rejections[robot_id] = "ambiguous occupancy match"
            # A locked robot that stops matching has to go back to searching
            # widely, or a bad lock is self-perpetuating.
            self.locked_dyaw.pop(robot_id, None)
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
                return

        self.registration_rejections.pop(robot_id, None)
        self.transforms[robot_id] = (wx, wy, candidate_yaw)
        self.locked_dyaw[robot_id] = result.dyaw

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
            return set(self.robot_grids)
        if self.merge_mode == "cslam":
            # Membership is the collaborative back end's call, not a correlation
            # score: a robot is in the map once it has actually closed a loop
            # with the fleet.
            return {rid for rid in self.robot_grids if self.in_common_frame(rid)}
        accepted = {
            rid
            for rid, result in self.registrations.items()
            if result.confident and rid not in self.registration_rejections
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
        """Occupied wins over free; free wins over unknown."""
        out = np.full_like(self.merged, UNKNOWN)
        members = self.global_members()
        for rid, (meta, cells) in self.robot_grids.items():
            if rid not in members:
                continue
            tf = self.transforms.get(rid, (0.0, 0.0, 0.0))
            warped = self._warp(meta, cells, tf)
            known = warped != UNKNOWN
            out[known] = np.maximum(out[known], warped[known])
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
                    "accepted": r.confident and k not in self.registration_rejections,
                    "rejection": self.registration_rejections.get(k),
                    "dyaw_deg": round(math.degrees(r.dyaw), 2),
                    "locked": k in self.locked_dyaw,
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
