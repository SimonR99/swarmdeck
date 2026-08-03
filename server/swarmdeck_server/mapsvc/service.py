"""Merged occupancy grid.

Per-robot grids arrive over HTTP, each in its OWN frame (SLAM puts the origin at
wherever that robot started). Transforms come either from config (`static`) or
from grid registration (`auto`). The merged grid is derived, diffed, and emitted
as bounding-box patches (architecture.md §5).
"""

from __future__ import annotations

import base64
import io
import math
import zlib
from dataclasses import dataclass
from typing import Any

import numpy as np

from .registration import Registration, register

UNKNOWN = -1


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
        self.merge_mode = "static"
        self.reference: str | None = None
        self.seq = 0

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

    def set_mode(self, mode: str) -> None:
        self.merge_mode = mode if mode in ("static", "auto") else "static"

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
        self._remerge()

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
        result = register(
            ref_cells, (ref_meta.resolution, ref_meta.origin_x, ref_meta.origin_y),
            mov_cells, (mov_meta.resolution, mov_meta.origin_x, mov_meta.origin_y),
        )
        self.registrations[robot_id] = result
        if not result.confident:
            self.registration_rejections[robot_id] = "ambiguous occupancy match"
            return

        # Compose with the reference's own world transform.
        rx, ry, ryaw = self.transforms.get(self.reference or "", (0.0, 0.0, 0.0))
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
                return

        self.registration_rejections.pop(robot_id, None)
        self.transforms[robot_id] = (wx, wy, candidate_yaw)

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

    @staticmethod
    def _grid_png(meta: GridMeta, cells: np.ndarray) -> bytes:
        """Render one occupancy grid in the frontend's top-down orientation."""
        from PIL import Image

        img = np.zeros((meta.height, meta.width, 3), dtype=np.uint8)
        img[...] = (229, 232, 236)
        img[cells == 0] = (255, 255, 255)
        img[cells >= 50] = (52, 58, 68)
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
                    "confident": r.confident,
                    "accepted": r.confident and k not in self.registration_rejections,
                    "rejection": self.registration_rejections.get(k),
                    "dyaw_deg": round(math.degrees(r.dyaw), 2),
                }
                for k, r in self.registrations.items()
            },
            "global_members": sorted(members),
            "view_by_robot": {
                rid: "global" if rid in members else "local"
                for rid in self.robot_grids
            },
        }


map_service = MapService()
