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
        self.transforms: dict[str, tuple[float, float, float]] = {}
        self.registrations: dict[str, Registration] = {}
        self.merge_mode = "static"
        self.reference: str | None = None
        self.seq = 0

        # Precomputed world-cell centre coordinates, for the backward warp.
        cx = self.meta.origin_x + (np.arange(n) + 0.5) * resolution
        cy = self.meta.origin_y + (np.arange(n) + 0.5) * resolution
        self._wx, self._wy = np.meshgrid(cx, cy)

    # ------------------------------------------------------------------ setup

    def set_transform(self, robot_id: str, x: float, y: float, yaw: float) -> None:
        self.transforms[robot_id] = (x, y, yaw)

    def set_mode(self, mode: str) -> None:
        self.merge_mode = mode if mode in ("static", "auto") else "static"

    # ---------------------------------------------------------------- ingest

    def ingest(self, robot_id: str, meta: GridMeta, cells: np.ndarray) -> None:
        self.robot_grids[robot_id] = (meta, cells)
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
        if result.confident:
            # Compose with the reference's own world transform.
            rx, ry, ryaw = self.transforms.get(self.reference or "", (0.0, 0.0, 0.0))
            c, s = math.cos(ryaw), math.sin(ryaw)
            wx = rx + result.dx * c - result.dy * s
            wy = ry + result.dx * s + result.dy * c
            self.transforms[robot_id] = (wx, wy, ryaw + result.dyaw)

    # --------------------------------------------------------------- merging

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
        for rid, (meta, cells) in self.robot_grids.items():
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

    def as_png(self) -> bytes:
        """Full grid for GET /api/map — browser-cacheable, survives reload."""
        from PIL import Image

        img = np.zeros((self.meta.height, self.meta.width, 3), dtype=np.uint8)
        img[...] = (26, 34, 48)  # unknown
        img[self.merged == 0] = (219, 231, 245)  # free
        img[self.merged >= 50] = (39, 56, 79)  # occupied
        # Grid row 0 is at origin_y (world "bottom"); image row 0 renders at the
        # top. Flip so the PNG matches the frontend's worldToGrid, which does
        # gy = height - (y - origin_y)/res.
        img = np.flipud(img)
        buf = io.BytesIO()
        Image.fromarray(img).save(buf, format="PNG", optimize=True)
        return buf.getvalue()

    def status(self) -> dict[str, Any]:
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
                    "dyaw_deg": round(math.degrees(r.dyaw), 2),
                }
                for k, r in self.registrations.items()
            },
        }


map_service = MapService()
