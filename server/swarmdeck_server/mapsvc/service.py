"""Merged occupancy grid.

Per-robot grids arrive over HTTP; transforms come from config (`static`) or a
merge backend (`auto`). The merged grid is derived, diffed, and emitted as
bounding-box patches (architecture.md §5).
"""

from __future__ import annotations

import base64
import io
import zlib
from dataclasses import dataclass
from typing import Any

import numpy as np

from ..bus import bus

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
    def __init__(self, resolution: float = 0.05, size_m: float = 40.0) -> None:
        n = int(size_m / resolution)
        self.meta = GridMeta(resolution, n, n, -size_m / 2, -size_m / 2)
        self.merged = np.full((n, n), UNKNOWN, dtype=np.int8)
        self._prev = self.merged.copy()
        self.robot_grids: dict[str, tuple[GridMeta, np.ndarray]] = {}
        self.transforms: dict[str, tuple[float, float, float]] = {}
        self.seq = 0

    def set_transform(self, robot_id: str, x: float, y: float, yaw: float) -> None:
        self.transforms[robot_id] = (x, y, yaw)

    def ingest(self, robot_id: str, meta: GridMeta, cells: np.ndarray) -> None:
        self.robot_grids[robot_id] = (meta, cells)
        self._remerge()

    def _remerge(self) -> None:
        """Occupied wins over free, free wins over unknown."""
        out = np.full_like(self.merged, UNKNOWN)
        for rid, (meta, cells) in self.robot_grids.items():
            tx, ty, _yaw = self.transforms.get(rid, (0.0, 0.0, 0.0))
            # Offset in cells between this grid's origin and the merged origin.
            ox = int(round((meta.origin_x + tx - self.meta.origin_x) / self.meta.resolution))
            oy = int(round((meta.origin_y + ty - self.meta.origin_y) / self.meta.resolution))

            x0, y0 = max(0, ox), max(0, oy)
            x1 = min(self.meta.width, ox + meta.width)
            y1 = min(self.meta.height, oy + meta.height)
            if x1 <= x0 or y1 <= y0:
                continue

            sub = cells[y0 - oy : y1 - oy, x0 - ox : x1 - ox]
            dst = out[y0:y1, x0:x1]
            known = sub != UNKNOWN
            dst[known] = np.maximum(dst[known], sub[known])
            out[y0:y1, x0:x1] = dst
        self.merged = out

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
        buf = io.BytesIO()
        Image.fromarray(img).save(buf, format="PNG", optimize=True)
        return buf.getvalue()


map_service = MapService()
