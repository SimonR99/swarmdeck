"""`GridMeta` lives on its own so `service.py` and `scan_grid.py` can both
import it without a cycle (`service.MapService` uses `scan_grid.ScanGridAccumulator`,
which needs `GridMeta` too)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


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
