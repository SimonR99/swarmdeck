"""Build a raytraced 2D occupancy grid from individual lidar scans.

For a robot whose own SLAM stack has no `OccupancyGrid` publisher — a 3D-only
pipeline like LVI-SAM, which registers a point cloud but never projects one to
2D — the adapter forwards each scan's already-registered points plus the
sensor's position instead of a finished grid. This accumulates them into the
exact `GridMeta`/int8-cells shape `MapService.ingest` expects from a robot
that DOES publish its own grid, so the entire merge/registration pipeline
downstream (`docs/collaborative-slam.md`) is unchanged and unaware of the
difference.

Marking cells "occupied" from points alone is not what an occupancy grid is
for — the free/unknown/occupied distinction is what grid registration keys off
(`docs/collaborative-slam.md` §2.2: known-free contradiction is what breaks
rotational symmetry). Every point is a lidar return, so the straight line from
the sensor to it is, by construction, free space the beam passed through
unobstructed.
"""

from __future__ import annotations

import numpy as np

from .grid_meta import GridMeta

UNKNOWN = -1
FREE = 0
OCCUPIED = 100


def _bresenham(x0: int, y0: int, x1: int, y1: int) -> tuple[np.ndarray, np.ndarray]:
    """Grid cells on the line from (x0,y0) up to but EXCLUDING (x1,y1).

    The endpoint is the lidar return itself (occupied) and is marked
    separately by the caller; everything strictly between the sensor and the
    return is free space the beam passed through.
    """
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    xs: list[int] = []
    ys: list[int] = []
    x, y = x0, y0
    while (x, y) != (x1, y1):
        xs.append(x)
        ys.append(y)
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy
    return np.array(xs, dtype=np.int64), np.array(ys, dtype=np.int64)


class ScanGridAccumulator:
    """One robot's persistent occupancy grid, built up scan by scan.

    Fixed size, anchored on the first scan's sensor position — the same way a
    SLAM stack anchors its own map at wherever the robot started. A point (or
    ray) that falls outside this window is dropped rather than growing the
    grid; that bounds a robot to roughly `size_m` of travel from its start in
    any direction, which is a real limitation, not a hidden one.
    """

    def __init__(
        self, origin_x: float, origin_y: float,
        resolution: float = 0.05, size_m: float = 40.0,
    ) -> None:
        n = int(size_m / resolution)
        self.meta = GridMeta(
            resolution=resolution, width=n, height=n,
            origin_x=origin_x - size_m / 2, origin_y=origin_y - size_m / 2,
        )
        self.cells = np.full((n, n), UNKNOWN, dtype=np.int8)

    def _to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        gx = int((x - self.meta.origin_x) / self.meta.resolution)
        gy = int((y - self.meta.origin_y) / self.meta.resolution)
        if 0 <= gx < self.meta.width and 0 <= gy < self.meta.height:
            return gx, gy
        return None

    def integrate(
        self, origin_x: float, origin_y: float, points_xy: np.ndarray
    ) -> None:
        """Raytrace one scan: free along each beam, occupied at the return.

        Occupied always wins — a cell already marked occupied is never
        downgraded back to free by a later beam passing near it, the same
        precedence `MapService._remerge` uses when merging robots' grids.
        """
        origin_cell = self._to_cell(origin_x, origin_y)
        if origin_cell is None or points_xy.size == 0:
            return
        ox, oy = origin_cell
        for px, py in points_xy:
            hit = self._to_cell(float(px), float(py))
            if hit is None:
                continue
            hx, hy = hit
            xs, ys = _bresenham(ox, oy, hx, hy)
            if len(xs):
                still_free = self.cells[ys, xs] != OCCUPIED
                self.cells[ys[still_free], xs[still_free]] = FREE
            self.cells[hy, hx] = OCCUPIED
