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

# Cells hold accumulated EVIDENCE rather than a latched verdict.
#
# The latch had two failures the operator could see. A single beam marked its
# whole ray free, so one spurious long return — a reflection, dust, a glimpse
# through a doorway — painted a free corridor straight through a wall; those are
# the spikes that radiated out of every live map. And `OCCUPIED` was permanent,
# so a robot or a person that passed through once became a wall forever. That
# second one is exactly the "stale cell is immortal" failure `MapService._remerge`
# documents and solves by majority vote across robots — but the per-robot
# accumulator feeding it never got the same treatment, so each robot kept
# manufacturing ghosts that the vote then had to outnumber.
#
# Asymmetric on purpose: one return is enough to SHOW an obstacle (the safe
# direction, and what the latch already did), while clearing one takes sustained
# contrary evidence.
#
# `FREE_AT = -1` deliberately keeps the old "one beam marks its path free"
# behaviour. Requiring a second observation was tried on the live fleet and had
# to be reverted: free space is not cosmetic here, it is the registration signal.
# `docs/collaborative-slam.md` §2.2 keys grid matching on known-free
# contradiction, and withholding it collapsed pairwise overlap from 1252 cells to
# 26, `support` from 0.98 to 0.05, and threw every robot out of the merged map.
# The long free spikes a lone spurious return still paints are the price of that
# signal; suppressing them belongs in range/return filtering upstream, not in the
# evidence threshold the merge depends on.
HIT_GAIN = 3
MISS_GAIN = 1
EVIDENCE_CLAMP = 12  # bounds how stubborn any cell can get, both ways
OCCUPIED_AT = 2      # one hit (+3) is immediately occupied
FREE_AT = -1         # one pass-through clears, as before — see above


# A return standing far BEHIND its angular neighbours is almost always a stray:
# a reflection, a mote, or a glimpse through a gap. Raytracing it carves a free
# corridor from the sensor all the way out past whatever it passed through,
# which is the long radial spike that has been decorating every live map.
#
# Neighbours are chosen by BEARING, not by distance, and that is the whole point.
# A fixed metric neighbourhood cannot work: real returns thin out linearly with
# range, so tuning one tight enough to catch strays at 15 m also deletes every
# genuine far wall. Measured against live fleet scans, a 0.30 m metric radius
# dropped 16% of tars_0's returns and capped its perception at 7.8 m; this test
# drops 0.6-2.6% and leaves the farthest real return (17.9 m) untouched.
#
# Only FARTHER-than-neighbours counts. A return nearer than its neighbours is an
# obstacle in front of a wall, which is exactly what must never be discarded.
#
# The cost is honest: a single-scan peek through a doorway has the same
# signature and is dropped too. That geometry comes back properly the moment a
# robot drives through the opening, whereas a stray never becomes correct.
STRAY_NEIGHBOURS = 4
STRAY_JUMP_M = 2.0


def drop_range_outliers(
    origin_x: float,
    origin_y: float,
    points_xy: np.ndarray,
    k: int = STRAY_NEIGHBOURS,
    jump_m: float = STRAY_JUMP_M,
) -> np.ndarray:
    """Drop returns standing more than `jump_m` behind their angular neighbours."""
    if len(points_xy) < 2 * k + 1:
        return points_xy
    dx = points_xy[:, 0] - origin_x
    dy = points_xy[:, 1] - origin_y
    ranges = np.hypot(dx, dy)
    order = np.argsort(np.arctan2(dy, dx))
    sorted_ranges = ranges[order]
    # Wraps at +/-pi via np.roll, so the scan is treated as the closed ring it is.
    offsets = [*range(-k, 0), *range(1, k + 1)]
    neighbours = np.stack([np.roll(sorted_ranges, s) for s in offsets])
    stray_sorted = (sorted_ranges - np.median(neighbours, axis=0)) > jump_m
    stray = np.zeros(len(points_xy), dtype=bool)
    stray[order] = stray_sorted
    return points_xy[~stray]


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
        # int32, not int8: within ONE scan the sensor's own cell is crossed by
        # every beam, so the running total before clamping is on the order of the
        # point count. In int8 that wraps, and the robot's own position would
        # flip to occupied.
        self._evidence = np.zeros((n, n), dtype=np.int32)

    def _to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        gx = int((x - self.meta.origin_x) / self.meta.resolution)
        gy = int((y - self.meta.origin_y) / self.meta.resolution)
        if 0 <= gx < self.meta.width and 0 <= gy < self.meta.height:
            return gx, gy
        return None

    def integrate(
        self, origin_x: float, origin_y: float, points_xy: np.ndarray
    ) -> None:
        """Raytrace one scan: evidence against each traversed cell, for the return.

        A return is evidence its own cell is occupied and evidence every cell the
        beam crossed to reach it is not. Neither is treated as proof — see the
        gain constants for why a verdict now needs corroboration in the free
        direction and stays revisable in the occupied one.
        """
        origin_cell = self._to_cell(origin_x, origin_y)
        if origin_cell is None or points_xy.size == 0:
            return
        ox, oy = origin_cell
        width = self.meta.width
        crossed: list[np.ndarray] = []
        hits: list[int] = []
        for px, py in points_xy:
            hit = self._to_cell(float(px), float(py))
            if hit is None:
                continue
            hx, hy = hit
            xs, ys = _bresenham(ox, oy, hx, hy)
            if len(xs):
                # Flatten now: one bincount over the whole scan is far cheaper
                # than an unbuffered `np.subtract.at` per beam, and a cell
                # crossed by many beams must count once per crossing.
                crossed.append(ys.astype(np.int64) * width + xs.astype(np.int64))
            hits.append(hy * width + hx)
        if not hits:
            return

        evidence = self._evidence
        n_cells = evidence.size
        if crossed:
            misses = np.bincount(np.concatenate(crossed), minlength=n_cells)
            evidence -= (misses * MISS_GAIN).reshape(evidence.shape).astype(np.int32)
        struck = np.bincount(np.asarray(hits, dtype=np.int64), minlength=n_cells)
        evidence += (struck * HIT_GAIN).reshape(evidence.shape).astype(np.int32)
        np.clip(evidence, -EVIDENCE_CLAMP, EVIDENCE_CLAMP, out=evidence)

        self.cells = np.where(
            evidence >= OCCUPIED_AT,
            np.int8(OCCUPIED),
            np.where(evidence <= FREE_AT, np.int8(FREE), np.int8(UNKNOWN)),
        ).astype(np.int8)
