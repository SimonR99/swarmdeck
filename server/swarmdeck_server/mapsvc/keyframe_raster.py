"""Rasterize world-frame keyframe points into a 2D occupancy grid.

This is a display product for the 2D map, built from the replicated Swarm-SLAM
keyframe geometry (the same chunks the Global 3D view renders). It is not a
navigation input: the robots plan on their own onboard terrain products, and
slam_toolbox's grids remain the "slam" map source.

Rule, per cell of ``cell_m`` (0.2 m by default):

* the ground height is the 10th percentile (linear interpolation, as
  ``numpy.percentile``) of the z of the points that fall in the cell, with z
  read at 1 mm resolution (``Z_QUANTUM``) by the grouping sort;
* the cell is OCCUPIED when it holds at least one point between
  ``ground + 0.15 m`` and ``ground + 2.0 m``, the band a ground robot must not
  drive into. The 0.15 m step keeps kerb tops, speed bumps and the road
  surface texture out of the obstacle class; the 2.0 m ceiling keeps tree
  canopies and awnings out;
* the cell is FREE when it holds points but none in that band;
* the cell is UNKNOWN when no point falls in it.

The cell values follow the ROS occupancy convention every other grid here
uses (``-1`` unknown, ``0`` free, ``100`` occupied) and the cells are stored
bottom-up, row-major, row 0 at ``origin_y``, exactly like the grids the SLAM
back-end posts to ``/api/slam/optimized_map``.

Bounds: the grid covers the points' XY extent plus ``margin_m`` on every side,
snapped to a lattice of the cell size so successive rebuilds of a growing map
do not jitter. The cell count is capped at ``max_cells``; a larger extent is
COARSENED (the cell size doubled) until it fits, up to
``MAX_COARSENING_DOUBLINGS`` times, after which the raster is refused with a
``ValueError`` (an extent that still needs more than 3.2 m cells at the
default budget is a stray point, not a map). The applied doubling count is
reported so the caller can log it.

Known display artefacts: a cell whose only points are an overhang (a canopy
seen without the road beneath it) takes the overhang as its ground and may
show occupied; a slope steeper than the step band within one cell shows
occupied. Both are acceptable for a fleet overview raster.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Mapping

import numpy as np

from .grid_meta import GridMeta

UNKNOWN = np.int8(-1)
FREE = np.int8(0)
OCCUPIED = np.int8(100)

DEFAULT_CELL_M = 0.2
DEFAULT_MARGIN_M = 1.0
GROUND_PERCENTILE = 10.0
OBSTACLE_MIN_M = 0.15
OBSTACLE_MAX_M = 2.0
# 4M int8 cells: 4 MB of grid, and a PNG the browser decodes in well under a
# second. At 0.2 m that is a 400 m by 400 m site.
MAX_CELLS = 4_000_000
MAX_COARSENING_DOUBLINGS = 4
# Height quantum of the packed sort key: 1 mm, 32 bits, so 4295 km of height
# range and 2^31 cells fit one int64.
Z_QUANTUM = 0.001
Z_BITS = 32


@dataclass(frozen=True)
class KeyframeRaster:
    meta: GridMeta
    cells: np.ndarray
    points: int
    known_cells: int
    occupied_cells: int
    coarsened: int


def rasterize_points(
    points: np.ndarray,
    *,
    cell_m: float = DEFAULT_CELL_M,
    margin_m: float = DEFAULT_MARGIN_M,
    max_cells: int = MAX_CELLS,
    ground_percentile: float = GROUND_PERCENTILE,
    obstacle_min_m: float = OBSTACLE_MIN_M,
    obstacle_max_m: float = OBSTACLE_MAX_M,
) -> KeyframeRaster:
    """Apply the module rule to ``points`` (N x 3, world frame)."""
    if not math.isfinite(cell_m) or cell_m <= 0.0:
        raise ValueError("cell size must be finite and positive")
    if not math.isfinite(margin_m) or margin_m < 0.0:
        raise ValueError("margin must be finite and non-negative")
    if type(max_cells) is not int or max_cells < 1:
        raise ValueError("max_cells must be a positive integer")
    if not 0.0 <= ground_percentile <= 100.0:
        raise ValueError("ground percentile must lie in [0, 100]")
    if (
        not math.isfinite(obstacle_min_m)
        or not math.isfinite(obstacle_max_m)
        or obstacle_min_m < 0.0
        or obstacle_max_m <= obstacle_min_m
    ):
        raise ValueError("obstacle band must be 0 <= min < max")

    cloud = np.asarray(points, dtype=np.float64)
    if cloud.ndim != 2 or cloud.shape[1] != 3:
        raise ValueError("points must have shape Nx3")
    cloud = cloud[np.isfinite(cloud).all(axis=1)]
    if len(cloud) == 0:
        raise ValueError("no finite points to rasterize")

    low = cloud[:, :2].min(axis=0) - margin_m
    high = cloud[:, :2].max(axis=0) + margin_m
    cell = float(cell_m)
    coarsened = 0
    while True:
        origin = np.floor(low / cell) * cell
        # +1 so a point on the far edge lands inside the last column or row.
        size = np.ceil((high - origin) / cell).astype(np.int64) + 1
        width, height = int(size[0]), int(size[1])
        if width * height <= max_cells:
            break
        if coarsened >= MAX_COARSENING_DOUBLINGS:
            raise ValueError(
                f"extent {float(high[0] - low[0]):.0f} m by "
                f"{float(high[1] - low[1]):.0f} m exceeds {max_cells} cells even at "
                f"{cell:.2f} m"
            )
        cell *= 2.0
        coarsened += 1
    origin_x, origin_y = float(origin[0]), float(origin[1])

    col = np.floor((cloud[:, 0] - origin_x) / cell).astype(np.int64)
    row = np.floor((cloud[:, 1] - origin_y) / cell).astype(np.int64)
    np.clip(col, 0, width - 1, out=col)
    np.clip(row, 0, height - 1, out=row)
    index = row * width + col
    z = cloud[:, 2]

    # Group the points by cell with z ascending inside each group, so a cell's
    # percentile is an index into its run and the obstacle band is one
    # vectorized comparison against the ground repeated over the run. One
    # int64 sort of a packed key (cell index above, z quantized to Z_QUANTUM
    # below) is several times faster than a two-key lexsort plus gathers for
    # millions of points; the quantization error is far below the step band.
    z_floor = float(z.min())
    z_quantized = np.rint((z - z_floor) / Z_QUANTUM).astype(np.int64)
    if index.max() >= 1 << (63 - Z_BITS) or z_quantized.max() >= 1 << Z_BITS:
        raise ValueError("grid or height range too large to pack")
    key = np.sort((index << Z_BITS) | z_quantized)
    index_sorted = key >> Z_BITS
    z_sorted = (key & ((1 << Z_BITS) - 1)).astype(np.float64) * Z_QUANTUM + z_floor
    starts = np.concatenate(([0], np.flatnonzero(np.diff(index_sorted)) + 1))
    counts = np.diff(np.append(starts, len(key)))
    cells_idx = index_sorted[starts]
    position = starts + (ground_percentile / 100.0) * (counts - 1)
    lower = np.floor(position).astype(np.int64)
    upper = np.minimum(lower + 1, starts + counts - 1)
    fraction = position - lower
    ground = z_sorted[lower] + (z_sorted[upper] - z_sorted[lower]) * fraction
    ground_per_point = np.repeat(ground, counts)
    band = (z_sorted >= ground_per_point + obstacle_min_m) & (
        z_sorted <= ground_per_point + obstacle_max_m
    )
    occupied = np.add.reduceat(band.astype(np.int32), starts) > 0

    cells = np.full((height, width), UNKNOWN, dtype=np.int8)
    flat = cells.reshape(-1)
    flat[cells_idx] = FREE
    flat[cells_idx[occupied]] = OCCUPIED
    return KeyframeRaster(
        meta=GridMeta(cell, width, height, origin_x, origin_y),
        cells=cells,
        points=int(len(cloud)),
        known_cells=int(len(cells_idx)),
        occupied_cells=int(occupied.sum()),
        coarsened=coarsened,
    )


def composite_world_points(
    view: Mapping[str, Any],
    chunk_points: Callable[[Mapping[str, Any]], np.ndarray | None],
    *,
    max_points: int,
) -> tuple[np.ndarray, dict[str, int]]:
    """Every point of a composite view, placed by its submaps' transforms.

    ``view`` is a replica component view whose ``selected["submaps"]`` carry
    ``T_component_submap`` already re-expressed in the target frame (the
    deployment composite does that, so the result is in the world frame).
    ``chunk_points`` returns one chunk's decoded N x 3 points, or ``None`` when
    the chunk is unavailable (retired between the manifest read and this call).
    The declared point total is checked against ``max_points`` before any chunk
    is read, so an oversized composite costs nothing; it raises ``OverflowError``.
    """
    submaps = view["selected"]["submaps"]
    declared = 0
    for submap in submaps:
        for chunk in submap["chunks"]:
            declared += int(chunk["point_count"])
    if declared > max_points:
        raise OverflowError(
            f"composite declares {declared} points, above the {max_points} budget"
        )
    parts: list[np.ndarray] = []
    chunks = 0
    missing = 0
    for submap in submaps:
        transform = np.asarray(submap["T_component_submap"], dtype=np.float64)
        if transform.shape != (4, 4):
            raise ValueError("submap transform must be a 4x4 matrix")
        rotation, translation = transform[:3, :3], transform[:3, 3]
        for chunk in submap["chunks"]:
            chunks += 1
            local = chunk_points(chunk)
            if local is None:
                missing += 1
                continue
            if len(local):
                parts.append(
                    np.asarray(local, dtype=np.float64) @ rotation.T + translation
                )
    points = np.concatenate(parts) if parts else np.zeros((0, 3), dtype=np.float64)
    return points, {"chunks": chunks, "missing_chunks": missing, "declared": declared}
