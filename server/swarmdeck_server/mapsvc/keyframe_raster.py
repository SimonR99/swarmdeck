"""Rasterize world-frame keyframe points into a 2D occupancy grid.

This is the only server-side 2D display product, built from replicated
Swarm-SLAM keyframe geometry (the same chunks the Global 3D view renders). It
is not a navigation input: robots plan on onboard terrain products.

Rule, per cell of ``cell_m`` (0.2 m by default):

* the ground height is the 10th percentile (linear interpolation, as
  ``numpy.percentile``) of the z of the points that fall in the cell, with z
  read at 1 mm resolution (``Z_QUANTUM``) by the grouping sort;
* the cell is OCCUPIED when it holds at least one point between
  ``ground + 0.30 m`` and ``ground + 2.0 m``: walls, furniture, vehicles,
  the things the operator reads as obstacles. The 0.30 m floor of the band
  keeps out kerb tops (0.12 to 0.19 m on the Bistro street), speed bumps, and
  floor scatter. Source heights remain in the surveyed world frame: each
  source's points and keyframe ray origin share one stable SE(3) placement, and
  this display raster never estimates a cross-robot vertical offset from scene
  statistics. The 2.0 m ceiling keeps tree canopies and awnings out. A 0.2 m
  planter is below the band and drawn free: this raster is a display, the
  robots plan on their own terrain products;
* the cell is FREE when it holds points but none in that band, and also when
  a keyframe's rays swept it (``rays``): each replicated keyframe carries a
  4096-point subsample of its scan, so far ground has few returns and a
  raster of returns alone left most of a street unknown while every robot's
  own map showed it free (1418 m2 known against 2175 m2 for one robot's
  local map, 2026-09-20). Per keyframe, the returns within
  ``RAY_BAND_M`` of the keyframe height (the band a ground robot occupies)
  are grouped into ``RAY_SECTORS`` bearings, and the cells from the origin to
  one cell short of each sector's farthest return are swept free, as a laser
  scan is written into an occupancy grid. A swept cell that holds an
  obstacle return stays OCCUPIED; sweeping never overrides a return;
* the cell is UNKNOWN when no point falls in it and no ray swept it.

The cell values follow the ROS occupancy convention (``-1`` unknown, ``0``
free, ``100`` occupied) and the cells are stored bottom-up, row-major, row 0
at ``origin_y``. They are published only through the optimized deployment
raster endpoint.

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
OBSTACLE_MIN_M = 0.30
OBSTACLE_MAX_M = 2.0
# 4M int8 cells: 4 MB of grid, and a PNG the browser decodes in well under a
# second. At 0.2 m that is a 400 m by 400 m site.
MAX_CELLS = 4_000_000
MAX_COARSENING_DOUBLINGS = 4
# Height quantum of the packed sort key: 1 mm, 32 bits, so 4295 km of height
# range and 2^31 cells fit one int64.
Z_QUANTUM = 0.001
Z_BITS = 32
# Free-space sweeps: returns within this height of the keyframe origin claim
# free space along their bearing, one sector per degree.
RAY_BAND_M = 1.0
RAY_SECTORS = 360


@dataclass(frozen=True)
class KeyframeRaster:
    meta: GridMeta
    cells: np.ndarray
    points: int
    known_cells: int
    occupied_cells: int
    coarsened: int


def swept_free_cells(
    points: np.ndarray,
    origins: np.ndarray,
    spans: np.ndarray,
    *,
    origin_x: float,
    origin_y: float,
    cell: float,
    width: int,
    height: int,
    band_m: float = RAY_BAND_M,
    sectors: int = RAY_SECTORS,
) -> np.ndarray:
    """Boolean mask (height x width, flat) of the cells the keyframes' rays sweep.

    ``origins`` is K x 3 (one keyframe origin each) and ``spans`` K x 2, the
    ``[start, end)`` rows of ``points`` that keyframe returned. For every
    keyframe the returns within ``band_m`` of its height are binned by
    bearing into ``sectors`` and the farthest return of each sector, less one
    cell, bounds a visibility polygon; every cell of the window around the
    keyframe whose centre lies inside that polygon is swept. The window's
    bearings and ranges are a lookup table computed once per raster (the
    origin is taken at its cell's centre, a tenth of a cell of error), so a
    keyframe costs one comparison per window cell.
    """
    mask = np.zeros(width * height, dtype=bool)
    if len(origins) == 0:
        return mask
    # Pass 1: the farthest in-band return per sector of every keyframe.
    farthest_all = np.zeros((len(origins), sectors), dtype=np.float64)
    for k in range(len(origins)):
        start, end = int(spans[k, 0]), int(spans[k, 1])
        if end <= start:
            continue
        ox, oy, oz = (float(v) for v in origins[k])
        local = points[start:end]
        local = local[np.abs(local[:, 2] - oz) <= band_m]
        if len(local) == 0:
            continue
        dx, dy = local[:, 0] - ox, local[:, 1] - oy
        radius = np.hypot(dx, dy) - cell
        keep = radius > 0.0
        if not keep.any():
            continue
        bearing = np.floor(
            (np.arctan2(dy[keep], dx[keep]) + math.pi) / (2.0 * math.pi) * sectors
        ).astype(np.int64)
        np.clip(bearing, 0, sectors - 1, out=bearing)
        order = np.argsort(bearing, kind="stable")
        sorted_bearing = bearing[order]
        starts = np.concatenate(([0], np.flatnonzero(np.diff(sorted_bearing)) + 1))
        farthest_all[k, sorted_bearing[starts]] = np.maximum.reduceat(
            radius[keep][order], starts
        )
    reach = float(farthest_all.max())
    if reach <= 0.0:
        return mask
    # Pass 2: the window lookup table, then one comparison per cell and keyframe.
    half = int(math.ceil(reach / cell))
    offsets = np.arange(-half, half + 1)
    wx, wy = np.meshgrid(offsets, offsets)  # wy varies by row
    window_range = np.hypot(wx, wy) * cell
    window_sector = np.floor(
        (np.arctan2(wy, wx) + math.pi) / (2.0 * math.pi) * sectors
    ).astype(np.int64)
    np.clip(window_sector, 0, sectors - 1, out=window_sector)
    grid = mask.reshape(height, width)
    for k in range(len(origins)):
        farthest = farthest_all[k]
        if not farthest.any():
            continue
        col0 = int(math.floor((float(origins[k, 0]) - origin_x) / cell))
        row0 = int(math.floor((float(origins[k, 1]) - origin_y) / cell))
        inside = window_range <= farthest[window_sector]
        # Clip the window to the grid.
        c_lo, c_hi = max(0, col0 - half), min(width, col0 + half + 1)
        r_lo, r_hi = max(0, row0 - half), min(height, row0 + half + 1)
        if c_lo >= c_hi or r_lo >= r_hi:
            continue
        sub = inside[
            r_lo - (row0 - half) : r_hi - (row0 - half),
            c_lo - (col0 - half) : c_hi - (col0 - half),
        ]
        grid[r_lo:r_hi, c_lo:c_hi] |= sub
    return mask


def rasterize_points(
    points: np.ndarray,
    *,
    cell_m: float = DEFAULT_CELL_M,
    margin_m: float = DEFAULT_MARGIN_M,
    max_cells: int = MAX_CELLS,
    ground_percentile: float = GROUND_PERCENTILE,
    obstacle_min_m: float = OBSTACLE_MIN_M,
    obstacle_max_m: float = OBSTACLE_MAX_M,
    rays: tuple[np.ndarray, np.ndarray] | None = None,
) -> KeyframeRaster:
    """Apply the module rule to ``points`` (N x 3, world frame).

    ``rays`` is ``(origins, spans)`` as ``composite_world_points`` gathers
    them; with it the keyframes' rays sweep free space (``swept_free_cells``).
    """
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
    finite = np.isfinite(cloud).all(axis=1)
    if rays is not None and not finite.all():
        # Spans index the rows as gathered; a dropped row would shift them.
        rays = None
    cloud = cloud[finite]
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
    if rays is not None:
        origins = np.asarray(rays[0], dtype=np.float64)
        spans = np.asarray(rays[1], dtype=np.int64)
        if (
            origins.ndim != 2
            or origins.shape[1] != 3
            or spans.shape != (len(origins), 2)
        ):
            raise ValueError("rays must be (K x 3 origins, K x 2 spans)")
        swept = swept_free_cells(
            cloud,
            origins,
            spans,
            origin_x=origin_x,
            origin_y=origin_y,
            cell=cell,
            width=width,
            height=height,
        )
        flat[swept] = FREE
    flat[cells_idx] = FREE
    flat[cells_idx[occupied]] = OCCUPIED
    return KeyframeRaster(
        meta=GridMeta(cell, width, height, origin_x, origin_y),
        cells=cells,
        points=int(len(cloud)),
        known_cells=int((flat != UNKNOWN).sum()),
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
    When a submap carries the single ``sensor_origins`` entry guaranteed by the
    capture contract, that local sensor origin is transformed by the same
    full SE(3) as its points and used for ray sweeps. Legacy originless
    fixtures fall back to the submap translation.
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
    origins: list[np.ndarray] = []
    spans: list[tuple[int, int]] = []
    chunks = 0
    missing = 0
    gathered = 0
    for submap in submaps:
        transform = np.asarray(submap["T_component_submap"], dtype=np.float64)
        if transform.shape != (4, 4):
            raise ValueError("submap transform must be a 4x4 matrix")
        rotation, translation = transform[:3, :3], transform[:3, 3]
        sensor_origins = submap.get("sensor_origins") or ()
        ray_origin: np.ndarray | None = translation
        if len(sensor_origins) > 1:
            # A submap can carry several origins for legacy/multi-capture
            # products, but no point-to-origin association is available here.
            # Keep all geometry and omit only its unsafe ray sweep.
            ray_origin = None
        elif sensor_origins:
            local_origin = np.asarray(sensor_origins[0], dtype=np.float64)
            if local_origin.shape != (3,) or not np.isfinite(local_origin).all():
                raise ValueError("sensor origin must be a finite XYZ triple")
            # The capture's sensor origin is in the same local frame as the
            # chunk points. Apply the full submap SE(3), not just its
            # translation, so free-space rays and returns agree exactly.
            ray_origin = local_origin @ rotation.T + translation
        first = gathered
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
                gathered += len(local)
        if gathered > first and ray_origin is not None:
            origins.append(ray_origin)
            spans.append((first, gathered))
    points = np.concatenate(parts) if parts else np.zeros((0, 3), dtype=np.float64)
    return points, {
        "chunks": chunks,
        "missing_chunks": missing,
        "declared": declared,
        # One origin and ``[start, end)`` row span per keyframe that
        # contributed points: what ``rasterize_points`` sweeps free space with.
        "origins": (
            np.asarray(origins, dtype=np.float64)
            if origins
            else np.zeros((0, 3), dtype=np.float64)
        ),
        "spans": (
            np.asarray(spans, dtype=np.int64)
            if spans
            else np.zeros((0, 2), dtype=np.int64)
        ),
    }
