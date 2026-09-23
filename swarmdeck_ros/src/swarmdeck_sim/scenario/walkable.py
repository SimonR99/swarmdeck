"""Where a ground robot can get to in a mesh world, and how far it is.

Detection targets in a mesh world have to stand where the fleet can reach
them. A hand-picked list covers one entrance tile; a tunnel network of several
hundred tiles needs the floor itself. This rasterises the collision mesh on a
horizontal grid, keeps the surfaces with headroom above them as floor, and
floods that floor from the fleet's start with a step-height limit, so every
cell it reaches carries its distance from the start along the tunnels, not in
a straight line through rock.

numpy only: it runs in the simulation container at session generation.
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math

import numpy as np

# A triangle this steep is wall; a vertical ray grazing it says nothing.
_MIN_RAY_NORMAL_Z = 0.2
# Floor has to be walkable, i.e. within about 35 degrees of level.
_MIN_FLOOR_NORMAL_Z = 0.8
# Hits closer than this in one column are one (double-sided) surface.
_SAME_SURFACE_M = 0.05
_NEIGHBOURS = tuple(
    (dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if (dx, dy) != (0, 0)
)


@dataclass(frozen=True)
class Floor:
    """The reachable floor: one row per grid cell and level."""

    x: np.ndarray
    y: np.ndarray
    z: np.ndarray
    # Metres from the start along the floor.
    distance: np.ndarray
    # Every neighbour within `ring` cells is reachable floor at a similar
    # height: the cell is at least that far from a wall or a drop.
    clear: np.ndarray
    # Some neighbour within two rings is not: the cell is beside a wall.
    by_wall: np.ndarray


def _column_hits(triangles: np.ndarray, x0: float, y0: float, cell: float):
    """Every surface a vertical ray through each cell centre crosses.

    Returns (column index pairs, heights, whether the surface is walkable).
    Rays go through cell centres x0 + i * cell; a triangle is paired with the
    centres inside its xy bounding box, then kept where the centre falls
    inside it.
    """
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    normal = np.cross(b - a, c - a)
    length = np.linalg.norm(normal, axis=1)
    normal_z = np.abs(normal[:, 2]) / np.maximum(length, 1e-12)
    usable = (length > 1e-12) & (normal_z > _MIN_RAY_NORMAL_Z)
    triangles = triangles[usable]
    normal_z = normal_z[usable]
    lo = triangles[:, :, :2].min(axis=1)
    hi = triangles[:, :, :2].max(axis=1)
    i0 = np.ceil((lo[:, 0] - x0) / cell).astype(np.int64)
    i1 = np.floor((hi[:, 0] - x0) / cell).astype(np.int64)
    j0 = np.ceil((lo[:, 1] - y0) / cell).astype(np.int64)
    j1 = np.floor((hi[:, 1] - y0) / cell).astype(np.int64)
    nx = np.maximum(i1 - i0 + 1, 0)
    ny = np.maximum(j1 - j0 + 1, 0)
    counts = nx * ny

    columns_i, columns_j, heights, walkable = [], [], [], []
    # Chunked so a world of large triangles cannot allocate every pair at once.
    order = np.flatnonzero(counts)
    bounds = np.searchsorted(
        np.cumsum(counts[order]),
        np.arange(4_000_000, counts.sum() + 4_000_000, 4_000_000),
    )
    for idx in np.split(order, np.unique(np.minimum(bounds + 1, len(order)))):
        if len(idx) == 0:
            continue
        per = counts[idx]
        tri = np.repeat(idx, per)
        offset = np.arange(per.sum()) - np.repeat(np.cumsum(per) - per, per)
        ci = i0[tri] + offset % nx[tri]
        cj = j0[tri] + offset // nx[tri]
        px = x0 + ci * cell
        py = y0 + cj * cell
        ta, tb, tc = triangles[tri, 0], triangles[tri, 1], triangles[tri, 2]
        den = (tb[:, 1] - tc[:, 1]) * (ta[:, 0] - tc[:, 0]) + (tc[:, 0] - tb[:, 0]) * (
            ta[:, 1] - tc[:, 1]
        )
        den = np.where(np.abs(den) > 1e-12, den, np.nan)
        u = (
            (tb[:, 1] - tc[:, 1]) * (px - tc[:, 0])
            + (tc[:, 0] - tb[:, 0]) * (py - tc[:, 1])
        ) / den
        v = (
            (tc[:, 1] - ta[:, 1]) * (px - tc[:, 0])
            + (ta[:, 0] - tc[:, 0]) * (py - tc[:, 1])
        ) / den
        inside = (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9)
        z = u * ta[:, 2] + v * tb[:, 2] + (1 - u - v) * tc[:, 2]
        columns_i.append(ci[inside])
        columns_j.append(cj[inside])
        heights.append(z[inside])
        walkable.append(normal_z[tri][inside] >= _MIN_FLOOR_NORMAL_Z)
    if not heights:
        empty = np.zeros(0)
        return empty.astype(np.int64), empty.astype(np.int64), empty, empty.astype(bool)
    return (
        np.concatenate(columns_i),
        np.concatenate(columns_j),
        np.concatenate(heights),
        np.concatenate(walkable),
    )


def reachable_floor(
    triangles: np.ndarray,
    start: tuple[float, float, float],
    *,
    cell: float = 0.5,
    headroom: float = 1.0,
    step: float = 0.3,
) -> Floor:
    """The floor a ground robot starting at `start` can drive to.

    A cell's floor is a walkable surface with at least `headroom` of clear
    space above it; two floors in neighbouring cells connect when their
    heights differ by at most `step` (about 30 degrees at 0.5 m cells, which
    takes the SubT ramps and refuses walls and shafts). Distances are summed
    along the grid (1 or sqrt(2) cells per move).
    """
    tri = np.asarray(triangles, dtype=float)
    x0 = math.floor(float(tri[:, :, 0].min()) / cell) * cell
    y0 = math.floor(float(tri[:, :, 1].min()) / cell) * cell
    ci, cj, z, walk = _column_hits(tri, x0, y0, cell)

    # Sort each column bottom-up and fold coincident hits into one surface.
    order = np.lexsort((z, cj, ci))
    ci, cj, z, walk = ci[order], cj[order], z[order], walk[order]
    same_column = (ci[1:] == ci[:-1]) & (cj[1:] == cj[:-1])
    duplicate = np.concatenate(
        [[False], same_column & (z[1:] - z[:-1] < _SAME_SURFACE_M)]
    )
    # A surface is walkable when any of its coincident hits is.
    surface = np.cumsum(~duplicate) - 1
    walk_any = np.bincount(surface, weights=walk.astype(float)) > 0
    keep = ~duplicate
    ci, cj, z, walk = ci[keep], cj[keep], z[keep], walk_any
    same_column = (ci[1:] == ci[:-1]) & (cj[1:] == cj[:-1])
    above = np.full(len(z), np.inf)
    above[:-1] = np.where(same_column, z[1:], np.inf)
    floor = walk & (above - z >= headroom)
    fi, fj, fz = ci[floor], cj[floor], z[floor]

    # For each floor and each cell offset, the floor in that cell nearest its
    # height within `step`, or -1: cells keyed and sorted once, looked up with
    # searchsorted, one level of a multi-level cell at a time.
    width = int(fj.max()) + 5
    keys = (fi + 2) * width + (fj + 2)
    order = np.argsort(keys, kind="stable")
    sorted_keys = keys[order]
    most_levels = int(np.unique(keys, return_counts=True)[1].max())

    def neighbour(di: int, dj: int) -> np.ndarray:
        target = keys + di * width + dj
        lo = np.searchsorted(sorted_keys, target, "left")
        hi = np.searchsorted(sorted_keys, target, "right")
        best = np.full(len(keys), -1)
        gap = np.full(len(keys), np.inf)
        for level in range(most_levels):
            at = lo + level
            candidate = order[np.minimum(at, len(order) - 1)]
            dz = np.abs(fz[candidate] - fz)
            better = (at < hi) & (dz <= step) & (dz < gap)
            best[better] = candidate[better]
            gap[better] = dz[better]
        return best

    ring2 = [(di, dj) for di in range(-2, 3) for dj in range(-2, 3) if di or dj]
    table = {offset: neighbour(*offset) for offset in ring2}

    # The start's floor: the highest one at or just below the start.
    sx, sy, sz = start
    si = round((sx - x0) / cell)
    sj = round((sy - y0) / cell)
    near = np.flatnonzero(
        (np.abs(fi - si) <= 1) & (np.abs(fj - sj) <= 1) & (fz <= sz + step)
    )
    if len(near) == 0:
        raise ValueError(f"no floor under the start ({sx:g}, {sy:g}, {sz:g})")
    seed = int(near[np.argmax(fz[near])])

    moves = [
        (table[offset].tolist(), cell * (math.sqrt(2.0) if all(offset) else 1.0))
        for offset in _NEIGHBOURS
    ]
    distance = [math.inf] * len(fz)
    distance[seed] = 0.0
    queue = [(0.0, seed)]
    while queue:
        d, n = heapq.heappop(queue)
        if d > distance[n]:
            continue
        for links, cost in moves:
            m = links[n]
            if m >= 0 and d + cost < distance[m]:
                distance[m] = d + cost
                heapq.heappush(queue, (d + cost, m))
    distance = np.asarray(distance)

    reached_mask = np.isfinite(distance)
    reached = np.flatnonzero(reached_mask)

    def ring_complete(ring: int) -> np.ndarray:
        complete = np.ones(len(fz), dtype=bool)
        for (di, dj), links in table.items():
            if max(abs(di), abs(dj)) <= ring:
                complete &= (links >= 0) & reached_mask[np.maximum(links, 0)]
        return complete[reached]

    clear = ring_complete(1)
    by_wall = ~ring_complete(2)
    return Floor(
        x=x0 + fi[reached] * cell,
        y=y0 + fj[reached] * cell,
        z=fz[reached],
        distance=distance[reached],
        clear=clear,
        by_wall=by_wall,
    )


def scatter(
    floor: Floor,
    count: int,
    rng,
    *,
    min_distance: float,
    spacing: float,
    far_fraction: float = 0.9,
) -> list[int]:
    """Indices of `count` floor cells spread over the reachable network.

    Every pick is at least `min_distance` along the floor from the start,
    stands clear of walls by one cell but beside one (so it does not stand
    in the middle of a passage), and is at least `spacing` from every other
    pick. The first pick lies in the far end: the last `1 - far_fraction` of
    the network's floor distance from the start. Fewer than `count` come back
    only when the network has no room for more at that spacing.
    """
    if count <= 0:
        return []
    candidates = np.flatnonzero(
        floor.clear & floor.by_wall & (floor.distance >= min_distance)
    )
    if len(candidates) == 0:
        raise ValueError("no reachable floor for detection targets")
    far_limit = far_fraction * float(floor.distance[candidates].max())
    picks = [rng.choice([int(n) for n in candidates if floor.distance[n] >= far_limit])]
    rest = [int(n) for n in candidates]
    rng.shuffle(rest)
    for n in rest:
        if len(picks) >= count:
            break
        if all(
            math.dist(
                (floor.x[n], floor.y[n], floor.z[n]),
                (floor.x[p], floor.y[p], floor.z[p]),
            )
            >= spacing
            for p in picks
        ):
            picks.append(n)
    return picks
