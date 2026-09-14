"""Collision-checked grid routes for robots without an onboard global planner."""

from __future__ import annotations

import heapq
import math

import numpy as np

from .grid_meta import GridMeta


class PathPlanningError(ValueError):
    """The map does not admit a route to the requested goal."""


def _segment_cells(a, b):
    """Supercover of a segment in grid coordinates, including corner touches."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    cuts = {0.0, 1.0}
    for start, delta, end in ((a[0], dx, b[0]), (a[1], dy, b[1])):
        if delta:
            for boundary in range(
                math.ceil(min(start, end)), math.floor(max(start, end)) + 1
            ):
                cuts.add((boundary - start) / delta)
    cuts = sorted(cuts)
    samples = cuts + [(lo + hi) / 2 for lo, hi in zip(cuts, cuts[1:])]
    for t in samples:
        x, y = a[0] + t * dx, a[1] + t * dy
        xs = {math.floor(x)}
        ys = {math.floor(y)}
        if abs(x - round(x)) < 1e-9:
            xs = {round(x) - 1, round(x)}
        if abs(y - round(y)) < 1e-9:
            ys = {round(y) - 1, round(y)}
        for cx in xs:
            for cy in ys:
                yield cx, cy


def plan_global_path(
    grid: np.ndarray,
    meta: GridMeta,
    start_world: dict[str, float],
    goal_world: dict[str, float],
    clearance_m: float = 0.35,
) -> list[dict[str, float]]:
    """Return a checked polyline, or raise rather than invent an unsafe route."""
    h, w = grid.shape
    res = float(meta.resolution)
    if not h or not w or not math.isfinite(res) or res <= 0:
        raise PathPlanningError("Navigation map is invalid")
    endpoints = [
        ((float(pt["x"]) - meta.origin_x) / res, (float(pt["y"]) - meta.origin_y) / res)
        for pt in (start_world, goal_world)
    ]
    if any(
        not (math.isfinite(x) and math.isfinite(y) and 0 <= x < w and 0 <= y < h)
        for x, y in endpoints
    ):
        raise PathPlanningError("Start or goal is outside the navigation map")
    start, goal = [(math.floor(x), math.floor(y)) for x, y in endpoints]

    # Use numpy so clearance is enforced in the base server installation too.
    # A square envelope is conservative at diagonal obstacle corners.
    radius = max(0, math.ceil(clearance_m / res))
    occupied = grid >= 50
    for axis, size in ((0, h), (1, w)):
        source = occupied.copy()
        for offset in range(1, min(radius, size - 1) + 1):
            dst, src = [slice(None)] * 2, [slice(None)] * 2
            dst[axis], src[axis] = slice(offset, None), slice(None, -offset)
            occupied[tuple(dst)] |= source[tuple(src)]
            occupied[tuple(src)] |= source[tuple(dst)]
    cost = np.where(occupied, 255, np.where(grid < 0, 8, 1))

    def visible(a, b, known_only=False):
        return all(
            0 <= x < w
            and 0 <= y < h
            and cost[y, x] < 255
            and (not known_only or cost[y, x] == 1)
            for x, y in _segment_cells(a, b)
        )

    def centre(cell):
        return cell[0] + 0.5, cell[1] + 0.5

    if not visible(endpoints[0], centre(start)) or not visible(
        centre(goal), endpoints[1]
    ):
        raise PathPlanningError("Start or goal has insufficient obstacle clearance")
    if visible(*endpoints, known_only=True):
        points = endpoints
    else:
        queue = [(0.0, 0.0, start)]
        scores = {start: 0.0}
        previous = {}
        found = False
        expansions = 0
        while queue and expansions < 40000:
            _, g, current = heapq.heappop(queue)
            if g > scores[current]:
                continue
            expansions += 1
            if current == goal:
                found = True
                break
            x, y = current
            for dx, dy in (
                (-1, 0),
                (1, 0),
                (0, -1),
                (0, 1),
                (-1, -1),
                (-1, 1),
                (1, -1),
                (1, 1),
            ):
                nx, ny = x + dx, y + dy
                if not (0 <= nx < w and 0 <= ny < h) or cost[ny, nx] == 255:
                    continue
                if dx and dy and (occupied[y, nx] or occupied[ny, x]):
                    continue
                ng = g + math.hypot(dx, dy) * float(cost[ny, nx])
                if ng < scores.get((nx, ny), math.inf):
                    scores[nx, ny] = ng
                    previous[nx, ny] = current
                    heapq.heappush(
                        queue,
                        (ng + math.hypot(nx - goal[0], ny - goal[1]), ng, (nx, ny)),
                    )
        if not found:
            raise PathPlanningError("No route with sufficient obstacle clearance")
        cells = [goal]
        while cells[-1] != start:
            cells.append(previous[cells[-1]])
        points = (
            [endpoints[0]] + [centre(cell) for cell in reversed(cells)] + [endpoints[1]]
        )
        # Remove bends only when every touched cell is clear. Never stride over
        # corners, and never shortcut a known-free detour through unknown space.
        simplified = [points[0]]
        i = 0
        while i < len(points) - 1:
            j = len(points) - 1
            while j > i + 1 and not visible(points[i], points[j], known_only=True):
                j -= 1
            simplified.append(points[j])
            i = j
        points = simplified
    return [
        {"x": meta.origin_x + x * res, "y": meta.origin_y + y * res} for x, y in points
    ]
