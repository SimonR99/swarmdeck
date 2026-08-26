"""Grid-based A* global path planner on occupancy maps.

Provides global room-scale trajectory planning for robots (such as Scout Mini
running local reactive avoidance) that lack an onboard global planner node like Nav2.
"""

from __future__ import annotations

import heapq
import math
from typing import Any

import numpy as np

from .grid_meta import GridMeta


def plan_global_path(
    grid: np.ndarray,
    meta: GridMeta,
    start_world: dict[str, float],
    goal_world: dict[str, float],
    clearance_m: float = 0.35,
) -> list[dict[str, float]]:
    """Compute an A* path from start_world to goal_world avoiding occupied cells."""
    h, w = grid.shape
    res = float(meta.resolution)
    ox = float(meta.origin_x)
    oy = float(meta.origin_y)

    # Convert start and goal to grid array indices (row index = y, col index = x)
    sx = int(round((float(start_world["x"]) - ox) / res))
    sy = int(round((float(start_world["y"]) - oy) / res))
    gx = int(round((float(goal_world["x"]) - ox) / res))
    gy = int(round((float(goal_world["y"]) - oy) / res))

    # Clamp inside grid boundaries
    sx = max(0, min(w - 1, sx))
    sy = max(0, min(h - 1, sy))
    gx = max(0, min(w - 1, gx))
    gy = max(0, min(h - 1, gy))

    if sx == gx and sy == gy:
        return [
            {"x": round(start_world["x"], 3), "y": round(start_world["y"], 3)},
            {"x": round(goal_world["x"], 3), "y": round(goal_world["y"], 3)},
        ]

    # Cost map: 0=free (cost 1), -1=unknown (cost 8), >=50=occupied (cost 255)
    clearance_px = max(1, int(round(clearance_m / res)))
    occupied_mask = grid >= 50
    if clearance_px > 1 and np.any(occupied_mask):
        try:
            from scipy.ndimage import binary_dilation

            occupied_mask = binary_dilation(occupied_mask, iterations=clearance_px)
        except ImportError:
            pass

    cost = np.where(occupied_mask, 255, np.where(grid == -1, 8, 1)).astype(np.int32)

    # 8-connectivity
    neighbors = [
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, 1.414),
        (-1, 1, 1.414),
        (1, -1, 1.414),
        (1, 1, 1.414),
    ]

    open_set: list[tuple[float, float, int, int]] = [(0.0, 0.0, sx, sy)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score: dict[tuple[int, int], float] = {(sx, sy): 0.0}

    found = False
    max_expansions = 40000
    expansions = 0

    while open_set and expansions < max_expansions:
        expansions += 1
        _, g, cx, cy = heapq.heappop(open_set)
        if (cx, cy) == (gx, gy) or math.hypot(cx - gx, cy - gy) <= 1.5:
            gx, gy = cx, cy
            found = True
            break
        if g > g_score.get((cx, cy), float("inf")):
            continue

        for dx, dy, step_cost in neighbors:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < w and 0 <= ny < h:
                c = int(cost[ny, nx])
                if c >= 200:  # obstacle
                    continue
                ng = g + step_cost * (1.0 + c * 0.15)
                if ng < g_score.get((nx, ny), float("inf")):
                    g_score[(nx, ny)] = ng
                    h_val = math.hypot(nx - gx, ny - gy)
                    heapq.heappush(open_set, (ng + h_val, ng, nx, ny))
                    came_from[(nx, ny)] = (cx, cy)

    if not found:
        return [
            {"x": round(start_world["x"], 3), "y": round(start_world["y"], 3)},
            {"x": round(goal_world["x"], 3), "y": round(goal_world["y"], 3)},
        ]

    curr = (gx, gy)
    pixel_path = []
    while curr in came_from:
        pixel_path.append(curr)
        curr = came_from[curr]
    pixel_path.append((sx, sy))
    pixel_path.reverse()

    stride = max(1, len(pixel_path) // 40)
    sampled = pixel_path[::stride]
    if sampled[-1] != pixel_path[-1]:
        sampled.append(pixel_path[-1])

    return [
        {
            "x": round(ox + px * res, 3),
            "y": round(oy + py * res, 3),
        }
        for px, py in sampled
    ]
