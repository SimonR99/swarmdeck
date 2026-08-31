"""Pure planning primitives for coordinated multi-robot exploration.

The ROS node in :mod:`coordinated_explore` is intentionally thin.  Map fusion,
frontier extraction and allocation live here so they can be tested without ROS
or Gazebo and replayed deterministically.

Frames follow the rest of SwarmDeck: ``T_world_map=(x, y, yaw)`` maps a point
from one robot's SLAM map into the shared study frame.  The transform is a weak
coordination prior (normally the configured spawn pose), not a measurement used
by the collaborative SLAM optimizer.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

import numpy as np

UNKNOWN = np.int8(-1)
FREE_MAX = 20
OCCUPIED_MIN = 65


@dataclass(frozen=True, slots=True)
class GridSnapshot:
    """One immutable occupancy grid in its source map frame."""

    cells: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0

    def __post_init__(self) -> None:
        cells = np.asarray(self.cells, dtype=np.int8)
        if cells.ndim != 2:
            raise ValueError(f"cells must be 2D, got {cells.shape}")
        if not math.isfinite(self.resolution) or self.resolution <= 0.0:
            raise ValueError("resolution must be finite and positive")
        object.__setattr__(self, "cells", cells)


@dataclass(frozen=True, slots=True)
class CommonGrid:
    """Fixed-frame union and how many robot maps observed each known cell."""

    cells: np.ndarray
    observations: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float

    @property
    def width(self) -> int:
        return int(self.cells.shape[1])

    @property
    def height(self) -> int:
        return int(self.cells.shape[0])

    def cell(self, x: float, y: float) -> tuple[int, int]:
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        return row, col

    def point(self, row: int, col: int) -> tuple[float, float]:
        return (
            self.origin_x + (col + 0.5) * self.resolution,
            self.origin_y + (row + 0.5) * self.resolution,
        )


@dataclass(frozen=True, slots=True)
class Frontier:
    row: int
    col: int
    x: float
    y: float
    cells: int
    information_m2: float


@dataclass(frozen=True, slots=True)
class RobotState:
    robot_id: str
    x: float
    y: float
    radius_m: float
    travelled_m: float = 0.0
    navigation_clearance_m: float | None = None


@dataclass(frozen=True, slots=True)
class Assignment:
    robot_id: str
    frontier: Frontier
    path_cost_m: float
    utility: float


def transform_point(
    point: tuple[float, float], transform: tuple[float, float, float]
) -> tuple[float, float]:
    """Apply an SE(2) ``T_target_source`` tuple to one point."""
    x, y = point
    tx, ty, yaw = transform
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return tx + x * cosine - y * sine, ty + x * sine + y * cosine


def inverse_transform_point(
    point: tuple[float, float], transform: tuple[float, float, float]
) -> tuple[float, float]:
    """Express a target-frame point in the source frame of ``transform``."""
    x, y = point[0] - transform[0], point[1] - transform[1]
    cosine, sine = math.cos(transform[2]), math.sin(transform[2])
    return x * cosine + y * sine, -x * sine + y * cosine


def rendezvous_slots(
    starts: dict[str, tuple[float, float, float]], spacing_m: float
) -> dict[str, tuple[float, float]]:
    """Place the fleet in ordered, collision-free slots around its centroid.

    Slots lie on the principal axis of the configured start positions and robot
    order along that axis is preserved.  Robots therefore do not cross through
    one another on their one deliberate common-observation leg.
    """
    if not starts:
        return {}
    ids = sorted(starts)
    positions = np.array([[starts[r][0], starts[r][1]] for r in ids], dtype=float)
    centre = positions.mean(axis=0)
    centred = positions - centre
    if len(ids) > 1 and float(np.linalg.norm(centred)) > 1e-9:
        _, _, vh = np.linalg.svd(centred, full_matrices=False)
        axis = vh[0]
    else:
        axis = np.array([1.0, 0.0])
    # Give the axis a deterministic sign, important for reproducible paths.
    if axis[0] < 0.0 or (abs(axis[0]) < 1e-9 and axis[1] < 0.0):
        axis = -axis
    by_id = {robot_id: positions[index] for index, robot_id in enumerate(ids)}
    ordered = sorted(ids, key=lambda rid: float(np.dot(by_id[rid], axis)))
    offsets = (np.arange(len(ids), dtype=float) - (len(ids) - 1) / 2.0) * spacing_m
    return {
        rid: tuple((centre + axis * offsets[index]).tolist())
        for index, rid in enumerate(ordered)
    }


def merge_grids(
    grids: dict[str, GridSnapshot],
    transforms: dict[str, tuple[float, float, float]],
    *,
    size_m: float,
    resolution: float,
    centre: tuple[float, float] = (0.0, 0.0),
    occupied_wins: bool = True,
) -> CommonGrid:
    """Rasterize known cells from robot maps into one conservative union.

    Occupied wins over free by default. ``occupied_wins=False`` builds an
    optimistic reachability layer in which any free observation clears a cell;
    it is suitable only for auction costs because Nav2 remains the full-
    resolution collision authority. Observation count is at most one per robot
    per output cell, even when rotation maps several source cells onto the same
    cell. This makes redundant-coverage statistics meaningful.
    """
    if size_m <= 0.0 or resolution <= 0.0:
        raise ValueError("size and resolution must be positive")
    width = int(math.ceil(size_m / resolution))
    height = width
    origin_x = float(centre[0]) - width * resolution / 2.0
    origin_y = float(centre[1]) - height * resolution / 2.0
    merged = np.full((height, width), UNKNOWN, dtype=np.int8)
    observations = np.zeros((height, width), dtype=np.uint8)

    for robot_id in sorted(grids):
        transform = transforms.get(robot_id)
        if transform is None:
            continue
        grid = grids[robot_id]
        rows, cols = np.nonzero(grid.cells >= 0)
        if not len(rows):
            continue

        # OccupancyGrid origins are poses, not merely lower-left translations.
        gx = (cols.astype(float) + 0.5) * grid.resolution
        gy = (rows.astype(float) + 0.5) * grid.resolution
        oc, os = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
        local_x = grid.origin_x + gx * oc - gy * os
        local_y = grid.origin_y + gx * os + gy * oc

        tc, ts = math.cos(transform[2]), math.sin(transform[2])
        world_x = transform[0] + local_x * tc - local_y * ts
        world_y = transform[1] + local_x * ts + local_y * tc
        out_cols = np.floor((world_x - origin_x) / resolution).astype(np.int64)
        out_rows = np.floor((world_y - origin_y) / resolution).astype(np.int64)
        valid = (
            (out_cols >= 0)
            & (out_cols < width)
            & (out_rows >= 0)
            & (out_rows < height)
        )
        if not np.any(valid):
            continue
        out_cols, out_rows = out_cols[valid], out_rows[valid]
        values = grid.cells[rows[valid], cols[valid]]
        flat = out_rows * width + out_cols
        unique_known = np.unique(flat)
        obs_flat = observations.reshape(-1)
        obs_flat[unique_known] = np.minimum(
            255, obs_flat[unique_known].astype(np.uint16) + 1
        ).astype(np.uint8)

        merged_flat = merged.reshape(-1)
        free_flat = np.unique(flat[values <= FREE_MAX])
        occupied_flat = np.unique(flat[values >= OCCUPIED_MIN])
        if occupied_wins:
            unset = merged_flat[free_flat] < 0
            merged_flat[free_flat[unset]] = 0
            merged_flat[occupied_flat] = 100
        else:
            unset = merged_flat[occupied_flat] < 0
            merged_flat[occupied_flat[unset]] = 100
            merged_flat[free_flat] = 0

    return CommonGrid(merged, observations, resolution, origin_x, origin_y)


def _dilate(mask: np.ndarray, iterations: int = 1) -> np.ndarray:
    """Eight-connected binary dilation using only NumPy."""
    result = np.asarray(mask, dtype=bool).copy()
    for _ in range(max(0, int(iterations))):
        padded = np.pad(result, 1, mode="constant", constant_values=False)
        neighbours = np.zeros_like(result)
        for dr in range(3):
            for dc in range(3):
                neighbours |= padded[
                    dr : dr + result.shape[0], dc : dc + result.shape[1]
                ]
        result = neighbours
    return result


def _components(mask: np.ndarray) -> list[list[tuple[int, int]]]:
    """Eight-connected components, visiting only true cells."""
    remaining = {tuple(index) for index in np.argwhere(mask)}
    groups: list[list[tuple[int, int]]] = []
    while remaining:
        seed = remaining.pop()
        queue = [seed]
        group = [seed]
        while queue:
            row, col = queue.pop()
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    neighbour = (row + dr, col + dc)
                    if neighbour in remaining:
                        remaining.remove(neighbour)
                        queue.append(neighbour)
                        group.append(neighbour)
        groups.append(group)
    return groups


def extract_frontiers(
    grid: CommonGrid,
    *,
    clearance_m: float,
    gain_radius_m: float = 4.0,
    min_cluster_cells: int = 5,
    min_separation_m: float = 1.5,
    max_candidates: int = 32,
) -> list[Frontier]:
    """Extract reachable free/unknown boundaries and representative viewpoints."""
    cells = grid.cells
    free = (cells >= 0) & (cells <= FREE_MAX)
    unknown = cells < 0
    adjacent_unknown = np.zeros_like(unknown)
    adjacent_unknown[1:, :] |= unknown[:-1, :]
    adjacent_unknown[:-1, :] |= unknown[1:, :]
    adjacent_unknown[:, 1:] |= unknown[:, :-1]
    adjacent_unknown[:, :-1] |= unknown[:, 1:]

    occupied = cells >= OCCUPIED_MIN
    inflation_cells = int(math.ceil(max(0.0, clearance_m) / grid.resolution))
    safe = ~_dilate(occupied, inflation_cells)
    frontier_mask = free & adjacent_unknown & safe
    groups = _components(frontier_mask)

    gain_cells = max(1, int(math.ceil(gain_radius_m / grid.resolution)))
    candidates: list[Frontier] = []
    for group in groups:
        if len(group) < min_cluster_cells:
            continue
        array = np.asarray(group, dtype=np.int64)
        centroid = array.mean(axis=0)
        # A medoid is guaranteed to remain on known free space.  A geometric
        # centroid can land across a wall or just inside unknown space.
        squared = np.sum((array - centroid) ** 2, axis=1)
        row, col = (int(value) for value in array[int(np.argmin(squared))])
        r0, r1 = max(0, row - gain_cells), min(grid.height, row + gain_cells + 1)
        c0, c1 = max(0, col - gain_cells), min(grid.width, col + gain_cells + 1)
        rr, cc = np.ogrid[r0:r1, c0:c1]
        disc = (rr - row) ** 2 + (cc - col) ** 2 <= gain_cells**2
        unknown_cells = int(np.count_nonzero(unknown[r0:r1, c0:c1] & disc))
        x, y = grid.point(row, col)
        candidates.append(
            Frontier(
                row=row,
                col=col,
                x=x,
                y=y,
                cells=len(group),
                information_m2=unknown_cells * grid.resolution**2,
            )
        )

    # Non-maximum suppression keeps two fragments of one doorway from being
    # assigned to two robots.  Prefer information, then boundary length.
    candidates.sort(key=lambda item: (item.information_m2, item.cells), reverse=True)
    kept: list[Frontier] = []
    for candidate in candidates:
        if any(
            math.hypot(candidate.x - other.x, candidate.y - other.y)
            < min_separation_m
            for other in kept
        ):
            continue
        kept.append(candidate)
        if len(kept) >= max_candidates:
            break
    return kept


def _nearest_true(mask: np.ndarray, row: int, col: int) -> tuple[int, int] | None:
    """Find a nearby traversable cell without a whole-grid distance transform."""
    height, width = mask.shape
    if 0 <= row < height and 0 <= col < width and mask[row, col]:
        return row, col
    for radius in range(1, max(height, width)):
        r0, r1 = max(0, row - radius), min(height - 1, row + radius)
        c0, c1 = max(0, col - radius), min(width - 1, col + radius)
        perimeter = []
        perimeter.extend((r0, c) for c in range(c0, c1 + 1))
        perimeter.extend((r1, c) for c in range(c0, c1 + 1))
        perimeter.extend((r, c0) for r in range(r0 + 1, r1))
        perimeter.extend((r, c1) for r in range(r0 + 1, r1))
        valid = [(r, c) for r, c in perimeter if mask[r, c]]
        if valid:
            return min(valid, key=lambda rc: (rc[0] - row) ** 2 + (rc[1] - col) ** 2)
    return None


def _coarse_navigation_grid(
    grid: CommonGrid, clearance_m: float, target_resolution_m: float = 0.20
) -> tuple[np.ndarray, int]:
    free = (grid.cells >= 0) & (grid.cells <= FREE_MAX)
    occupied = grid.cells >= OCCUPIED_MIN
    inflated = _dilate(
        occupied, int(math.ceil(max(0.0, clearance_m) / grid.resolution))
    )
    safe = free & ~inflated
    factor = max(1, int(round(target_resolution_m / grid.resolution)))
    if factor == 1:
        return safe, factor
    height = safe.shape[0] // factor
    width = safe.shape[1] // factor
    if height == 0 or width == 0:
        return safe, 1
    trimmed = safe[: height * factor, : width * factor]
    # One safe fine cell is enough for the allocation cost.  Nav2 remains the
    # collision authority and plans at full resolution before any motion.
    coarse = trimmed.reshape(height, factor, width, factor).any(axis=(1, 3))
    return coarse, factor


def _wavefront(
    traversable: np.ndarray, start: tuple[int, int], step_m: float
) -> np.ndarray:
    """Eight-connected Dijkstra distance over known free space."""
    import heapq

    distance = np.full(traversable.shape, np.inf, dtype=np.float32)
    if not traversable[start]:
        return distance
    distance[start] = 0.0
    queue: list[tuple[float, int, int]] = [(0.0, start[0], start[1])]
    neighbours = (
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (1, 1, math.sqrt(2.0)),
    )
    height, width = traversable.shape
    while queue:
        cost, row, col = heapq.heappop(queue)
        if cost > float(distance[row, col]) + 1e-6:
            continue
        for dr, dc, multiplier in neighbours:
            nr, nc = row + dr, col + dc
            if not (0 <= nr < height and 0 <= nc < width and traversable[nr, nc]):
                continue
            new_cost = cost + step_m * multiplier
            if new_cost + 1e-6 < float(distance[nr, nc]):
                distance[nr, nc] = new_cost
                heapq.heappush(queue, (new_cost, nr, nc))
    return distance


def allocate_frontiers(
    grid: CommonGrid,
    robots: list[RobotState],
    frontiers: list[Frontier],
    *,
    reserved: list[tuple[float, float]] | None = None,
    goal_separation_m: float = 2.5,
    active_handoff_margin_m: float = 4.0,
    candidate_limit: int = 20,
    navigation_grid: CommonGrid | None = None,
) -> list[Assignment]:
    """Jointly assign distinct, reachable frontiers to the small fleet.

    The objective is information gain per geodesic travel cost.  For four
    robots and at most twenty candidates, exhaustive assignment is small
    (20P4 = 116,280) and avoids the avoidable conflicts of independent greedy
    choices.  Existing robot goals are predictive reservations: another robot
    cannot select a frontier whose sensing disc substantially overlaps them.
    An idle robot also defers a long deadhead when an active goal will finish
    materially closer to that frontier.  With no reservations the gate is
    absent, so it cannot prevent eventual completion.
    """
    if not robots or not frontiers:
        return []
    navigation = navigation_grid or grid
    if (
        navigation.cells.shape != grid.cells.shape
        or not math.isclose(navigation.resolution, grid.resolution)
        or not math.isclose(navigation.origin_x, grid.origin_x)
        or not math.isclose(navigation.origin_y, grid.origin_y)
    ):
        raise ValueError("navigation grid must have the frontier grid geometry")
    reservations = list(reserved or [])
    candidates = [
        candidate
        for candidate in frontiers
        if not any(
            math.hypot(candidate.x - x, candidate.y - y) < goal_separation_m
            for x, y in reservations
        )
    ][: max(1, candidate_limit)]
    if not candidates:
        return []

    costs = np.full((len(robots), len(candidates)), np.inf, dtype=np.float64)
    paths = np.full_like(costs, np.inf)
    for robot_index, robot in enumerate(robots):
        clearance = (
            robot.navigation_clearance_m
            if robot.navigation_clearance_m is not None
            else robot.radius_m + 0.12
        )
        traversable, factor = _coarse_navigation_grid(
            navigation, clearance_m=clearance
        )
        start_row, start_col = navigation.cell(robot.x, robot.y)
        start = _nearest_true(traversable, start_row // factor, start_col // factor)
        if start is None:
            continue
        distances = _wavefront(traversable, start, navigation.resolution * factor)
        for candidate_index, candidate in enumerate(candidates):
            target = _nearest_true(
                traversable, candidate.row // factor, candidate.col // factor
            )
            if target is None:
                continue
            path = float(distances[target])
            if not math.isfinite(path):
                continue
            if reservations:
                active_goal_distance = min(
                    math.hypot(candidate.x - x, candidate.y - y)
                    for x, y in reservations
                )
                if path > active_goal_distance + active_handoff_margin_m:
                    # A teammate already in flight will finish closer. Wait
                    # for the next auction instead of paying avoidable fleet
                    # deadhead merely to maximize current cardinality.
                    continue
            # Unit information gain: long deadhead travel must buy proportionally
            # more unknown area.  sqrt prevents one huge, occluded unknown disc
            # from dominating every reachable nearby frontier.
            gain = max(0.05, math.sqrt(max(candidate.information_m2, 0.0)))
            paths[robot_index, candidate_index] = path
            costs[robot_index, candidate_index] = (path + 0.75) / (1.0 + gain)

    best: tuple[float, tuple[int, ...], tuple[int, ...]] | None = None
    # If one robot is temporarily isolated, allocate the reachable subset
    # instead of idling the whole fleet. Prefer more concurrently working
    # robots, then the minimum joint utility cost within that cardinality.
    for assign_count in range(min(len(robots), len(candidates)), 0, -1):
        robot_subsets = itertools.combinations(range(len(robots)), assign_count)
        for robot_indices in robot_subsets:
            candidate_orders = itertools.permutations(
                range(len(candidates)), assign_count
            )
            for candidate_indices in candidate_orders:
                values = [
                    costs[robot, candidate]
                    for robot, candidate in zip(robot_indices, candidate_indices)
                ]
                if not all(math.isfinite(value) for value in values):
                    continue
                total = float(sum(values))
                if best is None or total < best[0]:
                    best = (total, tuple(robot_indices), tuple(candidate_indices))
        if best is not None:
            break
    if best is None:
        return []

    assignments = []
    for robot_index, candidate_index in zip(best[1], best[2]):
        candidate = candidates[candidate_index]
        cost = float(costs[robot_index, candidate_index])
        assignments.append(
            Assignment(
                robot_id=robots[robot_index].robot_id,
                frontier=candidate,
                path_cost_m=float(paths[robot_index, candidate_index]),
                utility=1.0 / max(cost, 1e-9),
            )
        )
    return sorted(assignments, key=lambda item: item.robot_id)


def frontier_near(
    frontiers: list[Frontier], point: tuple[float, float], radius_m: float
) -> bool:
    return any(
        math.hypot(item.x - point[0], item.y - point[1]) <= radius_m
        for item in frontiers
    )
