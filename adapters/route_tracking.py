"""ROS-independent monotonic route tracking with checked forward rejoining.

A checker is required: local avoidance alone cannot justify skipping route bends.
The map check supplements, and never bypasses, the onboard collision controller.
"""

from __future__ import annotations

import math
import numpy as np


def segment_cells(a, b):
    """All cells touched by a grid-coordinate segment, including corner touches."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    cuts = {0.0, 1.0}
    for start, delta, end in ((a[0], dx, b[0]), (a[1], dy, b[1])):
        if delta:
            cuts.update(
                (k - start) / delta
                for k in range(
                    math.ceil(min(start, end)), math.floor(max(start, end)) + 1
                )
            )
    cuts = sorted(cuts)
    for t in cuts + [(lo + hi) / 2 for lo, hi in zip(cuts, cuts[1:])]:
        x, y = a[0] + t * dx, a[1] + t * dy
        xs = {round(x) - 1, round(x)} if abs(x - round(x)) < 1e-9 else {math.floor(x)}
        ys = {round(y) - 1, round(y)} if abs(y - round(y)) < 1e-9 else {math.floor(y)}
        for cx in xs:
            for cy in ys:
                yield cx, cy


class GridCollisionChecker:
    """Conservative square clearance, matching the global grid planner.

    Unknown cells and map boundaries block shortcuts. Input is the original,
    uncarved navigation-map snapshot, in the robot's navigation frame.
    """

    def __init__(self, snapshot, clearance_m):
        self.map = snapshot
        if not math.isfinite(clearance_m) or clearance_m < 0:
            raise ValueError("invalid route clearance")
        if not math.isfinite(snapshot.resolution) or snapshot.resolution <= 0:
            raise ValueError("invalid map resolution")
        self.radius = math.ceil(clearance_m / snapshot.resolution)

    def __call__(self, a, b):
        m = self.map
        pts = [
            ((p[0] - m.origin_x) / m.resolution, (p[1] - m.origin_y) / m.resolution)
            for p in (a, b)
        ]
        if any(not math.isfinite(v) for p in pts for v in p):
            return False
        h, w = m.cells.shape
        if any(not (0 <= x < w and 0 <= y < h) for x, y in pts):
            return False
        r = self.radius
        for x, y in segment_cells(*pts):
            if x - r < 0 or y - r < 0 or x + r >= w or y + r >= h:
                return False
            cells = m.cells[y - r : y + r + 1, x - r : x + r + 1]
            if np.any((cells < 0) | (cells >= 50)):
                return False
        return True


class RouteTracker:
    def __init__(self, path, lookahead_m=0.8):
        self.points = []
        for p in path:
            point = (float(p["x"]), float(p["y"]))
            if not all(math.isfinite(v) for v in point):
                raise ValueError("nonfinite route point")
            if not self.points or math.dist(self.points[-1], point) > 1e-6:
                self.points.append(point)
        self.distances = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            self.distances.append(self.distances[-1] + math.dist(a, b))
        if not math.isfinite(lookahead_m) or lookahead_m <= 0:
            raise ValueError("invalid lookahead")
        self.lookahead = lookahead_m
        self.progress = 0.0
        self.target_progress = 0.0
        self.blocked_reason = ""

    def point_at(self, distance):
        for i, end in enumerate(self.distances[1:]):
            if distance <= end:
                a, b = self.points[i : i + 2]
                fraction = (distance - self.distances[i]) / (end - self.distances[i])
                return (
                    a[0] + fraction * (b[0] - a[0]),
                    a[1] + fraction * (b[1] - a[1]),
                )
        return self.points[-1]

    def target(self, pose, clear):
        if not self.points or clear is None:
            self.blocked_reason = "navigation map unavailable"
            return None
        position = (float(pose["x"]), float(pose["y"]))
        if not all(math.isfinite(v) for v in position):
            self.blocked_reason = "invalid robot pose"
            return None
        # A bounded, local projection prevents jumping to a faraway return leg
        # where a looping route crosses itself. Ties prefer the earlier leg.
        ceiling = min(self.distances[-1], self.progress + 2 * self.lookahead)
        best = (math.inf, self.progress)
        for i, end in enumerate(self.distances[1:]):
            start = self.distances[i]
            if end < self.progress or start > ceiling:
                continue
            a, b = self.points[i : i + 2]
            length = end - start
            projection = (
                (position[0] - a[0]) * (b[0] - a[0])
                + (position[1] - a[1]) * (b[1] - a[1])
            ) / length
            s = max(self.progress, start, min(end, ceiling, start + projection))
            best = min(best, (math.dist(position, self.point_at(s)), s))
        projected = best[1]
        desired = min(self.distances[-1], projected + self.lookahead)
        desired = max(self.target_progress, desired)
        # Try shorter checked connections when a long shortcut cuts a bend,
        # but never move the target backward along the route.
        floor = max(self.target_progress, projected + min(0.1, self.lookahead))
        candidates = [desired]
        s = desired - 0.1
        while s >= floor:
            candidates.append(s)
            s -= 0.1
        for s in candidates:
            target = self.point_at(s)
            if clear(position, target):
                self.progress = max(self.progress, projected)
                self.target_progress = s
                self.blocked_reason = ""
                return {"x": target[0], "y": target[1]}
        self.blocked_reason = "no collision-free connection to route ahead"
        return None
