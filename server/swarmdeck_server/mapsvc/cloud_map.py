"""Bounded, immutable display history, independent of registration's latest scan."""

from dataclasses import dataclass

import numpy as np


def voxel_keys(points, size):
    keys = np.floor(points.astype(np.float64) / size).astype(np.int64)
    return np.ascontiguousarray(keys).view("V24").ravel()


@dataclass(frozen=True)
class CloudMap:
    points: np.ndarray
    colors: np.ndarray
    voxel_size: float = 0.10

    @classmethod
    def empty(cls):
        return cls(np.empty((0, 3), np.float32), np.empty((0, 3), np.uint8))

    def add(self, points, rgb=None, *, max_points=500_000):
        """Keep first surfaces/colors, coarsening at capacity instead of losing rooms.

        Sorted voxel keys make reobservations a scan-sized lookup. Only new
        geometry allocates/sorts a map. Grey (148) is the legacy upload protocol's
        unobserved-color sentinel; later uncolored scans cannot erase camera RGB.
        """
        if not len(points):
            return self
        incoming, first = np.unique(
            voxel_keys(points, self.voxel_size), return_index=True
        )
        colors = np.full(points.shape, 148, np.uint8) if rgb is None else rgb
        # Prefer the first measured color within each incoming voxel.
        colored = np.flatnonzero(np.any(colors != 148, axis=1))
        color_first = first.copy()
        if len(colored):
            keys, indices = np.unique(
                voxel_keys(points[colored], self.voxel_size), return_index=True
            )
            color_first[np.searchsorted(incoming, keys)] = colored[indices]
        points, colors = points[first], colors[color_first]
        existing = voxel_keys(self.points, self.voxel_size)
        positions = np.searchsorted(existing, incoming)
        present = positions < len(existing)
        present[present] &= existing[positions[present]] == incoming[present]
        upgrades = np.zeros(len(points), bool)
        if present.any():
            upgrades[present] = np.all(
                self.colors[positions[present]] == 148, axis=1
            ) & np.any(colors[present] != 148, axis=1)
        if present.all() and not upgrades.any():
            return self
        old_colors = self.colors
        if upgrades.any():
            old_colors = old_colors.copy()
            old_colors[positions[upgrades]] = colors[upgrades]
        if present.all():
            return CloudMap(self.points, old_colors, self.voxel_size)
        xyz = np.concatenate((self.points, points[~present]))
        rgb_out = np.concatenate((old_colors, colors[~present]))
        keys = np.concatenate((existing, incoming[~present]))
        size = self.voxel_size
        if len(xyz) > max_points:
            size *= 2
            # Reuse the same color-preserving aggregation on a coarser lattice.
            reduced = CloudMap(
                np.empty((0, 3), np.float32), np.empty((0, 3), np.uint8), size
            ).add(xyz, rgb_out, max_points=max_points)
            return reduced
        order = np.argsort(keys)
        return CloudMap(xyz[order], rgb_out[order], size)
