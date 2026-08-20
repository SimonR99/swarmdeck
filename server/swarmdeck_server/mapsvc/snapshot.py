"""Immutable publication snapshots for map readers.

Registration and extent expansion happen in worker threads.  The service keeps
its mutable working state private to those writers and publishes this compact,
read-only value for HTTP/websocket consumers.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .grid_meta import GridMeta

UNKNOWN = -1


@dataclass(frozen=True)
class MapSnapshot:
    """One coherent map, metadata, patch baseline, and sequence generation."""

    meta: GridMeta
    merged: np.ndarray
    patch_prev: np.ndarray
    seq: int


def copy_meta(meta: GridMeta) -> GridMeta:
    return GridMeta(meta.resolution, meta.width, meta.height, meta.origin_x, meta.origin_y)


def expand_patch_baseline(
    previous: np.ndarray, old_meta: GridMeta, new_meta: GridMeta
) -> np.ndarray:
    """Move a patch baseline into a newly expanded map extent."""
    if (
        old_meta.width == new_meta.width
        and old_meta.height == new_meta.height
        and old_meta.origin_x == new_meta.origin_x
        and old_meta.origin_y == new_meta.origin_y
    ):
        return previous.copy()

    baseline = np.full((new_meta.height, new_meta.width), UNKNOWN, dtype=np.int8)
    res = new_meta.resolution
    off_x = int(round((old_meta.origin_x - new_meta.origin_x) / res))
    off_y = int(round((old_meta.origin_y - new_meta.origin_y) / res))
    src_x0 = max(0, -off_x)
    src_y0 = max(0, -off_y)
    dst_x0 = max(0, off_x)
    dst_y0 = max(0, off_y)
    copy_w = min(old_meta.width - src_x0, new_meta.width - dst_x0)
    copy_h = min(old_meta.height - src_y0, new_meta.height - dst_y0)
    if copy_w > 0 and copy_h > 0:
        baseline[
            dst_y0 : dst_y0 + copy_h,
            dst_x0 : dst_x0 + copy_w,
        ] = previous[src_y0 : src_y0 + copy_h, src_x0 : src_x0 + copy_w]
    return baseline


def make_snapshot(
    meta: GridMeta,
    merged: np.ndarray,
    patch_prev: np.ndarray,
    seq: int,
) -> MapSnapshot:
    """Copy and freeze one coherent map/meta/baseline/sequence value."""
    frozen_merged = np.array(merged, dtype=np.int8, copy=True)
    frozen_prev = np.array(patch_prev, dtype=np.int8, copy=True)
    frozen_merged.setflags(write=False)
    frozen_prev.setflags(write=False)
    return MapSnapshot(copy_meta(meta), frozen_merged, frozen_prev, seq)
