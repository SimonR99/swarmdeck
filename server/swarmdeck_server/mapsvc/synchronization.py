"""Short-lock publication of coherent map generations.

The compositor mutates a working NumPy grid in a worker thread while HTTP and
websocket readers run on the event loop.  ``SnapshotStore`` is the narrow
boundary between those worlds: it copies a complete generation under a small
lock, then lets rendering and compression happen without holding that lock.
"""

from __future__ import annotations

from dataclasses import dataclass
import threading

import numpy as np

from .grid_meta import GridMeta
from .snapshot import MapSnapshot, expand_patch_baseline, make_snapshot


@dataclass(frozen=True)
class PatchCapture:
    snapshot: MapSnapshot
    x0: int
    y0: int
    x1: int
    y1: int
    cells: np.ndarray
    seq: int


class SnapshotStore:
    """Publish and advance map snapshots without exposing mixed state."""

    def __init__(
        self, meta: GridMeta, merged: np.ndarray, previous: np.ndarray
    ) -> None:
        self._lock = threading.Lock()
        self._snapshot = make_snapshot(meta, merged, previous, 0)

    def get(self) -> MapSnapshot:
        with self._lock:
            return self._snapshot

    def publish(self, meta: GridMeta, merged: np.ndarray) -> MapSnapshot:
        """Copy one working-map generation and preserve the patch baseline."""
        with self._lock:
            previous = self._snapshot
            baseline = expand_patch_baseline(previous.patch_prev, previous.meta, meta)
            self._snapshot = make_snapshot(meta, merged, baseline, previous.seq)
            return self._snapshot

    def capture_patch(self) -> PatchCapture | None:
        """Advance the baseline atomically and return a render-ready patch."""
        with self._lock:
            snapshot = self._snapshot
            changed = snapshot.merged != snapshot.patch_prev
            if not changed.any():
                return None
            ys, xs = np.where(changed)
            y0, y1 = int(ys.min()), int(ys.max()) + 1
            x0, x1 = int(xs.min()), int(xs.max()) + 1
            cells = np.ascontiguousarray(snapshot.merged[y0:y1, x0:x1])
            seq = snapshot.seq + 1
            self._snapshot = MapSnapshot(
                snapshot.meta,
                snapshot.merged,
                snapshot.merged,
                seq,
            )
            return PatchCapture(snapshot, x0, y0, x1, y1, cells, seq)
