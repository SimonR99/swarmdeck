"""Scan-to-grid raytracing tests — synthetic, deterministic, no ROS involved.

Covers the part most likely to be silently wrong: free space actually gets
marked (not just occupied points), occupied always wins over a later free
ray, and out-of-window points/origins are dropped rather than crashing.
"""

from __future__ import annotations

import numpy as np
import pytest

from swarmdeck_server.mapsvc.scan_grid import FREE, OCCUPIED, UNKNOWN, ScanGridAccumulator
from swarmdeck_server.mapsvc.service import MapService


def test_a_single_beam_marks_free_along_it_and_occupied_at_the_end():
    acc = ScanGridAccumulator(origin_x=0.0, origin_y=0.0, resolution=0.05, size_m=10.0)
    acc.integrate(0.0, 0.0, np.array([[1.0, 0.0]], dtype=np.float32))

    ox, oy = acc._to_cell(0.0, 0.0)
    hx, hy = acc._to_cell(1.0, 0.0)
    mx, my = acc._to_cell(0.5, 0.0)  # halfway along the beam

    assert acc.cells[hy, hx] == OCCUPIED
    assert acc.cells[my, mx] == FREE
    assert acc.cells[oy, ox] == FREE  # the robot's own cell: passed through, not hit
    # A cell nowhere near the beam stays unknown.
    fx, fy = acc._to_cell(-2.0, 3.0)
    assert acc.cells[fy, fx] == UNKNOWN


def test_occupied_is_never_downgraded_by_a_later_free_ray():
    """A wall seen once must not flicker to free because a later beam grazes it.

    This is the precedence `MapService._remerge` also uses when merging
    robots: occupied wins over free, free wins over unknown.
    """
    acc = ScanGridAccumulator(origin_x=0.0, origin_y=0.0, resolution=0.05, size_m=10.0)
    # First beam hits (1.0, 0.0) - occupied.
    acc.integrate(0.0, 0.0, np.array([[1.0, 0.0]], dtype=np.float32))
    # Second beam, from a different origin, passes straight through that same
    # cell on its way to a farther return.
    acc.integrate(1.0, 0.0, np.array([[2.0, 0.0]], dtype=np.float32))

    hx, hy = acc._to_cell(1.0, 0.0)
    assert acc.cells[hy, hx] == OCCUPIED


def test_points_and_origin_outside_the_window_are_dropped_not_crashed():
    acc = ScanGridAccumulator(origin_x=0.0, origin_y=0.0, resolution=0.05, size_m=4.0)
    # Point far outside the 4 m window.
    acc.integrate(0.0, 0.0, np.array([[100.0, 100.0]], dtype=np.float32))
    assert (acc.cells == OCCUPIED).sum() == 0

    # Origin itself outside the window: nothing to raytrace from, no crash.
    acc.integrate(500.0, 500.0, np.array([[500.5, 500.0]], dtype=np.float32))
    assert (acc.cells != UNKNOWN).sum() == 0


def test_empty_points_is_a_noop():
    acc = ScanGridAccumulator(origin_x=0.0, origin_y=0.0, resolution=0.05, size_m=4.0)
    acc.integrate(0.0, 0.0, np.zeros((0, 2), dtype=np.float32))
    assert (acc.cells != UNKNOWN).sum() == 0


def test_ingest_scan_feeds_the_same_pipeline_a_native_grid_uses():
    """A scan-built grid must be indistinguishable, downstream, from a robot
    that publishes its own OccupancyGrid — same merge/registration path."""
    svc = MapService(resolution=0.05, size_m=10.0)
    points = np.array([[1.0, 0.0], [0.0, 1.0], [-1.0, 0.0], [0.0, -1.0]], dtype=np.float32)
    svc.ingest_scan("r0", 0.0, 0.0, points)

    assert "r0" in svc.robot_grids
    meta, cells = svc.robot_grids["r0"]
    assert cells.dtype == np.int8
    assert (cells == OCCUPIED).any()
    assert (cells == FREE).any()
    # static mode (the default): an ingested robot is a global member immediately.
    assert "r0" in svc.global_members()


def test_a_second_scan_from_the_same_robot_accumulates_not_replaces():
    svc = MapService(resolution=0.05, size_m=10.0)
    svc.ingest_scan("r0", 0.0, 0.0, np.array([[1.0, 0.0]], dtype=np.float32))
    svc.ingest_scan("r0", 0.0, 0.0, np.array([[0.0, 1.0]], dtype=np.float32))

    _, cells = svc.robot_grids["r0"]
    assert (cells == OCCUPIED).sum() == 2, "both scans' hits must still be present"
