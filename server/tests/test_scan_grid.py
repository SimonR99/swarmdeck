"""Scan-to-grid raytracing tests — synthetic, deterministic, no ROS involved.

Covers the part most likely to be silently wrong: free space actually gets
marked (not just occupied points), occupied always wins over a later free
ray, and out-of-window points/origins are dropped rather than crashing.
"""

from __future__ import annotations

import numpy as np
import pytest

from swarmdeck_server.mapsvc.scan_grid import (
    EVIDENCE_CLAMP,
    FREE,
    FREE_DECAY,
    OCCUPIED,
    UNKNOWN,
    ScanGridAccumulator,
)
from swarmdeck_server.mapsvc.service import MapService


def test_a_single_beam_marks_free_along_it_and_occupied_at_the_end():
    """One beam clears its own path. Load-bearing, not cosmetic.

    Withholding free space until a second observation was tried on the live
    fleet and reverted: grid registration keys on known-free contradiction
    (`docs/architecture/collaborative-slam.md` §2.2), and the delay collapsed pairwise
    overlap from 1252 cells to 26 and emptied the merged map. See the gain
    constants in scan_grid.py.
    """
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


def test_a_dense_scan_clears_its_neighbourhood_in_one_pass():
    """Corroboration must not mean a real scan leaves the map unknown.

    Every beam of a 360-degree scan crosses the cells near the sensor, so the
    robot's own neighbourhood is confirmed free on the first scan. Only the
    far tip of an isolated ray stays unconfirmed, which is the point.
    """
    acc = ScanGridAccumulator(origin_x=0.0, origin_y=0.0, resolution=0.05, size_m=10.0)
    angles = np.linspace(-np.pi, np.pi, 360, endpoint=False)
    points = np.column_stack([3.0 * np.cos(angles), 3.0 * np.sin(angles)]).astype(
        np.float32
    )
    acc.integrate(0.0, 0.0, points)

    ox, oy = acc._to_cell(0.0, 0.0)
    assert acc.cells[oy, ox] == FREE, "the sensor's own cell must read free"
    assert int((acc.cells == FREE).sum()) > 500, "a dense scan must clear real area"


def test_a_ghost_can_be_cleared_by_driving_through_it():
    """Occupied must be revisable, or every passer-by is a permanent wall.

    `MapService._remerge` calls this out as the failure its majority vote exists
    to undo — but the vote can only erase a ghost another robot has seen through,
    and the per-robot accumulator that feeds it used to make every ghost
    immortal at source.
    """
    acc = ScanGridAccumulator(origin_x=0.0, origin_y=0.0, resolution=0.05, size_m=10.0)
    acc.integrate(0.0, 0.0, np.array([[1.0, 0.0]], dtype=np.float32))
    gx, gy = acc._to_cell(1.0, 0.0)
    assert acc.cells[gy, gx] == OCCUPIED

    # Whatever it was has moved on: beams now reach a wall beyond it.
    for _ in range(10):
        acc.integrate(0.0, 0.0, np.array([[2.0, 0.0]], dtype=np.float32))

    assert acc.cells[gy, gx] == FREE, "a ghost must not outlive the evidence for it"


def test_unobserved_free_space_fades_and_occupied_does_not():
    """Historical white must not outlive the pose that painted it.

    The accumulator cannot move old rays when SuperOdom / LIO-SAM loop-closes,
    so a room mapped before the jump stays as a white rectangle at the old
    coordinates. Occupied cells are the map and stay; free cells that no
    current beam crosses decay back to unknown.
    """
    acc = ScanGridAccumulator(origin_x=0.0, origin_y=0.0, resolution=0.05, size_m=10.0)
    # Paint a short beam at +x, then clamp its free evidence.
    for _ in range(EVIDENCE_CLAMP):
        acc.integrate(0.0, 0.0, np.array([[1.0, 0.0]], dtype=np.float32))
    mx, my = acc._to_cell(0.5, 0.0)
    hx, hy = acc._to_cell(1.0, 0.0)
    assert acc.cells[my, mx] == FREE
    assert acc.cells[hy, hx] == OCCUPIED

    scans_to_fade = int(np.ceil(EVIDENCE_CLAMP / FREE_DECAY))
    for _ in range(scans_to_fade):
        # Beams the other way: they do not cross the old +x cells.
        acc.integrate(0.0, 0.0, np.array([[0.0, 1.0]], dtype=np.float32))

    assert acc.cells[my, mx] == UNKNOWN, "unobserved free space must fade"
    assert acc.cells[hy, hx] == OCCUPIED, "walls persist when nobody is looking"


def test_retained_free_space_stays_white_while_unknown_stays_unknown():
    """Hardware profiles can retain explored floor without painting the void."""
    acc = ScanGridAccumulator(
        origin_x=0.0,
        origin_y=0.0,
        resolution=0.05,
        size_m=10.0,
        retain_free_space=True,
    )
    for _ in range(EVIDENCE_CLAMP):
        acc.integrate(0.0, 0.0, np.array([[1.0, 0.0]], dtype=np.float32))
    mx, my = acc._to_cell(0.5, 0.0)
    hx, hy = acc._to_cell(1.0, 0.0)
    ux, uy = acc._to_cell(-1.0, -1.0)

    for _ in range(EVIDENCE_CLAMP):
        acc.integrate(0.0, 0.0, np.array([[0.0, 1.0]], dtype=np.float32))

    assert acc.cells[my, mx] == FREE
    assert acc.cells[hy, hx] == OCCUPIED
    assert acc.cells[uy, ux] == UNKNOWN


def test_observed_free_space_does_not_fade():
    """A 360° lidar re-confirms visible free space; decay must not eat it."""
    acc = ScanGridAccumulator(origin_x=0.0, origin_y=0.0, resolution=0.05, size_m=10.0)
    acc.integrate(0.0, 0.0, np.array([[1.0, 0.0]], dtype=np.float32))
    mx, my = acc._to_cell(0.5, 0.0)
    for _ in range(EVIDENCE_CLAMP):
        acc.integrate(0.0, 0.0, np.array([[1.0, 0.0]], dtype=np.float32))
    assert acc.cells[my, mx] == FREE


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


def test_outlier_point_beyond_max_range_is_dropped():
    acc = ScanGridAccumulator(origin_x=0.0, origin_y=0.0, resolution=0.05, size_m=4.0)
    # Point far outside max lidar range (e.g. 100m, 100m).
    acc.integrate(0.0, 0.0, np.array([[100.0, 100.0]], dtype=np.float32))
    assert (acc.cells == OCCUPIED).sum() == 0


def test_robot_outside_initial_window_expands_grid_and_updates_map():
    """When a robot drives beyond its initial window, the grid dynamically
    expands so scans continue to update the map with free and occupied cells."""
    acc = ScanGridAccumulator(origin_x=0.0, origin_y=0.0, resolution=0.05, size_m=4.0)
    # Initial window is [-2.0, 2.0] in x and y.
    # Robot drives to (10.0, 0.0) (well outside the 4m window) and returns a hit at (10.5, 0.0).
    acc.integrate(10.0, 0.0, np.array([[10.5, 0.0]], dtype=np.float32))

    hx, hy = acc._to_cell(10.5, 0.0)
    ox, oy = acc._to_cell(10.0, 0.0)
    assert acc.cells[hy, hx] == OCCUPIED
    assert acc.cells[oy, ox] == FREE
    assert (acc.cells == OCCUPIED).sum() >= 1


def test_empty_points_is_a_noop():
    acc = ScanGridAccumulator(origin_x=0.0, origin_y=0.0, resolution=0.05, size_m=4.0)
    acc.integrate(0.0, 0.0, np.zeros((0, 2), dtype=np.float32))
    assert (acc.cells != UNKNOWN).sum() == 0


def test_ingest_scan_feeds_the_same_pipeline_a_native_grid_uses():
    """A scan-built grid must be indistinguishable, downstream, from a robot
    that publishes its own OccupancyGrid — same merge/registration path."""
    svc = MapService(resolution=0.05, size_m=10.0)
    points = np.array(
        [[1.0, 0.0], [0.0, 1.0], [-1.0, 0.0], [0.0, -1.0]], dtype=np.float32
    )
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


def test_reset_forgets_the_accumulated_scan_grid():
    """A fleet reset must not leave a scan-fed robot's whole map behind.

    `_scan_grids` IS the map for a robot with no OccupancyGrid of its own — the
    hardware path for both Bunkers and for tars. `MapService.reset()` cleared
    every OTHER map product but left the accumulator standing, so the next scan
    to arrive re-ingested the entire pre-reset grid: the "old map comes straight
    back" failure the reset ordering exists to prevent, arriving by a route the
    ordering could not cover. Invisible in simulation, where nothing uses this
    path at all.
    """
    from swarmdeck_server.mapsvc.service import MapService

    service = MapService(resolution=0.05, size_m=40.0)
    service.set_mode("static")
    service.ingest_scan(
        "botman_0",
        0.0,
        0.0,
        np.array([[1.0, 0.0], [1.0, 0.1], [1.0, 0.2]], dtype=np.float32),
    )
    assert int((service.merged >= 50).sum()) == 3

    service.reset()
    assert (service.merged == -1).all()
    assert "botman_0" not in service._scan_grids

    # One new return must produce exactly one occupied cell, not four.
    service.ingest_scan("botman_0", 0.0, 0.0, np.array([[2.0, 0.0]], dtype=np.float32))
    assert int((service.merged >= 50).sum()) == 1


def test_scan_window_follows_the_configured_map_extent():
    """The per-robot window must not silently stay at its own 40 m default."""
    from swarmdeck_server.mapsvc.service import MapService

    service = MapService(resolution=0.05, size_m=80.0)
    service.ingest_scan("r0", 0.0, 0.0, np.array([[1.0, 0.0]], dtype=np.float32))
    accumulator = service._scan_grids["r0"]
    assert accumulator.meta.width * accumulator.meta.resolution == 80.0


def test_a_stray_return_is_dropped_but_a_sparse_far_wall_is_kept():
    """The spike fix must not cost long-range perception.

    A stray return raytraces a free corridor from the sensor out past whatever
    it passed through — the long radial spikes on every live map. But real
    returns thin out linearly with range, so the obvious fix (drop returns with
    few nearby neighbours) deletes genuine far geometry: measured against live
    fleet scans a 0.30 m metric radius dropped 16% of tars_0's returns and
    capped its perception at 7.8 m. Comparing against ANGULAR neighbours instead
    is range-independent.
    """
    from swarmdeck_server.mapsvc.scan_grid import drop_range_outliers

    ang = np.linspace(-np.pi, np.pi, 360, endpoint=False)
    # A sparsely-but-evenly sampled wall at 15 m: every point is far from every
    # other in metres (0.26 m apart), yet none of them is a stray.
    far = np.column_stack([15.0 * np.cos(ang), 15.0 * np.sin(ang)]).astype(np.float32)
    assert len(drop_range_outliers(0.0, 0.0, far)) == len(
        far
    ), "a sparse but coherent far wall must survive intact"

    # Now push three returns 5 m out beyond their angular neighbours.
    strays = far.copy()
    for i in (10, 100, 250):
        strays[i] = far[i] * (20.0 / 15.0)
    kept = drop_range_outliers(0.0, 0.0, strays)
    assert (
        len(kept) == len(far) - 3
    ), f"expected 3 strays dropped, got {len(far) - len(kept)}"

    # A return NEARER than its neighbours is an obstacle in front of a wall and
    # must never be discarded — that is the one error that could hide a hazard.
    obstacles = far.copy()
    for i in (30, 200):
        obstacles[i] = far[i] * (3.0 / 15.0)
    assert len(drop_range_outliers(0.0, 0.0, obstacles)) == len(far)


def test_outlier_filter_is_a_noop_on_a_tiny_scan():
    from swarmdeck_server.mapsvc.scan_grid import drop_range_outliers

    tiny = np.array([[1.0, 0.0], [0.0, 1.0]], dtype=np.float32)
    assert len(drop_range_outliers(0.0, 0.0, tiny)) == 2
    assert len(drop_range_outliers(0.0, 0.0, np.zeros((0, 2), np.float32))) == 0


def test_ingest_scan_outside_initial_map_extent_expands_merged_map():
    """When a robot explores beyond the initial MapService extent, the merged map
    and accumulator both expand and update without dropping scans."""
    svc = MapService(resolution=0.1, size_m=10.0)
    svc.set_mode("static")
    svc.set_transform("r0", 0.0, 0.0, 0.0)

    # Initial extent is [-5.0, 5.0] in x and y.
    # Ingest a scan from robot at (20.0, 0.0) with returns at (21.0, 0.0).
    svc.ingest_scan("r0", 20.0, 0.0, np.array([[21.0, 0.0]], dtype=np.float32))

    # The accumulator and merged map should now cover x=21.0
    assert "r0" in svc.robot_grids
    meta, cells = svc.robot_grids["r0"]
    assert meta.origin_x + meta.width * meta.resolution >= 21.0
    assert (cells == OCCUPIED).any()

    # Merged map should also cover x=21.0 and contain the occupied return
    assert svc.meta.origin_x + svc.meta.width * svc.meta.resolution >= 21.0
    assert (svc.merged == OCCUPIED).any()

    patch = svc.take_patch()
    assert patch is not None
    assert patch["type"] == "map_patch"
    assert patch["width"] == svc.meta.width
    assert patch["height"] == svc.meta.height


def test_map_reset_restores_initial_extent_after_expansion():
    """Resetting after expansion returns the service to its configured initial bounds."""
    svc = MapService(resolution=0.1, size_m=10.0)
    svc.set_mode("static")
    svc.ingest_scan("r0", 30.0, 0.0, np.array([[31.0, 0.0]], dtype=np.float32))
    assert svc.meta.width * svc.meta.resolution > 10.0

    svc.reset()
    assert (svc.merged == UNKNOWN).all()
    assert svc.meta.width * svc.meta.resolution == 10.0
    assert svc.meta.origin_x == -5.0
    assert svc.meta.origin_y == -5.0
