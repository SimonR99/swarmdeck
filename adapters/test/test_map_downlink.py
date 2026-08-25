"""Nav2 must not start on a lethal cell of the collaborative grid."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from adapters.map_downlink import (
    DownloadedMap,
    apply_to_occupancy_grid,
    clear_robot_disc,
)


def test_clear_robot_disc_frees_occupied_cells_under_the_chassis():
    cells = np.full((20, 20), np.int8(100))
    clear_robot_disc(
        cells,
        origin_x=0.0,
        origin_y=0.0,
        resolution=0.1,
        x=1.0,
        y=1.0,
        radius_m=0.35,
    )
    # Cell centre of (10, 10) is (1.05, 1.05) — inside the disc.
    assert int(cells[10, 10]) == 0
    # A far corner stays occupied.
    assert int(cells[0, 0]) == 100


def test_occupancy_grid_carve_does_not_mutate_the_cached_download():
    cells = np.full((8, 8), np.int8(100))
    downloaded = DownloadedMap(
        seq=1,
        resolution=0.5,
        width=8,
        height=8,
        origin_x=0.0,
        origin_y=0.0,
        cells=cells,
    )
    grid = SimpleNamespace(
        header=SimpleNamespace(frame_id=""),
        info=SimpleNamespace(
            resolution=0.0,
            width=0,
            height=0,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
            ),
        ),
        data=[],
    )
    apply_to_occupancy_grid(
        grid,
        downloaded,
        "map_frame",
        pose_xy=(2.0, 2.0),
        clear_radius_m=0.8,
    )
    assert 0 in grid.data
    assert int(downloaded.cells[4, 4]) == 100
    assert grid.header.frame_id == "map_frame"
