"""The keyframe rasterizer: ground percentile, obstacle band, extent, budgets."""

from __future__ import annotations

import numpy as np
import pytest

from swarmdeck_server.mapsvc import keyframe_raster as kr


def cell_of(raster: kr.KeyframeRaster, x: float, y: float) -> int:
    meta = raster.meta
    col = int(np.floor((x - meta.origin_x) / meta.resolution))
    row = int(np.floor((y - meta.origin_y) / meta.resolution))
    return int(raster.cells[row, col])


def column(x: float, y: float, zs) -> np.ndarray:
    return np.array([[x, y, z] for z in zs], dtype=np.float64)


def test_ground_is_the_tenth_percentile_not_the_minimum():
    # Cell A: ten road returns and one at 0.3 m; ground 0, the 0.3 m point is
    # inside the band, occupied. Cell B: one stray low return and ten at 0.3 m;
    # the 10th percentile (linear, as numpy) lands on 0.3, so nothing rises
    # above ground + 0.15 and the cell is free. A minimum-based ground would
    # have called B occupied.
    a = column(0.1, 0.1, [0.0] * 10 + [0.3])
    b = column(1.1, 0.1, [0.0] + [0.3] * 10)
    assert np.percentile(b[:, 2], 10) == pytest.approx(0.3)
    raster = kr.rasterize_points(np.vstack([a, b]), margin_m=0.0)
    assert cell_of(raster, 0.1, 0.1) == 100
    assert cell_of(raster, 1.1, 0.1) == 0
    assert raster.known_cells == 2
    assert raster.occupied_cells == 1


def test_ground_percentile_interpolates_like_numpy():
    # Six road returns with a little texture; the seventh point is the
    # candidate and is the cell's highest point either way, so it leaves the
    # 10th percentile (position 0.6, between 0.0 and 0.02) where numpy puts it.
    zs = [0.0, 0.02, 0.04, 0.06, 0.08, 0.1]
    ground = np.percentile(zs + [1.0], 10)
    assert ground == pytest.approx(0.012)
    # A point 0.149 m above the interpolated ground is a kerb; 0.151 m is a wall.
    kerb = column(0.1, 0.1, zs + [ground + 0.149])
    wall = column(1.1, 0.1, zs + [ground + 0.151])
    raster = kr.rasterize_points(np.vstack([kerb, wall]), margin_m=0.0)
    assert cell_of(raster, 0.1, 0.1) == 0
    assert cell_of(raster, 1.1, 0.1) == 100


@pytest.mark.parametrize(
    "height,expected",
    [
        (0.10, 0),  # kerb top: below the step band
        (0.15, 100),  # band is inclusive at the step
        (1.0, 100),  # a wall
        (2.0, 100),  # band is inclusive at the ceiling
        (2.1, 0),  # canopy above the robots
    ],
)
def test_obstacle_band_is_ground_plus_015_to_ground_plus_2(height, expected):
    ground = 3.0  # an elevated site, so the band is relative, not absolute
    points = column(0.1, 0.1, [ground] * 9 + [ground + height])
    raster = kr.rasterize_points(points, margin_m=0.0)
    assert cell_of(raster, 0.1, 0.1) == expected


def test_cells_without_points_are_unknown_and_the_margin_is_unknown():
    points = np.array([[0.1, 0.1, 0.0], [0.9, 0.9, 0.0]])
    raster = kr.rasterize_points(points, cell_m=0.2, margin_m=1.0)
    meta = raster.meta
    assert meta.resolution == 0.2
    assert (meta.origin_x, meta.origin_y) == pytest.approx((-1.0, -1.0), abs=1e-9)
    # Extent (-0.9 .. 1.9) at 0.2 m: ceil(2.9 / 0.2) + 1 = 16 cells each way.
    assert (meta.width, meta.height) == (16, 16)
    known = raster.cells != -1
    assert known.sum() == 2
    assert cell_of(raster, 0.1, 0.1) == 0
    assert cell_of(raster, 0.9, 0.9) == 0
    assert raster.cells[0, 0] == -1
    assert raster.cells[-1, -1] == -1


def test_grid_is_bottom_up_row_major_on_a_world_lattice():
    # A point north of another must land in a higher row (ROS order), and the
    # origin snaps to the cell lattice so a growing map does not jitter.
    south = column(0.35, 0.05, [0.0])
    north = column(0.35, 1.05, [0.0, 1.0])
    raster = kr.rasterize_points(np.vstack([south, north]), cell_m=0.5, margin_m=0.0)
    meta = raster.meta
    assert (meta.origin_x, meta.origin_y) == pytest.approx((0.0, 0.0))
    rows_cols = np.argwhere(raster.cells != -1)
    assert rows_cols.tolist() == [[0, 0], [2, 0]]
    assert raster.cells[0, 0] == 0
    assert raster.cells[2, 0] == 100
    # The same cloud shifted by a fraction of a cell keeps the lattice origin.
    shifted = kr.rasterize_points(
        np.vstack([south, north]) + [0.1, 0.1, 0.0], cell_m=0.5, margin_m=0.0
    )
    assert (shifted.meta.origin_x, shifted.meta.origin_y) == pytest.approx((0.0, 0.0))


def test_cell_budget_coarsens_then_refuses():
    points = np.array([[0.0, 0.0, 0.0], [10.0, 10.0, 0.0]])
    fine = kr.rasterize_points(points, cell_m=0.2, margin_m=0.0)
    assert fine.coarsened == 0
    assert fine.meta.width * fine.meta.height > 400
    coarse = kr.rasterize_points(points, cell_m=0.2, margin_m=0.0, max_cells=400)
    # 0.2 -> 0.4 (26x26 = 676) -> 0.8 (14x14 = 196 <= 400)
    assert coarse.coarsened == 2
    assert coarse.meta.resolution == pytest.approx(0.8)
    assert coarse.meta.width * coarse.meta.height <= 400
    assert coarse.known_cells == 2
    with pytest.raises(ValueError, match="exceeds"):
        kr.rasterize_points(points, cell_m=0.2, margin_m=0.0, max_cells=4)


def test_non_finite_points_are_ignored_and_an_empty_cloud_is_refused():
    points = np.array([[0.1, 0.1, 0.0], [np.nan, 0.0, 0.0], [0.0, np.inf, 0.0]])
    raster = kr.rasterize_points(points, margin_m=0.0)
    assert raster.points == 1
    assert raster.known_cells == 1
    with pytest.raises(ValueError, match="no finite points"):
        kr.rasterize_points(points[1:], margin_m=0.0)
    with pytest.raises(ValueError, match="Nx3"):
        kr.rasterize_points(np.zeros((3, 2)))
    with pytest.raises(ValueError):
        kr.rasterize_points(points, cell_m=0.0)
    with pytest.raises(ValueError):
        kr.rasterize_points(points, obstacle_min_m=2.0, obstacle_max_m=1.0)


def test_composite_points_are_placed_by_each_submap_transform():
    # Submap 1: rotated 90 degrees and translated; submap 2: identity, one
    # chunk unavailable. The declared total is checked before any read.
    rotation = [[0.0, -1.0, 0.0, 10.0], [1.0, 0.0, 0.0, 20.0], [0.0, 0.0, 1.0, 2.0]]
    view = {
        "selected": {
            "submaps": [
                {
                    "T_component_submap": rotation + [[0.0, 0.0, 0.0, 1.0]],
                    "chunks": [{"sha256": "a", "encoding": "xyz", "point_count": 1}],
                },
                {
                    "T_component_submap": np.eye(4).tolist(),
                    "chunks": [
                        {"sha256": "b", "encoding": "xyz", "point_count": 1},
                        {"sha256": "gone", "encoding": "xyz", "point_count": 1},
                    ],
                },
            ]
        }
    }
    chunks = {
        "a": np.array([[1.0, 0.0, 0.0]], dtype=np.float32),
        "b": np.array([[5.0, 6.0, 7.0]], dtype=np.float32),
    }
    reads = []

    def chunk_points(chunk):
        reads.append(chunk["sha256"])
        return chunks.get(chunk["sha256"])

    points, stats = kr.composite_world_points(view, chunk_points, max_points=3)
    assert points.tolist() == [[10.0, 21.0, 2.0], [5.0, 6.0, 7.0]]
    assert stats == {"chunks": 3, "missing_chunks": 1, "declared": 3}
    assert reads == ["a", "b", "gone"]

    reads.clear()
    with pytest.raises(OverflowError, match="budget"):
        kr.composite_world_points(view, chunk_points, max_points=2)
    assert reads == []
