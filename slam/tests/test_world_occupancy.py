"""Ground-truth occupancy of the indoor world, and scoring against it."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from swarmdeck_slam.world_occupancy import (
    Occupancy,
    boxes_from_sdf,
    rasterize,
    score_occupancy,
)


SDF = (
    Path(__file__).resolve().parents[2]
    / "swarmdeck_ros"
    / "src"
    / "swarmdeck_sim"
    / "worlds"
    / "indoor.sdf"
)


def test_indoor_sdf_raster_covers_the_building_shell() -> None:
    occupancy = rasterize(boxes_from_sdf(SDF), resolution=0.10, pad_m=0.5)
    xs = occupancy.origin_x + (np.arange(occupancy.width) + 0.5) * occupancy.resolution
    ys = occupancy.origin_y + (np.arange(occupancy.height) + 0.5) * occupancy.resolution
    xx, yy = np.meshgrid(xs, ys)
    south = occupancy.cells & (np.abs(yy + 11.925) < 0.20) & (np.abs(xx) < 10.0)
    north = occupancy.cells & (np.abs(yy - 11.925) < 0.20) & (np.abs(xx) < 10.0)
    assert south.sum() > 50
    assert north.sum() > 50
    assert occupancy.cells.sum() > 500


def test_identical_occupancy_scores_perfect_iou() -> None:
    occupancy = rasterize(boxes_from_sdf(SDF), resolution=0.20, pad_m=0.4)
    score = score_occupancy(occupancy, occupancy, yaw_step_deg=90.0)
    assert score.iou == 1.0
    assert score.precision == 1.0
    assert score.recall == 1.0


def test_translated_occupancy_recovers_after_alignment() -> None:
    truth = rasterize(boxes_from_sdf(SDF), resolution=0.20, pad_m=0.4)
    shifted = Occupancy(
        truth.cells,
        truth.origin_x + 0.80,
        truth.origin_y + 0.60,
        truth.resolution,
    )
    score = score_occupancy(shifted, truth, yaw_step_deg=90.0)
    assert score.iou > 0.95
