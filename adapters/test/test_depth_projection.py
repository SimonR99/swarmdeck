from __future__ import annotations

import math
import struct
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from adapters.perception.depth_projection import (
    point_for_bbox,
    point_for_depth_image,
    transform_point,
)


class Field:
    def __init__(self, name: str, offset: int) -> None:
        self.name = name
        self.offset = offset


def organised_cloud(rows: list[list[tuple[float, float, float]]], *, padding: int = 0):
    height = len(rows)
    width = len(rows[0])
    point_step = 16  # intensity before XYZ proves offsets are honoured
    row_step = width * point_step + padding
    data = bytearray(height * row_step)
    for y, row in enumerate(rows):
        for x, point in enumerate(row):
            struct.pack_into("<ffff", data, y * row_step + x * point_step, 99.0, *point)
    return type(
        "Cloud",
        (),
        {
            "width": width,
            "height": height,
            "point_step": point_step,
            "row_step": row_step,
            "is_bigendian": False,
            "fields": [Field("intensity", 0), Field("x", 4), Field("y", 8), Field("z", 12)],
            "data": bytes(data),
        },
    )()


def test_bbox_depth_prefers_coherent_foreground_and_honours_row_padding():
    background = (0.0, 0.0, 4.0)
    rows = [[background for _ in range(8)] for _ in range(8)]
    for y in range(2, 6):
        for x in range(2, 6):
            rows[y][x] = (1.0 + 0.01 * x, 2.0 + 0.01 * y, 2.0)
    rows[3][3] = (0.01, 0.01, 0.2)  # isolated flying pixel
    cloud = organised_cloud(rows, padding=12)

    point = point_for_bbox(cloud, (0.20, 0.20, 0.60, 0.60), inset=0.0)

    assert point is not None
    assert point.tolist() == pytest.approx([1.04, 2.04, 2.0], abs=0.04)


def test_unorganised_or_sparse_cloud_cannot_claim_a_position():
    cloud = organised_cloud([[(0.0, 0.0, 1.0)] * 8])
    assert point_for_bbox(cloud, (0.0, 0.0, 1.0, 1.0)) is None

    sparse_rows = [list(row) for row in [[(math.nan, math.nan, math.nan)] * 4 for _ in range(4)]]
    sparse_rows[1][1] = (0.0, 0.0, 1.0)
    sparse = organised_cloud(sparse_rows)
    assert point_for_bbox(sparse, (0.0, 0.0, 1.0, 1.0)) is None


def test_transform_point_applies_rotation_and_translation():
    half = math.sqrt(0.5)
    transform = type(
        "Transform",
        (),
        {
            "translation": type("V", (), {"x": 10.0, "y": -2.0, "z": 0.5})(),
            "rotation": type("Q", (), {"x": 0.0, "y": 0.0, "z": half, "w": half})(),
        },
    )()

    result = transform_point((1.0, 0.0, 0.0), transform)

    assert result is not None
    assert result.tolist() == pytest.approx([10.0, -1.0, 0.5])


def test_aligned_depth_image_deprojects_foreground_with_camera_intrinsics():
    width = height = 8
    values = np.full((height, width), 4000, dtype="<u2")
    values[2:6, 2:6] = 2000
    values[3, 3] = 100  # invalid under min_range, not a false foreground
    image = type(
        "Image",
        (),
        {
            "width": width,
            "height": height,
            "encoding": "16UC1",
            "is_bigendian": False,
            "step": width * 2,
            "data": values.tobytes(),
        },
    )()
    info = type(
        "Info",
        (),
        {"K": [4.0, 0.0, 3.5, 0.0, 4.0, 3.5, 0.0, 0.0, 1.0]},
    )()

    point = point_for_depth_image(image, info, (0.20, 0.20, 0.60, 0.60), inset=0.0)

    assert point is not None
    assert point.tolist() == pytest.approx([0.0, 0.0, 2.0], abs=0.26)
