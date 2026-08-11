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


def depth_image(values: np.ndarray):
    height, width = values.shape
    return type(
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


def camera_info(width: int, height: int):
    centre = (width - 1) / 2.0
    return type(
        "Info",
        (),
        {"K": [4.0, 0.0, centre, 0.0, 4.0, (height - 1) / 2.0, 0.0, 0.0, 1.0]},
    )()


def test_aligned_depth_image_deprojects_foreground_with_camera_intrinsics():
    width = height = 8
    values = np.full((height, width), 4000, dtype="<u2")
    values[2:6, 2:6] = 2000
    values[3, 3] = 100  # invalid under min_range, not a false foreground
    image = depth_image(values)

    point = point_for_depth_image(
        image, camera_info(width, height), (0.20, 0.20, 0.60, 0.60), inset=0.0
    )

    assert point is not None
    assert point.tolist() == pytest.approx([0.0, 0.0, 2.0], abs=0.26)


def test_outline_reads_the_object_where_the_box_reads_the_wall_behind_it():
    """The reason detections carry a segmentation mask at all.

    A diagonal object -- the pool noodle is the real case -- occupies a
    minority of its own bounding box.  Here the object is a 2 m diagonal band
    across a 6 m wall, so a box-shaped reading is dominated by the wall and
    lands metres past the thing it is supposed to have found.
    """
    width = height = 16
    values = np.full((height, width), 6000, dtype="<u2")
    diagonal = [(index, index) for index in range(2, 14)]
    for y, x in diagonal:
        values[y, x - 1 : x + 2] = 2000
    image = depth_image(values)
    info = camera_info(width, height)
    bbox = (1 / 16, 1 / 16, 14 / 16, 14 / 16)
    # A band three pixels wide down the diagonal of its own box.
    polygon = ((1 / 16, 3 / 16), (3 / 16, 1 / 16), (15 / 16, 13 / 16), (13 / 16, 15 / 16))

    with_outline = point_for_depth_image(image, info, bbox, polygon=polygon, inset=0.0)
    box_only = point_for_depth_image(image, info, bbox, inset=0.0)

    assert with_outline is not None and box_only is not None
    assert with_outline[2] == pytest.approx(2.0, abs=0.05)
    assert box_only[2] == pytest.approx(6.0, abs=0.05)


def test_an_unusable_outline_falls_back_to_the_inset_box():
    """Fail soft: a bad polygon costs precision, never the whole detection."""
    width = height = 8
    values = np.full((height, width), 4000, dtype="<u2")
    values[2:6, 2:6] = 2000
    image = depth_image(values)
    info = camera_info(width, height)
    bbox = (0.20, 0.20, 0.60, 0.60)
    baseline = point_for_depth_image(image, info, bbox, inset=0.0)

    for unusable in (
        (),                                        # no outline at all
        ((0.1, 0.1), (0.2, 0.2)),                  # too few points to be an area
        ((0.4, 0.4), (0.41, 0.4), (0.41, 0.41)),   # a sliver covering no pixels
        "not a polygon",
    ):
        assert point_for_depth_image(
            image, info, bbox, polygon=unusable, inset=0.0
        ).tolist() == pytest.approx(baseline.tolist())


def test_point_cloud_path_honours_the_same_outline():
    background = (0.0, 0.0, 5.0)
    rows = [[background for _ in range(8)] for _ in range(8)]
    for index in range(1, 7):
        rows[index][index] = (0.1 * index, 0.1 * index, 2.0)
    cloud = organised_cloud(rows)
    bbox = (0.0, 0.0, 1.0, 1.0)
    polygon = ((0.0, 0.125), (0.125, 0.0), (1.0, 0.875), (0.875, 1.0))

    with_outline = point_for_bbox(cloud, bbox, polygon=polygon, inset=0.0)
    box_only = point_for_bbox(cloud, bbox, inset=0.0)

    assert with_outline is not None and box_only is not None
    assert with_outline[2] == pytest.approx(2.0, abs=0.05)
    assert box_only[2] == pytest.approx(5.0, abs=0.05)
