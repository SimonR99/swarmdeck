from types import SimpleNamespace

import numpy as np
import pytest

from adapters.costmap import normalize_costmap


def _message(width=2, height=2, data=None, *, frame="odom", origin=(0.0, 0.0)):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame),
        info=SimpleNamespace(
            resolution=1.0,
            width=width,
            height=height,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=origin[0], y=origin[1]),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=data if data is not None else [0, 25, -1, 100],
    )


def test_normalize_costmap_keeps_ros_grid_order_when_already_in_target_frame():
    result = normalize_costmap(_message(), target_frame="map")

    assert result.frame_id == "map"
    assert result.cells.tolist() == [[0, 25], [-1, 100]]


def test_normalize_costmap_applies_planar_translation():
    result = normalize_costmap(
        _message(width=2, height=1, data=[100, 40]),
        target_frame="map",
        transform=(2.0, -3.0, 0.0),
    )

    assert (result.origin_x, result.origin_y) == pytest.approx((2.0, -3.0))
    assert result.cells.tolist() == [[100, 40]]


def test_normalize_costmap_rotates_and_keeps_the_most_restrictive_cost():
    result = normalize_costmap(
        _message(width=2, height=1, data=[100, 40]),
        target_frame="map",
        transform=(0.0, 0.0, np.pi / 2),
    )

    assert (result.width, result.height) == (1, 2)
    assert result.cells[:, 0].tolist() == [100, 40]


def test_normalize_costmap_rejects_malformed_data():
    with pytest.raises(ValueError, match="data size"):
        normalize_costmap(_message(data=[0]), target_frame="map")
