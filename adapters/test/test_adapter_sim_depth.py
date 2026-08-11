"""Turning a simulated duck detection into a point on the operator's map.

The detector is RGB-only, so a detection is a box on a video frame until depth
gives it a range and the robot's pose gives it a place. This covers the two ends
of that: the geometry (which is where a wrong sign or a confused frame
convention would put ducks in a mirrored world), and the refusals (which is what
stops a missing or frozen depth stream inventing a coordinate instead).

adapter_sim imports the whole ROS stack at module scope; conftest.py stubs it.
The arithmetic under test needs none of it.
"""

from __future__ import annotations

import math
import types
from unittest.mock import MagicMock

import numpy as np
import pytest


# The scout_mini mount, from spawn_fleet.ROBOT_PROFILES. Restated as a literal
# rather than imported, so that a change to the profile has to be acknowledged
# here instead of silently changing what these tests assert.
CAM_X = 0.322
CAM_Z = 0.090


@pytest.fixture
def bridge(sim_module):
    """A RobotBridge with no subscriptions, holding no camera data yet."""
    instance = sim_module.RobotBridge.__new__(sim_module.RobotBridge)
    instance.node = MagicMock()
    instance.id = "robot_0"
    instance.camera_x = CAM_X
    instance.camera_z = CAM_Z
    instance._camera_depth = None
    instance._camera_info = None
    instance._last_depth_warning_at = 0.0
    instance._map_to_odom = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    instance._odom_to_base = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    instance._odom_topic_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    instance._warned_no_tf_base = False
    return instance


def depth_image(metres: np.ndarray, *, sec: int = 10, nanosec: int = 0):
    """A 32FC1 depth image shaped like the one Gazebo's rgbd_camera bridges."""
    values = np.ascontiguousarray(metres, dtype="<f4")
    height, width = values.shape
    return types.SimpleNamespace(
        header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=sec, nanosec=nanosec),
            frame_id="robot_0/base_link/camera",
        ),
        width=width,
        height=height,
        encoding="32FC1",
        is_bigendian=False,
        step=width * 4,
        data=values.tobytes(),
    )


def camera_info(width: int = 64, height: int = 48, fov: float = 1.2):
    """Intrinsics for the simulated lens: 1.2 rad horizontal, centred.

    The matrix is `k`, lowercase, because that is what a ROS 2 CameraInfo
    carries — ROS 1's `K` was renamed in the IDL. depth_projection accepts
    either, and this is the one the simulated fleet actually delivers, so it is
    the one these tests exercise.
    """
    fx = (width / 2.0) / math.tan(fov / 2.0)
    return types.SimpleNamespace(
        k=[fx, 0.0, width / 2.0, 0.0, fx, height / 2.0, 0.0, 0.0, 1.0]
    )


def scene(width: int = 64, height: int = 48, *, background: float = 6.0):
    return np.full((height, width), background, dtype="<f4")


def header(sec: int = 10, nanosec: int = 0):
    return types.SimpleNamespace(stamp=types.SimpleNamespace(sec=sec, nanosec=nanosec))


# --------------------------------------------------------------- the geometry


def test_a_duck_dead_ahead_lands_in_front_of_the_robot(sim_module):
    """Optical z is the boresight, and the camera sits forward of base_link."""
    position = sim_module.camera_point_to_map(
        (0.0, 0.0, 2.0), {"x": 0.0, "y": 0.0, "yaw": 0.0}, CAM_X, CAM_Z
    )
    assert position == {"x": pytest.approx(CAM_X + 2.0), "y": pytest.approx(0.0)}


def test_a_duck_to_the_right_of_frame_lands_to_the_robots_right(sim_module):
    """Optical x is RIGHT and base_link y is LEFT: the sign must flip.

    Getting this backwards mirrors every marker about the robot's heading, which
    on a symmetric indoor map looks entirely plausible.
    """
    position = sim_module.camera_point_to_map(
        (0.5, 0.0, 2.0), {"x": 0.0, "y": 0.0, "yaw": 0.0}, CAM_X, CAM_Z
    )
    assert position == {"x": pytest.approx(CAM_X + 2.0), "y": pytest.approx(-0.5)}


def test_height_in_the_camera_frame_does_not_move_the_marker(sim_module):
    """`map_position` is a point on a 2D map; optical y (down) is dropped."""
    floor = sim_module.camera_point_to_map(
        (0.0, 0.4, 2.0), {"x": 0.0, "y": 0.0, "yaw": 0.0}, CAM_X, CAM_Z
    )
    shelf = sim_module.camera_point_to_map(
        (0.0, -0.4, 2.0), {"x": 0.0, "y": 0.0, "yaw": 0.0}, CAM_X, CAM_Z
    )
    assert floor == shelf


@pytest.mark.parametrize(
    "yaw, expected",
    [
        (math.pi / 2, (0.0, 2.322)),
        (math.pi, (-2.322, 0.0)),
        (-math.pi / 2, (0.0, -2.322)),
    ],
)
def test_the_marker_rotates_with_the_robot(sim_module, yaw, expected):
    position = sim_module.camera_point_to_map(
        (0.0, 0.0, 2.0), {"x": 0.0, "y": 0.0, "yaw": yaw}, CAM_X, CAM_Z
    )
    assert (position["x"], position["y"]) == pytest.approx(expected, abs=1e-3)


def test_the_marker_is_offset_by_where_the_robot_actually_is(sim_module):
    """The pose is the robot's SLAM pose, not the origin: both terms apply."""
    position = sim_module.camera_point_to_map(
        (0.0, 0.0, 2.0), {"x": 5.0, "y": -3.0, "yaw": math.pi / 2}, CAM_X, CAM_Z
    )
    assert (position["x"], position["y"]) == pytest.approx((5.0, -0.678), abs=1e-3)


def test_a_non_finite_depth_sample_is_refused_rather_than_placed(sim_module):
    """Gazebo returns +inf where a ray hits nothing, and inf is not a location."""
    pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    assert sim_module.camera_point_to_map((0.0, 0.0, math.inf), pose, CAM_X, CAM_Z) is None
    assert sim_module.camera_point_to_map((math.nan, 0.0, 2.0), pose, CAM_X, CAM_Z) is None


# ------------------------------------------------------- end to end, in metres


def test_a_box_in_the_depth_image_becomes_a_metric_map_position(sim_module, bridge):
    """The whole chain: depth pixels -> camera XYZ -> map, with real intrinsics.

    A duck-sized patch 2 m ahead, slightly right of centre, in front of a wall at
    6 m. The wall must not win: the box is mostly background, and averaging it
    in would drag the marker metres past the duck.
    """
    depth = scene()
    depth[20:28, 36:44] = 2.0  # the duck, right of the 32-pixel centre column
    bridge._camera_depth = depth_image(depth)
    bridge._camera_info = camera_info()

    # The same patch, as a normalised detection box.
    position = bridge._depth_map_position((36 / 64, 20 / 48, 8 / 64, 8 / 48), header())

    assert position is not None
    # The patch straddles the optical centre 7.5 px to its right; through a
    # 64 px / 1.2 rad lens that is 0.32 m of offset at 2 m of range.
    assert position["x"] == pytest.approx(CAM_X + 2.0, abs=0.02)
    assert position["y"] == pytest.approx(-0.32, abs=0.02)


def test_the_same_duck_seen_from_the_far_side_lands_in_the_same_place(bridge):
    """The marker must be anchored to the map, not carried around by the robot.

    The backend merges detections from the whole fleet onto one map, so a
    position that is only right when the robot happens to face north is worse
    than no position at all. The second robot here is the first one reflected
    through the duck and turned around: same range, mirrored bearing. If any
    sign in the optical -> base_link -> map chain is wrong, the two disagree.
    """
    depth = scene()
    depth[20:28, 28:36] = 3.0
    bridge._camera_depth = depth_image(depth)
    bridge._camera_info = camera_info()
    bbox = (28 / 64, 20 / 48, 8 / 64, 8 / 48)

    bridge._odom_to_base = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    from_south = bridge._depth_map_position(bbox, header())

    bridge._odom_to_base = {
        "x": 2 * from_south["x"],
        "y": 2 * from_south["y"],
        "yaw": math.pi,
    }
    from_north = bridge._depth_map_position(bbox, header())

    assert from_north["x"] == pytest.approx(from_south["x"], abs=1e-3)
    assert from_north["y"] == pytest.approx(from_south["y"], abs=1e-3)


# ------------------------------------------------------------- the refusals


def test_no_depth_yet_reports_no_position_rather_than_a_guess(bridge):
    bridge._camera_info = camera_info()
    assert bridge._depth_map_position((0.4, 0.4, 0.2, 0.2), header()) is None


def test_intrinsics_are_required_before_anything_is_deprojected(bridge):
    bridge._camera_depth = depth_image(scene())
    assert bridge._depth_map_position((0.4, 0.4, 0.2, 0.2), header()) is None


def test_a_frozen_depth_stream_stops_producing_positions(sim_module, bridge):
    """A stale depth frame is stale geometry: the robot has moved since.

    Both streams come off one sensor at one rate, so this cannot fire on
    ordinary jitter — only on a depth stream that has actually stopped.
    """
    depth = scene()
    depth[16:32, 24:40] = 2.0
    bridge._camera_info = camera_info()
    bbox = (24 / 64, 16 / 48, 16 / 64, 16 / 48)

    bridge._camera_depth = depth_image(depth, sec=10)
    assert bridge._depth_map_position(bbox, header(sec=10)) is not None

    # The colour frame moves on; the depth frame stays where it stopped.
    stale_by = sim_module.DEPTH_MAX_AGE_S + 0.2
    assert bridge._depth_map_position(
        bbox, header(sec=10, nanosec=int(stale_by * 1e9))
    ) is None


def test_a_duck_beyond_the_usable_depth_range_is_not_placed(bridge):
    """Past the far limit a few pixels of box cover metres of room."""
    depth = scene(background=30.0)
    depth[20:28, 28:36] = 24.0
    bridge._camera_depth = depth_image(depth)
    bridge._camera_info = camera_info()

    assert bridge._depth_map_position((28 / 64, 20 / 48, 8 / 64, 8 / 48), header()) is None


def test_open_sky_pixels_never_become_a_position(bridge):
    """Gazebo fills unhit pixels with +inf; every one of them must be dropped."""
    depth = scene(background=math.inf)
    bridge._camera_depth = depth_image(depth)
    bridge._camera_info = camera_info()

    assert bridge._depth_map_position((0.3, 0.3, 0.4, 0.4), header()) is None


def test_the_operator_is_told_why_markers_are_missing_but_not_flooded(bridge):
    """One warning per 10 s: this runs per detection, per frame, per robot."""
    for _ in range(20):
        bridge._depth_map_position((0.4, 0.4, 0.2, 0.2), header())
    assert bridge.node.get_logger().warn.call_count == 1
