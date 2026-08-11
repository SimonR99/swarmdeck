"""Bridge wiring in session.launch.py.

Needs the `launch` packages, which the ROS-free server venv does not have, so these
skip locally and run inside the Gazebo image.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

pytest.importorskip("launch_ros", reason="ROS 2 launch not on the path")

LAUNCH_FILE = Path(__file__).resolve().parents[1] / "launch" / "session.launch.py"


def _load():
    spec = importlib.util.spec_from_file_location("session_launch", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


bridge_args = _load().bridge_args

TF_BRIDGE = "/robot_0/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
SCAN_BRIDGE = "/robot_0/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"


def test_drive_plugin_tf_is_dropped_when_the_ekf_owns_it():
    """Both publish odom -> base_link. Two publishers make the TF tree flicker
    between a slip-corrupted estimate and a corrected one."""
    assert TF_BRIDGE not in bridge_args("robot_0", lidar_rings=1, fuse_imu=True)


def test_drive_plugin_tf_is_bridged_when_not_fusing():
    """Without the EKF nothing else publishes odom -> base_link, and SLAM cannot
    transform a scan without it."""
    assert TF_BRIDGE in bridge_args("robot_0", lidar_rings=1, fuse_imu=False)


def test_single_ring_bridges_gazebos_own_laserscan():
    assert SCAN_BRIDGE in bridge_args("robot_0", lidar_rings=1)


def test_multi_ring_leaves_scan_to_the_cloud_slicer():
    """pointcloud_to_laserscan publishes <ns>/scan itself; bridging it too would
    put two publishers on one topic and let SLAM alternate between them."""
    assert SCAN_BRIDGE not in bridge_args("robot_0", lidar_rings=9)


@pytest.mark.parametrize("rings", [1, 9])
@pytest.mark.parametrize("fuse", [True, False])
def test_imu_and_odom_are_always_bridged(rings, fuse):
    """The EKF needs both, whichever lidar and fusion mode is selected."""
    args = bridge_args("robot_0", lidar_rings=rings, fuse_imu=fuse)
    joined = "\n".join(args)
    assert "/robot_0/imu@sensor_msgs/msg/Imu" in joined
    assert "/robot_0/odom@nav_msgs/msg/Odometry" in joined


@pytest.mark.parametrize("rings", [1, 9])
@pytest.mark.parametrize("fuse", [True, False])
def test_the_whole_rgbd_camera_reaches_ros(rings, fuse):
    """Colour alone makes a duck a box on a video frame and nothing on the map.

    The names are Gazebo's: an `rgbd_camera` on base topic `<ns>/camera`
    publishes `image`, `depth_image`, `camera_info` and `points`. They must
    match robot.sdf.jinja exactly — a bridge for a topic Gazebo does not
    publish is silent, and presents as a camera that never appears.
    """
    joined = "\n".join(bridge_args("robot_0", lidar_rings=rings, fuse_imu=fuse))
    assert "/robot_0/camera/image@sensor_msgs/msg/Image" in joined
    assert "/robot_0/camera/depth_image@sensor_msgs/msg/Image" in joined
    assert "/robot_0/camera/camera_info@sensor_msgs/msg/CameraInfo" in joined


def test_the_organised_cloud_stays_inside_gazebo():
    """`<ns>/camera/points` is the depth image again at about 70 MB/s per robot,
    and nothing downstream reads it. See adapter_sim._depth_map_position."""
    assert "/robot_0/camera/points" not in "\n".join(bridge_args("robot_0"))
