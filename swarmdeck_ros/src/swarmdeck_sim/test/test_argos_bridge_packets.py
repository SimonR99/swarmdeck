"""Exercise actual binary packet decoding without a running ROS graph."""

import io
import hashlib
import json
from pathlib import Path
import struct
import sys
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "nodes"))
import swarmdeck_argos_bridge as bridge


class Socket:
    def __init__(self, data):
        self.data = io.BytesIO(data)

    def recv(self, count):
        return self.data.read(min(count, 17))  # Deliberately fragmented TCP reads.


def message(**kwargs):
    return NS(
        header=NS(stamp=NS(sec=0, nanosec=0)),
        pose=NS(pose=NS()),
        twist=NS(twist=NS()),
        transform=NS(),
        **kwargs,
    )


@pytest.fixture
def rig(monkeypatch):
    for name in (
        "Odometry",
        "TransformStamped",
        "LaserScan",
        "Image",
        "CameraInfo",
        "PointCloud2",
        "String",
    ):
        monkeypatch.setattr(bridge, name, message, raising=False)
    for name in ("Point", "Quaternion", "Vector3", "TFMessage"):
        monkeypatch.setattr(bridge, name, NS, raising=False)
    monkeypatch.setattr(
        bridge, "Clock", lambda: NS(clock=NS(sec=0, nanosec=0)), raising=False
    )
    robot = NS(
        last_scan_tick=-1,
        last_camera_tick=-1,
        last_odom_tick=-1,
        _warned_invalid=False,
        lidar_x=0.0,
        lidar_z=0.4,
        base_height=0.1,
        prox_min_height=0.10 + bridge.PROX_HEIGHT_EPSILON,
        prox_range_max=8.0,
    )
    for name in ("base", "odom", "lidar", "camera"):
        setattr(robot, "frame_" + name, name)
    for name in (
        "truth",
        "odom",
        "tf",
        "points",
        "capture",
        "scan",
        "prox",
        "image",
        "info",
        "depth",
    ):
        setattr(robot, "pub_" + name, Mock())
    node = bridge.ArgosBridge.__new__(bridge.ArgosBridge)
    node._robot = lambda _: robot
    node.get_logger = Mock()
    node.capture_producer_id = "a" * 32
    node.sensor_epoch = 3
    return node, robot


def packet(sensor_tick=90, valid=True, hits=0, max_range=30.0):
    pose = (1.0, 2.0, 0.0, 1.0, 0.0, 0.0, 0.0) + (0.0,) * 6
    points = np.zeros(hits, dtype=bridge.LIDAR_DTYPE)
    points["x"] = 2.0
    points["hit"] = 1
    return (
        b"\x01r"
        + struct.pack("<13d", *pose)
        + struct.pack("<BB13dI", 1, valid, *pose, sensor_tick)
        + b"\x00\x00"  # No encoders or IMU.
        + struct.pack("<BIIIfI", 1, sensor_tick, 1, 1, max_range, hits)
        + points.tobytes()
        + struct.pack("<BIIIf", 1, sensor_tick, 1, 1, 60.0)
        + b"\xff\x00\x00\x01"
        + struct.pack("<f", 2.0)
    )


def read(node, sock, tick=100):
    return node._read_robot(sock, bridge._stamp_of(tick, 100), 100, tick / 100, tick)


def test_capture_timestamps_and_empty_scan(rig):
    node, robot = rig
    assert read(node, Socket(packet())) == "r"
    for name in ("odom", "scan", "prox", "image", "info", "depth"):
        stamp = getattr(robot, "pub_" + name).publish.call_args.args[0].header.stamp
        assert (stamp.sec, stamp.nanosec) == (0, 900_000_000)
    tf = robot.pub_tf.publish.call_args.args[0].transforms[0]
    assert tf.header.stamp == robot.pub_odom.publish.call_args.args[0].header.stamp
    assert robot.pub_truth.publish.call_args.args[0].header.stamp.sec == 1
    assert all(
        x == float("inf") for x in robot.pub_prox.publish.call_args.args[0].ranges
    )
    robot.pub_points.publish.assert_not_called()


def test_duplicate_rgbd_is_drained_before_next_robot(rig):
    node, robot = rig
    sock = Socket(packet() + packet() + packet(95))
    for _ in range(3):
        read(node, sock)
    for name in ("odom", "tf", "scan", "prox", "image", "info", "depth"):
        assert getattr(robot, "pub_" + name).publish.call_count == 2
    assert sock.recv(1) == b""


def test_nonempty_cloud_and_scans_share_capture_time(rig):
    node, robot = rig
    read(node, Socket(packet(hits=1)))
    stamps = [
        getattr(robot, "pub_" + name).publish.call_args.args[0].header.stamp
        for name in ("points", "scan", "prox")
    ]
    assert all(stamp == stamps[0] for stamp in stamps)
    assert stamps[0].nanosec == 900_000_000
    metadata = json.loads(robot.pub_capture.publish.call_args.args[0].data)
    expected_points = np.asarray([[2.0, 0.0, 0.0]], dtype="<f4")
    assert metadata == {
        "clock": "ros_sim_time",
        "first_return": True,
        "frame_id": "lidar",
        "geometry": "raw_ray_capture",
        "instantaneous": True,
        "producer_id": "a" * 32,
        "point_count": 1,
        "points_sha256": hashlib.sha256(expected_points.tobytes()).hexdigest(),
        "provider": "simulation",
        "schema": "swarmdeck.raw-capture.v1",
        "sensor_epoch": 3,
        "single_sensor_origin": True,
        "source_contract": "argos.photorealistic_lidar.hit_endpoints.single_tick.v1",
        "stamp_ns": 900_000_000,
    }


def test_tick_zero_sensors_are_drained_but_withheld_until_capture_time_exists(rig):
    node, robot = rig
    sock = Socket(packet(sensor_tick=0, hits=1) + packet(sensor_tick=1, hits=1))

    # Tick-zero odometry and its TF retain their existing startup behaviour,
    # while sensor geometry is withheld because ROS TF interprets time zero as
    # "latest" rather than as this capture instant.
    assert read(node, sock, tick=0) == "r"
    robot.pub_odom.publish.assert_called_once()
    robot.pub_tf.publish.assert_called_once()
    for name in ("points", "capture", "scan", "prox", "image", "info", "depth"):
        getattr(robot, "pub_" + name).publish.assert_not_called()

    # Draining the rejected packet preserves framing, and the first positive
    # sensor tick is published normally with exact capture-time provenance.
    assert read(node, sock, tick=1) == "r"
    for name in ("points", "capture", "scan", "prox", "image", "info", "depth"):
        getattr(robot, "pub_" + name).publish.assert_called_once()
    assert sock.recv(1) == b""


def test_fallback_scan_timestamp_never_claims_raw_capture_provenance(rig):
    node, robot = rig
    read(node, Socket(packet(sensor_tick=500, hits=1)))
    robot.pub_points.publish.assert_called_once()
    robot.pub_capture.publish.assert_not_called()


def test_invalid_scan_tick_cannot_publish_zero_fallback_timestamp(rig):
    node, robot = rig
    sock = Socket(packet(sensor_tick=500, hits=1) + packet(sensor_tick=1, hits=1))
    read(node, sock, tick=0)
    for name in ("points", "capture", "scan", "prox"):
        getattr(robot, "pub_" + name).publish.assert_not_called()
    read(node, sock, tick=1)
    robot.pub_points.publish.assert_called_once()
    assert sock.recv(1) == b""


def test_future_camera_and_odometry_not_relabelled_as_current(rig):
    node, robot = rig
    read(node, Socket(packet(101)))
    for name in ("odom", "tf", "image", "info", "depth"):
        getattr(robot, "pub_" + name).publish.assert_not_called()


def test_unconverged_estimator_does_not_publish_placeholder(rig):
    node, robot = rig
    read(node, Socket(packet(valid=False)))
    robot.pub_odom.publish.assert_not_called()
    robot.pub_tf.publish.assert_not_called()


def test_zero_length_and_truncated_socket():
    assert bridge.recv_exact(Socket(b""), 0) == b""
    with pytest.raises(EOFError):
        bridge.recv_exact(Socket(b"ab"), 3)


def test_tick_stamp_remains_normalized_at_uint32_limit(rig):
    stamp = bridge._stamp_of(2**32 - 1, 100)
    assert (stamp.sec, stamp.nanosec) == (42949672, 950000000)


def test_reconnect_and_clock_rewind_clear_sensor_epoch(rig):
    node, robot = rig
    robot.last_scan_tick = robot.last_camera_tick = robot.last_odom_tick = 500
    node.robots = {"r": robot}
    node.running = True
    node.pub_clock = Mock()
    node._send_commands = Mock()
    sock = Socket(
        b"".join(
            struct.pack("<4sIII", bridge.OBSERVATION_MAGIC, tick, 100, 1) + packet(tick)
            for tick in (100, 10)
        )
    )
    with pytest.raises(EOFError):
        node._handle(sock)
    for name in ("odom", "image", "scan"):
        assert getattr(robot, "pub_" + name).publish.call_count == 2


@pytest.mark.parametrize("max_range", [0.0, float("nan"), float("inf")])
def test_unrendered_scan_does_not_poison_laser_calibration(rig, max_range):
    node, robot = rig
    sock = Socket(packet(max_range=max_range) + packet(hits=1))
    read(node, sock)
    robot.pub_scan.publish.assert_not_called()
    robot.pub_prox.publish.assert_not_called()
    robot.pub_image.publish.assert_called_once()  # The packet was fully drained.
    # The valid first render can share the placeholder's tick.
    read(node, sock)
    robot.pub_scan.publish.assert_called_once()
    robot.pub_points.publish.assert_called_once()
