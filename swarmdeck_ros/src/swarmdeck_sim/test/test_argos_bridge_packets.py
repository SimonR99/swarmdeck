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
        last_scan_raw=None,
        last_scan_products=None,
        last_camera_tick=-1,
        last_odom_tick=-1,
        last_odom_pose=None,
        odom_pose_history={},
        _warned_invalid=False,
        lidar_x=0.0,
        lidar_z=0.4,
        base_height=0.1,
        prox_min_height=0.10 + bridge.PROX_HEIGHT_EPSILON,
        prox_range_max=8.0,
    )
    for name in ("base", "odom", "lidar", "camera", "nav_scan", "nav_prox"):
        setattr(robot, "frame_" + name, name)
    for name in (
        "truth",
        "odom",
        "tf",
        "points",
        "capture",
        "scan",
        "nav_scan",
        "prox",
        "nav_prox",
        "image",
        "info",
        "depth",
    ):
        publisher = Mock()
        publisher.get_subscription_count.return_value = 1
        setattr(robot, "pub_" + name, publisher)
    node = bridge.ArgosBridge.__new__(bridge.ArgosBridge)
    node._robot = lambda _: robot
    node.get_logger = Mock()
    node.capture_producer_id = "a" * 32
    node.sensor_epoch = 3
    return node, robot


def packet(sensor_tick=90, valid=True, hits=0, max_range=30.0, odom_pose=None):
    pose = (1.0, 2.0, 0.0, 1.0, 0.0, 0.0, 0.0) + (0.0,) * 6
    odom_pose = pose if odom_pose is None else odom_pose
    points = np.zeros(hits, dtype=bridge.LIDAR_DTYPE)
    points["x"] = 2.0
    points["hit"] = 1
    return (
        b"\x01r"
        + struct.pack("<13d", *pose)
        + struct.pack(
            "<BB13dI", 1, valid, *(odom_pose if valid else (0.0,) * 13), sensor_tick
        )
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
    odom_msg = robot.pub_odom.publish.call_args.args[0]
    tf = robot.pub_tf.publish.call_args.args[0].transforms
    assert tf[0].header.stamp == odom_msg.header.stamp
    assert odom_msg.pose.pose.position.z == pytest.approx(robot.base_height)
    assert tf[1].child_frame_id == robot.frame_nav_scan
    assert tf[1].transform.translation.z == 0.0
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
    assert (
        robot.pub_points.publish.call_args.args[0].data
        == np.asarray([[2.0, 0.0, 0.0, 0.0]], dtype="<f4").tobytes()
    )
    assert robot.pub_scan.publish.call_args.args[0].ranges == [
        (
            2.0
            if i == int(np.floor(-bridge.SCAN_ANGLE_MIN * bridge.INV_ANGLE_INC))
            else float("inf")
        )
        for i in range(bridge.SCAN_BEAMS)
    ]
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


def test_unchanged_scan_and_pose_reuses_all_projections_across_odom_ticks(
    rig, monkeypatch
):
    node, robot = rig
    calls = {"cloud": 0, "slice": 0, "prox": 0, "nav": 0, "nav_prox": 0}
    for key, name in (
        ("slice", "project_laserscan_slice"),
        ("prox", "project_laserscan_proximity"),
        ("nav", "project_laserscan_navigation"),
        ("nav_prox", "project_laserscan_proximity_navigation"),
    ):
        original = getattr(bridge, name)

        def counted(*args, _key=key, _original=original, **kwargs):
            calls[_key] += 1
            return _original(*args, **kwargs)

        monkeypatch.setattr(bridge, name, counted)
    original_sha = bridge.hashlib.sha256

    def counted_sha(*args, **kwargs):
        calls["cloud"] += 1
        return original_sha(*args, **kwargs)

    monkeypatch.setattr(bridge.hashlib, "sha256", counted_sha)
    read(node, Socket(packet(sensor_tick=90, hits=1)))
    first = robot.pub_points.publish.call_args.args[0].data
    first_ranges = robot.pub_scan.publish.call_args.args[0].ranges
    nav_ranges = robot.pub_nav_scan.publish.call_args.args[0].ranges
    nav_prox_ranges = robot.pub_nav_prox.publish.call_args.args[0].ranges
    read(node, Socket(packet(sensor_tick=91, hits=1)))
    # A new velocity sample is not a new pose for the scan projection.
    moving_velocity = (1.0, 2.0, 0.0, 1.0, 0.0, 0.0, 0.0) + (0.5,) + (0.0,) * 5
    read(node, Socket(packet(sensor_tick=92, hits=1, odom_pose=moving_velocity)))
    assert robot.pub_points.publish.call_args.args[0].data == first
    assert robot.pub_scan.publish.call_args.args[0].ranges == first_ranges
    assert robot.pub_nav_scan.publish.call_args.args[0].ranges == nav_ranges
    assert robot.pub_nav_prox.publish.call_args.args[0].ranges == nav_prox_ranges
    assert calls == {"cloud": 1, "slice": 1, "prox": 1, "nav": 1, "nav_prox": 1}


def test_changed_pose_recomputes_nav_projection_with_unchanged_raw_scan(
    rig, monkeypatch
):
    node, robot = rig
    project = bridge.project_laserscan_navigation
    nav_calls = Mock(wraps=project)
    monkeypatch.setattr(bridge, "project_laserscan_navigation", nav_calls)
    read(node, Socket(packet(sensor_tick=90, hits=1)))
    first = robot.pub_nav_scan.publish.call_args.args[0].ranges
    pitch = np.deg2rad(16.0)
    changed = (1.0, 2.0, 0.0, np.cos(pitch / 2), 0.0, np.sin(pitch / 2), 0.0) + (
        0.0,
    ) * 6
    read(node, Socket(packet(sensor_tick=91, hits=1, odom_pose=changed)))
    second = robot.pub_nav_scan.publish.call_args.args[0].ranges
    assert nav_calls.call_count == 2
    assert second != first
    assert (
        second
        == project(*nav_calls.call_args.args, **nav_calls.call_args.kwargs).tolist()
    )


def test_unsubscribed_scan_products_are_not_built_and_late_subscriber_receives_next(
    rig, monkeypatch
):
    node, robot = rig
    for name in (
        "points",
        "capture",
        "scan",
        "nav_scan",
        "prox",
        "nav_prox",
        "image",
        "info",
        "depth",
    ):
        getattr(robot, "pub_" + name).get_subscription_count.return_value = 0
    project = Mock(side_effect=AssertionError("unsubscribed projection"))
    monkeypatch.setattr(bridge, "project_laserscan_slice", project)
    read(node, Socket(packet(sensor_tick=90, hits=1)))
    project.assert_not_called()
    robot.pub_scan.get_subscription_count.return_value = 1
    # Later readers see the next observation even when the raycast is unchanged.
    monkeypatch.setattr(
        bridge,
        "project_laserscan_slice",
        lambda *a, **kw: np.full(bridge.SCAN_BEAMS, np.inf),
    )
    read(node, Socket(packet(sensor_tick=91, hits=1)))
    robot.pub_scan.publish.assert_called_once()
    robot.pub_image.publish.assert_not_called()


def test_late_capture_subscriber_gets_digest_after_point_cloud_cache_warmed(rig):
    node, robot = rig
    robot.pub_capture.get_subscription_count.return_value = 0
    robot.pub_nav_scan.get_subscription_count.return_value = 0
    robot.pub_nav_prox.get_subscription_count.return_value = 0
    read(node, Socket(packet(sensor_tick=90, hits=1)))
    robot.pub_capture.publish.assert_not_called()
    robot.pub_capture.get_subscription_count.return_value = 1
    read(node, Socket(packet(sensor_tick=91, hits=1)))
    digest = json.loads(robot.pub_capture.publish.call_args.args[0].data)[
        "points_sha256"
    ]
    assert (
        digest
        == hashlib.sha256(
            np.asarray([[2.0, 0.0, 0.0]], dtype="<f4").tobytes()
        ).hexdigest()
    )


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


def full_packet(sensor_tick=90):
    """Every optional block present: encoders, IMU, a mixed scan and RGB-D."""
    half = np.sqrt(0.5)
    truth = (3.0, -1.0, 0.2, half, 0.0, 0.0, half, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
    odom = (1.0, 2.0, 0.3, half, 0.0, 0.0, half, 0.5, 0.0, 0.0, 0.0, 0.0, 0.25)
    points = np.zeros(4, dtype=bridge.LIDAR_DTYPE)
    points["x"] = [2.0, 0.0, -3.0, 1.0]
    points["y"] = [0.0, 2.5, 0.0, 1.0]
    points["z"] = [0.0, 0.01, -0.3, 0.0]
    points["range"] = np.hypot(points["x"], points["y"])
    points["ring"] = [1, 2, 3, 4]
    points["hit"] = [1, 1, 1, 0]
    return (
        (
            b"\x01r"
            + struct.pack("<13d", *truth)
            + struct.pack("<BB13dI", 1, 1, *odom, sensor_tick)
            + struct.pack("<B4d", 1, 0.1, 0.2, 3.0, 4.0)
            + struct.pack("<B6d", 1, 0.01, 0.02, 0.03, 0.1, 0.2, 9.8)
            + struct.pack("<BIIIfI", 1, sensor_tick, 16, 360, 20.0, 4)
            + points.tobytes()
            + struct.pack("<BIIIf", 1, sensor_tick, 2, 1, 90.0)
            + bytes(range(6))
            + b"\x01"
            + struct.pack("<2f", 1.5, 2.5)
        ),
        points,
        odom,
    )


def published(publisher):
    (call,) = publisher.publish.call_args_list
    return call.args[0]


def test_full_observation_publishes_every_product(rig, monkeypatch):
    node, robot = rig
    monkeypatch.setattr(
        bridge,
        "Imu",
        lambda: message(orientation_covariance=[0.0] * 9),
        raising=False,
    )
    robot.pub_imu = Mock()
    robot.frame_imu = "imu"
    data, points, odom_origin = full_packet()
    sock = Socket(data)

    assert read(node, sock) == "r"
    assert sock.recv(1) == b"", "encoder and IMU blocks must be drained exactly"

    truth = published(robot.pub_truth)
    assert (truth.header.frame_id, truth.child_frame_id) == ("world", "base")
    assert vars(truth.twist.twist.angular) == {"x": 0.4, "y": 0.5, "z": 0.6}

    odo = bridge._base_pose_from_origin(odom_origin, robot.base_height)
    odom = published(robot.pub_odom)
    assert (odom.header.frame_id, odom.child_frame_id) == ("odom", "base")
    assert vars(odom.pose.pose.position) == {"x": odo[0], "y": odo[1], "z": odo[2]}
    assert vars(odom.twist.twist.linear) == {"x": odo[7], "y": odo[8], "z": odo[9]}
    tf = published(robot.pub_tf).transforms
    assert [t.child_frame_id for t in tf] == ["base", "nav_scan", "nav_prox"]
    assert vars(tf[1].transform.translation) == {"x": odo[0], "y": odo[1], "z": 0.0}
    assert tf[2].transform.rotation.w == pytest.approx(np.sqrt(0.5))
    assert tf[2].transform.rotation.z == pytest.approx(np.sqrt(0.5))

    imu = published(robot.pub_imu)
    assert imu.header.frame_id == "imu"
    assert vars(imu.angular_velocity) == {"x": 0.01, "y": 0.02, "z": 0.03}
    assert vars(imu.linear_acceleration) == {"x": 0.1, "y": 0.2, "z": 9.8}
    assert imu.orientation_covariance[0] == -1.0

    hits = points[points["hit"] != 0]
    cloud = published(robot.pub_points)
    assert (cloud.width, cloud.point_step, cloud.row_step) == (3, 16, 48)
    expected_cloud = np.stack(
        [hits["x"], hits["y"], hits["z"], hits["ring"].astype("<f4")], axis=1
    ).astype("<f4")
    assert cloud.data == expected_cloud.tobytes()
    capture = json.loads(published(robot.pub_capture).data)
    assert capture["point_count"] == 3
    assert (
        capture["points_sha256"]
        == hashlib.sha256(expected_cloud[:, :3].tobytes()).hexdigest()
    )

    nav_pose = robot.odom_pose_history[90]
    expected = {
        "scan": ("lidar", 20.0, bridge.project_laserscan_slice(hits, range_max=20.0)),
        "prox": (
            "base",
            robot.prox_range_max,
            bridge.project_laserscan_proximity(
                hits,
                robot.lidar_x,
                robot.lidar_z,
                robot.base_height,
                prox_min_height=robot.prox_min_height,
                prox_range_max=robot.prox_range_max,
            ),
        ),
        "nav_scan": (
            "nav_scan",
            20.0,
            bridge.project_laserscan_navigation(
                hits, robot.lidar_x, robot.lidar_z, nav_pose, range_max=20.0
            ),
        ),
        "nav_prox": (
            "nav_prox",
            robot.prox_range_max,
            bridge.project_laserscan_proximity_navigation(
                hits,
                robot.lidar_x,
                robot.lidar_z,
                robot.base_height,
                nav_pose,
                prox_min_height=robot.prox_min_height,
                prox_range_max=robot.prox_range_max,
            ),
        ),
    }
    for name, (frame, range_max, ranges) in expected.items():
        scan = published(getattr(robot, "pub_" + name))
        assert scan.header.frame_id == frame
        assert (scan.header.stamp.sec, scan.header.stamp.nanosec) == (0, 900_000_000)
        assert (scan.angle_min, scan.angle_max, scan.angle_increment) == (
            float(bridge.SCAN_ANGLE_MIN),
            float(bridge.SCAN_ANGLE_MAX),
            float(bridge.SCAN_ANGLE_INC),
        )
        assert (scan.time_increment, scan.scan_time) == (0.0, bridge.SCAN_TIME)
        assert (scan.range_min, scan.range_max) == (bridge.SCAN_RANGE_MIN, range_max)
        assert scan.ranges == ranges.tolist()

    image = published(robot.pub_image)
    assert (image.height, image.width, image.step) == (1, 2, 6)
    assert (image.encoding, image.data) == ("rgb8", bytes(range(6)))
    info = published(robot.pub_info)
    focal = 1 / (2.0 * np.tan(np.radians(90.0) / 2.0))
    assert info.k == pytest.approx([focal, 0.0, 1.0, 0.0, focal, 0.5, 0.0, 0.0, 1.0])
    assert info.p[:7] == pytest.approx([focal, 0.0, 1.0, 0.0, 0.0, focal, 0.5])
    assert (info.distortion_model, info.d) == ("plumb_bob", [0.0] * 5)
    depth = published(robot.pub_depth)
    assert (depth.encoding, depth.step) == ("32FC1", 8)
    assert depth.data == struct.pack("<2f", 1.5, 2.5)
