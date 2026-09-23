"""One latched writer replaces all simulated static-mount processes."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import time

import pytest

pytest.importorskip("rclpy", reason="ROS 2 is only available in the launch-test image")
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage

LAUNCH = Path(__file__).resolve().parents[1] / "launch"


def load(name):
    spec = importlib.util.spec_from_file_location(name, LAUNCH / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_session_sensor_mount_transforms_preserve_frames_and_geometry():
    session = load("session.launch")
    robot = SimpleNamespace(lidar_x=0.123456, lidar_z=0.7, camera_x=0.2, camera_z=0.8)
    transforms = session.sensor_mount_transforms("scout_2", robot)
    assert transforms == [
        {
            "frame_id": "scout_2/base_link",
            "child_frame_id": f"scout_2/base_link/{sensor}",
            "x": x,
            "z": z,
        }
        for sensor, x, z in (
            ("lidar", 0.1235, 0.7),
            ("imu", 0.0, 0.0),
            ("proximity_lidar", 0.24, 0.05),
            ("camera", 0.2, 0.8),
        )
    ]


def test_static_mounts_replay_all_edges_to_late_namespaced_subscribers():
    session = load("session.launch")
    mounts = load("static_mounts")
    robot = SimpleNamespace(lidar_x=0.1, lidar_z=0.7, camera_x=0.2, camera_z=0.8)
    fleet = {
        ns: session.sensor_mount_transforms(ns, robot) for ns in ("robot_0", "robot_1")
    }
    rclpy.init()
    writer = rclpy.create_node("sensor_mounts")
    reader = rclpy.create_node("late_tf_reader")
    try:
        # Publish before any subscribers exist: late-join replay is the contract.
        publishers = mounts.publish_mounts(writer, fleet)
        assert len(publishers) == 2
        received = {}
        subscriptions = [
            reader.create_subscription(
                TFMessage,
                f"/{ns}/tf_static",
                lambda msg, ns=ns: received.__setitem__(ns, msg),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
            for ns in fleet
        ]
        deadline = time.monotonic() + 10
        while len(received) != 2 and time.monotonic() < deadline:
            rclpy.spin_once(reader, timeout_sec=0.1)
        assert set(received) == set(fleet)
        for ns, message in received.items():
            assert len(message.transforms) == 4
            for transform, expected in zip(message.transforms, fleet[ns]):
                assert transform.header.frame_id == expected["frame_id"]
                assert transform.child_frame_id == expected["child_frame_id"]
                assert transform.transform.translation.x == expected["x"]
                assert transform.transform.translation.z == expected["z"]
                assert transform.transform.translation.y == 0
                rotation = transform.transform.rotation
                assert (rotation.x, rotation.y, rotation.z, rotation.w) == (0, 0, 0, 1)
            assert reader.count_publishers(f"/{ns}/tf_static") == 1
        assert subscriptions
    finally:
        reader.destroy_node()
        writer.destroy_node()
        rclpy.shutdown()
