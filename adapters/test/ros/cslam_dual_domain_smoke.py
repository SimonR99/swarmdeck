#!/usr/bin/env python3
"""Prove sensor isolation and bounded coordination relays across two DDS domains.

This starts only SwarmDeck's bridge, not the native Swarm-SLAM frontends. A
fake keyframe pair on the peer domain drives the real persistence/publication
path after capture-time TF and a raw cloud arrive exclusively on the robot's
sensor domain.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid

import numpy as np
import rclpy
from cslam_common_interfaces.msg import KeyframeOdom, KeyframePointCloud
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32, read_points_numpy
from std_msgs.msg import Header, String
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


def require(predicate: bool, detail: str) -> None:
    if not predicate:
        raise RuntimeError(detail)


def make_node(
    name: str, domain_id: int
) -> tuple[Context, Node, SingleThreadedExecutor]:
    context = Context()
    context.init(args=[], initialize_logging=False, domain_id=domain_id)
    node = Node(name, context=context, enable_rosout=False)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    return context, node, executor


def transform(stamp, parent: str, child: str, *, x=0.0, z=0.0):
    value = TransformStamped()
    value.header = Header(stamp=stamp, frame_id=parent)
    value.child_frame_id = child
    value.transform.translation.x = x
    value.transform.translation.z = z
    value.transform.rotation.w = 1.0
    return value


def spin_pair(sensor_executor, peer_executor, duration_s: float = 0.1) -> None:
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        sensor_executor.spin_once(timeout_sec=0.01)
        peer_executor.spin_once(timeout_sec=0.01)


def main() -> None:
    peer_domain = 224
    sensor_domain = 225
    robot = "dual_probe"
    mission = str(uuid.uuid4())
    with tempfile.TemporaryDirectory(prefix="cslam-dual-domain-") as directory:
        root = Path(directory)
        log_path = root / "bridge.log"
        environment = {
            **os.environ,
            "ROS_DOMAIN_ID": str(peer_domain),
            "PYTHONPATH": f"/app:{os.environ.get('PYTHONPATH', '')}",
        }
        command = [
            "/usr/bin/python3",
            "/app/deploy/autonomy/cslam_bridge.py",
            "--ros-args",
            "-r",
            "__ns:=/r0",
            "-p",
            f"robot_id:={robot}",
            "-p",
            "robot_index:=0",
            "-p",
            f"robot_names:='[\"{robot}\"]'",
            "-p",
            f"mission_id:={mission}",
            "-p",
            "sensor_namespace:=''",
            "-p",
            "base_frame:=base_link",
            "-p",
            "odom_frame:=odom",
            "-p",
            "navigation_frame:=map",
            "-p",
            "cloud_topic:=/raw_points",
            "-p",
            f"sensor_domain_id:={sensor_domain}",
            "-p",
            "tf_topic:=/tf",
            "-p",
            "tf_static_topic:=/tf_static",
            "-p",
            f"store_root:={root / 'maps'}",
        ]
        with log_path.open("w") as log:
            process = subprocess.Popen(
                command,
                env=environment,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )

        sensor_context, sensor_node, sensor_executor = make_node(
            "sensor_fixture", sensor_domain
        )
        peer_context, peer_node, peer_executor = make_node("peer_fixture", peer_domain)
        sensor_tf = sensor_node.create_publisher(TFMessage, "/tf", 20)
        peer_tf = peer_node.create_publisher(TFMessage, "/tf", 20)
        raw_cloud = sensor_node.create_publisher(PointCloud2, "/raw_points", 10)
        key_cloud = peer_node.create_publisher(
            KeyframePointCloud, "/r0/cslam/keyframe_data", 10
        )
        key_odom = peer_node.create_publisher(
            KeyframeOdom, "/r0/cslam/keyframe_odom", 10
        )
        local_intention = sensor_node.create_publisher(
            String, "/swarmdeck/intentions", 20
        )
        local_report = sensor_node.create_publisher(
            String, "/swarmdeck/exploration_reports", 20
        )
        peer_intention = peer_node.create_publisher(String, "/swarmdeck/intentions", 20)
        peer_report = peer_node.create_publisher(
            String, "/swarmdeck/exploration_reports", 20
        )

        normalized_clouds = []
        normalized_odometry = []
        sensor_domain_normalized = []
        local_authorities = []
        peer_authorities = []
        local_metadata = []
        local_intentions = []
        peer_intentions = []
        local_reports = []
        peer_reports = []
        peer_node.create_subscription(
            PointCloud2,
            "/r0/normalized_cloud",
            normalized_clouds.append,
            10,
        )
        peer_node.create_subscription(
            Odometry,
            "/r0/normalized_odom",
            normalized_odometry.append,
            10,
        )
        sensor_node.create_subscription(
            PointCloud2,
            "/r0/normalized_cloud",
            sensor_domain_normalized.append,
            10,
        )
        sensor_node.create_subscription(
            String, f"/{robot}/map_authority", local_authorities.append, 10
        )
        peer_node.create_subscription(
            String, f"/{robot}/map_authority", peer_authorities.append, 10
        )
        sensor_node.create_subscription(
            String, f"/{robot}/keyframes", local_metadata.append, 10
        )
        sensor_node.create_subscription(
            String, "/swarmdeck/intentions", local_intentions.append, 20
        )
        peer_node.create_subscription(
            String, "/swarmdeck/intentions", peer_intentions.append, 20
        )
        sensor_node.create_subscription(
            String, "/swarmdeck/exploration_reports", local_reports.append, 20
        )
        peer_node.create_subscription(
            String, "/swarmdeck/exploration_reports", peer_reports.append, 20
        )

        try:
            deadline = time.monotonic() + 20.0
            while time.monotonic() < deadline and (
                raw_cloud.get_subscription_count() < 1
                or key_cloud.get_subscription_count() < 1
                or local_intention.get_subscription_count() < 2
                or peer_intention.get_subscription_count() < 2
            ):
                require(process.poll() is None, "bridge exited during DDS discovery")
                spin_pair(sensor_executor, peer_executor)
            require(
                raw_cloud.get_subscription_count() >= 1, "raw sensor subscriber missing"
            )
            require(
                key_cloud.get_subscription_count() >= 1, "keyframe subscriber missing"
            )

            stamp = sensor_node.get_clock().now().to_msg()
            local_transforms = TFMessage(
                transforms=[
                    transform(stamp, "odom", "base_link", x=1.0),
                    transform(stamp, "base_link", "lidar", z=0.5),
                    transform(stamp, "odom", "map"),
                ]
            )
            conflicting_peer_tf = TFMessage(
                transforms=[transform(stamp, "odom", "base_link", x=99.0)]
            )
            cloud = create_cloud_xyz32(
                Header(stamp=stamp, frame_id="lidar"),
                np.asarray([[2.0, 0.0, 0.0]], dtype=np.float32),
            )
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline and not normalized_clouds:
                sensor_tf.publish(local_transforms)
                peer_tf.publish(conflicting_peer_tf)
                raw_cloud.publish(cloud)
                spin_pair(sensor_executor, peer_executor, 0.1)
            require(
                normalized_clouds and normalized_odometry,
                "normalization did not cross domains",
            )
            xyz = np.asarray(
                read_points_numpy(
                    normalized_clouds[-1], field_names=("x", "y", "z"), skip_nans=True
                )
            ).reshape(-1, 3)
            require(
                np.allclose(xyz, [[2.0, 0.0, 0.5]], atol=1e-6),
                f"sensor extrinsic was not applied: {xyz}",
            )
            require(
                abs(normalized_odometry[-1].pose.pose.position.x - 1.0) < 1e-9,
                "peer-domain x=99 TF contaminated sensor-domain odometry",
            )
            require(
                not sensor_domain_normalized,
                "normalized cloud was unexpectedly published in the sensor domain",
            )

            key_cloud.publish(
                KeyframePointCloud(id=0, pointcloud=normalized_clouds[-1])
            )
            key_odom.publish(KeyframeOdom(id=0, odom=normalized_odometry[-1]))

            own_intention = f'{{ "robot_id": "{robot}", "exact": "intention" }}'
            remote_intention = '{ "robot_id": "remote", "exact": "intention" }'
            own_report = f'{{ "robot_id": "{robot}", "exact": "report" }}'
            remote_report = '{ "robot_id": "remote", "exact": "report" }'
            local_intention.publish(String(data=own_intention))
            local_report.publish(String(data=own_report))
            peer_intention.publish(String(data=remote_intention))
            peer_report.publish(String(data=remote_report))

            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline and (
                not local_authorities
                or not local_metadata
                or own_intention not in [message.data for message in peer_intentions]
                or remote_intention
                not in [message.data for message in local_intentions]
                or own_report not in [message.data for message in peer_reports]
                or remote_report not in [message.data for message in local_reports]
            ):
                sensor_tf.publish(local_transforms)
                raw_cloud.publish(cloud)
                spin_pair(sensor_executor, peer_executor, 0.1)
            require(local_authorities, "map authority did not return to sensor domain")
            require(local_metadata, "keyframe metadata did not return to sensor domain")
            require(not peer_authorities, "map authority leaked into the peer domain")
            require(
                json.loads(local_authorities[-1].data)["robot_id"] == robot,
                "local authority payload is invalid",
            )
            for payload, received, label in (
                (own_intention, peer_intentions, "outbound intention"),
                (remote_intention, local_intentions, "inbound intention"),
                (own_report, peer_reports, "outbound report"),
                (remote_report, local_reports, "inbound report"),
            ):
                require(
                    [message.data for message in received].count(payload) == 1,
                    f"{label} was altered, lost, or looped",
                )

            oversized = json.dumps({"robot_id": robot, "padding": "x" * 32_768})
            local_intention.publish(String(data=oversized))
            spin_pair(sensor_executor, peer_executor, 0.5)
            require(
                oversized not in [message.data for message in peer_intentions],
                "oversized coordination payload crossed the bridge",
            )
            print(
                "PASS: isolated sensor TF/cloud -> peer normalized capture; "
                "local authority/metadata and bounded bidirectional coordination relays",
                flush=True,
            )
        except Exception:
            if log_path.exists():
                print(log_path.read_text(), flush=True)
            raise
        finally:
            for executor, node, context in (
                (sensor_executor, sensor_node, sensor_context),
                (peer_executor, peer_node, peer_context),
            ):
                executor.shutdown(timeout_sec=1.0)
                node.destroy_node()
                rclpy.try_shutdown(context=context)
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
            if process.returncode not in (0, -signal.SIGTERM):
                if log_path.exists():
                    print(log_path.read_text(), flush=True)
                raise RuntimeError(
                    f"bridge did not stop cleanly: return code {process.returncode}"
                )


if __name__ == "__main__":
    main()
