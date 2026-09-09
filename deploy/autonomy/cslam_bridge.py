#!/usr/bin/env python3
"""Capture-time normalization and persistent onboard Swarm-SLAM mapping."""

import json
import os
import time
from pathlib import Path
from threading import Event, Thread

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformListener, TransformException
from cslam_common_interfaces.msg import (
    KeyframePointCloud,
    KeyframeOdom,
    OptimizationResult,
)

from autonomy.contracts import (
    ComponentRevision,
    GraphSolution,
    KeyframeId,
    component_id_for_anchor,
)
from autonomy.cslam import CslamMapper, pose_matrix
from autonomy.mapping import CorrectionAwareMapper, SubmapStore
from autonomy.replication import ReplicaClient


def transform_pose(transform):
    from geometry_msgs.msg import Pose

    result = Pose()
    result.position.x, result.position.y, result.position.z = (
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    )
    result.orientation = transform.rotation
    return result


class Bridge(Node):
    def __init__(self):
        super().__init__("onboard_mapper")
        params = {
            "robot_id": "robot_0",
            "robot_index": 0,
            "robot_names": '["robot_0"]',
            "mission_id": "",
            "sensor_namespace": "robot_0",
            "base_frame": "robot_0/base_link",
            "odom_frame": "robot_0/odom",
            "cloud_topic": "/robot_0/scan/points",
            "server_url": "",
            "store_root": "/maps",
            "max_range_m": 30.0,
            "navigation_frame": "robot_0/map_frame",
        }
        for key, value in params.items():
            self.declare_parameter(key, value)
        p = {key: self.get_parameter(key).value for key in params}
        self.robot = p["robot_id"]
        self.base, self.odom_frame = p["base_frame"], p["odom_frame"]
        self.max_range = p["max_range_m"]
        self.sensor_ns = p["sensor_namespace"].strip("/")
        self.navigation_frame = p["navigation_frame"]
        names = dict(enumerate(json.loads(p["robot_names"])))
        if names[p["robot_index"]] != self.robot:
            raise ValueError("robot_names and robot_index disagree")
        KeyframeId(self.robot, p["mission_id"], 0)
        root = Path(p["store_root"]) / p["mission_id"] / self.robot
        root.mkdir(parents=True, exist_ok=True)
        # A crashed frontend must not reuse seq=0 against a live peer graph.
        with (root / "frontend-lifetime").open("x") as stream:
            stream.write(
                "Start a fresh fleet mission and ROS domain after a frontend restart.\n"
            )
        self.core = CslamMapper(
            CorrectionAwareMapper(SubmapStore(root / "geometry")),
            self.robot,
            p["robot_index"],
            p["mission_id"],
            names,
        )
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.authority_pub = self.create_publisher(
            String, f"/{self.robot}/map_authority", 5
        )
        keyframe_qos = QoSProfile(depth=100)
        keyframe_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        keyframe_qos.reliability = ReliabilityPolicy.RELIABLE
        self.keyframe_metadata_pub = self.create_publisher(
            String, f"/{self.robot}/keyframes", keyframe_qos
        )
        self.cloud_pub = self.create_publisher(PointCloud2, "normalized_cloud", 5)
        self.odom_pub = self.create_publisher(Odometry, "normalized_odom", 5)
        self.pending_cloud = None
        self.last_sensor_at = 0.0
        self.clouds, self.odoms, self.capture_calibrations = {}, {}, {}
        self.normalized_count = self.capture_count = self.solution_count = (
            self.dropped
        ) = 0
        self.create_subscription(
            PointCloud2, p["cloud_topic"], self.raw_cloud, qos_profile_sensor_data
        )
        self.create_subscription(
            KeyframePointCloud, "cslam/keyframe_data", self.key_cloud, 100
        )
        self.create_subscription(
            KeyframeOdom, "cslam/keyframe_odom", self.key_odom, 100
        )
        self.create_subscription(
            OptimizationResult, "cslam/optimized_estimates", self.optimized, 100
        )
        self.create_timer(0.05, self.normalize)
        self.latest_envelope = None
        self.create_timer(1.0, self.snapshot)
        self.closed = Event()
        self.replica_error, self.acked_revision = "", -1
        self.worker = None
        if p["server_url"]:
            self.client = ReplicaClient(p["server_url"])
            self.worker = Thread(target=self.replicate, daemon=True)
            self.worker.start()
        self.status_file = root / "status.json"
        self.snapshot_file = root / "snapshot.json"
        self.graph_solution_file = root / "graph_solution.json"
        self.graph_solution_revision = -1

    def raw_cloud(self, cloud):
        self.pending_cloud = cloud  # latest-only under sensor overload

    def normalize(self):
        cloud = self.pending_cloud
        if cloud is None:
            return
        stamp = Time.from_msg(cloud.header.stamp)
        try:
            mount = self.tf.lookup_transform(self.base, cloud.header.frame_id, stamp)
            local = self.tf.lookup_transform(self.odom_frame, self.base, stamp)
        except TransformException:
            return  # never substitute a latest transform for capture-time TF
        self.pending_cloud = None
        xyz = point_cloud2.read_points_numpy(
            cloud, field_names=("x", "y", "z"), skip_nans=True
        )
        xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
        xyz = xyz[
            np.isfinite(xyz).all(axis=1)
            & (np.linalg.norm(xyz, axis=1) <= self.max_range)
        ]
        T = np.asarray(pose_matrix(transform_pose(mount.transform)))
        xyz = xyz @ T[:3, :3].T + T[:3, 3]
        header = Header(stamp=cloud.header.stamp, frame_id=self.base)
        out = point_cloud2.create_cloud_xyz32(header, xyz)
        odom = Odometry()
        odom.header = Header(stamp=cloud.header.stamp, frame_id=self.odom_frame)
        odom.child_frame_id = self.base
        odom.pose.pose = transform_pose(local.transform)
        # TF has no covariance. Zero is explicitly unknown at this boundary;
        # optimizer noise remains configured independently, never inferred here.
        stamp_ns = cloud.header.stamp.sec * 1_000_000_000 + cloud.header.stamp.nanosec
        self.capture_calibrations[stamp_ns] = (T, cloud.header.frame_id)
        while len(self.capture_calibrations) > 1024:
            self.capture_calibrations.pop(next(iter(self.capture_calibrations)))
        self.odom_pub.publish(odom)
        self.cloud_pub.publish(out)
        self.normalized_count += 1
        self.last_sensor_at = time.monotonic()

    def key_cloud(self, msg):
        self.clouds[msg.id] = msg.pointcloud
        self.consume(msg.id)

    def key_odom(self, msg):
        self.odoms[msg.id] = msg.odom
        self.consume(msg.id)

    def consume(self, seq):
        if seq in self.clouds and seq in self.odoms:
            cloud, odom = self.clouds.pop(seq), self.odoms.pop(seq)
            xyz = point_cloud2.read_points_numpy(
                cloud, field_names=("x", "y", "z"), skip_nans=True
            )
            stamp = odom.header.stamp.sec * 1_000_000_000 + odom.header.stamp.nanosec
            mount, sensor_frame = self.capture_calibrations[stamp]
            accepted = self.core.capture(
                seq,
                stamp,
                pose_matrix(odom.pose.pose),
                xyz,
                T_base_sensor=mount,
                sensor_frame=sensor_frame,
            )
            if accepted:
                keyframe = self.core.key(seq)
                self.keyframe_metadata_pub.publish(
                    String(
                        data=json.dumps(
                            {
                                "schema": "swarmdeck.keyframe-metadata.v1",
                                "keyframe_id": keyframe.stable_id,
                                "stamp_ns": stamp,
                                "odom_frame": self.odom_frame,
                                "T_odom_keyframe": self.core.local_poses[keyframe],
                                "component_id": component_id_for_anchor(
                                    self.core.anchor
                                ),
                            },
                            allow_nan=False,
                        )
                    )
                )
            self.capture_count += 1
        for cache in (self.clouds, self.odoms):
            while len(cache) > 100:
                cache.pop(min(cache))
                self.dropped += 1

    def optimized(self, msg):
        if self.core.solution(msg):
            self.solution_count += 1

    def snapshot(self):
        if self.core.revision and self.graph_solution_revision != self.core.revision:
            revision = ComponentRevision(
                component_id_for_anchor(self.core.anchor),
                self.core.epoch,
                self.core.revision,
            )
            solution = GraphSolution(
                revision,
                self.core.anchor,
                tuple(self.core.poses),
                self.core.poses,
            )
            temporary = self.graph_solution_file.with_suffix(".tmp")
            temporary.write_text(
                json.dumps(
                    {
                        "schema": "swarmdeck.pose-snapshot.v1",
                        "solution": solution.canonical_dict(),
                    },
                    allow_nan=False,
                )
            )
            os.replace(temporary, self.graph_solution_file)
            self.graph_solution_revision = self.core.revision
        if self.core.revision and (
            self.latest_envelope is None
            or self.latest_envelope["revision"] != self.core.revision
        ):
            self.latest_envelope = self.core.envelope()
            temporary = self.snapshot_file.with_suffix(".tmp")
            temporary.write_text(
                json.dumps(self.latest_envelope["snapshot"], allow_nan=False)
            )
            os.replace(temporary, self.snapshot_file)
        if self.core.revision and time.monotonic() - self.last_sensor_at < 3.0:
            try:
                transform = self.tf.lookup_transform(
                    self.odom_frame, self.navigation_frame, Time()
                )
                local_navigation = np.asarray(
                    pose_matrix(transform_pose(transform.transform))
                )
                component_id = self.latest_envelope["component_id"]
                manifests = [
                    manifest
                    for manifest in self.latest_envelope["snapshot"]["manifests"]
                    if manifest["graph_revision"]["component_id"] == component_id
                    and manifest["graph_revision"]["epoch"] == self.core.epoch
                    and manifest["graph_revision"]["revision"] == self.core.revision
                ]
                manifest = manifests[0] if len(manifests) == 1 else None
                if manifest is not None:
                    source_stamp_ns = max(
                        (
                            int(submap["observed_at_ns"])
                            for submap in manifest["submaps"]
                        ),
                        default=0,
                    )
                    authority = {
                        "robot_id": self.robot,
                        "mission_id": self.core.mission_id,
                        "participants": list(self.core.robot_names.values()),
                        "component_id": component_id,
                        "solution_order": list(self.core.solution_order),
                        "correction_revision": self.core.correction_revision,
                        "map_epoch": manifest["graph_revision"]["epoch"],
                        "mapping_graph_revision": manifest["graph_revision"][
                            "revision"
                        ],
                        "geometry_revision": manifest["geometry_revision"],
                        "map_source_stamp": {
                            "sec": source_stamp_ns // 1_000_000_000,
                            "nanosec": source_stamp_ns % 1_000_000_000,
                        },
                        "navigation_frame": self.navigation_frame,
                        "T_component_navigation": (
                            self.core.T_component_local @ local_navigation
                        ).tolist(),
                    }
                    home_key = self.core.key(0)
                    if home_key in self.core.poses:
                        T_navigation_component = np.linalg.inv(
                            self.core.T_component_local @ local_navigation
                        )
                        authority["home"] = {
                            "keyframe_id": home_key.stable_id,
                            "T_navigation_home": (
                                T_navigation_component
                                @ np.asarray(self.core.poses[home_key])
                            ).tolist(),
                        }
                    self.authority_pub.publish(
                        String(data=json.dumps(authority, allow_nan=False))
                    )
            except TransformException:
                pass
        status = {
            "robot_id": self.robot,
            "mission_id": self.core.mission_id,
            "normalized_scans": self.normalized_count,
            "keyframes": self.capture_count,
            "solutions": self.solution_count,
            "revision": self.core.revision,
            "replicated_revision": self.acked_revision,
            "replication_error": self.replica_error,
            "dropped_pairs": self.dropped,
        }
        temporary = self.status_file.with_suffix(".tmp")
        temporary.write_text(json.dumps(status))
        os.replace(temporary, self.status_file)

    def replicate(self):
        while not self.closed.wait(2.0):
            envelope = self.latest_envelope
            if envelope is None or envelope["revision"] <= self.acked_revision:
                continue
            try:
                self.client.sync(envelope, self.core.mapper.get_chunk)
                self.acked_revision = envelope["revision"]
                self.replica_error = ""
            except Exception as exc:
                self.replica_error = f"{type(exc).__name__}: {exc}"


def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.closed.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
