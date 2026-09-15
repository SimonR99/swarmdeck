#!/usr/bin/env python3
"""Capture-time normalization and persistent onboard Swarm-SLAM mapping."""

import hashlib
import json
import os
import time
from pathlib import Path
from threading import Event, Lock, Thread

import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformListener, TransformException
from cslam_common_interfaces.msg import (
    InterRobotLoopClosure,
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
from autonomy.capture_providers import (
    MAX_RAW_CAPTURE_POINTS,
    RawCaptureMetadata,
    endpoint_preserving_sample,
)
from autonomy.capture_color import (
    retain_bounded_image,
    select_geometry_and_color,
    select_rgbd_observation,
)
from autonomy.cslam import CslamMapper, pose_matrix, publish_snapshot_if_new
from autonomy.mapping import CorrectionAwareMapper, SubmapStore
from autonomy.replication import ReplicaClient

DEFAULT_STORED_RAW_CAPTURE_POINTS = 4_096
MAX_RAW_CAPTURE_CACHE_BYTES = 32 * 1024 * 1024
MAX_RAW_CAPTURE_RECORDS = 32
MAX_RGBD_RECORDS = 8
# Eight 1080p 32FC1 frames fit while malformed dimensions and unusually large
# individual messages remain unable to consume the whole cache allowance.
MAX_RGBD_STREAM_BYTES = 64 * 1024 * 1024
MAX_RGBD_MESSAGE_BYTES = 16 * 1024 * 1024
RAW_CAPTURE_JOIN_GRACE_S = 0.5
AUTHORITY_SENSOR_TTL_S = 3.0


def create_steady_timer(node, period_s, callback):
    """Create a wall-time timer which remains live under slow simulated time."""

    clock = Clock(clock_type=ClockType.STEADY_TIME)
    return clock, node.create_timer(period_s, callback, clock=clock)


def sensor_input_is_fresh(last_sensor_at, now=None):
    """Keep authority liveness tied to real sensor delivery, not ROS time."""

    current = time.monotonic() if now is None else float(now)
    age = current - float(last_sensor_at)
    return last_sensor_at > 0.0 and 0.0 <= age < AUTHORITY_SENSOR_TTL_S


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
            "sensor_domain_id": -1,
            "tf_topic": "/tf",
            "tf_static_topic": "/tf_static",
            "capture_provider": "unknown",
            "capture_provenance_topic": "",
            "color_topic": "",
            "depth_topic": "",
            "color_info_topic": "",
            "color_frame": "",
            "color_frame_convention": "optical",
            "max_stored_raw_capture_points": DEFAULT_STORED_RAW_CAPTURE_POINTS,
        }
        for key, value in params.items():
            self.declare_parameter(key, value)
        p = {key: self.get_parameter(key).value for key in params}
        use_sim_time = bool(self.get_parameter("use_sim_time").value)
        self.robot = p["robot_id"]
        self.base, self.odom_frame = p["base_frame"], p["odom_frame"]
        self.max_range = p["max_range_m"]
        self.sensor_ns = p["sensor_namespace"].strip("/")
        self.navigation_frame = p["navigation_frame"]
        sensor_domain = int(p["sensor_domain_id"])
        peer_domain = self.context.get_domain_id()
        if sensor_domain < 0:
            sensor_domain = peer_domain
        if not 0 <= sensor_domain <= 232:
            raise ValueError("sensor_domain_id must be between 0 and 232")
        self.sensor_domain_id = sensor_domain
        self.peer_domain_id = peer_domain
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
            p["capture_provider"],
        )
        self.closed = Event()
        self._shared_lock = Lock()
        self.sensor_context = None
        self.sensor_executor = None
        self.sensor_thread = None
        self.sensor_error = ""
        self.sensor_node = self
        if sensor_domain != peer_domain:
            self.sensor_context = Context()
            self.sensor_context.init(
                args=[], initialize_logging=False, domain_id=sensor_domain
            )
            self.sensor_node = Node(
                "onboard_mapper_sensor",
                context=self.sensor_context,
                namespace=self.get_namespace(),
                cli_args=[
                    "--ros-args",
                    "-r",
                    f"/tf:={p['tf_topic']}",
                    "-r",
                    f"/tf_static:={p['tf_static_topic']}",
                ],
                use_global_arguments=False,
                enable_rosout=False,
                start_parameter_services=False,
                parameter_overrides=[Parameter("use_sim_time", value=use_sim_time)],
                automatically_declare_parameters_from_overrides=True,
            )
        self.tf = Buffer(node=self.sensor_node)
        self.listener = TransformListener(self.tf, self.sensor_node)
        self.authority_pub = self.sensor_node.create_publisher(
            String, f"/{self.robot}/map_authority", 5
        )
        keyframe_qos = QoSProfile(depth=100)
        keyframe_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        keyframe_qos.reliability = ReliabilityPolicy.RELIABLE
        self.keyframe_metadata_pub = self.sensor_node.create_publisher(
            String, f"/{self.robot}/keyframes", keyframe_qos
        )
        self.cloud_pub = self.create_publisher(PointCloud2, "normalized_cloud", 5)
        self.odom_pub = self.create_publisher(Odometry, "normalized_odom", 5)
        self.pending_cloud = None
        self.last_sensor_at = 0.0
        self.clouds, self.odoms, self.capture_calibrations = {}, {}, {}
        self.color_images, self.depth_images = {}, {}
        self.color_info = None
        self.color_images_received = self.depth_images_received = 0
        self.color_frames_rejected = 0
        self.color_capture_attempts = self.color_pairs_selected = 0
        self.color_pair_rejections = self.color_tf_rejections = 0
        self.color_projection_rejections = self.colored_captures = 0
        self.color_frame = str(p["color_frame"] or "")
        self.color_frame_convention = str(p["color_frame_convention"])
        if self.color_frame_convention not in {"optical", "body"}:
            raise ValueError("color_frame_convention must be optical or body")
        self.pending_capture_since = {}
        self.raw_captures, self.raw_capture_metadata = {}, {}
        self.raw_capture_collisions = {}
        self.raw_capture_cache_bytes = 0
        self.raw_capture_source = None
        self.raw_capture_source_reset = False
        self.raw_capture_invalid_metadata = 0
        self.raw_capture_proof_mismatches = 0
        self.raw_capture_warned_at = 0.0
        self.qualified_capture_count = 0
        self.raw_capture_points_received = 0
        self.raw_capture_points_stored = 0
        self.max_stored_raw_capture_points = int(p["max_stored_raw_capture_points"])
        if not 1 <= self.max_stored_raw_capture_points <= MAX_RAW_CAPTURE_POINTS:
            raise ValueError(
                "max_stored_raw_capture_points must be between 1 and "
                f"{MAX_RAW_CAPTURE_POINTS}"
            )
        provenance_topic = str(p["capture_provenance_topic"] or "")
        self.raw_capture_enabled = bool(
            provenance_topic
            and self.core.capture_provider.spec.raw_source_contract is not None
        )
        self.normalized_count = self.capture_count = self.solution_count = (
            self.dropped
        ) = 0
        self.solution_results_received = self.solution_results_accepted = (
            self.solution_results_unchanged
        ) = 0
        self.closure_candidates = self.verified_closures = 0
        self.rejected_closures = 0
        self.closures_by_peer = {}
        self.sensor_node.create_subscription(
            PointCloud2, p["cloud_topic"], self.raw_cloud, qos_profile_sensor_data
        )
        if p["color_topic"] and p["depth_topic"] and p["color_info_topic"]:
            self.sensor_node.create_subscription(
                Image, p["color_topic"], self._color_image, qos_profile_sensor_data
            )
            self.sensor_node.create_subscription(
                Image, p["depth_topic"], self._depth_image, qos_profile_sensor_data
            )
            self.sensor_node.create_subscription(
                CameraInfo,
                p["color_info_topic"],
                self._color_info,
                qos_profile_sensor_data,
            )
        if self.raw_capture_enabled:
            self.sensor_node.create_subscription(
                String,
                provenance_topic,
                self.raw_capture_provenance,
                qos_profile_sensor_data,
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
        # Verification outcomes are published on a fleet-global topic. Keep
        # bounded counters in the local status file so an absent descriptor
        # exchange can be distinguished from geometric rejection and from a
        # later optimizer/replication failure without recording sensor data.
        self.create_subscription(
            InterRobotLoopClosure,
            "/cslam/inter_robot_loop_closure",
            self.inter_robot_closure,
            100,
        )
        self.sensor_node.create_timer(0.05, self.normalize)
        self.create_timer(0.1, self.flush_captures)
        if self.sensor_context is not None:
            self._configure_coordination_relays()
            self.sensor_executor = SingleThreadedExecutor(context=self.sensor_context)
            self.sensor_executor.add_node(self.sensor_node)
            self.sensor_thread = Thread(
                target=self._spin_sensor,
                name=f"{self.robot}-sensor-domain-{sensor_domain}",
                daemon=True,
            )
            self.sensor_thread.start()
        self.latest_envelope = None
        self.snapshot_clock, self.snapshot_timer = create_steady_timer(
            self, 1.0, self.snapshot
        )
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
        self.snapshot_file_revision = -1

    def _color_image(self, message):
        accepted = self._remember_color_frame(self.color_images, message, "color")
        with self._shared_lock:
            self.color_images_received += 1
            self.color_frames_rejected += not accepted

    def _depth_image(self, message):
        accepted = self._remember_color_frame(self.depth_images, message, "depth")
        with self._shared_lock:
            self.depth_images_received += 1
            self.color_frames_rejected += not accepted

    def _color_info(self, message):
        with self._shared_lock:
            self.color_info = message

    def _remember_color_frame(self, cache, message, kind):
        with self._shared_lock:
            return retain_bounded_image(
                cache,
                message,
                kind,
                MAX_RGBD_RECORDS,
                MAX_RGBD_STREAM_BYTES,
                MAX_RGBD_MESSAGE_BYTES,
            )

    def _capture_colors(self, points_base, capture_header):
        """Return measured RGBA only for a timestamp-qualified RGB-D observation."""
        from adapters.reconstruction import colorize_ros_rgbd

        self.color_capture_attempts += 1
        try:
            cloud_ns = int(capture_header.stamp.sec) * 1_000_000_000 + int(
                capture_header.stamp.nanosec
            )
        except (AttributeError, TypeError, ValueError):
            self.color_pair_rejections += 1
            return None
        with self._shared_lock:
            images = tuple(self.color_images.values())
            depths = tuple(self.depth_images.values())
            info = self.color_info
        selected = select_rgbd_observation(
            cloud_ns,
            images,
            depths,
            info,
            self.color_frame,
        )
        if selected is None:
            self.color_pair_rejections += 1
            return None
        image, depth, frame = selected
        self.color_pairs_selected += 1
        try:
            # The selected endpoints are expressed in base at LiDAR capture
            # time, while RGB-D may come from a nearby camera tick.  Ask tf2
            # for that exact temporal transform through fixed odometry so
            # robot motion between the two observations is preserved.
            transform = self.tf.lookup_transform_full(
                frame,
                Time.from_msg(image.header.stamp),
                self.base,
                Time.from_msg(capture_header.stamp),
                self.odom_frame,
            )
        except TransformException:
            self.color_tf_rejections += 1
            return None
        camera_from_base = np.asarray(
            pose_matrix(transform_pose(transform.transform)), dtype=np.float64
        )
        if self.color_frame_convention == "body":
            camera_from_base = (
                np.asarray(
                    [[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]],
                    dtype=np.float64,
                )
                @ camera_from_base
            )
        try:
            rgba = colorize_ros_rgbd(
                points_base,
                image,
                depth,
                info,
                camera_from_base,
            )
        except (BufferError, TypeError, ValueError):
            rgba = None
        if rgba is None or not np.any(rgba[:, 3]):
            self.color_projection_rejections += 1
            return None
        self.colored_captures += 1
        return rgba

    @staticmethod
    def _relay_robot_id(message):
        if not isinstance(message.data, str) or len(message.data.encode()) > 32_768:
            return None
        try:
            value = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return None
        robot_id = value.get("robot_id") if isinstance(value, dict) else None
        return robot_id if isinstance(robot_id, str) and robot_id else None

    def _configure_coordination_relays(self):
        """Bridge only bounded coordination JSON across the DDS boundary."""

        for topic in (
            "/swarmdeck/intentions",
            "/swarmdeck/exploration_reports",
        ):
            peer_publisher = self.create_publisher(String, topic, 20)
            local_publisher = self.sensor_node.create_publisher(String, topic, 20)
            self.sensor_node.create_subscription(
                String,
                topic,
                lambda message, publisher=peer_publisher: self._relay_from_sensor(
                    message, publisher
                ),
                20,
            )
            self.create_subscription(
                String,
                topic,
                lambda message, publisher=local_publisher: self._relay_from_peer(
                    message, publisher
                ),
                20,
            )

    def _relay_from_sensor(self, message, publisher):
        if self._relay_robot_id(message) == self.robot:
            publisher.publish(String(data=message.data))

    def _relay_from_peer(self, message, publisher):
        robot_id = self._relay_robot_id(message)
        if robot_id is not None and robot_id != self.robot:
            publisher.publish(String(data=message.data))

    def _spin_sensor(self):
        try:
            self.sensor_executor.spin()
        except ExternalShutdownException:
            pass
        except Exception as exc:
            self.sensor_error = f"{type(exc).__name__}: {exc}"
            self.get_logger().error(
                f"sensor DDS domain executor failed: {self.sensor_error}"
            )
            rclpy.try_shutdown(context=self.context)

    def close(self):
        self.closed.set()
        if self.sensor_executor is not None:
            self.sensor_executor.shutdown(timeout_sec=2.0)
        if self.sensor_thread is not None:
            self.sensor_thread.join(timeout=2.0)
        if self.sensor_context is not None:
            self.sensor_node.destroy_node()
            rclpy.try_shutdown(context=self.sensor_context)

    def raw_cloud(self, cloud):
        self.pending_cloud = cloud  # latest-only under sensor overload

    def raw_capture_provenance(self, message):
        try:
            metadata = RawCaptureMetadata.from_json(message.data)
            if metadata.provider != self.core.capture_provider.spec.name:
                raise ValueError("raw capture metadata names another provider")
        except (AttributeError, ValueError) as exc:
            self.raw_capture_invalid_metadata += 1
            now = time.monotonic()
            if now - self.raw_capture_warned_at >= 10.0:
                self.raw_capture_warned_at = now
                self.get_logger().warn(f"raw capture provenance rejected: {exc}")
            return
        source = (metadata.producer_id, metadata.sensor_epoch)
        with self._shared_lock:
            if self.raw_capture_source_reset:
                return
            if self.raw_capture_source is None:
                self.raw_capture_source = source
            elif source != self.raw_capture_source:
                self.raw_capture_source_reset = True
                self.raw_captures.clear()
                self.raw_capture_metadata.clear()
                self.raw_capture_collisions.clear()
                self.raw_capture_cache_bytes = 0
                self.get_logger().error(
                    "raw capture producer epoch changed; a new mission is required"
                )
                return
            previous = self.raw_capture_metadata.get(metadata.stamp_ns)
            if previous is not None and previous != metadata:
                self.raw_capture_collisions[metadata.stamp_ns] = None
                self.raw_capture_metadata.pop(metadata.stamp_ns, None)
            elif metadata.stamp_ns not in self.raw_capture_collisions:
                self.raw_capture_metadata[metadata.stamp_ns] = metadata
            while len(self.raw_capture_metadata) > MAX_RAW_CAPTURE_RECORDS:
                self.raw_capture_metadata.pop(next(iter(self.raw_capture_metadata)))

    def _cache_raw_capture(
        self,
        stamp_ns,
        points_base,
        mount,
        sensor_frame,
        point_count,
        points_sha256,
        source_points_finite,
    ):
        if not self.raw_capture_enabled:
            return
        with self._shared_lock:
            self.raw_capture_points_received += point_count
            source_reset = self.raw_capture_source_reset
        if source_reset:
            return
        if point_count > MAX_RAW_CAPTURE_POINTS or not source_points_finite:
            with self._shared_lock:
                self.raw_capture_collisions[stamp_ns] = None
                while len(self.raw_capture_collisions) > 2 * MAX_RAW_CAPTURE_RECORDS:
                    self.raw_capture_collisions.pop(
                        next(iter(self.raw_capture_collisions))
                    )
            return
        points = endpoint_preserving_sample(
            np.asarray(points_base, dtype=np.float64),
            self.max_stored_raw_capture_points,
        )
        points.setflags(write=False)
        record = (
            points,
            np.asarray(mount).copy(),
            sensor_frame,
            point_count,
            points_sha256,
        )
        with self._shared_lock:
            previous = self.raw_captures.get(stamp_ns)
            if previous is not None:
                same = (
                    previous[2] == sensor_frame
                    and np.array_equal(previous[0], points)
                    and np.array_equal(previous[1], mount)
                    and previous[3] == point_count
                    and previous[4] == points_sha256
                )
                if not same:
                    self.raw_capture_cache_bytes -= previous[0].nbytes
                    self.raw_captures.pop(stamp_ns, None)
                    self.raw_capture_collisions[stamp_ns] = None
                return
            if stamp_ns in self.raw_capture_collisions:
                return
            self.raw_captures[stamp_ns] = record
            self.raw_capture_cache_bytes += points.nbytes
            self.raw_capture_points_stored += len(points)
            while self.raw_captures and (
                len(self.raw_captures) > MAX_RAW_CAPTURE_RECORDS
                or self.raw_capture_cache_bytes > MAX_RAW_CAPTURE_CACHE_BYTES
            ):
                old_stamp = next(iter(self.raw_captures))
                old = self.raw_captures.pop(old_stamp)
                self.raw_capture_cache_bytes -= old[0].nbytes
                self.raw_capture_metadata.pop(old_stamp, None)
            while len(self.raw_capture_collisions) > 2 * MAX_RAW_CAPTURE_RECORDS:
                self.raw_capture_collisions.pop(next(iter(self.raw_capture_collisions)))

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
        raw_xyz = point_cloud2.read_points_numpy(
            cloud, field_names=("x", "y", "z"), skip_nans=False
        )
        raw_xyz = np.asarray(raw_xyz).reshape(-1, 3)
        raw_point_count = len(raw_xyz)
        raw_source_finite = bool(np.isfinite(raw_xyz).all())
        # Oversized captures can still feed the legacy normalized topic, but
        # never spend another full-cloud allocation computing an attestation.
        raw_points_sha256 = (
            hashlib.sha256(
                np.ascontiguousarray(raw_xyz, dtype="<f4").tobytes()
            ).hexdigest()
            if raw_point_count <= MAX_RAW_CAPTURE_POINTS and raw_source_finite
            else ""
        )
        xyz = np.asarray(raw_xyz, dtype=np.float64)
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
        with self._shared_lock:
            self.capture_calibrations[stamp_ns] = (T, cloud.header.frame_id)
            while len(self.capture_calibrations) > 1024:
                self.capture_calibrations.pop(next(iter(self.capture_calibrations)))
        self._cache_raw_capture(
            stamp_ns,
            xyz,
            T,
            cloud.header.frame_id,
            raw_point_count,
            raw_points_sha256,
            raw_source_finite,
        )
        self.odom_pub.publish(odom)
        self.cloud_pub.publish(out)
        with self._shared_lock:
            self.normalized_count += 1
            self.last_sensor_at = time.monotonic()

    def key_cloud(self, msg):
        self.clouds[msg.id] = msg.pointcloud
        self.pending_capture_since.setdefault(msg.id, time.monotonic())
        self.consume(msg.id)

    def key_odom(self, msg):
        self.odoms[msg.id] = msg.odom
        self.pending_capture_since.setdefault(msg.id, time.monotonic())
        self.consume(msg.id)

    def flush_captures(self):
        for seq in tuple(self.clouds.keys() & self.odoms.keys()):
            self.consume(seq)

    def _take_raw_capture(self, stamp, keyframe):
        with self._shared_lock:
            if stamp in self.raw_capture_collisions or self.raw_capture_source_reset:
                return None
            raw = self.raw_captures.get(stamp)
            metadata = self.raw_capture_metadata.get(stamp)
            if raw is None or metadata is None:
                return None
            if (
                metadata.frame_id != raw[2]
                or metadata.point_count != raw[3]
                or metadata.points_sha256 != raw[4]
            ):
                self.raw_captures.pop(stamp)
                self.raw_capture_metadata.pop(stamp)
                self.raw_capture_cache_bytes -= raw[0].nbytes
                self.raw_capture_collisions[stamp] = None
                self.raw_capture_proof_mismatches += 1
                return None
            self.raw_captures.pop(stamp)
            self.raw_capture_metadata.pop(stamp)
            self.raw_capture_cache_bytes -= raw[0].nbytes
        return raw[0], raw[1], raw[2], metadata.provenance(keyframe)

    def consume(self, seq):
        if seq in self.clouds and seq in self.odoms:
            cloud, odom = self.clouds[seq], self.odoms[seq]
            stamp = odom.header.stamp.sec * 1_000_000_000 + odom.header.stamp.nanosec
            if self.raw_capture_enabled:
                with self._shared_lock:
                    raw_ready = (
                        stamp in self.raw_captures
                        and stamp in self.raw_capture_metadata
                    )
                    terminal = (
                        stamp in self.raw_capture_collisions
                        or self.raw_capture_source_reset
                    )
                age = time.monotonic() - self.pending_capture_since.get(
                    seq, time.monotonic()
                )
                if not raw_ready and not terminal and age < RAW_CAPTURE_JOIN_GRACE_S:
                    return
            cloud, odom = self.clouds.pop(seq), self.odoms.pop(seq)
            self.pending_capture_since.pop(seq, None)
            xyz = point_cloud2.read_points_numpy(
                cloud, field_names=("x", "y", "z"), skip_nans=True
            )
            with self._shared_lock:
                calibration = self.capture_calibrations.get(stamp)
            if calibration is None:
                self.dropped += 1
                return
            raw = self._take_raw_capture(stamp, self.core.key(seq))
            xyz, mount, sensor_frame, provenance, colors = select_geometry_and_color(
                xyz,
                calibration,
                raw,
                # Swarm-SLAM rebuilds the keyframe PointCloud2 with a default
                # zero header. Its paired odometry retains the original
                # normalized scan timestamp used by calibration/raw joins.
                lambda selected: self._capture_colors(selected, odom.header),
            )
            accepted = self.core.capture(
                seq,
                stamp,
                pose_matrix(odom.pose.pose),
                xyz,
                T_base_sensor=mount,
                sensor_frame=sensor_frame,
                provenance=provenance,
                colors_rgba=colors,
            )
            if accepted:
                if provenance is not None:
                    self.qualified_capture_count += 1
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
        self.solution_results_received += 1
        previous_order = self.core.solution_order
        corrected = self.core.solution(msg)
        accepted = self.core.solution_order != previous_order
        if accepted:
            self.solution_results_accepted += 1
        if corrected:
            self.solution_count += 1
        elif accepted:
            self.solution_results_unchanged += 1

    def inter_robot_closure(self, msg):
        try:
            first, second = int(msg.robot0_id), int(msg.robot1_id)
        except (AttributeError, TypeError, ValueError):
            return
        if self.core.robot_index not in (first, second):
            return
        other = second if first == self.core.robot_index else first
        peer = self.core.robot_names.get(other)
        if peer is None:
            return
        self.closure_candidates += 1
        if bool(getattr(msg, "success", False)):
            self.verified_closures += 1
            self.closures_by_peer[peer] = self.closures_by_peer.get(peer, 0) + 1
        else:
            self.rejected_closures += 1

    def snapshot(self):
        with self._shared_lock:
            last_sensor_at = self.last_sensor_at
            normalized_count = self.normalized_count
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
            or self.latest_envelope["revision"] != self.core.replica_revision
        ):
            self.latest_envelope = self.core.envelope()
        if self.core.revision:
            self.snapshot_file_revision = publish_snapshot_if_new(
                self.snapshot_file,
                self.latest_envelope["snapshot"],
                self.core.revision,
                self.snapshot_file_revision,
            )
        if self.core.revision and sensor_input_is_fresh(last_sensor_at):
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
                        # MGG may use continuous odometry while the UI remains
                        # in the SLAM navigation frame. Publish both pairs from
                        # one snapshot so no consumer composes different times.
                        "planning_frame": self.odom_frame,
                        "T_component_planning": self.core.T_component_local.tolist(),
                        "peer_slam": {
                            "robot_id": self.robot,
                            "mission_id": self.core.mission_id,
                            "keyframes": len(self.core.local_poses),
                            "verified": self.verified_closures,
                            "rejected": self.rejected_closures,
                            "by_peer": dict(self.closures_by_peer),
                        },
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
            "normalized_scans": normalized_count,
            "keyframes": self.capture_count,
            "qualified_raw_captures": self.qualified_capture_count,
            "capture_provider": self.core.capture_provider.spec.name,
            "raw_capture_source_reset": self.raw_capture_source_reset,
            "raw_capture_invalid_metadata": self.raw_capture_invalid_metadata,
            "raw_capture_proof_mismatches": self.raw_capture_proof_mismatches,
            "raw_capture_timestamp_collisions": len(self.raw_capture_collisions),
            "raw_capture_points_received": self.raw_capture_points_received,
            "raw_capture_points_stored": self.raw_capture_points_stored,
            "max_stored_raw_capture_points": self.max_stored_raw_capture_points,
            "color_images_received": self.color_images_received,
            "depth_images_received": self.depth_images_received,
            "color_frames_rejected": self.color_frames_rejected,
            "color_capture_attempts": self.color_capture_attempts,
            "color_pairs_selected": self.color_pairs_selected,
            "color_pair_rejections": self.color_pair_rejections,
            "color_tf_rejections": self.color_tf_rejections,
            "color_projection_rejections": self.color_projection_rejections,
            "colored_captures": self.colored_captures,
            # `solutions` is retained for compatibility and has always counted
            # pose-changing corrections rather than native optimizer messages.
            "solutions": self.solution_count,
            "solution_results_received": self.solution_results_received,
            "solution_results_accepted": self.solution_results_accepted,
            "solution_results_unchanged": self.solution_results_unchanged,
            "inter_robot_closure_candidates": self.closure_candidates,
            "inter_robot_closures_verified": self.verified_closures,
            "inter_robot_closures_rejected": self.rejected_closures,
            "inter_robot_closures_by_peer": dict(sorted(self.closures_by_peer.items())),
            "corrections_applied": self.solution_count,
            "last_solution_order": list(self.core.solution_order),
            # `revision` remains the replica publication sequence so it stays
            # comparable with `replicated_revision`. Map consumers use the
            # independent graph revision carried by the snapshot authority.
            "revision": self.core.replica_revision,
            "mapping_graph_revision": self.core.revision,
            "replicated_revision": self.acked_revision,
            "replication_error": self.replica_error,
            "dropped_pairs": self.dropped,
            "peer_domain_id": self.peer_domain_id,
            "sensor_domain_id": self.sensor_domain_id,
            "sensor_error": self.sensor_error,
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
