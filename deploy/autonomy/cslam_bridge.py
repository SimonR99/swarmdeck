#!/usr/bin/env python3
"""Capture-time normalization and persistent onboard Swarm-SLAM mapping.

The map authority published on ``/<robot>/map_authority`` is product-gated:
it names the newest MOLA product the worker has published for this peer
(``<root>/mola/index.json`` with its ``source.json``), never a pose-graph
revision that has no product yet. Because a product built at revision R
places its geometry with the correction, solution order and home pose in
effect at R, the authority carries that revision's frame state from
``CslamMapper.frame_history`` rather than the current one. Revision
increments alone are not frame changes, so consumers keep executing routes
across product transitions.
"""

import hashlib
import json
import math
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
    RobotHeartbeat,
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
from autonomy.peer_mask import (
    MAX_PEER_POSE_TOLERANCE_S,
    PeerBodyMask,
    idle_mask_counters,
    peer_body,
)
from autonomy.cslam import (
    CslamMapper,
    pose_matrix,
    publish_snapshot_if_new,
    remove_stale_unique_temporaries,
    write_unique_temporary,
)
from autonomy.mapping import CorrectionAwareMapper, SubmapStore
from autonomy.map_epochs import (
    map_epoch_lock,
    read_map_epoch,
    robot_run_id,
    read_peer_epochs,
    write_peer_epochs,
)
from autonomy.product_authority import (
    build_authority,
    product_authority_key,
    read_published_product,
    read_worker_status,
)
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
# The peer-root files the bridge writes through `write_unique_temporary`;
# a start deletes their temporaries that a crashed bridge left behind.
BRIDGE_WRITTEN_FILES = ("snapshot.json", "status.json", "graph_solution.json")
# The wire cadence of the map authority heartbeat, independent of
# `_snapshot`'s own 1 Hz timer: `AuthorityHeartbeatPublisher.run` publishes
# on this period from its own thread, so a snapshot tick that never runs at
# all (an executor starved by other callbacks) never silences the topic.
AUTHORITY_HEARTBEAT_PERIOD_S = 1.0
# How long the heartbeat keeps re-sending the last fresh authority while no
# fresh one can be built. A healthy robot, parked or not, builds a fresh
# authority every `_snapshot` tick, so this only bounds a stall: sensors, TF
# or `_snapshot` itself that stay dead past it lose map authority (the
# heartbeat falls back to `resetting`), as they did before the re-send.
AUTHORITY_RESEND_MAX_S = 10.0
# The longest a heartbeat send waits for map_epoch_lock. Other holders
# (claims, the worker's publication renames and directory fsyncs, the peer
# epoch watermark) can be slow; a send that cannot get the lock in time is
# skipped ("epoch lock busy"), never blocked behind them.
AUTHORITY_EPOCH_LOCK_WAIT_S = AUTHORITY_HEARTBEAT_PERIOD_S / 2
# Bounds Bridge.close()'s wait for the heartbeat thread to exit. Safety does
# not depend on it: close() fences every heartbeat publish and log first
# (`_authority_closed`), so a thread still blocked, e.g. on map_epoch_lock,
# can touch no ROS object once close() returns.
AUTHORITY_HEARTBEAT_JOIN_TIMEOUT_S = AUTHORITY_HEARTBEAT_PERIOD_S + 5.0
# A parked robot's scans are the same scan. Swarm-SLAM earns keyframes by
# distance, and by scene change at most once per
# keyframe_scene_change_min_period_s (cslam_lidar.yaml), so a scan that
# repeats the previously normalized pose is only worth normalizing at that
# period: everything in between would be deserialized, hashed, masked and
# published for nothing.
PARKED_TRANSLATION_M = 0.02
PARKED_ROTATION_RAD = 0.05
PARKED_REPUBLISH_S = 5.0


def parked_since(previous, pose, now, last_at):
    """True when `pose` repeats `previous` and the last normalization is recent.

    `previous` and `pose` are (x, y, z, qx, qy, qz, qw) tuples.
    """

    if previous is None or now - last_at >= PARKED_REPUBLISH_S:
        return False
    if math.dist(previous[:3], pose[:3]) > PARKED_TRANSLATION_M:
        return False
    dot = abs(sum(a * b for a, b in zip(previous[3:], pose[3:])))
    return 2.0 * math.acos(min(1.0, dot)) <= PARKED_ROTATION_RAD


def create_steady_timer(node, period_s, callback):
    """Create a wall-time timer which remains live under slow simulated time."""

    clock = Clock(clock_type=ClockType.STEADY_TIME)
    return clock, node.create_timer(period_s, callback, clock=clock)


def write_text_if_changed(path, text, last, replace=os.replace):
    """Atomically write ``text`` to ``path`` only when it differs from ``last``.

    Returns the text now on disk, to pass as ``last`` on the next call. A
    peer's status.json was rewritten every tick whether or not anything in it
    had changed; most of a parked peer's ticks change nothing.
    ``replace(temporary, path)`` moves the written temporary into place.
    """

    if text == last:
        return last
    temporary = write_unique_temporary(path, text)
    replace(temporary, path)
    return text


class MapEpochRetired(Exception):
    """A newer map epoch was claimed; this bridge's run may write nothing."""


def sensor_input_is_fresh(last_sensor_at, now=None):
    """Keep authority liveness tied to real sensor delivery, not ROS time."""

    current = time.monotonic() if now is None else float(now)
    age = current - float(last_sensor_at)
    return last_sensor_at > 0.0 and 0.0 <= age < AUTHORITY_SENSOR_TTL_S


def authority_is_serializable(authority):
    try:
        json.dumps(authority, allow_nan=False)
    except (TypeError, ValueError):
        return False
    return True


def resetting_authority(*, robot_id, mission_id, robot_map_epoch, run_id):
    """The authority message that tells MGG this run has no map to offer."""

    return {
        "robot_id": robot_id,
        "mission_id": mission_id,
        "robot_map_epoch": robot_map_epoch,
        "run_id": run_id,
        "state": "resetting",
    }


def authority_heartbeat(
    built, cached, *, robot_id, mission_id, robot_map_epoch, run_id, cache_age_s
):
    """Decide the map-authority message to publish this tick.

    Sensor freshness, TF and the product read gate a *fresh* authority, but
    they never gate the heartbeat itself: a freshly built ``authority``
    (``build_authority``'s dict, or None when this tick could not produce
    one) is published and cached; otherwise the last cached authority is
    re-sent byte-for-byte so MGG's snapshot never expires from a transient
    stall. The cache is honoured only while it still names the peer's
    current mission, run and map epoch, so a real reset (a new run_id) falls
    through to a ``resetting`` status instead of an authority for a map that
    no longer exists. ``cache_age_s`` is the time since ``cached`` was last
    built fresh; once it reaches `AUTHORITY_RESEND_MAX_S` the stall is no
    longer transient and the cache falls through to ``resetting`` too.

    Returns ``(message, new_cache)``. ``new_cache`` is the dict to keep for
    the next tick: the fresh authority, the same cached authority when it was
    re-sent, or None once there is nothing left worth re-sending.

    A ``built`` authority that cannot be serialized for the wire (a NaN) is
    treated as not built, so it never evicts the last good one.
    """

    if built is not None and authority_is_serializable(built):
        return built, built
    if (
        cached is not None
        and cached.get("robot_id") == robot_id
        and cached.get("mission_id") == mission_id
        and cached.get("robot_map_epoch") == robot_map_epoch
        and cached.get("run_id") == run_id
        and cache_age_s < AUTHORITY_RESEND_MAX_S
    ):
        return cached, cached
    return (
        resetting_authority(
            robot_id=robot_id,
            mission_id=mission_id,
            robot_map_epoch=robot_map_epoch,
            run_id=run_id,
        ),
        None,
    )


class AuthorityHeartbeatPublisher:
    """Send the cached map authority on its own schedule and thread.

    `update` (called from `Bridge._snapshot`, on the busy main executor)
    only replaces the cached message; it never sends. `tick` (called from
    `run`, on a dedicated thread that never touches the executor) sends
    whatever is cached. A `_snapshot` tick that takes arbitrarily long, or
    does not run at all for a while, delays only the next *fresh* authority;
    the heartbeat keeps firing every `period_s` regardless, which is the fix
    for MGG's snapshot expiring from an executor stall. The stall is bounded:
    a message updated with ``fresh_at`` (the `clock` time its authority was
    last built fresh) is sent as its ``resetting`` form once
    `AUTHORITY_RESEND_MAX_S` has passed since then, so a `_snapshot` that
    stops running altogether still loses map authority.

    `send(message)` returns None once it has published `message`, or the
    reason it did not. `Bridge._send_authority` validates the durable map
    epoch and publishes as one step under map_epoch_lock, so an authority is
    never sent for a retired epoch, even when a new epoch is claimed between
    this tick reading the cache and sending. An exception from `send` (the
    epoch record unreadable, say) is a reason too: nothing was sent, and the
    thread keeps running.

    `log(reason)` is called whenever the interval between two completed
    sends exceeds 1.5 × `period_s`; that interruption of the wire heartbeat
    is what MGG observes as a gap. (`run` waits a full period after each
    send, so every interval is a little over one period; only the excess
    beyond half a period is a real delay.) A completed send is stamped only
    after `send` returns, so a send that blocks shows up as its own gap,
    and the reason says whether the send blocked or the tick started late. A
    `_snapshot` build failure that still leaves a valid cached message to
    re-send is never a gap (`status.json`'s ``authority_gap_*`` records
    those). Callers rate-limit repeated reasons themselves.
    """

    def __init__(self, send, *, period_s, log, clock=time.monotonic):
        self._send = send
        self._period_s = period_s
        self._gap_s = 1.5 * period_s
        self._log = log
        self._clock = clock
        # Guards the cached message against `update`/`invalidate` from the
        # main executor.
        self._lock = Lock()
        self._message = None
        self._fresh_at = None
        self._no_message_reason = "no authority yet"
        # Heartbeat-thread only: the monotonic time of the last completed
        # send, or None before the first; `_started_at` stands in for it
        # until then, so a robot that never gets a first authority still
        # logs "no authority yet" rather than staying silent forever.
        self._last_sent_at = None
        self._started_at = clock()

    def update(self, message, fresh_at=None):
        """Replace the message the next tick(s) send; never sends.

        ``fresh_at`` bounds how long ``message`` is re-sent (see the class
        docstring); None sends it until the next update.
        """

        with self._lock:
            self._message = message
            self._fresh_at = fresh_at
            self._no_message_reason = "no authority yet"

    def invalidate(self):
        """Discard the cached message: a durable epoch change retired it.

        Called from `Bridge.snapshot()` when it notices; `send` enforces the
        same fence on every send whether or not this has run.
        """

        with self._lock:
            self._message = None
            self._no_message_reason = "map epoch retired"

    def tick(self):
        """Send the cached message, if any, and log an actual send gap."""

        started = self._clock()
        with self._lock:
            message = self._message
            fresh_at = self._fresh_at
            reason = self._no_message_reason
        if (
            message is not None
            and fresh_at is not None
            and started - fresh_at >= AUTHORITY_RESEND_MAX_S
        ):
            message = resetting_authority(
                robot_id=message["robot_id"],
                mission_id=message["mission_id"],
                robot_map_epoch=message["robot_map_epoch"],
                run_id=message["run_id"],
            )
        last = self._started_at if self._last_sent_at is None else self._last_sent_at
        if message is not None:
            try:
                reason = self._send(message)
            except Exception as exc:
                reason = f"send failed ({type(exc).__name__}: {exc})"
            if reason is None:
                self._last_sent_at = self._clock()
                blocked = self._last_sent_at - started
                if blocked > self._period_s / 2:
                    reason = f"send blocked {blocked:.1f}s (map epoch lock or DDS)"
                else:
                    reason = (
                        f"heartbeat tick started {started - last:.1f}s "
                        "after the last send"
                    )
        now = self._clock()
        if now - last > self._gap_s:
            self._log(f"{now - last:.1f}s between sends: {reason}")

    def run(self, closed):
        """Tick every `period_s` until `closed` is set (a `threading.Event`)."""

        while not closed.wait(self._period_s):
            self.tick()


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
            "map_epoch": 0,
            "run_id": "",
            "sensor_namespace": "robot_0",
            "base_frame": "robot_0/base_link",
            "odom_frame": "robot_0/odom",
            "cloud_topic": "/robot_0/scan/points",
            "server_url": "",
            "store_root": "/maps",
            "max_range_m": 30.0,
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
            # Peer-body masking. Off unless a profile turns it on; see the
            # block that reads these below for why hardware leaves it off.
            "peer_body_mask": False,
            "peer_platforms": "{}",
            "peer_pose_topic_template": "/{robot}/ground_truth",
            "peer_pose_frame": "world",
            "peer_mask_margin_m": 0.15,
            "peer_mask_pose_tolerance_s": 0.05,
        }
        for key, value in params.items():
            self.declare_parameter(key, value)
        p = {key: self.get_parameter(key).value for key in params}
        use_sim_time = bool(self.get_parameter("use_sim_time").value)
        self.robot = p["robot_id"]
        self.base, self.odom_frame = p["base_frame"], p["odom_frame"]
        self.max_range = p["max_range_m"]
        self.sensor_ns = p["sensor_namespace"].strip("/")
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
        run_id = robot_run_id(p["mission_id"], self.robot, int(p["map_epoch"]))
        if p["run_id"] != run_id:
            raise ValueError("bridge run_id does not match its map epoch")
        root = Path(p["store_root"]) / p["mission_id"] / self.robot
        root.mkdir(parents=True, exist_ok=True)
        # The peer root: snapshot.json, status.json and graph_solution.json
        # live here, and the MOLA worker publishes its products under mola/.
        self.root = root
        remove_stale_unique_temporaries(root, BRIDGE_WRITTEN_FILES)
        reset_root = os.environ.get("SWARMDECK_SIM_RESET_DIR", "")
        self.reset_root = Path(reset_root) if reset_root else None
        record = read_map_epoch(root)
        if record is None or record["run_id"] != run_id:
            raise ValueError("peer launch must durably claim the map epoch")
        self.core = CslamMapper(
            CorrectionAwareMapper(SubmapStore(root / "geometry")),
            self.robot,
            p["robot_index"],
            p["mission_id"],
            names,
            p["capture_provider"],
            map_epoch=int(p["map_epoch"]),
        )
        known = read_peer_epochs(root)
        if known is not None:
            if known["mission_id"] != self.core.mission_id:
                raise ValueError("peer epoch watermark belongs to another mission")
            for index, name in names.items():
                if index != self.core.robot_index and name in known["robot_map_epochs"]:
                    self.core.observe_epoch(index, known["robot_map_epochs"][name])
        self._persist_peer_epochs()
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
        self.pending_cloud_since = 0.0
        self.normalize_retry_s = 0.05
        self.normalize_retry_timer = None
        self.capture_retry_timer = None
        # Sensor callbacks run on a separate executor in split-domain mode.
        # Wake the peer executor rather than consuming keyframes across threads.
        self.capture_ready = self.create_guard_condition(self.flush_captures)
        self.last_sensor_at = 0.0
        self.last_normalized_pose = None
        self.last_normalized_at = 0.0
        self.captures_skipped_parked = 0
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
        # Peer-body mask. A grouped start has every robot scanning its
        # neighbours, and those returns describe a body that drives away, so
        # they are removed here rather than left for later free-space rays to
        # argue with. It stays off unless a profile enables it: masking needs a
        # peer pose in a frame shared with this capture at the capture stamp,
        # which simulation publishes and a hardware fleet does not guarantee.
        self.captures_held_unmaskable = 0
        self.peer_mask = None
        self.peer_mask_margin_m = float(p["peer_mask_margin_m"])
        self.peer_mask_tolerance_s = float(p["peer_mask_pose_tolerance_s"])
        self.peer_pose_frame = str(p["peer_pose_frame"]).strip("/")
        if bool(p["peer_body_mask"]):
            if not 0.0 < self.peer_mask_tolerance_s <= MAX_PEER_POSE_TOLERANCE_S:
                raise ValueError(
                    "peer_mask_pose_tolerance_s must be positive and at most "
                    f"{MAX_PEER_POSE_TOLERANCE_S}"
                )
            if not self.peer_pose_frame:
                raise ValueError("peer_pose_frame must name the shared pose frame")
            template = str(p["peer_pose_topic_template"])
            if "{robot}" not in template:
                raise ValueError("peer_pose_topic_template must contain {robot}")
            platforms = json.loads(p["peer_platforms"] or "{}")
            fleet = sorted(names.values())
            missing = sorted(set(fleet) - set(platforms))
            if missing:
                raise ValueError(
                    "peer_platforms must give every fleet robot a platform; "
                    f"missing {missing}"
                )
            # This robot's own body is resolved too: an unknown platform must
            # fail at startup rather than silently leave one robot unmasked.
            self.peer_mask = PeerBodyMask(
                self.robot,
                {name: peer_body(platforms[name]) for name in fleet},
                int(self.peer_mask_tolerance_s * 1e9),
                self.peer_mask_margin_m,
            )
            for name in fleet:
                self.sensor_node.create_subscription(
                    Odometry,
                    template.format(robot=name),
                    self._peer_pose_callback(name),
                    qos_profile_sensor_data,
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
        ) = self.solution_results_deferred = 0
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
        for index in names:
            self.create_subscription(
                RobotHeartbeat, f"/r{index}/cslam/heartbeat", self.peer_heartbeat, 10
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
        # Product-gated authority diagnostics: the product revision advertised
        # on the last tick, and the ticks on which a product existed but no
        # frame state could be paired with it.
        self.authority_revision = None
        self.authority_skipped = 0
        # The last authority actually published (fresh or re-sent), kept so
        # the heartbeat can re-send it verbatim while a fresh one cannot be
        # built; see `authority_heartbeat`. `authority_fresh_at` is the
        # monotonic time that cache was last built fresh, which bounds the
        # re-send (`AUTHORITY_RESEND_MAX_S`).
        self.authority_cache = None
        self.authority_fresh_at = None
        self.authority_gap_ticks = 0
        self.authority_gap_reason = ""
        self._product_memo = {}
        # status.json is only rewritten when its content actually changes.
        self._last_status_json = None
        # Publishes the cached authority on its own thread, never the main
        # executor `_snapshot` runs on; see `AuthorityHeartbeatPublisher`.
        # `_authority_send_lock` guards `_authority_closed` and every
        # heartbeat publish and gap log; close() sets the flag under it.
        # Lock order: map_epoch_lock, then `_authority_send_lock`.
        self._authority_send_lock = Lock()
        self._authority_closed = False
        self._authority_heartbeat = AuthorityHeartbeatPublisher(
            self._send_authority,
            period_s=AUTHORITY_HEARTBEAT_PERIOD_S,
            log=self._log_authority_gap,
        )
        self.authority_heartbeat_thread = Thread(
            target=self._authority_heartbeat.run,
            args=(self.closed,),
            name=f"{self.robot}-authority-heartbeat",
            daemon=True,
        )
        self.authority_heartbeat_thread.start()

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
        # Fence the heartbeat before anything it touches (self.authority_pub
        # on self.sensor_node, and the node's logger) is destroyed below or
        # by `main()`: this waits for a publish or log already in flight,
        # and every later one sees the flag and does nothing, whether or
        # not the join below succeeds.
        with self._authority_send_lock:
            self._authority_closed = True
        self.closed.set()
        self.authority_heartbeat_thread.join(timeout=AUTHORITY_HEARTBEAT_JOIN_TIMEOUT_S)
        if self.authority_heartbeat_thread.is_alive():
            self.get_logger().error(
                "authority heartbeat thread did not stop within "
                f"{AUTHORITY_HEARTBEAT_JOIN_TIMEOUT_S:.1f}s of close(); it is "
                "fenced and can no longer publish or log"
            )
        if self.sensor_executor is not None:
            self.sensor_executor.shutdown(timeout_sec=2.0)
        if self.sensor_thread is not None:
            self.sensor_thread.join(timeout=2.0)
        if self.sensor_context is not None:
            self.sensor_node.destroy_node()
            rclpy.try_shutdown(context=self.sensor_context)

    def _peer_pose_callback(self, robot):
        """Buffer one robot's timestamped pose in the shared reference frame.

        Every sample is retained with its own stamp. The mask joins a capture
        to the sample that was true at the capture stamp, so nothing here may
        collapse the history down to a latest pose.
        """

        def handle(message):
            # Composing poses only means anything if they share a frame. A
            # sample in a robot-local odometry frame is not comparable with
            # ours and must never be silently treated as if it were.
            if message.header.frame_id.strip("/") != self.peer_pose_frame:
                self.peer_mask.reject_pose()
                return
            stamp_ns = (
                message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            )
            self.peer_mask.add_pose(robot, stamp_ns, pose_matrix(message.pose.pose))

        return handle

    def raw_cloud(self, cloud):
        self.pending_cloud = cloud  # latest-only while capture-time TF is missing
        self.pending_cloud_since = time.monotonic()
        self.normalize_retry_s = 0.05
        self.normalize()

    def _cancel_normalize_retry(self):
        if self.normalize_retry_timer is not None:
            self.normalize_retry_timer.cancel()
            self.sensor_node.destroy_timer(self.normalize_retry_timer)
            self.normalize_retry_timer = None

    def _retry_normalize(self):
        remaining = AUTHORITY_SENSOR_TTL_S - (
            time.monotonic() - self.pending_cloud_since
        )
        delay = max(0.001, min(self.normalize_retry_s, remaining))
        self.normalize_retry_clock, self.normalize_retry_timer = create_steady_timer(
            self.sensor_node, delay, self.normalize
        )
        self.normalize_retry_s = min(1.0, 2 * self.normalize_retry_s)

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
        self.capture_ready.trigger()

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
        self._cancel_normalize_retry()
        cloud = self.pending_cloud
        if cloud is None:
            return
        if time.monotonic() - self.pending_cloud_since >= AUTHORITY_SENSOR_TTL_S:
            self.pending_cloud = None
            self.get_logger().warn(
                "dropping cloud waiting for capture-time TF", throttle_duration_sec=5.0
            )
            return
        stamp = Time.from_msg(cloud.header.stamp)
        try:
            mount = self.tf.lookup_transform(self.base, cloud.header.frame_id, stamp)
            local = self.tf.lookup_transform(self.odom_frame, self.base, stamp)
        except TransformException:
            self._retry_normalize()
            return  # never substitute a latest transform for capture-time TF
        self.pending_cloud = None
        pose = (
            local.transform.translation.x,
            local.transform.translation.y,
            local.transform.translation.z,
            local.transform.rotation.x,
            local.transform.rotation.y,
            local.transform.rotation.z,
            local.transform.rotation.w,
        )
        now = time.monotonic()
        if parked_since(self.last_normalized_pose, pose, now, self.last_normalized_at):
            # The sensor is live; only the repeated scan is not worth the work.
            self.captures_skipped_parked += 1
            with self._shared_lock:
                self.last_sensor_at = now
            return
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
        stamp_ns = cloud.header.stamp.sec * 1_000_000_000 + cloud.header.stamp.nanosec
        # Before anything downstream sees the capture: the normalized topic
        # Swarm-SLAM keyframes from, the cached raw capture that carries
        # provenance, and every MOLA product that derives free-space evidence
        # from it. Masked points are dropped endpoints, not free space.
        if self.peer_mask is not None:
            if not self.peer_mask.can_place_peers(stamp_ns):
                # The mask cannot place this robot (the first moments after a
                # reset) or a tracked neighbour (a pose relay gap) at this
                # stamp. A keyframe taken from an unmasked capture keeps a
                # parked neighbour in the map for as long as it stays, and a
                # moving one as a phantom rise until free-space rays retire
                # it, so this capture is not published. The next scan comes
                # 50 ms later.
                self.captures_held_unmaskable += 1
                return
            xyz = self.peer_mask.apply(xyz, stamp_ns)
        header = Header(stamp=cloud.header.stamp, frame_id=self.base)
        out = point_cloud2.create_cloud_xyz32(header, xyz)
        odom = Odometry()
        odom.header = Header(stamp=cloud.header.stamp, frame_id=self.odom_frame)
        odom.child_frame_id = self.base
        odom.pose.pose = transform_pose(local.transform)
        # TF has no covariance. Zero is explicitly unknown at this boundary;
        # optimizer noise remains configured independently, never inferred here.
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
        self.capture_ready.trigger()
        self.odom_pub.publish(odom)
        self.cloud_pub.publish(out)
        self.last_normalized_pose = pose
        self.last_normalized_at = time.monotonic()
        with self._shared_lock:
            self.normalized_count += 1
            self.last_sensor_at = self.last_normalized_at

    def key_cloud(self, msg):
        if (
            msg.mission_id != self.core.mission_id
            or msg.map_epoch != self.core.map_epoch
        ):
            return
        self.clouds[msg.id] = msg.pointcloud
        self.pending_capture_since.setdefault(msg.id, time.monotonic())
        self.flush_captures()

    def key_odom(self, msg):
        if (
            msg.mission_id != self.core.mission_id
            or msg.map_epoch != self.core.map_epoch
        ):
            return
        self.odoms[msg.id] = msg.odom
        self.pending_capture_since.setdefault(msg.id, time.monotonic())
        self.flush_captures()

    def flush_captures(self):
        if self.capture_retry_timer is not None:
            self.capture_retry_timer.cancel()
            self.destroy_timer(self.capture_retry_timer)
            self.capture_retry_timer = None
        for seq in tuple(self.clouds.keys() | self.odoms.keys()):
            self.consume(seq)
        waiting = self.clouds.keys() & self.odoms.keys()
        if waiting:
            # Only the bounded raw/provenance join needs a deadline. New raw
            # data wakes this method via capture_ready before that deadline.
            deadline = min(self.pending_capture_since[seq] for seq in waiting)
            delay = max(0.001, deadline + RAW_CAPTURE_JOIN_GRACE_S - time.monotonic())
            self.capture_retry_clock, self.capture_retry_timer = create_steady_timer(
                self, delay, self.flush_captures
            )

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
                                "mission_id": self.core.mission_id,
                                "robot_map_epoch": self.core.map_epoch,
                                "run_id": self.core.run_id,
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
                expired = min(cache)
                cache.pop(expired)
                self.pending_capture_since.pop(expired, None)
                self.dropped += 1

    def optimized(self, msg):
        """Feed one optimizer result to the core and count what became of it.

        The core accepts a result when it is newer than any it has seen
        (``solver_order`` advances); it adopts one only when the poses moved
        beyond the change tolerance and the adoption interval allows. An
        accepted result is therefore exactly one of adopted, deferred (held
        by the interval, ``core.deferred_solution``) or unchanged.
        """

        self.solution_results_received += 1
        previous_order = self.core.solver_order
        previous_epochs = self.core.peer_epoch_revision
        adopted = self.core.solution(msg)
        if self.core.peer_epoch_revision != previous_epochs:
            self._persist_peer_epochs()
        if self.core.solver_order == previous_order:
            return
        self.solution_results_accepted += 1
        if adopted:
            self.solution_count += 1
        elif self.core.deferred_solution is not None:
            self.solution_results_deferred += 1
        else:
            self.solution_results_unchanged += 1

    def _persist_peer_epochs(self):
        with map_epoch_lock(self.root):
            current = read_map_epoch(self.root)
            if current is None or current["run_id"] != self.core.run_id:
                return
            write_peer_epochs(
                self.root,
                self.core.mission_id,
                {
                    self.core.robot_names[index]: epoch
                    for index, epoch in self.core.robot_map_epochs.items()
                },
            )

    def peer_heartbeat(self, msg):
        if msg.mission_id != self.core.mission_id:
            return
        index, epoch = int(msg.robot_id), int(msg.map_epoch)
        previous = self.core.robot_map_epochs.get(index)
        if previous is None or not self.core.observe_epoch(index, epoch):
            return
        if epoch > previous:
            self._persist_peer_epochs()
            peer = self.core.robot_names[index]
            removed = self.closures_by_peer.pop(peer, 0)
            self.verified_closures = max(0, self.verified_closures - removed)
            self._product_memo.clear()

    def inter_robot_closure(self, msg):
        previous_epochs = self.core.peer_epoch_revision
        if not self.core.accepts_epoch_vector(msg):
            return
        if self.core.peer_epoch_revision != previous_epochs:
            self._persist_peer_epochs()
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

    def source_reset_stamp(self):
        """Read the supervisor's real source-reset ACK, never infer from a launch."""
        if self.core.map_epoch == 0:
            return {}
        if self.reset_root is None:
            return None
        try:
            status = json.loads(
                (
                    self.reset_root / "robots" / self.robot / "mgg-status.json"
                ).read_text()
            )
            stamp = status["source_reset_stamp"]
            if (
                status["source_reset_run_id"] != self.core.run_id
                or type(stamp["sec"]) is not int
                or type(stamp["nanosec"]) is not int
                or stamp["sec"] < 0
                or not 0 <= stamp["nanosec"] < 1_000_000_000
            ):
                return None
            return stamp
        except (OSError, ValueError, KeyError, TypeError):
            return None

    def _log_authority_gap(self, reason):
        """Called from the heartbeat thread: the actual publish interval
        just exceeded one heartbeat period.
        """

        with self._authority_send_lock:
            if self._authority_closed:
                return
            self.get_logger().warn(
                f"map authority heartbeat gap ({reason})", throttle_duration_sec=5.0
            )

    def _send_authority(self, message):
        """Publish `message` only while the durable map epoch names its run.

        Called on the heartbeat thread. The epoch record is re-read and the
        message published under map_epoch_lock, the lock `claim_map_epoch()`
        holds while it retires a run, so no claim can land between this
        check and the publish. The lock wait is bounded
        (`AUTHORITY_EPOCH_LOCK_WAIT_S`): a busy lock skips this send. Returns
        None once published, otherwise the
        reason; an unreadable record raises, which the heartbeat reports as
        a failed send (fail closed).
        """

        data = json.dumps(message, allow_nan=False)
        with map_epoch_lock(self.root, timeout=AUTHORITY_EPOCH_LOCK_WAIT_S) as locked:
            if not locked:
                return "epoch lock busy"
            record = read_map_epoch(self.root)
            if record is None:
                return "no map epoch claimed"
            if record["run_id"] != message["run_id"]:
                return "map epoch retired"
            with self._authority_send_lock:
                if self._authority_closed:
                    return "bridge closed"
                self.authority_pub.publish(String(data=data))
        return None

    def _replace_if_current(self, temporary, path):
        """Rename ``temporary`` onto ``path`` while this run's epoch is current.

        graph_solution.json, snapshot.json and status.json are files
        `claim_map_epoch()` deletes when it retires this run, so each rename
        re-reads the durable epoch under map_epoch_lock. Only the small epoch
        read and the rename happen under the lock; the heartbeat takes the
        same lock before every send. Raises MapEpochRetired, after removing
        ``temporary``, when the epoch has moved on. ``temporary`` must be
        this writer's own (`write_unique_temporary`): a bridge process of
        another epoch writing the same file uses a different one, so neither
        can overwrite or delete the bytes the other renames.
        """

        try:
            with map_epoch_lock(self.root):
                record = read_map_epoch(self.root)
                if record is not None and record["run_id"] == self.core.run_id:
                    os.replace(temporary, path)
                    return
        finally:
            temporary.unlink(missing_ok=True)
        raise MapEpochRetired(self.core.run_id)

    def snapshot(self):
        # Not under map_epoch_lock: `_snapshot` reads the product (up to
        # 64 MiB, with retries) and builds the authority, and the heartbeat
        # needs that lock before every send. Each file write re-validates
        # the epoch under the lock (`_replace_if_current`), and the heartbeat
        # re-validates it before every send.
        record = read_map_epoch(self.root)
        if record is None or record["run_id"] != self.core.run_id:
            # A fresh launch retired this robot's map epoch.
            self._authority_heartbeat.invalidate()
            return
        try:
            self._snapshot()
        except MapEpochRetired:
            self._authority_heartbeat.invalidate()

    def _snapshot(self):
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
            temporary = write_unique_temporary(
                self.graph_solution_file,
                json.dumps(
                    {
                        "schema": "swarmdeck.pose-snapshot.v1",
                        "solution": solution.canonical_dict(),
                    },
                    allow_nan=False,
                ),
            )
            self._replace_if_current(temporary, self.graph_solution_file)
            self.graph_solution_revision = self.core.revision
        if self.core.revision and (
            self.latest_envelope is None
            or self.latest_envelope["revision"] != self.core.replica_revision
        ):
            self.latest_envelope = self.core.envelope()
        if self.core.revision:
            self.snapshot_file_revision = publish_snapshot_if_new(
                self.snapshot_file,
                {
                    **self.latest_envelope["snapshot"],
                    "mission_id": self.core.mission_id,
                    "robot_id": self.robot,
                    "robot_map_epoch": self.core.map_epoch,
                    "run_id": self.core.run_id,
                    "robot_map_epochs": self.latest_envelope["robot_map_epochs"],
                    "participant_robot_ids": self.latest_envelope[
                        "participant_robot_ids"
                    ],
                },
                self.core.revision,
                self.snapshot_file_revision,
                replace=self._replace_if_current,
            )
        self.authority_revision = None
        source_reset_stamp = self.source_reset_stamp()
        # A fresh authority needs sensor freshness, the reset ACK and a
        # product paired to the current revision; none of that gates the
        # heartbeat below, only whether this tick's authority is fresh or
        # re-sent (`authority_heartbeat`). `gap_reason` names why this tick
        # could not build a fresh one, for the rate-limited log.
        built_authority = None
        gap_reason = None
        if not self.core.revision:
            gap_reason = "no graph revision yet"
        elif not sensor_input_is_fresh(last_sensor_at):
            gap_reason = "sensor input stale"
        elif source_reset_stamp is None:
            gap_reason = "map reset acknowledgement unavailable"
        else:
            try:
                # Corrections are data, never a second TF authority. The
                # navigation/planning frame is the continuous odometry frame.
                local_navigation = np.eye(4)
                # Advertise the newest product the MOLA worker has published,
                # paired with the frame state of the revision it was built
                # from; a revision without a product is never advertised.
                product = read_published_product(self.root, memo=self._product_memo)
                selected = product_authority_key(self.core, product)
                if selected is not None:
                    artifact, frame = selected
                    authority = build_authority(
                        artifact,
                        frame,
                        robot_id=self.robot,
                        mission_id=self.core.mission_id,
                        participants=list(self.core.robot_names.values()),
                        robot_map_epoch=self.core.map_epoch,
                        run_id=self.core.run_id,
                        navigation_frame=self.odom_frame,
                        planning_frame=self.odom_frame,
                        T_local_navigation=local_navigation,
                        home_keyframe_id=self.core.key(0).stable_id,
                        peer_slam={
                            "robot_id": self.robot,
                            "mission_id": self.core.mission_id,
                            "robot_map_epoch": self.core.map_epoch,
                            "run_id": self.core.run_id,
                            "keyframes": len(self.core.local_poses),
                            "verified": self.verified_closures,
                            "rejected": self.rejected_closures,
                            "by_peer": dict(self.closures_by_peer),
                        },
                    )
                    if source_reset_stamp:
                        authority["source_reset_stamp"] = source_reset_stamp
                    built_authority = authority
                elif product is not None:
                    self.authority_skipped += 1
                    gap_reason = "worker product lags the current graph revision"
                else:
                    gap_reason = "no product published yet"
            except TransformException:
                gap_reason = "transform lookup failed"
        now = time.monotonic()
        message, self.authority_cache = authority_heartbeat(
            built_authority,
            self.authority_cache,
            robot_id=self.robot,
            mission_id=self.core.mission_id,
            robot_map_epoch=self.core.map_epoch,
            run_id=self.core.run_id,
            cache_age_s=(
                0.0
                if self.authority_fresh_at is None
                else now - self.authority_fresh_at
            ),
        )
        if built_authority is not None and message is built_authority:
            self.authority_fresh_at = now
        elif built_authority is not None:
            gap_reason = "built authority is not serializable"
        if self.authority_cache is None:
            self.authority_fresh_at = None
        self.authority_revision = message.get("mapping_graph_revision")
        self.authority_gap_reason = gap_reason or ""
        if gap_reason is not None:
            self.authority_gap_ticks += 1
        # Only the cached message changes here; `authority_heartbeat_thread`
        # sends it on its own schedule, off this (possibly busy) main
        # executor, re-validating the durable epoch before every send
        # (`_send_authority`). `authority_gap_reason`/`authority_gap_ticks`
        # above are this tick's own build-failure diagnostic; the heartbeat
        # logs an *actual* send gap on its own terms.
        self._authority_heartbeat.update(message, fresh_at=self.authority_fresh_at)
        # The MOLA worker's last build attempt for this peer. Its product stays
        # at the last revision that fit once the component outgrows the point
        # budget; the failure is only visible here and in the worker's log.
        worker_status = read_worker_status(self.root)
        status = {
            "robot_id": self.robot,
            "mission_id": self.core.mission_id,
            "robot_map_epoch": self.core.map_epoch,
            "run_id": self.core.run_id,
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
            # Peer-body mask. `points_dropped` counts removed endpoints, which
            # were never converted into free space; `peers_skipped_stale`
            # counts peer/capture pairs left unmasked for want of a pose close
            # enough to the capture stamp.
            "peer_body_mask": self.peer_mask is not None,
            "peer_body_mask_margin_m": self.peer_mask_margin_m,
            "peer_body_mask_pose_tolerance_s": self.peer_mask_tolerance_s,
            "peer_body_mask_captures_held": self.captures_held_unmaskable,
            "parked_scans_skipped": self.captures_skipped_parked,
            **(
                idle_mask_counters()
                if self.peer_mask is None
                else self.peer_mask.counters()
            ),
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
            # adopted corrections rather than native optimizer messages.
            # Accepted results split into adopted (`solutions`), deferred
            # (moved a pose beyond the change tolerance but held by the
            # adoption interval) and unchanged.
            "solutions": self.solution_count,
            "solution_results_received": self.solution_results_received,
            "solution_results_accepted": self.solution_results_accepted,
            "solution_results_unchanged": self.solution_results_unchanged,
            "solution_results_deferred": self.solution_results_deferred,
            # The newest accepted result held back by the interval: how far
            # the adopted frame lags the solver, and by which order.
            "deferred_solution": (
                None
                if self.core.deferred_solution is None
                else {
                    "order": list(self.core.deferred_solution.order),
                    "translation_m": self.core.deferred_solution.translation_m,
                    "rotation_deg": math.degrees(
                        self.core.deferred_solution.rotation_rad
                    ),
                }
            ),
            "inter_robot_closure_candidates": self.closure_candidates,
            "inter_robot_closures_verified": self.verified_closures,
            "inter_robot_closures_rejected": self.rejected_closures,
            "inter_robot_closures_by_peer": dict(sorted(self.closures_by_peer.items())),
            "corrections_applied": self.solution_count,
            # The adopted frame's order, and the newest solver clock seen;
            # they differ after a result that was deferred or moved nothing.
            "last_solution_order": list(self.core.solution_order),
            "last_solver_order": list(self.core.solver_order),
            # `revision` remains the replica publication sequence so it stays
            # comparable with `replicated_revision`. Map consumers use the
            # independent graph revision carried by the snapshot authority.
            "revision": self.core.replica_revision,
            "mapping_graph_revision": self.core.revision,
            # The product-gated authority: the product revision advertised on
            # this tick, how far the worker lags the graph, and the ticks on
            # which a product existed but no frame state could be paired.
            "authority_revision": self.authority_revision,
            "product_lag_revisions": (
                None
                if self.authority_revision is None
                else self.core.revision - self.authority_revision
            ),
            "authority_skipped": self.authority_skipped,
            # Heartbeat gaps: ticks that could not build a fresh authority
            # (re-sent the last good one, or reported resetting when there
            # was none) and why the most recent one happened. The headline
            # metric for the heartbeat fix is MGG never seeing the map go
            # unavailable despite this counting above zero.
            "authority_gap_ticks": self.authority_gap_ticks,
            "authority_gap_reason": self.authority_gap_reason,
            # The worker's last build outcome (`mola/worker.json`): null until
            # a worker has reported, "" after a published build, otherwise the
            # failure, such as the point budget (`manifest exceeds point count
            # limit of 2000000 points`), with `product_lag_revisions` then
            # growing while the last product that fit stays in service.
            "product_error": None if worker_status is None else worker_status.error,
            "replicated_revision": self.acked_revision,
            "replication_error": self.replica_error,
            "dropped_pairs": self.dropped,
            "peer_domain_id": self.peer_domain_id,
            "sensor_domain_id": self.sensor_domain_id,
            "sensor_error": self.sensor_error,
        }
        self._last_status_json = write_text_if_changed(
            self.status_file,
            json.dumps(status),
            self._last_status_json,
            replace=self._replace_if_current,
        )

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
