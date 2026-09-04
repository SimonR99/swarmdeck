#!/usr/bin/env python3
"""ROS 2 side of the ARGoS bridge.

Binds the Unix socket that `argos/loop_functions/swarmdeck_bridge_loop_functions.cpp`
dials, turns each observation into ROS messages, and sends back the commands
collected from `cmd_vel` and the reset services. The wire format is documented
at the top of that file; this module and it must be changed together, which is
what the "SDB2" magic is for.

    ros2 run swarmdeck_sim swarmdeck_argos_bridge.py --socket /run/swarmdeck/argos.sock

Three things here are load-bearing and easy to get wrong.

**Frame names.** They are the ones `swarmdeck_slam/launch/slam.launch.py`
already publishes static transforms for: `<ns>/base_link/lidar`,
`<ns>/base_link/imu`, `<ns>/base_link/camera`. Inventing `<ns>/lidar_link`
instead does not fail: the messages publish, SLAM subscribes, and every scan is
silently dropped by the TF message filter, which reports only that its queue is
full.

**This node owns `odom -> base_link`.** On the Gazebo backend an EKF fused
wheel odometry with the gyro and published it. Here the pose arrives already
fused, from Ultra-Fusion, so there is no filter and this is the only publisher.
Adding a second one gives a TF tree that flickers between two estimates, which
is worse than either.

**A stale cmd_vel expires.** The ARGoS controller latches whatever velocity it
was last given, and this bridge sends the newest `cmd_vel` it has on every
exchange, so without a deadman a robot whose publisher stops keeps driving on
its last command indefinitely. That is not hypothetical: exploration yields
`cmd_vel` to Nav2 per robot and deliberately says nothing while it does, and a
finished goal or a crashed node leaves the same silence. Measured before the
timeout existed: one `cmd_vel` of 0.25 m/s, no further publication, and the
robot was still doing 0.25 m/s forty seconds later with its mode reading
`idle`. The hardware adapters carry the same protection as `drive_timeout_s`.

**Odometry is not ground truth.** `/<ns>/odom` carries the estimator's drifting
pose and `/<ns>/ground_truth` carries the simulator's, on separate topics, and
nothing but evaluation tooling should read the second. An earlier version of
this file assigned the ground-truth pose into the odometry message, which made
`swarmdeck-slam`'s whole job trivial and its results meaningless.
"""

from __future__ import annotations

import argparse
import math
import os
import socket
import struct
import sys
import threading
import time
from typing import Optional

import numpy as np
try:
    import rclpy
    from geometry_msgs.msg import Point, Quaternion, TransformStamped, Twist, Vector3
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import (
        CameraInfo,
        Image,
        Imu,
        LaserScan,
        PointCloud2,
        PointField,
    )
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage
except ImportError:
    rclpy = None
    Node = object
    PointField = None

try:
    from robot_localization.srv import SetPose
except ImportError:  # pragma: no cover - only when robot_localization is absent
    SetPose = None

# How long a cmd_vel stays valid, in SIMULATION seconds.
#
# Simulation time rather than wall clock because it is the robot's own clock,
# and because the publishers do not share a wall-clock rate: Nav2 runs on
# `use_sim_time` at roughly 20 Hz of SIM time whatever the real-time factor is,
# while explore.py runs on `time.monotonic()` and so publishes MORE often in sim
# time as the simulation slows. Nav2 is therefore the binding constraint at
# ~0.05 s, and this leaves it ten missed control cycles before intervening.
CMD_VEL_TIMEOUT_SIM_S = 0.5

# Diagnostic only. See the encoder block in _read_robot().
DEBUG_ENCODERS = os.environ.get("SWARMDECK_DEBUG_ENCODERS", "") == "1"

OBSERVATION_MAGIC = b"SDB2"
COMMAND_MAGIC = b"SDCMD"

# range f32, x f32, y f32, z f32, ring u16, hit u8. Written field by field on
# the C++ side, so there is no padding and '<' formats line up exactly.
LIDAR_READING = struct.Struct("<ffffHB")
LIDAR_DTYPE = np.dtype([
    ("range", "<f4"),
    ("x", "<f4"),
    ("y", "<f4"),
    ("z", "<f4"),
    ("ring", "<u2"),
    ("hit", "u1"),
])

if PointField is not None:
    LIDAR_POINT_FIELDS = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
else:
    LIDAR_POINT_FIELDS = []

SCAN_BEAMS = 360
SCAN_ANGLE_MIN = -3.14159
SCAN_ANGLE_MAX = 3.14159
SCAN_ANGLE_INC = 0.0174533
INV_ANGLE_INC = 1.0 / SCAN_ANGLE_INC
SCAN_RANGE_MIN = 0.45
SCAN_TIME = 0.1

PROX_MIN_HEIGHT = 0.15
PROX_MAX_HEIGHT = 1.80

ROBOT_SPECS = {
    "bunker": {
        "lidar_x": -0.07,
        "lidar_z": 0.402,
        "base_height": 0.138,
        "prox_range_max": 8.0,
    },
    "scout_mini": {
        "lidar_x": 0.0,
        "lidar_z": 0.280,
        "base_height": 0.100,
        "prox_range_max": 8.0,
    },
    "spot": {
        "lidar_x": -0.180,
        "lidar_z": 0.470,
        "base_height": 0.500,
        "prox_range_max": 8.0,
    },
}


def project_laserscan_slice(
    hit_pts: np.ndarray,
    range_max: float = 30.0,
) -> np.ndarray:
    """Derive a 2D planar slice in the sensor frame for SLAM Toolbox.

    Selects the horizontal ring band [-0.05, 0.05] m in sensor frame.
    """
    ranges = np.full(SCAN_BEAMS, np.inf, dtype=np.float32)
    if hit_pts.size == 0:
        return ranges

    hz = hit_pts["z"]
    slice_mask = (hz >= -0.05) & (hz <= 0.05)
    if not np.any(slice_mask):
        return ranges

    sx = hit_pts["x"][slice_mask]
    sy = hit_pts["y"][slice_mask]
    sr = np.hypot(sx, sy)
    valid = (sr >= SCAN_RANGE_MIN) & (sr <= range_max)
    if not np.any(valid):
        return ranges

    sr = sr[valid]
    stheta = np.arctan2(sy[valid], sx[valid])
    sbins = np.clip(
        np.floor((stheta - SCAN_ANGLE_MIN) * INV_ANGLE_INC).astype(np.int32),
        0,
        SCAN_BEAMS - 1,
    )
    np.minimum.at(ranges, sbins, sr)
    return ranges


def project_laserscan_proximity(
    hit_pts: np.ndarray,
    lidar_x: float,
    lidar_z: float,
    base_height: float,
    prox_range_max: float = 8.0,
) -> np.ndarray:
    """Derive a 2.5D obstacle projection in base_link frame for Nav2 costmaps.

    Projects obstacles in the physical height band [0.15, 1.80] m above ground.
    """
    ranges = np.full(SCAN_BEAMS, np.inf, dtype=np.float32)
    if hit_pts.size == 0:
        return ranges

    hx = hit_pts["x"]
    hy = hit_pts["y"]
    hz = hit_pts["z"]

    px = hx + lidar_x
    py = hy
    pz_floor = hz + lidar_z + base_height

    prox_mask = (pz_floor >= PROX_MIN_HEIGHT) & (pz_floor <= PROX_MAX_HEIGHT)
    if not np.any(prox_mask):
        return ranges

    px = px[prox_mask]
    py = py[prox_mask]
    pr = np.hypot(px, py)
    valid = (pr >= SCAN_RANGE_MIN) & (pr <= prox_range_max)
    if not np.any(valid):
        return ranges

    pr = pr[valid]
    ptheta = np.arctan2(py[valid], px[valid])
    pbins = np.clip(
        np.floor((ptheta - SCAN_ANGLE_MIN) * INV_ANGLE_INC).astype(np.int32),
        0,
        SCAN_BEAMS - 1,
    )
    np.minimum.at(ranges, pbins, pr)
    return ranges


def recv_exact(sock: socket.socket, count: int) -> bytes:
    chunks, got = [], 0
    while got < count:
        chunk = sock.recv(min(1 << 20, count - got))
        if not chunk:
            raise EOFError("ARGoS closed the bridge socket")
        chunks.append(chunk)
        got += len(chunk)
    return b"".join(chunks) if len(chunks) > 1 else chunks[0]


class RobotInterface:
    """Publishers, subscribers and pending commands for one simulated robot."""

    def __init__(self, node: Node, robot_id: str):
        self.node = node
        self.id = robot_id
        self.cmd_vel = (0.0, 0.0)
        # Bumped on arrival and compared at send time, which is how the send
        # path learns a command is NEW without needing the simulation clock in
        # a ROS callback thread that does not have it.
        self.cmd_seq = 0
        self._seen_seq = 0
        self._cmd_at_sim: float | None = None
        self._expired = False
        self._encoder_log_at = -1e9
        self.pending_teleport: Optional[tuple] = None

        # RELIABLE for everything this node publishes, sensor streams included.
        #
        # A publisher's reliability has to be at least what a subscriber asks
        # for, and this stack's consumers ask for both kinds: explore.py takes
        # `scan` BEST_EFFORT but `odom` RELIABLE, adapter_sim takes `odom`
        # RELIABLE, and pointcloud_to_laserscan takes `cloud_in` RELIABLE.
        # RELIABLE satisfies all of them; BEST_EFFORT satisfies only the
        # BEST_EFFORT ones, and the rest silently receive NOTHING.
        #
        # This is what ros_gz_bridge offered on the Gazebo backend, which is why
        # the same consumers worked there. Publishing sensor_data QoS here
        # instead cost two failures that both present as "the fleet does not
        # move": no `<ns>/scan` at all (pointcloud_to_laserscan never received a
        # cloud) and an explorer that never saw an odometry message. The only
        # hint is a rclpy warning about an incompatible RELIABILITY policy.
        reliable = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                              reliability=QoSReliabilityPolicy.RELIABLE)
        # TF has to be latched-ish for late joiners the same way tf2_ros does it.
        tf_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=100,
                            reliability=QoSReliabilityPolicy.RELIABLE,
                            durability=QoSDurabilityPolicy.VOLATILE)

        ns = robot_id
        spec = node.robot_specs.get(robot_id, ROBOT_SPECS["bunker"])
        self.lidar_x = float(spec.get("lidar_x", -0.07))
        self.lidar_z = float(spec.get("lidar_z", 0.402))
        self.base_height = float(spec.get("base_height", 0.138))
        self.prox_range_max = float(spec.get("prox_range_max", 8.0))

        self.pub_points = node.create_publisher(
            PointCloud2, f"/{ns}/scan/points", reliable)
        self.pub_scan = node.create_publisher(
            LaserScan, f"/{ns}/scan", reliable)
        self.pub_prox = node.create_publisher(
            LaserScan, f"/{ns}/proximity_scan", reliable)
        self.pub_imu = node.create_publisher(
            Imu, f"/{ns}/imu", reliable)
        self.pub_odom = node.create_publisher(
            Odometry, f"/{ns}/odom", reliable)
        self.pub_truth = node.create_publisher(
            Odometry, f"/{ns}/ground_truth", reliable)
        self.pub_image = node.create_publisher(
            Image, f"/{ns}/camera/image", reliable)
        self.pub_depth = node.create_publisher(
            Image, f"/{ns}/camera/depth_image", reliable)
        self.pub_info = node.create_publisher(
            CameraInfo, f"/{ns}/camera/camera_info", reliable)
        # Namespaced, and remapped to `tf` by every consumer in this stack, so
        # the four robots' trees stay separate.
        self.pub_tf = node.create_publisher(TFMessage, f"/{ns}/tf", tf_qos)

        node.create_subscription(
            Twist, f"/{ns}/cmd_vel", self._on_cmd_vel, reliable)

        if SetPose is not None:
            self.srv_set_pose = node.create_service(
                SetPose, f"/{ns}/set_pose", self._on_set_pose)

        self.frame_base = f"{ns}/base_link"
        self.frame_odom = f"{ns}/odom"
        self.frame_lidar = f"{ns}/base_link/lidar"
        self.frame_imu = f"{ns}/base_link/imu"
        self.frame_camera = f"{ns}/base_link/camera"
        self._warned_invalid = False

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.cmd_vel = (float(msg.linear.x), float(msg.angular.z))
        self.cmd_seq += 1

    def _encoder_log_due(self, sim_now: float) -> bool:
        """Rate-limit the diagnostic to once a second of simulation time."""
        if sim_now - self._encoder_log_at < 1.0:
            return False
        self._encoder_log_at = sim_now
        return True

    def velocity_at(self, sim_now: float) -> tuple[float, float]:
        """The velocity to command, zeroed once the last one has gone stale."""
        if self.cmd_seq != self._seen_seq:
            self._seen_seq = self.cmd_seq
            self._cmd_at_sim = sim_now
            if self._expired:
                self._expired = False
        if self._cmd_at_sim is None:
            return (0.0, 0.0)
        if sim_now - self._cmd_at_sim <= CMD_VEL_TIMEOUT_SIM_S:
            return self.cmd_vel
        if not self._expired and self.cmd_vel != (0.0, 0.0):
            self._expired = True
            # Once per lapse, not once per tick: a robot parked with no
            # publisher would otherwise fill the log forever.
            self.node.get_logger().info(
                f"[{self.id}] no cmd_vel for {CMD_VEL_TIMEOUT_SIM_S}s of "
                f"simulation time; commanding zero"
            )
        return (0.0, 0.0)

    def _on_set_pose(self, request, response):
        """Teleport the robot. Simulation-only, and the reset path's only mover.

        On the Gazebo backend this was a `gz service` call on
        `/world/<name>/set_pose`; there is no such service here, so the request
        rides back to the loop function on the command channel.
        """
        p = request.pose.pose.pose.position
        o = request.pose.pose.pose.orientation
        self.pending_teleport = (p.x, p.y, p.z, o.w, o.x, o.y, o.z)
        self.node.get_logger().info(
            f"[{self.id}] teleport queued to ({p.x:.2f}, {p.y:.2f})")
        return response


class ArgosBridge(Node):
    @staticmethod
    def _load_robot_specs(config_path: str | None) -> dict[str, dict]:
        specs: dict[str, dict] = {}
        if config_path and os.path.exists(config_path):
            try:
                import yaml
                with open(config_path, "r", encoding="utf-8") as f:
                    cfg = yaml.safe_load(f) or {}
                fleet = cfg.get("fleet") or {}
                default_type = fleet.get("robot_type", "bunker")
                overrides = fleet.get("robot_types") or {}
                count = int(fleet.get("robot_count", 4))
                prefix = fleet.get("robot_prefix", "robot_")
                for i in range(count):
                    rid = f"{prefix}{i}"
                    ptype = overrides.get(rid, default_type)
                    base_spec = ROBOT_SPECS.get(ptype, ROBOT_SPECS["bunker"]).copy()
                    specs[rid] = base_spec
            except Exception:
                pass
        return specs

    def __init__(self, socket_path: str, config_path: str | None = None):
        super().__init__("swarmdeck_argos_bridge")
        self.socket_path = socket_path
        self.robot_specs = self._load_robot_specs(config_path)
        self.robots: dict[str, RobotInterface] = {}
        self.running = True
        self.world_reset_pending = False

        self.pub_clock = self.create_publisher(Clock, "/clock", 10)
        self.create_service(Trigger, "/swarmdeck_sim/reset_world",
                            self._on_reset_world)

        parent = os.path.dirname(socket_path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        if os.path.exists(socket_path):
            os.unlink(socket_path)
        self.server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.server.bind(socket_path)
        self.server.listen(1)
        self.get_logger().info(f"listening for ARGoS on {socket_path}")

        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    # -- services ----------------------------------------------------------

    def _on_reset_world(self, request, response):
        self.world_reset_pending = True
        response.success = True
        response.message = "world reset queued"
        self.get_logger().info("world reset queued")
        return response

    # -- socket ------------------------------------------------------------

    def _serve(self) -> None:
        while self.running:
            try:
                client, _ = self.server.accept()
            except OSError:
                if not self.running:
                    return
                time.sleep(0.5)
                continue
            self.get_logger().info("ARGoS connected")
            try:
                self._handle(client)
            except EOFError:
                self.get_logger().info("ARGoS disconnected")
            except Exception as exc:  # noqa: BLE001 - report and re-listen
                self.get_logger().error(f"bridge error: {exc}")
            finally:
                try:
                    client.close()
                except OSError:
                    pass

    def _robot(self, robot_id: str) -> RobotInterface:
        if robot_id not in self.robots:
            self.robots[robot_id] = RobotInterface(self, robot_id)
            self.get_logger().info(f"registered topics for '{robot_id}'")
        return self.robots[robot_id]

    def _handle(self, sock: socket.socket) -> None:
        while self.running:
            magic, tick, ticks_per_second, count = struct.unpack(
                "<4sIII", recv_exact(sock, 16))
            if magic != OBSERVATION_MAGIC:
                raise RuntimeError(
                    f"observation magic {magic!r} is not {OBSERVATION_MAGIC!r}: "
                    f"the ARGoS loop function and this bridge are different "
                    f"protocol versions")

            seconds = tick / float(ticks_per_second or 1)
            stamp = Clock().clock
            stamp.sec = int(seconds)
            stamp.nanosec = int(round((seconds - stamp.sec) * 1e9))
            clock_msg = Clock()
            clock_msg.clock = stamp
            self.pub_clock.publish(clock_msg)

            ids = []
            for _ in range(count):
                ids.append(self._read_robot(sock, stamp, ticks_per_second, seconds))
            self._send_commands(sock, tick, ids, seconds)

    def _read_robot(self, sock, stamp, ticks_per_second, seconds=0.0) -> str:
        robot_id = recv_exact(
            sock, struct.unpack("<B", recv_exact(sock, 1))[0]).decode("utf-8")
        robot = self._robot(robot_id)

        # -- ground truth ---------------------------------------------------
        gt = struct.unpack("<13d", recv_exact(sock, 13 * 8))
        truth = Odometry()
        truth.header.stamp = stamp
        truth.header.frame_id = "world"
        truth.child_frame_id = robot.frame_base
        truth.pose.pose.position = Point(x=gt[0], y=gt[1], z=gt[2])
        truth.pose.pose.orientation = Quaternion(w=gt[3], x=gt[4], y=gt[5],
                                                 z=gt[6])
        truth.twist.twist.linear = Vector3(x=gt[7], y=gt[8], z=gt[9])
        truth.twist.twist.angular = Vector3(x=gt[10], y=gt[11], z=gt[12])
        robot.pub_truth.publish(truth)

        # -- odometry -------------------------------------------------------
        if struct.unpack("<B", recv_exact(sock, 1))[0]:
            valid = struct.unpack("<B", recv_exact(sock, 1))[0]
            odo = struct.unpack("<13d", recv_exact(sock, 13 * 8))
            struct.unpack("<I", recv_exact(sock, 4))  # estimate tick, unused
            if valid:
                odom = Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id = robot.frame_odom
                odom.child_frame_id = robot.frame_base
                odom.pose.pose.position = Point(x=odo[0], y=odo[1], z=odo[2])
                odom.pose.pose.orientation = Quaternion(
                    w=odo[3], x=odo[4], y=odo[5], z=odo[6])
                odom.twist.twist.linear = Vector3(x=odo[7], y=odo[8], z=odo[9])
                odom.twist.twist.angular = Vector3(
                    x=odo[10], y=odo[11], z=odo[12])
                robot.pub_odom.publish(odom)

                transform = TransformStamped()
                transform.header.stamp = stamp
                transform.header.frame_id = robot.frame_odom
                transform.child_frame_id = robot.frame_base
                transform.transform.translation = Vector3(
                    x=odo[0], y=odo[1], z=odo[2])
                transform.transform.rotation = odom.pose.pose.orientation
                robot.pub_tf.publish(TFMessage(transforms=[transform]))
            elif not robot._warned_invalid:
                robot._warned_invalid = True
                # Not an error: an external estimator needs motion and a few
                # seconds of sensor data before it has a pose at all. Publishing
                # a placeholder would put the robot at the origin of its own map.
                self.get_logger().info(
                    f"[{robot_id}] estimator has no pose yet; withholding "
                    f"/{robot_id}/odom and odom->base_link until it converges")

        # -- wheel encoders --------------------------------------------------
        # Normally read and discarded: Ultra-Fusion consumes the encoders inside
        # ARGoS, through the external_estimator medium, and nothing in ROS wants
        # a second dead-reckoned pose to disagree with the fused one.
        #
        # SWARMDECK_DEBUG_ENCODERS=1 logs them instead. They are the only view
        # of what the controller actually commanded, which is what separates a
        # controller fault from a physics one when a robot will not turn.
        if struct.unpack("<B", recv_exact(sock, 1))[0]:
            encoders = struct.unpack("<4d", recv_exact(sock, 4 * 8))
            if DEBUG_ENCODERS and robot._encoder_log_due(seconds):
                left, right = encoders[0], encoders[1]
                self.get_logger().info(
                    f"[{robot_id}] wheels L={left:+.4f} R={right:+.4f} m/s "
                    f"diff={right - left:+.4f}"
                )

        # -- IMU -------------------------------------------------------------
        if struct.unpack("<B", recv_exact(sock, 1))[0]:
            imu_data = struct.unpack("<6d", recv_exact(sock, 6 * 8))
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = robot.frame_imu
            imu.angular_velocity = Vector3(x=imu_data[0], y=imu_data[1],
                                           z=imu_data[2])
            imu.linear_acceleration = Vector3(x=imu_data[3], y=imu_data[4],
                                              z=imu_data[5])
            # No orientation estimate: this is a 6-DOF IMU, and -1 in the first
            # covariance element is how sensor_msgs/Imu says so. Filling it from
            # ground truth would hand a localizer the answer.
            imu.orientation_covariance[0] = -1.0
            robot.pub_imu.publish(imu)

        # -- lidar ------------------------------------------------------------
        if struct.unpack("<B", recv_exact(sock, 1))[0]:
            _scan_tick, _rings, _azimuths, _max_range, readings = struct.unpack(
                "<IIIfI", recv_exact(sock, 20))
            raw = recv_exact(sock, readings * LIDAR_READING.size)
            arr = np.frombuffer(raw, dtype=LIDAR_DTYPE)
            hit_mask = arr["hit"] != 0
            hits = int(np.count_nonzero(hit_mask))
            hit_pts = arr[hit_mask] if hits else np.empty(0, dtype=LIDAR_DTYPE)

            # 1. PointCloud2 (Fast-LIVO2 & 3D consumers - UNTOUCHED)
            if hits:
                out = np.empty((hits, 4), dtype="<f4")
                out[:, 0] = hit_pts["x"]
                out[:, 1] = hit_pts["y"]
                out[:, 2] = hit_pts["z"]
                out[:, 3] = hit_pts["ring"]

                cloud = PointCloud2()
                cloud.header.stamp = stamp
                cloud.header.frame_id = robot.frame_lidar
                cloud.height = 1
                cloud.width = hits
                # `intensity` carries the laser channel index. A real unit puts
                # return strength there, which this sensor does not model; the
                # ring is what a 3D SLAM front-end actually wants from the
                # fourth field, and it costs nothing to carry.
                cloud.fields = LIDAR_POINT_FIELDS
                cloud.is_bigendian = False
                cloud.point_step = 16
                cloud.row_step = 16 * hits
                cloud.is_dense = True
                cloud.data = out.tobytes()
                robot.pub_points.publish(cloud)

            # 2. Planar LaserScan (SLAM Toolbox: horizontal ring slice in sensor frame)
            scan_ranges = project_laserscan_slice(hit_pts, range_max=float(_max_range))
            scan_msg = LaserScan()
            scan_msg.header.stamp = stamp
            scan_msg.header.frame_id = robot.frame_lidar
            scan_msg.angle_min = float(SCAN_ANGLE_MIN)
            scan_msg.angle_max = float(SCAN_ANGLE_MAX)
            scan_msg.angle_increment = float(SCAN_ANGLE_INC)
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = float(SCAN_TIME)
            scan_msg.range_min = float(SCAN_RANGE_MIN)
            scan_msg.range_max = float(_max_range)
            scan_msg.ranges = scan_ranges.tolist()
            robot.pub_scan.publish(scan_msg)

            # 3. Proximity 2.5D LaserScan (Nav2: 0.15..1.80 m obstacle band in base_link)
            prox_ranges = project_laserscan_proximity(
                hit_pts,
                robot.lidar_x,
                robot.lidar_z,
                robot.base_height,
                prox_range_max=robot.prox_range_max,
            )
            prox_msg = LaserScan()
            prox_msg.header.stamp = stamp
            prox_msg.header.frame_id = robot.frame_base
            prox_msg.angle_min = float(SCAN_ANGLE_MIN)
            prox_msg.angle_max = float(SCAN_ANGLE_MAX)
            prox_msg.angle_increment = float(SCAN_ANGLE_INC)
            prox_msg.time_increment = 0.0
            prox_msg.scan_time = float(SCAN_TIME)
            prox_msg.range_min = float(SCAN_RANGE_MIN)
            prox_msg.range_max = float(robot.prox_range_max)
            prox_msg.ranges = prox_ranges.tolist()
            robot.pub_prox.publish(prox_msg)

        # -- camera ------------------------------------------------------------
        if struct.unpack("<B", recv_exact(sock, 1))[0]:
            _cam_tick, width, height, fov_deg = struct.unpack(
                "<IIIf", recv_exact(sock, 16))
            rgb = recv_exact(sock, width * height * 3)

            image = Image()
            image.header.stamp = stamp
            image.header.frame_id = robot.frame_camera
            image.height, image.width = height, width
            image.encoding = "rgb8"
            image.is_bigendian = False
            image.step = width * 3
            image.data = rgb
            robot.pub_image.publish(image)

            # The sensor reports a VERTICAL field of view, so the focal length
            # comes from the height. Deriving it from the width instead scales
            # every deprojected detection by the aspect ratio, which looks like
            # a calibration error nobody made.
            fov = math.radians(fov_deg if fov_deg > 0 else 60.0)
            fy = height / (2.0 * math.tan(fov / 2.0))
            fx = fy
            cx, cy = width / 2.0, height / 2.0
            info = CameraInfo()
            info.header.stamp = stamp
            info.header.frame_id = robot.frame_camera
            info.height, info.width = height, width
            info.distortion_model = "plumb_bob"
            info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            robot.pub_info.publish(info)

            if struct.unpack("<B", recv_exact(sock, 1))[0]:
                depth = Image()
                depth.header.stamp = stamp
                depth.header.frame_id = robot.frame_camera
                depth.height, depth.width = height, width
                depth.encoding = "32FC1"
                depth.is_bigendian = False
                depth.step = width * 4
                depth.data = recv_exact(sock, width * height * 4)
                robot.pub_depth.publish(depth)

        return robot_id

    @staticmethod
    def _iter_hits(raw: bytes, readings: int):
        arr = np.frombuffer(raw, dtype=LIDAR_DTYPE)
        hit_mask = arr["hit"] != 0
        for pt in arr[hit_mask]:
            yield float(pt["x"]), float(pt["y"]), float(pt["z"]), int(pt["ring"])

    def _send_commands(self, sock: socket.socket, tick: int, ids,
                       sim_now: float) -> None:
        out = bytearray(COMMAND_MAGIC)
        out += struct.pack("<II", tick, len(ids))
        for robot_id in ids:
            robot = self.robots[robot_id]
            name = robot_id.encode("utf-8")
            out += struct.pack("<B", len(name)) + name
            out += struct.pack("<ff", *robot.velocity_at(sim_now))
            teleport = robot.pending_teleport
            if teleport is None:
                out += struct.pack("<B", 0)
            else:
                out += struct.pack("<B", 1)
                out += struct.pack("<3d", *teleport[:3])
                out += struct.pack("<4d", *teleport[3:])
                robot.pending_teleport = None
        out += struct.pack("<B", 1 if self.world_reset_pending else 0)
        self.world_reset_pending = False
        sock.sendall(out)

    def shutdown(self) -> None:
        self.running = False
        try:
            self.server.close()
        except OSError:
            pass
        if os.path.exists(self.socket_path):
            try:
                os.unlink(self.socket_path)
            except OSError:
                pass


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--socket", default="/run/swarmdeck/argos.sock")
    parser.add_argument("--config", default=None, help="Path to session YAML config")
    args, ros_args = parser.parse_known_args(argv if argv is not None
                                             else sys.argv[1:])

    rclpy.init(args=ros_args)
    node = ArgosBridge(socket_path=args.socket, config_path=args.config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
