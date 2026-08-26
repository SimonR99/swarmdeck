"""ROS-free keyframe production: voxel, motion-gate, encode, drop-queue.

Adapters stream keyframes instead of waiting for the server to register grids.
The upload MUST drop rather than block -- a synchronous POST on this path is
what stalled telemetry and knocked the live fleet offline
(``adapters/adapter_ros2/config/bunker.yaml``).

Points are expected in the robot's **map** frame (the same registered cloud
already used for ``/api/adapter/scan``). They are transformed into the base
frame at capture before encoding, which is the wire contract.
"""

from __future__ import annotations

import math
import os
import threading
import time
import urllib.error
import urllib.request
import uuid
from collections import deque
from typing import Any

import numpy as np

try:
    from swarmdeck_protocol import ProtocolError, encode_keyframe
except ImportError:
    import sys
    from pathlib import Path

    _proto_dir = Path(__file__).resolve().parent / "protocol"
    if _proto_dir.exists() and str(_proto_dir) not in sys.path:
        sys.path.insert(0, str(_proto_dir))
    _root_dir = Path(__file__).resolve().parent.parent
    if _root_dir.exists() and str(_root_dir) not in sys.path:
        sys.path.insert(0, str(_root_dir))
    from swarmdeck_protocol import ProtocolError, encode_keyframe

# Matches the occupancy grid's 0.05 m cells. At the previous 0.2 m the cloud
# arrived FOUR TIMES coarser than the cells it was rasterized into, so a wall
# could not render sharper than 0.2 m however good the poses were -- which is
# most of why the optimized single-robot map looked worse than SLAM Toolbox's,
# and none of it was the optimizer's fault.
#
# This is a RENDERING budget, not a registration one: verify.py downsamples both
# clouds to 0.15 m before GICP, so loop closure sees no extra detail and costs
# no extra time. What it does cost is wire bytes and memory per keyframe.
#
# On hardware, weigh that against the link: an OS1-128 voxelized at 0.05 m is a
# far larger cloud than this simulated 33-ring unit, and botman's single NIC
# carries the lidar VLAN as well. Override per robot rather than editing this.
DEFAULT_VOXEL_M = float(os.environ.get("SWARMDECK_KEYFRAME_VOXEL_M", "0.05"))
DEFAULT_MIN_TRANSLATION_M = 0.5
DEFAULT_MIN_YAW_RAD = math.radians(15.0)
DEFAULT_MIN_PERIOD_S = 2.0
DEFAULT_QUEUE = 2
DEFAULT_TIMEOUT_S = 2.0
DEFAULT_MIN_POINTS = 50
# Reject a capture taken while turning faster than this.
#
# A spinning lidar sweeps over a finite time. Rotate fast enough during one
# revolution and the returns are stitched across a moving pose, so the cloud is
# the wrong SHAPE rather than merely noisy -- and a wrong shape is the dangerous
# kind of error, because registration fits it confidently. Measured on
# sessions/captures/3d-run-01: keyframes above ~8 deg/s produced loop closures
# that passed every geometric gate (20 deg yaw deviation, 85% inlier ratio,
# per-block conditioning) untouched, while the pose graph they fed turned a
# 3.65 deg front-end yaw error into 7.38 deg. Dropping them inverted that --
# the same graph then IMPROVED the trajectory, to 3.26 deg.
#
# 8 deg/s is where the effect appears; 12 deg/s changed nothing measurable.
#
# Sim-specific severity, general mechanism: Gazebo publishes no per-point
# timestamps, so nothing downstream can de-skew. Real Ouster hardware does stamp
# points, and a lidar-inertial front end de-skews with them -- so on the robots
# this gate is a cheap safeguard rather than the only defence, and it can be
# loosened once de-skewing is actually in the path.
DEFAULT_MAX_YAW_RATE = math.radians(
    float(os.environ.get("SWARMDECK_KEYFRAME_MAX_YAW_RATE_DEG", "8.0"))
)


def se3_from_quat_xyz(pose7: np.ndarray) -> np.ndarray:
    """``T`` from ``[x, y, z, qx, qy, qz, qw]``. Duplicated from slam/types
    because adapters cannot import gtsam-pinned code."""
    values = np.asarray(pose7, dtype=np.float64).reshape(-1)
    if values.shape != (7,):
        raise ValueError(f"expected 7 values, got {values.shape}")
    translation, quat = values[:3], values[3:]
    norm = float(np.linalg.norm(quat))
    if norm < 1e-9:
        raise ValueError("quaternion has zero norm")
    x, y, z, w = quat / norm
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    matrix[:3, 3] = translation
    return matrix


def points_map_to_base(points_map: np.ndarray, t_map_base: np.ndarray) -> np.ndarray:
    """Apply ``T_base_map`` so the cloud lives in the base frame at capture."""
    matrix = se3_from_quat_xyz(t_map_base)
    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    pts = np.asarray(points_map, dtype=np.float64)
    return ((pts - translation) @ rotation).astype(np.float32)


def voxel_downsample(points: np.ndarray, voxel_m: float) -> np.ndarray:
    if points.shape[0] == 0:
        return points
    keys = np.round(np.asarray(points, dtype=np.float64) / voxel_m).astype(np.int32)
    _, keep = np.unique(keys, axis=0, return_index=True)
    return np.asarray(points)[keep]


def pose7_from_xy_yaw(x: float, y: float, yaw: float, z: float = 0.0) -> np.ndarray:
    return np.array(
        [x, y, z, 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)],
        dtype=np.float64,
    )


def points_lidar_to_map(
    points_lidar: np.ndarray,
    pose_xy_yaw: tuple[float, float, float],
    lidar_x: float = 0.0,
    lidar_z: float = 0.0,
) -> np.ndarray:
    """Lift lidar-frame XYZ into the robot's map frame.

    ``pose_xy_yaw`` is ``T_map_base``. The lidar is assumed yaw-aligned with
    base_link and offset by ``(lidar_x, 0, lidar_z)``, matching the simulated
    mounts and the hardware adapters' static extrinsics.
    """
    pts = np.asarray(points_lidar, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float32)
    x = pts[:, 0] + float(lidar_x)
    y = pts[:, 1]
    z = pts[:, 2] + float(lidar_z)
    px, py, yaw = pose_xy_yaw
    cosine, sine = math.cos(yaw), math.sin(yaw)
    out = np.empty((pts.shape[0], 3), dtype=np.float32)
    out[:, 0] = px + x * cosine - y * sine
    out[:, 1] = py + x * sine + y * cosine
    out[:, 2] = z
    return out


def laser_scan_to_map_points(
    ranges: np.ndarray,
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    pose_xy_yaw: tuple[float, float, float],
    lidar_x: float = 0.0,
    lidar_z: float = 0.0,
    z_layers: tuple[float, ...] = (0.0, 0.12, 0.24),
) -> np.ndarray:
    """Convert a planar LaserScan into a thickened map-frame cloud.

    A single ring has no vertical structure for GICP or Scan Context. Copying
    the hitpoints at a few heights around the lidar mount is the same trick
    the producer tests use, and it is what lets the 2D Gazebo fleet speak the
    same keyframe contract as an Ouster.
    """
    values = np.asarray(ranges, dtype=np.float64).reshape(-1)
    if values.size == 0:
        return np.zeros((0, 3), dtype=np.float32)
    angles = angle_min + np.arange(values.size, dtype=np.float64) * angle_increment
    valid = np.isfinite(values) & (values > range_min) & (values < range_max)
    if not np.any(valid):
        return np.zeros((0, 3), dtype=np.float32)
    r = values[valid]
    a = angles[valid]
    lidar = np.stack([r * np.cos(a), r * np.sin(a), np.zeros(r.shape[0])], axis=1)
    layers = []
    for offset in z_layers:
        layer = lidar.copy()
        layer[:, 2] = float(offset)
        layers.append(
            points_lidar_to_map(layer, pose_xy_yaw, lidar_x=lidar_x, lidar_z=lidar_z)
        )
    return np.vstack(layers)


def _moved(
    previous: np.ndarray, current: np.ndarray, min_t: float, min_yaw: float
) -> bool:
    delta = current[:3] - previous[:3]
    if float(np.linalg.norm(delta)) >= min_t:
        return True

    # Yaw from quaternion (z-axis): atan2(2(wz+xy), 1-2(y^2+z^2)) with ROS order.
    def yaw_of(q: np.ndarray) -> float:
        x, y, z, w = q
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    dyaw = abs(
        (yaw_of(current[3:]) - yaw_of(previous[3:]) + math.pi) % (2 * math.pi) - math.pi
    )
    return dyaw >= min_yaw


def mint_session() -> str:
    """A fresh boot id for one adapter process.

    ``seq`` restarts at zero every time this process does, so on its own it
    cannot identify a keyframe: the back-end keys nodes by ``(robot_id, seq)``
    and drops a repeat as a duplicate. After a reboot that silently discards
    the robot's first N keyframes, where N is however many it had sent before
    -- observed live as aslan_0 losing ~97 m of driving with an empty
    ``last_error`` throughout.

    Minting this ONCE, at construction, is the whole contract: every packet
    from one run of this process carries the same value, so
    ``(robot_id, session, seq)`` is unique and ``(robot_id, session)`` names
    one continuous trajectory in one map frame. Re-minting per keyframe would
    be worse than not having it -- every keyframe would become its own
    trajectory and no odometry edge would ever be built.

    Wall-clock seconds first so the ids sort chronologically in an operator's
    trajectory list, then random bytes because two robots can boot inside the
    same second (and a robot with no RTC boots at the same fake second every
    time). Only characters ``swarmdeck_protocol`` allows in a session.
    """
    return f"{int(time.time()):010d}-{uuid.uuid4().hex[:8]}"


class KeyframeUploader:
    """Motion-gated producer with a bounded drop-oldest upload queue."""

    def __init__(
        self,
        robot_id: str,
        http_url: str,
        *,
        voxel_m: float = DEFAULT_VOXEL_M,
        min_translation_m: float = DEFAULT_MIN_TRANSLATION_M,
        min_yaw_rad: float = DEFAULT_MIN_YAW_RAD,
        min_period_s: float = DEFAULT_MIN_PERIOD_S,
        queue_size: int = DEFAULT_QUEUE,
        timeout_s: float = DEFAULT_TIMEOUT_S,
        min_points: int = DEFAULT_MIN_POINTS,
        max_yaw_rate: float = DEFAULT_MAX_YAW_RATE,
        session: str | None = None,
    ) -> None:
        self.robot_id = robot_id
        #: Minted once per process. See :func:`mint_session`. Passing one in is
        #: for tests and for a supervisor that wants to name the run itself; it
        #: must never change while this object lives.
        self.session = mint_session() if session is None else session
        self.http_url = http_url.rstrip("/")
        self.voxel_m = voxel_m
        self.min_translation_m = min_translation_m
        self.min_yaw_rad = min_yaw_rad
        self.min_period_s = min_period_s
        self.timeout_s = timeout_s
        self.min_points = min_points
        self.max_yaw_rate = max_yaw_rate
        self._queue: deque[bytes] = deque(maxlen=max(1, queue_size))
        self._lock = threading.Lock()
        self._seq = 0
        self._last_pose: np.ndarray | None = None
        self._last_at = 0.0
        self.dropped = 0
        self.sent = 0
        self.spun = 0
        # Tracked on EVERY consider(), not just accepted keyframes: yaw rate
        # between two keyframes 2 s apart is an average that hides exactly the
        # brief fast turns this gate exists to catch.
        self._prev_yaw: float | None = None
        self._prev_stamp: float | None = None
        self.last_error = ""

    def consider(
        self,
        points_map: np.ndarray,
        t_map_base: np.ndarray,
        stamp: float,
    ) -> bool:
        """Non-blocking. Returns True if a keyframe was enqueued."""
        pose = np.asarray(t_map_base, dtype=np.float64).reshape(-1)
        if pose.shape != (7,) or not np.isfinite(pose).all():
            return False
        if self._turning_too_fast(pose, float(stamp)):
            self.spun += 1
            return False
        now = time.monotonic()
        if self._last_pose is not None:
            if now - self._last_at < self.min_period_s:
                return False
            if not _moved(
                self._last_pose, pose, self.min_translation_m, self.min_yaw_rad
            ):
                return False
        pts = np.asarray(points_map)
        if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] < self.min_points:
            return False
        try:
            base_points = voxel_downsample(points_map_to_base(pts, pose), self.voxel_m)
        except ValueError:
            return False
        if base_points.shape[0] < self.min_points:
            return False
        with self._lock:
            seq = self._seq
            self._seq += 1
        try:
            blob = encode_keyframe(
                robot_id=self.robot_id,
                seq=seq,
                stamp=float(stamp),
                points=base_points,
                t_odom_base=pose,
                session=self.session,
            )
        except ProtocolError:
            return False
        with self._lock:
            if (
                self._queue.maxlen is not None
                and len(self._queue) >= self._queue.maxlen
            ):
                self.dropped += 1
            self._queue.append(blob)
            self._last_pose = pose
            self._last_at = now
        return True

    def _turning_too_fast(self, pose: np.ndarray, stamp: float) -> bool:
        """Yaw rate since the previous scan, against ``max_yaw_rate``.

        Always records the current sample before returning, so the estimate
        stays anchored to the most recent scan even when a capture is rejected;
        otherwise a run of fast frames would be compared against an ever more
        stale reference and the rate would read low exactly when it is highest.
        """
        yaw = math.atan2(
            2.0 * (pose[6] * pose[5] + pose[3] * pose[4]),
            1.0 - 2.0 * (pose[4] * pose[4] + pose[5] * pose[5]),
        )
        previous_yaw, previous_stamp = self._prev_yaw, self._prev_stamp
        self._prev_yaw, self._prev_stamp = yaw, stamp
        if self.max_yaw_rate <= 0.0 or previous_yaw is None or previous_stamp is None:
            return False
        dt = stamp - previous_stamp
        if dt <= 1e-3:
            return False  # duplicate or out-of-order stamp: no usable rate
        delta = abs((yaw - previous_yaw + math.pi) % (2 * math.pi) - math.pi)
        return (delta / dt) > self.max_yaw_rate

    def upload_one(self) -> bool:
        """Blocking POST of at most one queued blob. Safe for an executor."""
        with self._lock:
            if not self._queue:
                return False
            blob = self._queue.popleft()
        url = f"{self.http_url}/api/adapter/keyframe?robot_id={self.robot_id}"
        try:
            urllib.request.urlopen(
                urllib.request.Request(
                    url,
                    data=blob,
                    headers={"Content-Type": "application/octet-stream"},
                ),
                timeout=self.timeout_s,
            ).read()
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            self.last_error = str(exc)
            return False
        self.sent += 1
        self.last_error = ""
        return True

    def pending(self) -> int:
        with self._lock:
            return len(self._queue)
