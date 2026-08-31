"""ROS-independent adapter protocol building blocks.

The ROS 1 and ROS 2 bridges deliberately keep their transport and message
construction in separate files.  Their protocol-facing behaviour, however,
must stay the same: a stale link stops autonomy, image/cloud decoding honours
the message wire format, and dashboard settings are applied in the same way.
This module owns those small runtime policies so a fix does not have to be
copied into both adapters (and so the policies can be tested without ROS).

The mixins only depend on attributes and hooks supplied by a bridge.  They do
not import rospy or rclpy at module import time; this is important for the
offline adapter tests and for deploying either bridge into a minimal image.
"""

from __future__ import annotations

import json
import logging
import math
import time
import urllib.parse
import urllib.request
from typing import Any

import numpy as np

from adapters.network_quality import read_link_quality

_LOG = logging.getLogger(__name__)


def deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge a profile override without losing sibling settings."""
    out = dict(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = value
    return out


def map_cloud_height_limits(band: dict[str, Any] | None) -> tuple[float, float]:
    """Return the cloud filter limits in the registered map frame.

    Older profiles expressed ``min_z``/``max_z`` directly in the map frame.
    Hardware profiles can also provide ``floor_z``; in that form the two
    values are physical heights above the floor, which keeps the requested
    clearance independent of where SLAM placed the map origin.
    """
    band = band or {}
    min_z = float(band.get("min_z", -1e9))
    max_z = float(band.get("max_z", 1e9))
    if "floor_z" in band:
        floor_z = float(band["floor_z"])
        min_z += floor_z
        max_z += floor_z
    return min_z, max_z


def project_occupied_cloud(
    points_xy: np.ndarray,
    *,
    resolution: float = 0.05,
    padding_m: float = 1.0,
    max_cells: int = 8_000_000,
) -> tuple[float, int, int, float, float, np.ndarray] | None:
    """Project XY returns into an occupied-only 2D grid.

    This is intentionally a projection, not a raytracer: an accumulated SLAM
    point cloud contains surface returns but does not retain the sensor origin
    for every point, so cells that are not hit remain UNKNOWN rather than being
    guessed FREE. The tuple is ``(resolution, width, height, origin_x,
    origin_y, cells)`` and ``cells`` is row-major ``int8`` occupancy data.
    """
    points = np.asarray(points_xy)
    if points.ndim != 2 or points.shape[1] < 2:
        return None
    try:
        resolution = float(resolution)
        padding_m = max(0.0, float(padding_m))
        max_cells = int(max_cells)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(resolution) or resolution <= 0.0 or max_cells <= 0:
        return None

    xy = np.asarray(points[:, :2], dtype=np.float64)
    xy = xy[np.isfinite(xy).all(axis=1)]
    if not len(xy):
        return None

    # GridMeta/OccupancyGrid use a lower-left origin and half-open cells. Using
    # the integer lattice here keeps the origin stable at exact resolution
    # boundaries, including for negative SLAM coordinates.
    lattice = np.floor(xy / resolution).astype(np.int64)
    min_cell_x = int(lattice[:, 0].min())
    max_cell_x = int(lattice[:, 0].max())
    min_cell_y = int(lattice[:, 1].min())
    max_cell_y = int(lattice[:, 1].max())
    padding_cells = int(math.ceil(padding_m / resolution))
    min_cell_x -= padding_cells
    max_cell_x += padding_cells
    min_cell_y -= padding_cells
    max_cell_y += padding_cells

    width = max_cell_x - min_cell_x + 1
    height = max_cell_y - min_cell_y + 1
    if width <= 0 or height <= 0 or width * height > max_cells:
        return None

    cells = np.full((height, width), -1, dtype=np.int8)
    gx = lattice[:, 0] - min_cell_x
    gy = lattice[:, 1] - min_cell_y
    cells[gy, gx] = 100
    return (
        resolution,
        width,
        height,
        min_cell_x * resolution,
        min_cell_y * resolution,
        cells,
    )


def yaw_of(q) -> float:
    """Return planar yaw from a ROS quaternion-like object."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y**2 + q.z**2),
    )


def stamp_seconds(header) -> float | None:
    """Read either ROS 1 ``to_sec`` or ROS 2 ``sec/nanosec`` timestamps."""
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    try:
        to_sec = getattr(stamp, "to_sec", None)
        if callable(to_sec):
            value = float(to_sec())
        else:
            value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except (AttributeError, TypeError, ValueError):
        return None
    return value if value > 0.0 and math.isfinite(value) else None


def unique_row_index(keys: np.ndarray) -> np.ndarray:
    """Indices of the first occurrence of each unique row, without axis=0.

    ``np.unique(..., axis=0)`` builds a structured view and lexsorts it, which
    on a 32k-point scan costs 39 ms. Packing the integer voxel keys into one
    linear index and running the ordinary 1-D unique is the same answer in
    9.6 ms (measured on Botman). At the 9 Hz this runs on /registered_scan the
    difference is roughly a third of a core.
    """
    keys = np.asarray(keys)
    if keys.ndim != 2 or keys.shape[0] == 0:
        return np.arange(keys.shape[0])
    k = keys.astype(np.int64, copy=False)
    k = k - k.min(axis=0)
    span = k.max(axis=0) + 1
    # Fall back when the packed index would overflow int64.
    if float(np.prod(span.astype(float))) >= 9.0e18:
        return np.unique(keys, axis=0, return_index=True)[1]
    lin = k[:, 0]
    for c in range(1, k.shape[1]):
        lin = lin * span[c] + k[:, c]
    return np.unique(lin, return_index=True)[1]


def cloud_xyz(msg) -> np.ndarray:
    """Extract finite XYZ points using PointCloud2's declared field offsets.

    ``sensor_msgs.point_cloud2`` is not available in every robot image, and
    assuming x/y/z are the first three fields breaks clouds that include
    intensity or ring data.  This implementation is shared by all adapters.
    """
    offsets = {f.name: f.offset for f in msg.fields if f.name in ("x", "y", "z")}
    if len(offsets) != 3:
        return np.zeros((0, 3), dtype=np.float32)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    count = len(raw) // msg.point_step if msg.point_step else 0
    if not count:
        return np.zeros((0, 3), dtype=np.float32)
    rows = raw[: count * msg.point_step].reshape(count, msg.point_step)
    columns = [
        rows[:, offsets[axis] : offsets[axis] + 4].copy().view(np.float32).ravel()
        for axis in ("x", "y", "z")
    ]
    points = np.stack(columns, axis=1)
    return points[np.isfinite(points).all(axis=1)]


def image_to_bgr(msg):
    """Decode a sensor image while respecting row padding and encoding."""
    try:
        import cv2
    except ImportError:  # pragma: no cover - depends on robot image
        return None
    encoding = str(getattr(msg, "encoding", "")).lower()
    channels = {
        "rgb8": 3,
        "8uc3": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
        "mono8": 1,
    }.get(encoding)
    if channels is None:
        return None
    try:
        step = int(getattr(msg, "step", 0)) or msg.width * channels
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, step)
        frame = rows[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
    except (ValueError, TypeError):
        return None
    if encoding in ("rgb8", "8uc3"):
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    if encoding == "mono8":
        return frame.reshape(msg.height, msg.width)
    return frame


class AdapterSensorMixin:
    """Message decoding and simple sensor callbacks common to ROS bridges."""

    @staticmethod
    def _cloud_xyz(msg) -> np.ndarray:
        return cloud_xyz(msg)

    @staticmethod
    def _image_to_bgr(msg):
        return image_to_bgr(msg)

    @staticmethod
    def _stamp_seconds(header) -> float | None:
        return stamp_seconds(header)

    def _on_odom(self, msg) -> None:
        p = msg.pose.pose
        self._odom_frame = getattr(getattr(msg, "header", None), "frame_id", "")
        self._odom_pose = {
            "x": p.position.x,
            "y": p.position.y,
            "yaw": yaw_of(p.orientation),
        }
        orientation = p.orientation
        self._odom_pose7 = np.array(
            [
                p.position.x,
                p.position.y,
                float(getattr(p.position, "z", 0.0) or 0.0),
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ],
            dtype=np.float64,
        )

    def _on_map(self, msg) -> None:
        self.grid = msg
        self._grid_dirty = True

    @staticmethod
    def _battery_fraction(value: Any, *, whole_percent: bool = False) -> float | None:
        """Normalise a battery percentage reported as either 0..1 or 0..100."""
        try:
            value = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(value) or value < 0.0:
            # sensor_msgs/BatteryState uses -1 for an unknown percentage.
            return None
        if whole_percent or value > 1.0:
            value /= 100.0
        return max(0.0, min(1.0, value))

    def _on_battery(self, msg) -> None:
        # Spot publishes one BatteryState per pack inside a BatteryStateArray.
        # The dashboard has one gauge, so use the lowest valid pack level: it
        # is the safe whole-robot value when packs are not perfectly balanced.
        if hasattr(msg, "battery_states"):
            levels = []
            for state in (getattr(msg, "battery_states", None) or []):
                raw = getattr(
                    state,
                    "charge_percentage",
                    getattr(state, "percentage", None),
                )
                # Spot's custom `charge_percentage` is explicitly 0..100,
                # unlike sensor_msgs/BatteryState's 0..1 `percentage`.
                level = self._battery_fraction(raw, whole_percent=True)
                if level is not None:
                    levels.append(level)
            self.battery = min(levels) if levels else None
            return

        if hasattr(msg, "percentage"):
            self.battery = self._battery_fraction(msg.percentage)
            return

        if hasattr(msg, "battery_voltage") or hasattr(msg, "voltage"):
            try:
                voltage = float(
                    getattr(msg, "battery_voltage", getattr(msg, "voltage", float("nan")))
                )
            except (TypeError, ValueError):
                voltage = float("nan")
            if not math.isfinite(voltage) or voltage <= 0.0:
                self.battery = None
                return
            min_v = float(self.cfg.get("battery_voltage_min", 23.0))
            max_v = float(self.cfg.get("battery_voltage_max", 29.2))
            if max_v > min_v:
                pct = (voltage - min_v) / (max_v - min_v)
                self.battery = max(0.0, min(1.0, pct))
            else:
                self.battery = None

    def _on_camera_depth_cloud(self, msg) -> None:
        self._camera_depth_cloud = msg

    def _on_camera_depth(self, msg) -> None:
        self._camera_depth_image = msg

    def _on_camera_info(self, msg) -> None:
        self._camera_info = msg

    def _on_camera_color_info(self, msg) -> None:
        self._camera_color_info = msg


class AdapterDetectionMixin:
    """Detector scheduling, batching, and dashboard settings synchronisation."""

    _TRACK_IDS = None

    def _detection_due(self) -> bool:
        if not self._detection_enabled or self._detector is None:
            return False
        now = time.monotonic()
        if now - self._last_detection_at < self._detection_period_s:
            return False
        self._last_detection_at = now
        return True

    def _detect_bgr(
        self,
        frame: np.ndarray,
        *,
        due_checked: bool = False,
        image_header=None,
    ) -> None:
        if not due_checked and not self._detection_due():
            return
        tracker = self._TRACK_IDS
        if tracker is None or self._detector is None:
            return
        from adapters.perception.object_detector import crop_detection_jpeg_base64

        detections = []
        for detection, track_id in tracker(self._detector.detect_bgr(frame)):
            item = detection.as_protocol(track_id)
            item["map_position"] = self._depth_map_position(
                detection.bbox, image_header, detection.polygon
            )
            item["image"] = crop_detection_jpeg_base64(frame, detection.bbox)
            detections.append(item)
        self._detections = detections

    def take_detections(self) -> list[dict] | None:
        current = self._detections
        self._detections = None
        return current

    def refresh_settings(self) -> None:
        """Apply dashboard perception controls without coupling them to ROS."""
        try:
            with urllib.request.urlopen(
                f"{self.http_url}/api/settings", timeout=2
            ) as response:
                payload = json.loads(response.read())
            settings = payload.get("settings", {})
            enabled = bool(settings.get("detection_enabled", True))
            if self._detection_enabled and not enabled:
                self._detections = []
            self._detection_enabled = enabled
            if self._detector is not None:
                self._detector.sensitivity = max(
                    0.1,
                    min(1.0, float(settings.get("detection_sensitivity", 0.55))),
                )
                self._detector.classes = settings.get("detection_classes")
                self._detector.class_floors = settings.get(
                    "detection_capture_floors"
                ) or settings.get("detection_class_floors")
        except Exception as exc:
            self._log_warning(f"[{self.id}] settings refresh failed: {exc}")


class AdapterLinkMixin:
    """Deadman and websocket freshness policy shared by hardware bridges."""

    def _on_nav_cmd_vel(self, msg) -> None:
        if self.nav_status == "active" and self.pub_cmd is not None:
            if self.link_ok():
                self.pub_cmd.publish(msg)

    def note_drive_command(self, linear: float, angular: float) -> None:
        self._pending_drive = (float(linear), float(angular))
        self._last_link_at = time.monotonic()

    def apply_pending_drive(self) -> None:
        pending = self._pending_drive
        if pending is None:
            return
        self._pending_drive = None
        self.drive(*pending)

    def _watchdogs(self) -> None:
        self.apply_pending_drive()
        self.drive_watchdog()
        self.link_watchdog()

    def drive_watchdog(self) -> None:
        if self.mode != "teleop" or self._last_drive_at == 0.0:
            return
        if time.monotonic() - self._last_drive_at > self.cfg["drive_timeout_s"]:
            self.drive(0.0, 0.0)
            self.mode = "idle"
            self._last_drive_at = 0.0

    def link_ok(self) -> bool:
        return time.monotonic() - self._last_link_at <= float(
            self.cfg["link_timeout_s"]
        )

    def note_link_activity(self) -> None:
        self._last_link_at = time.monotonic()

    def link_watchdog(self) -> None:
        if self.nav_status != "active" or self.link_ok():
            return
        self._log_warning(
            f"[{self.id}] operator link stale > {self.cfg['link_timeout_s']}s "
            "with a goal active; cancelling and stopping"
        )
        self.cancel_goal()
        self.drive(0.0, 0.0)

    def stop(self) -> None:
        self.drive(0.0, 0.0)
        self.cancel_goal()
        self.mode = "estop"

    def stop_for_exit(self) -> None:
        for _ in range(3):
            try:
                self.cancel_goal()
                self.drive(0.0, 0.0)
            except Exception:
                pass
            time.sleep(0.05)


class AdapterTelemetryMixin:
    """Protocol state envelope shared by ROS 1 and ROS 2 adapters."""

    def _network_quality(self, iface: str):
        host = getattr(self, "_server_host", None)
        port = getattr(self, "_server_port", None)
        if not host and getattr(self, "http_url", None):
            try:
                parsed = urllib.parse.urlparse(self.http_url)
                host = parsed.hostname
                port = parsed.port
            except Exception:
                pass
        try:
            return read_link_quality(iface, host=host, port=port)
        except TypeError:
            return read_link_quality(iface)

    def state(self) -> dict[str, Any]:
        planned_path = list(getattr(self, "planned_path", []) or [])
        global_planned_path = getattr(self, "global_planned_path", None)
        if global_planned_path is None:
            # ROS 2 Nav2 keeps these caches private for now; accept that name
            # here so the shared telemetry envelope remains adapter-agnostic.
            global_planned_path = getattr(self, "_global_planned_path", None)
        local_planned_path = getattr(self, "local_planned_path", None)
        if local_planned_path is None:
            local_planned_path = getattr(self, "_local_planned_path", None)
        # Older adapters only know the single `planned_path` field. Treat that
        # route as global so it remains visible after the split is introduced.
        if global_planned_path is None and local_planned_path is None:
            global_planned_path = planned_path
        state = {
            "type": "robot_state",
            "robot_id": self.id,
            "t_mono": round(time.monotonic() - self.t0, 4),
            "pose": self.map_pose(),
            "battery": self.battery,
            "mode": self.mode,
            "nav_status": self.nav_status,
            "goal": self.goal,
            # Backward-compatible effective route: local when available,
            # otherwise global. The two explicit fields below let the UI show
            # both routes at once for planners that expose both.
            "planned_path": planned_path,
            "global_planned_path": list(global_planned_path or []),
            "local_planned_path": list(local_planned_path or []),
        }
        network_iface = str(self.cfg.get("network_iface", ""))
        if network_iface:
            state["network"] = self._network_quality(network_iface)
        return state


def log_warning(bridge, message: str) -> None:
    """Log through the active ROS node, with an offline-test fallback."""
    node = getattr(bridge, "node", None)
    if node is not None:
        try:
            node.get_logger().warn(message)
            return
        except Exception:
            pass
    try:
        import rospy

        rospy.logwarn(message)
        return
    except Exception:
        _LOG.warning(message)


# Keep the mixins free of ROS imports while giving them one logging hook.
AdapterDetectionMixin._log_warning = log_warning
AdapterLinkMixin._log_warning = log_warning
