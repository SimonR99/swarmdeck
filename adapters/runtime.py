"""ROS-independent adapter protocol building blocks.

Hardware and simulation keep ROS wiring in separate files. Hello, robot_state,
keepalive, and reconnect live here so those clients cannot drift.

Mixins do not import rospy or rclpy at module import time.
"""

from __future__ import annotations

import asyncio
import json
import logging
import math
import time
import urllib.parse
import urllib.request
from collections.abc import Mapping, Sequence
from contextlib import nullcontext
from typing import Any

import numpy as np

from adapters.network_quality import read_link_quality

_LOG = logging.getLogger(__name__)

# 2 adds optional `slam_graph`. The server still accepts 1 so an old binary
# on a robot is not kicked off a mixed fleet.
PROTOCOL_VERSION = 2

# Ping 2/4 s, not the library 20/20: a dead radio would otherwise look live
# for ~40 s while autonomy keeps running. Tighter than ~6 s flaps a lossy
# link (Botman, 60% loss). Upload 25 s, not 5: scans queue on one server lock
# and 5 s discarded the four-robot fleet.
TRANSPORT_DEFAULTS: dict[str, Any] = {
    "ping_interval_s": 2.0,
    "ping_timeout_s": 4.0,
    "drive_timeout_s": 0.45,
    "link_timeout_s": 1.5,
    "upload_timeout_s": 25.0,
    # Route progress watchdog (adapters/route_progress.py): cancel a FollowPath
    # goal as a no-progress failure when the closest route point has not
    # advanced by route_progress_min_m within route_progress_timeout_s. Nav2's
    # own checker measures displacement and misses a robot rocking on a step.
    # A timeout of 0 disables it.
    "route_progress_timeout_s": 30.0,
    "route_progress_min_m": 0.5,
    "rates": {
        "state_hz": 5.0,
        "camera_period_s": 0.2,
        "settings_period_s": 5.0,
    },
}

RECONNECT_BACKOFF_S = 1.0
RECONNECT_BACKOFF_MAX_S = 30.0

HELLO_FIELDS = (
    "type",
    "protocol",
    "robot_id",
    "robot_type",
    "adapter",
    "ros",
    "coordinate_frame",
    "capabilities",
    "footprint_radius",
    "footprint",
)


def deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge a profile override without losing sibling settings."""
    out = dict(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = value
    return out


def load_yaml_profile(path: str | None, defaults: dict) -> dict:
    """Shallow-copy defaults, then deep-merge an optional YAML profile."""
    cfg = dict(defaults)
    if path:
        import yaml
        from pathlib import Path

        loaded = yaml.safe_load(Path(path).read_text()) or {}
        cfg = deep_merge(cfg, loaded)
    return cfg


def hello_message(
    *,
    robot_id: str,
    robot_type: str,
    adapter: str,
    ros: str,
    capabilities: Sequence[str],
    footprint_radius: float,
    footprint: Any = None,
    coordinate_frame: str = "local",
) -> dict[str, Any]:
    """Registration envelope. Every adapter sends this key set on connect."""
    return {
        "type": "hello",
        "protocol": PROTOCOL_VERSION,
        "robot_id": robot_id,
        "robot_type": robot_type,
        "adapter": adapter,
        "ros": ros,
        "coordinate_frame": ("merged" if coordinate_frame == "merged" else "local"),
        "capabilities": list(capabilities),
        "footprint_radius": float(footprint_radius),
        "footprint": footprint or None,
    }


def detections_message(
    robot_id: str,
    t0: float,
    items: list[dict[str, Any]],
    *,
    now: float | None = None,
    camera: str = "front",
) -> dict[str, Any]:
    """Detections batch, including `t_mono`."""
    stamp = time.monotonic() if now is None else now
    return {
        "type": "detections",
        "robot_id": robot_id,
        "t_mono": round(stamp - t0, 4),
        "camera": camera,
        "items": items,
    }


def websocket_connect_kwargs(cfg: Mapping[str, Any]) -> dict[str, float]:
    """Ping interval/timeout every adapter must pass to `websockets.connect`."""
    return {
        "ping_interval": float(cfg["ping_interval_s"]),
        "ping_timeout": float(cfg["ping_timeout_s"]),
    }


def next_backoff(current: float) -> float:
    return min(float(current) * 2.0, RECONNECT_BACKOFF_MAX_S)


async def run_until_first_failure(*coros: Any) -> None:
    """First failure cancels the rest. They share one socket; a sibling that
    keeps writing after close surfaces later as 'never retrieved'.

    Cancelling a task blocked in ``run_in_executor`` does not stop the worker.
    The in-flight upload finishes unobserved, which is safe.
    """
    tasks = [asyncio.ensure_future(c) for c in coros]
    try:
        done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_EXCEPTION)
        for task in done:
            task.result()
    finally:
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)


class AdapterHelloMixin:
    """`hello()` from `self.cfg`. Subclasses set `adapter_name` and `capabilities()`."""

    adapter_name: str = ""
    coordinate_frame: str = "local"

    def hello(self) -> dict[str, Any]:
        cfg = getattr(self, "cfg", {}) or {}
        return hello_message(
            robot_id=self.id,
            robot_type=str(cfg.get("robot_type", "unknown")),
            adapter=self.adapter_name,
            ros=str(cfg.get("ros_distro", "")),
            capabilities=list(self.capabilities()),
            footprint_radius=float(cfg.get("footprint_radius", 0.3)),
            footprint=cfg.get("footprint"),
            coordinate_frame=self.coordinate_frame,
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
            "z": p.position.z,
            "yaw": yaw_of(p.orientation),
        }

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
            for state in getattr(msg, "battery_states", None) or []:
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
                    getattr(
                        msg, "battery_voltage", getattr(msg, "voltage", float("nan"))
                    )
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
    """Shared sim/hardware velocity gating, deadman and link watchdog policy."""

    def _on_nav_cmd_vel(self, msg) -> None:
        if self.nav_status == "active" and self.pub_cmd is not None:
            if self.link_ok() and not getattr(self, "_nav_route_blocked", False):
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
        self.route_progress_watchdog()

    def route_progress_watchdog(self) -> None:
        """Cancel a FollowPath goal whose progress along the route stalled."""
        if not callable(getattr(self, "_fail_route_progress", None)):
            return  # This bridge has no full-path controller to supervise.
        from adapters.route_progress import route_progress_tick

        try:
            route_progress_tick(self)
        except Exception as exc:
            # A supervisor must never take the ROS timer, and with it the
            # deadman watchdogs, down with it.
            self._log_warning(f"[{self.id}] route progress watchdog failed: {exc}")

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
        with getattr(self, "_goal_lock", nullcontext()):
            if self.nav_status != "active" or self.link_ok():
                return
            self._log_warning(
                f"[{self.id}] operator link stale > {self.cfg['link_timeout_s']}s "
                "with a goal active; cancelling and stopping"
            )
            self.cancel_goal()
            self.drive(0.0, 0.0)
            # A manual zero drive retains each adapter's normal semantics;
            # this watchdog stop is the terminal cancellation of its route.
            self.nav_status = "cancelled"

    def stop(self) -> None:
        exploration = getattr(self, "exploration", None)
        if exploration is not None:
            exploration.stop()
        self.drive(0.0, 0.0)
        self.cancel_goal()
        self.mode = "estop"

    def stop_for_exit(self) -> None:
        exploration = getattr(self, "exploration", None)
        if exploration is not None:
            exploration.stop()
        for _ in range(3):
            try:
                self.cancel_goal()
                self.drive(0.0, 0.0)
            except Exception:
                pass
            time.sleep(0.05)


class AdapterGoalOwnershipMixin:
    """Generation-guarded goal commands used by exploration and objective planning.

    Each call acts only while ``expected_generation`` still owns the bridge's
    goal, so a planner can never cancel or relabel a newer operator command.
    The bridge provides ``_goal_lock``, ``_goal_generation``, ``nav_status``,
    ``_nav_execution_enabled``, ``pub_cmd`` and ``cancel_goal()``. Place this
    mixin before ``AdapterLinkMixin`` to gate its velocity relay on accepted
    execution as well as link freshness.
    """

    def _on_nav_cmd_vel(self, msg) -> None:
        with self._goal_lock:
            if not self._nav_execution_enabled:
                return
            super()._on_nav_cmd_vel(msg)

    def _hold_goal_motion(self) -> None:
        """Close the velocity relay before ownership returns to a planner."""
        self._nav_execution_enabled = False

    def _finish_goal_motion(self) -> None:
        """Replace the last relayed velocity when a terminal result closes the gate.

        The smoother's stop may arrive after the action result and be dropped
        by the closed gate. Do not leave the driver holding its last velocity.
        """
        with self._goal_lock:
            was_executing = self._nav_execution_enabled
            self._hold_goal_motion()
            if was_executing and self.pub_cmd is not None:
                from geometry_msgs.msg import Twist

                zero = Twist()
                zero.linear.x = zero.linear.y = zero.linear.z = 0.0
                zero.angular.x = zero.angular.y = zero.angular.z = 0.0
                self.pub_cmd.publish(zero)

    def cancel_goal_if_current(self, expected_generation: int, *, pending=False):
        """Cancel only the command owning ``expected_generation``."""
        with self._goal_lock:
            if expected_generation != self._goal_generation:
                return None
            self.cancel_goal()
            if pending:
                self.nav_status = "active"
            return self._goal_generation

    def set_nav_status_if_current(self, expected_generation: int, status: str) -> bool:
        with self._goal_lock:
            if expected_generation != self._goal_generation:
                return False
            if status != "active":
                self._hold_goal_motion()
            self.nav_status = status
            return True

    def set_goal_pending_if_current(self, expected_generation: int) -> bool:
        with self._goal_lock:
            if expected_generation != self._goal_generation:
                return False
            self._hold_goal_motion()
            self.nav_status = "active"
            return True


class AdapterTelemetryMixin:
    """Protocol `robot_state` envelope shared by every adapter."""

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

    def navigation_route_target(self, path, pose):
        """Shared opt-in tracker for adapters following supplied route points.

        ROS 1/ROS 2/simulation inherit this API. Native Nav2 controllers retain
        their own path tracking; callers must hold motion when this returns None.
        """
        from adapters.route_tracking import GridCollisionChecker, RouteTracker

        key = tuple((float(p["x"]), float(p["y"])) for p in path)
        if key != getattr(self, "_nav_route_key", None):
            self._nav_route_key = key
            self._nav_route_tracker = RouteTracker(
                path, float(self.cfg.get("nav_route_lookahead_m", 0.8))
            )
        client = getattr(self, "_nav_map", None)
        snapshot = getattr(client, "cached", None)
        checker = None
        if (
            snapshot is not None
            and time.monotonic() - getattr(client, "last_success_at", 0.0) <= 10.0
        ):
            checker = GridCollisionChecker(
                snapshot,
                float(
                    self.cfg.get(
                        "nav_route_clearance_m", self.cfg.get("footprint_radius", 0.35)
                    )
                ),
            )
        result = self._nav_route_tracker.target(pose, checker)
        self._nav_route_blocked = result is None
        return result

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
            "home_pose": getattr(self, "_home_pose", None),
            "battery": self.battery,
            "mode": (
                "explore"
                if getattr(getattr(self, "exploration", None), "active", False)
                else self.mode
            ),
            "exploration_status": getattr(
                getattr(self, "exploration", None), "status", "idle"
            ),
            "exploration_reason": getattr(
                getattr(self, "exploration", None), "reason", None
            ),
            # The waypoint being explored toward, while it has no known route.
            "exploration_goal": getattr(
                getattr(self, "goal_exploration", None), "display_goal", None
            ),
            "fleet_exploration_status": getattr(
                getattr(getattr(self, "exploration", None), "coordinator", None),
                "completion_state",
                "unknown",
            ),
            "exploration_coordination": (
                coordinator.summary()
                if (
                    coordinator := getattr(
                        getattr(self, "exploration", None), "coordinator", None
                    )
                )
                is not None
                and callable(getattr(coordinator, "summary", None))
                else None
            ),
            "nav_status": self.nav_status,
            "nav_failure_reason": getattr(self, "_nav_failure_reason", None),
            "goal": self.goal,
            # Backward-compatible effective route: local when available,
            # otherwise global. The two explicit fields below let the UI show
            # both routes at once for planners that expose both.
            "planned_path": planned_path,
            "global_planned_path": list(global_planned_path or []),
            "local_planned_path": list(local_planned_path or []),
        }
        readiness = getattr(self, "navigation_ready", None)
        if callable(readiness):
            # A connected adapter can precede Nav2 activation. Publish the
            # action-server fact so lifecycle consumers do not confuse an open
            # fleet websocket with a robot that can accept a route.
            state["navigation_ready"] = bool(readiness())
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
