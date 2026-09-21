"""One fresh onboard map authority reader shared by planning consumers."""

from __future__ import annotations

import json
from copy import deepcopy
from contextlib import nullcontext
import math
import os
import re
import time

import numpy as np

from autonomy.contracts import KeyframeId, validate_se3
from autonomy.map_epochs import robot_run_id

_INDEXED_FIELDS = frozenset(
    {
        "map_epoch",
        "mapping_graph_revision",
        "geometry_revision",
        "map_source_stamp",
    }
)

PLANNING_FRAME_TEMPLATE_ENV = "SWARMDECK_PLANNING_FRAME_TEMPLATE"


def planning_frame(bridge):
    """Return the explicitly configured MGG frame, defaulting to the UI frame."""
    template = os.environ.get(PLANNING_FRAME_TEMPLATE_ENV)
    if template is None:
        value = getattr(bridge, "map_frame", "")
    else:
        if template.count("{robot}") != 1:
            raise ValueError("invalid planning frame template")
        value = template.replace("{robot}", str(bridge.id))
    value = str(value).lstrip("/")
    if (
        not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_/-]*", value)
        or "//" in value
        or value.endswith("/")
    ):
        raise ValueError("invalid planning frame")
    return value


def authority_for_frame(authority, frame):
    """Return one validated authority expressed in ``frame``.

    The published navigation pair remains the UI contract. An alternate
    planning pair is accepted only when both its frame and transform are
    present and exactly name the requested frame.
    """
    if not isinstance(authority, dict):
        raise ValueError("map authority is unavailable")
    target = str(frame).lstrip("/")
    navigation = str(authority.get("navigation_frame", "")).lstrip("/")
    if not target:
        raise ValueError("planning frame is empty")
    selected = deepcopy(authority)
    if target == navigation:
        try:
            validate_se3(authority["T_component_navigation"])
        except KeyError as exc:
            raise ValueError("map authority has no navigation transform") from exc
        return selected
    else:
        planning = str(authority.get("planning_frame", "")).lstrip("/")
        if planning != target or "T_component_planning" not in authority:
            raise ValueError(f"map authority has no transform for frame {target!r}")
        transform = validate_se3(authority["T_component_planning"])
        selected["navigation_frame"] = authority["planning_frame"]
        selected["T_component_navigation"] = transform

    # Home is transported as navigation<-home. Re-express it through the
    # component frame so its physical landmark remains unchanged.
    home = authority.get("home")
    if target != navigation and isinstance(home, dict) and "T_navigation_home" in home:
        source = np.asarray(validate_se3(authority["T_component_navigation"]))
        target_transform = np.asarray(transform)
        source_home = np.asarray(validate_se3(home["T_navigation_home"]))
        selected_home = deepcopy(home)
        selected_home["T_navigation_home"] = (
            np.linalg.inv(target_transform) @ source @ source_home
        ).tolist()
        selected["home"] = selected_home
    return selected


def _snapshot_order(authority):
    present = _INDEXED_FIELDS.intersection(authority)
    if present and present != _INDEXED_FIELDS:
        raise ValueError("Partial indexed map authority")
    if not present:
        return None
    epoch, revision = authority["map_epoch"], authority["mapping_graph_revision"]
    if any(type(v) is not int or v < 0 or v >= 2**64 for v in (epoch, revision)):
        raise ValueError("Invalid map revision")
    digest = authority["geometry_revision"]
    if not isinstance(digest, str) or not re.fullmatch("[0-9a-f]{64}", digest):
        raise ValueError("Invalid geometry digest")
    stamp = authority["map_source_stamp"]
    sec, nanosec = stamp["sec"], stamp["nanosec"]
    if (
        type(sec) is not int
        or type(nanosec) is not int
        or not 0 <= sec < 2**31
        or not 0 <= nanosec < 10**9
    ):
        raise ValueError("Invalid source stamp")
    return epoch, revision


def authority_order(authority):
    """Return comparable causal fields without ordering mission UUIDs."""
    mission = KeyframeId(authority["robot_id"], authority["mission_id"], 0).session_id
    raw = authority.get("solution_order")
    if raw is None:
        solution = None
    else:
        if (
            not isinstance(raw, (list, tuple))
            or len(raw) != 2
            or any(type(value) is not int for value in raw)
        ):
            raise ValueError("Invalid solution order")
        solution = tuple(raw)
    return mission, solution, _snapshot_order(authority)


def expected_mission_id(robot_id):
    mission = os.environ.get("SWARMDECK_MISSION_ID") or None
    if mission is not None:
        mission = KeyframeId(robot_id, mission, 0).session_id
    return mission


def accepts_authority_update(candidate, current, expected_mission=None):
    """Reject causal rollback while allowing identical heartbeats."""
    mission, solution, snapshot = authority_order(candidate)
    epoch = candidate["robot_map_epoch"]
    run_id = robot_run_id(mission, candidate["robot_id"], epoch)
    if candidate["run_id"] != run_id:
        return False
    if expected_mission is not None and mission != expected_mission:
        return False
    if current is None:
        return True
    old_mission, old_solution, old_snapshot = authority_order(current)
    # A process has no causal evidence for ordering two UUID missions. The
    # documented mission transition restarts the adapter/reader.
    if mission != old_mission:
        return False
    previous_epoch = current["robot_map_epoch"]
    if epoch != previous_epoch:
        return epoch > previous_epoch
    if old_solution is not None and (solution is None or solution < old_solution):
        return False
    if old_snapshot is not None and (snapshot is None or snapshot < old_snapshot):
        return False
    if snapshot is not None and snapshot == old_snapshot:
        for field in ("component_id", "geometry_revision", "map_source_stamp"):
            if candidate.get(field) != current.get(field):
                return False
    return True


def transform_change_squared(candidate, current):
    return sum(
        (value - previous) ** 2
        for candidate_row, previous_row in zip(candidate, current)
        for value, previous in zip(candidate_row, previous_row)
    )


def snapshot_values(authority):
    """Validate and convert the transport-independent authority to ROS fields."""
    matrix = np.asarray(validate_se3(authority["T_component_navigation"]))
    order = _snapshot_order(authority)
    if order is None:
        raise ValueError("Indexed map authority is required")
    epoch, revision = order
    digest = authority["geometry_revision"]
    stamp = authority["map_source_stamp"]
    sec, nanosec = stamp["sec"], stamp["nanosec"]
    r = matrix[:3, :3]
    # Choose the largest quaternion component to remain stable at 180 degrees.
    squared = (
        np.array(
            [
                1 + r[0, 0] - r[1, 1] - r[2, 2],
                1 - r[0, 0] + r[1, 1] - r[2, 2],
                1 - r[0, 0] - r[1, 1] + r[2, 2],
                1 + np.trace(r),
            ]
        )
        / 4
    )
    axis = int(np.argmax(squared))
    q = np.zeros(4)
    q[axis] = math.sqrt(max(0.0, squared[axis]))
    denominator = 4 * q[axis]
    if axis == 3:
        q[:3] = [r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]
        q[:3] /= denominator
    else:
        j, k = (axis + 1) % 3, (axis + 2) % 3
        q[j] = (r[j, axis] + r[axis, j]) / denominator
        q[k] = (r[k, axis] + r[axis, k]) / denominator
        q[3] = (r[k, j] - r[j, k]) / denominator
    q /= np.linalg.norm(q)
    return epoch, revision, digest, sec, nanosec, matrix[:3, 3], q


class MappingAuthority:
    def __init__(self, bridge):
        from std_msgs.msg import String

        self.bridge = bridge
        self.value, self.received_at = None, 0.0
        self.clock = time.monotonic
        self.expected_mission = expected_mission_id(bridge.id)
        self.robot_map_epoch = -1
        self.source_reset_stamp_ns = None
        self.planning_frame = planning_frame(bridge)
        self.publisher = None
        try:
            from mgg_msgs.msg import MappingSnapshot
            from rclpy.qos import QoSProfile, DurabilityPolicy

            self.message_type = MappingSnapshot
            self.publisher = bridge.node.create_publisher(
                MappingSnapshot,
                f"/{bridge.id}/mgg/mapping_snapshot",
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
        except ImportError:
            logger = getattr(bridge.node, "get_logger", lambda: None)()
            if logger is not None:
                logger.warning(
                    "Indexed mapping relay unavailable: rebuild the adapter image with current mgg_msgs"
                )
        self.subscription = bridge.node.create_subscription(
            String, f"/{bridge.id}/map_authority", self.receive, 5
        )

    def current(self):
        return (
            self.value
            if self.value is not None and self.clock() - self.received_at <= 3.0
            else None
        )

    def current_for_frame(self, frame):
        value = self.current()
        return None if value is None else authority_for_frame(value, frame)

    def _discard_map_uploads(self):
        for name in (
            "grid",
            "_cloud",
            "_cloud_points",
            "_cloud_rgb",
            "_cloud_snapshot",
            "_scan_points",
            "_scan_origin",
        ):
            if hasattr(self.bridge, name):
                setattr(self.bridge, name, None)
        for name in ("_grid_dirty", "_cloud_dirty", "_scan_dirty"):
            if hasattr(self.bridge, name):
                setattr(self.bridge, name, False)
        lock = getattr(self.bridge, "_costmap_lock", None)
        if lock is not None:
            with lock:
                self.bridge._costmaps.clear()
                self.bridge._costmap_dirty.clear()

    def _advance_lifetime(self, epoch):
        with getattr(self.bridge, "_goal_lock", nullcontext()):
            if epoch > self.robot_map_epoch and self.robot_map_epoch >= 0:
                exploration = getattr(self.bridge, "exploration", None)
                if exploration is not None:
                    exploration.stop()
                self.bridge.cancel_goal()
                self.bridge.drive(0.0, 0.0)
                self.bridge._display_anchor = None
                self.value = None
                self.source_reset_stamp_ns = None
                self._discard_map_uploads()
            self.robot_map_epoch = epoch

    def receive(self, message):
        try:
            if len(message.data) > 32768:
                return
            value = json.loads(message.data)
            epoch = value["robot_map_epoch"]
            if (
                value["robot_id"] != self.bridge.id
                or (
                    self.expected_mission is not None
                    and value["mission_id"] != self.expected_mission
                )
                or value["run_id"]
                != robot_run_id(value["mission_id"], self.bridge.id, epoch)
                or epoch < self.robot_map_epoch
            ):
                return
            if self.expected_mission is None:
                self.expected_mission = value["mission_id"]
            if value.get("state") == "resetting":
                self._advance_lifetime(epoch)
                return
            if value["robot_id"] != self.bridge.id or value["navigation_frame"].lstrip(
                "/"
            ) != self.bridge.map_frame.lstrip("/"):
                return
            if not accepts_authority_update(value, self.value, self.expected_mission):
                return
            transform = validate_se3(value["T_component_navigation"])
            if not isinstance(value["component_id"], str) or not value["component_id"]:
                return
            # Old authorities remain readable for non-indexed planning. A
            # partial new snapshot is invalid rather than a fallback to old data.
            selected = authority_for_frame(value, self.planning_frame)
            converted = (
                snapshot_values(selected) if _INDEXED_FIELDS.issubset(value) else None
            )
            cutoff = None
            if epoch > 0:
                stamp = value["source_reset_stamp"]
                sec, nanosec = stamp["sec"], stamp["nanosec"]
                if (
                    type(sec) is not int
                    or type(nanosec) is not int
                    or sec < 0
                    or not 0 <= nanosec < 10**9
                ):
                    return
                cutoff = sec * 1_000_000_000 + nanosec
            self._advance_lifetime(epoch)
            if cutoff is not None and self.source_reset_stamp_ns is None:
                self._discard_map_uploads()
            self.source_reset_stamp_ns = cutoff
            if converted is not None and self.publisher is not None:
                out = self.message_type()
                out.component_id = value["component_id"]
                (
                    out.epoch,
                    out.graph_revision,
                    out.geometry_revision,
                    out.source_stamp.sec,
                    out.source_stamp.nanosec,
                    xyz,
                    q,
                ) = converted
                (
                    out.component_from_navigation.translation.x,
                    out.component_from_navigation.translation.y,
                    out.component_from_navigation.translation.z,
                ) = map(float, xyz)
                (
                    out.component_from_navigation.rotation.x,
                    out.component_from_navigation.rotation.y,
                    out.component_from_navigation.rotation.z,
                    out.component_from_navigation.rotation.w,
                ) = map(float, q)
                self.publisher.publish(out)
            self.value = value
            self.received_at = self.clock()
        except (ValueError, TypeError, KeyError, AttributeError):
            return


def get_mapping_authority(bridge):
    reader = getattr(bridge, "_mapping_authority", None)
    if reader is None:
        reader = bridge._mapping_authority = MappingAuthority(bridge)
    return reader
