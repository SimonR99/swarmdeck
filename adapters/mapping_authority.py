"""One fresh onboard map authority reader shared by planning consumers."""

from __future__ import annotations

import json
import math
import re
import time

import numpy as np

from autonomy.contracts import KeyframeId, validate_se3


def snapshot_values(authority):
    """Validate and convert the transport-independent authority to ROS fields."""
    matrix = np.asarray(validate_se3(authority["T_component_navigation"]))
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

    def receive(self, message):
        try:
            if len(message.data) > 32768:
                return
            value = json.loads(message.data)
            if value["robot_id"] != self.bridge.id or value["navigation_frame"].lstrip(
                "/"
            ) != self.bridge.map_frame.lstrip("/"):
                return
            KeyframeId(value["robot_id"], value["mission_id"], 0)
            validate_se3(value["T_component_navigation"])
            if not isinstance(value["component_id"], str) or not value["component_id"]:
                return
            # Old authorities remain readable for non-indexed planning. A
            # partial new snapshot is invalid rather than a fallback to old data.
            fields = {
                "map_epoch",
                "mapping_graph_revision",
                "geometry_revision",
                "map_source_stamp",
            }
            converted = snapshot_values(value) if fields.intersection(value) else None
            if (
                converted is not None
                and self.value is not None
                and self.value["mission_id"] == value["mission_id"]
                and fields.issubset(self.value)
                and converted[:2]
                < (self.value["map_epoch"], self.value["mapping_graph_revision"])
            ):
                return
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
            self.value, self.received_at = value, self.clock()
        except (ValueError, TypeError, KeyError, AttributeError):
            return


def get_mapping_authority(bridge):
    reader = getattr(bridge, "_mapping_authority", None)
    if reader is None:
        reader = bridge._mapping_authority = MappingAuthority(bridge)
    return reader
