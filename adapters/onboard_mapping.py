"""Strict local-map feed for decentralised navigation.

The adapter may publish its SLAM OccupancyGrid directly to Nav2 when central
map authority is disabled.  No frame transform is attempted here: a grid is
safe to reuse only when its header already names the navigation frame.
"""

from __future__ import annotations

import os
import time

from autonomy.capture_providers import (
    CAPTURE_PROVIDER_SPECS,
    CaptureProvider,
    capture_provider,
)

MAPPING_AUTHORITY_ENV = "SWARMDECK_MAPPING_AUTHORITY"
MAPPING_AUTHORITY_MODES = frozenset({"central", "onboard"})
CAPTURE_PROVIDER_ENV = "SWARMDECK_CAPTURE_PROVIDER"


def mapping_authority_mode(value: str | None = None) -> str:
    """Return a validated authority mode; legacy central maps remain default."""

    raw = os.environ.get(MAPPING_AUTHORITY_ENV, "central") if value is None else value
    mode = str(raw).strip().lower()
    if mode not in MAPPING_AUTHORITY_MODES:
        choices = ", ".join(sorted(MAPPING_AUTHORITY_MODES))
        raise ValueError(f"{MAPPING_AUTHORITY_ENV} must be one of: {choices}")
    return mode


def capture_provider_name(value: str | None = None) -> str:
    """Return the explicitly selected onboard capture provider.

    Unknown is the safe default for existing deployments: selecting a named
    provider still requires per-capture raw-source provenance before free-space
    evidence can be issued.
    """

    raw = os.environ.get(CAPTURE_PROVIDER_ENV, "unknown") if value is None else value
    name = str(raw).strip().lower()
    if name not in CAPTURE_PROVIDER_SPECS:
        choices = ", ".join(sorted(CAPTURE_PROVIDER_SPECS))
        raise ValueError(f"{CAPTURE_PROVIDER_ENV} must be one of: {choices}")
    return name


def selected_capture_provider(value: str | None = None) -> CaptureProvider:
    """Construct the selected ROS-free provider boundary."""

    return capture_provider(capture_provider_name(value))


def publish_onboard_map(bridge, msg) -> bool:
    """Republish an already aligned local OccupancyGrid, or fail closed.

    The original message is forwarded intact so its stamp, frame, origin and
    cell geometry remain one coherent SLAM snapshot.  In particular, this
    function never relabels a grid from another frame as the navigation frame.
    """

    if not bool(getattr(bridge, "onboard_mapping", False)):
        return False
    publisher = getattr(bridge, "pub_global_map", None)
    if publisher is None:
        return False

    source = str(getattr(getattr(msg, "header", None), "frame_id", "") or "")
    target = str(getattr(bridge, "map_frame", "") or "")
    if not source or not target or source != target:
        now = time.monotonic()
        warned_at = float(getattr(bridge, "_onboard_map_warned_at", 0.0))
        if now - warned_at >= 10.0:
            bridge._onboard_map_warned_at = now
            bridge.node.get_logger().warn(
                f"[{bridge.id}] onboard navigation map rejected: source frame "
                f"{source!r} does not exactly match {target!r}"
            )
        return False

    publisher.publish(msg)
    return True
