"""Gazebo world reset. Per-robot pose/SLAM reset stays on RobotBridge."""

from __future__ import annotations

import subprocess
import threading
import time

from sim_cslam import CSLAM_GRID, SLAM_GRAPHS

WORLD_RESET_COALESCE_S = 5.0

_WORLD_RESET_LOCK = threading.Lock()
_world_reset_at = 0.0
_world_name: str | None = None


def reset_module_state() -> None:
    global _world_name, _world_reset_at
    _world_name = None
    _world_reset_at = 0.0
    CSLAM_GRID.clear()
    SLAM_GRAPHS.clear()


def gz_world_name(logger) -> str | None:
    """Find the running world's name instead of hardcoding it."""
    global _world_name
    if _world_name:
        return _world_name
    try:
        listing = subprocess.run(
            ["gz", "service", "-l"], capture_output=True, text=True, timeout=10
        ).stdout
    except (OSError, subprocess.SubprocessError) as exc:
        logger.warn(f"[adapter_sim] cannot list gz services: {exc}")
        return None
    for line in listing.splitlines():
        line = line.strip()
        # Exactly /world/<name>/control — playback/control also ends in /control.
        if line.startswith("/world/") and line.endswith("/control"):
            name = line[len("/world/") : -len("/control")]
            if name and "/" not in name:
                _world_name = name
                return name
    logger.warn("[adapter_sim] no /world/<name>/control service found")
    return None


def reset_world(logger) -> bool:
    """Restore scenery once per fleet-wide reset. Does not move robots.

    `model_only`, not `all`: `all` also zeros /clock, and every node here
    runs on use_sim_time. Robots are spawned via /create, so they stay put
    until RobotBridge teleports each one.
    """
    global _world_reset_at
    with _WORLD_RESET_LOCK:
        if time.monotonic() - _world_reset_at < WORLD_RESET_COALESCE_S:
            return True
        world = gz_world_name(logger)
        if world is None:
            return False
        try:
            done = subprocess.run(
                [
                    "gz",
                    "service",
                    "-s",
                    f"/world/{world}/control",
                    "--reqtype",
                    "gz.msgs.WorldControl",
                    "--reptype",
                    "gz.msgs.Boolean",
                    "--timeout",
                    "5000",
                    "--req",
                    "reset: {model_only: true}",
                ],
                capture_output=True,
                text=True,
                timeout=15,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            logger.warn(f"[adapter_sim] world reset failed: {exc}")
            return False
        if "true" not in done.stdout.lower():
            logger.warn(
                f"[adapter_sim] world reset rejected: "
                f"{done.stdout.strip()} {done.stderr.strip()}"
            )
            return False
        _world_reset_at = time.monotonic()
        CSLAM_GRID.clear()
        SLAM_GRAPHS.clear()
        return True
