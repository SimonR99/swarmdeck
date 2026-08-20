"""Read a robot's local Wi-Fi link quality without a ROS dependency.

Linux exposes the currently associated wireless interfaces in
``/proc/net/wireless``.  Reading that file is cheap enough for the adapters'
5 Hz state loop and, unlike browser/WebRTC latency, measures the robot's own
radio link -- the quantity that belongs at the robot's map pose.
"""

from __future__ import annotations

import math
from pathlib import Path


WIRELESS_PATH = Path("/proc/net/wireless")


def parse_link_quality(text: str, interface: str = "auto") -> dict[str, float | str] | None:
    """Parse one ``/proc/net/wireless`` row.

    ``interface='auto'`` selects the first associated wireless interface.  The
    kernel link-quality column conventionally ranges from 0 to 70; RSSI is in
    dBm.  Some drivers expose dBm as an unsigned byte, which is normalised here.
    """
    wanted = interface.strip()
    for line in text.splitlines()[2:]:
        if ":" not in line:
            continue
        name, values = line.split(":", 1)
        name = name.strip()
        if wanted not in ("", "auto") and name != wanted:
            continue
        fields = values.split()
        if len(fields) < 3:
            continue
        try:
            link = float(fields[1].rstrip("."))
            level = float(fields[2].rstrip("."))
        except ValueError:
            continue
        if level > 0:
            level -= 256.0
        if not math.isfinite(link) or not math.isfinite(level):
            continue
        return {
            "interface": name,
            "quality_pct": round(max(0.0, min(100.0, link / 70.0 * 100.0)), 1),
            "rssi_dbm": round(level, 1),
        }
    return None


def read_link_quality(interface: str) -> dict[str, float | str] | None:
    """Return current Wi-Fi telemetry, or ``None`` when unavailable."""
    if not interface:
        return None
    try:
        return parse_link_quality(WIRELESS_PATH.read_text(), interface)
    except (OSError, UnicodeError):
        return None
