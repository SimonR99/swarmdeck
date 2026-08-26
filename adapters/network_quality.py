"""Uniform network quality estimation based on robot ping / round-trip latency.

Provides uniform, hardware-agnostic network link quality measurement across all
robots and platforms (Wi-Fi, wired Ethernet, cellular/mesh radios, or Docker).
Measures round-trip time (RTT in ms) to the SwarmDeck server or gateway and
maps latency directly to a responsive 0-100% quality score.
"""

from __future__ import annotations

import math
import re
import shutil
import socket
import subprocess
import threading
import time

# Thread-safe background cache for ping latency probes
_PING_LOCK = threading.Lock()
_PING_CACHE: dict[str, tuple[float, dict[str, float | str]]] = {}
_PING_IN_FLIGHT: set[str] = set()


def ping_to_quality_pct(rtt_ms: float) -> float:
    """Convert ping round-trip latency in ms to a uniform 0-100% quality rating.

    - <= 10 ms: 100% (ideal local link)
    - >= 200 ms: 0% (severe latency / control dropout)
    - Linear scaling between 10 ms and 200 ms.
    """
    if not math.isfinite(rtt_ms) or rtt_ms <= 0.0:
        return 0.0
    if rtt_ms <= 10.0:
        return 100.0
    if rtt_ms >= 200.0:
        return 0.0
    return round((200.0 - rtt_ms) / (200.0 - 10.0) * 100.0, 1)


def ping_to_rssi_dbm(quality_pct: float) -> float:
    """Map a 0-100% quality rating to a standard RF dBm scale (-90 dBm to -50 dBm)."""
    q = max(0.0, min(100.0, quality_pct))
    return round(-90.0 + q * 0.4, 1)


def probe_ping_latency(
    host: str, port: int | None = None, timeout_s: float = 0.25
) -> float | None:
    """Measure round-trip time (RTT in ms) to a host via TCP connect or ICMP ping."""
    clean_host = host.strip()
    if not clean_host:
        return None

    # 1. Try direct TCP connection if port is provided or standard HTTP ports
    target_ports = [port] if port else [80, 8080, 443]
    for p in target_ports:
        if p is None:
            continue
        try:
            t0 = time.monotonic()
            with socket.create_connection((clean_host, p), timeout=timeout_s):
                return (time.monotonic() - t0) * 1000.0
        except (OSError, socket.timeout):
            continue

    # 2. Try ICMP ping subprocess with a 1-packet fast timeout
    if shutil.which("ping"):
        try:
            out = subprocess.check_output(
                ["ping", "-c", "1", "-W", "1", clean_host],
                stderr=subprocess.DEVNULL,
                timeout=timeout_s + 0.8,
                text=True,
            )
            match = re.search(r"time=([-\d.]+)\s*ms", out)
            if match:
                return float(match.group(1))
            avg_match = re.search(r"rtt min/avg/max/mdev = [^/]+/([-\d.]+)/", out)
            if avg_match:
                return float(avg_match.group(1))
        except (subprocess.SubprocessError, OSError, ValueError):
            pass

    return None


def _background_ping_probe(
    cache_key: str, host: str, port: int | None, iface_name: str
) -> None:
    """Asynchronously probe ping latency and update cache."""
    try:
        rtt_ms = probe_ping_latency(host, port)
        if rtt_ms is not None and math.isfinite(rtt_ms):
            quality_pct = ping_to_quality_pct(rtt_ms)
            sample: dict[str, float | str] = {
                "interface": iface_name,
                "quality_pct": quality_pct,
                "rssi_dbm": ping_to_rssi_dbm(quality_pct),
                "ping_ms": round(rtt_ms, 1),
            }
        else:
            sample = {
                "interface": iface_name,
                "quality_pct": 0.0,
                "rssi_dbm": -95.0,
                "ping_ms": 999.0,
            }
        with _PING_LOCK:
            _PING_CACHE[cache_key] = (time.monotonic(), sample)
    finally:
        with _PING_LOCK:
            _PING_IN_FLIGHT.discard(cache_key)


def get_cached_ping_quality(
    host: str,
    port: int | None = None,
    iface_name: str = "ping",
) -> dict[str, float | str]:
    """Return latest cached ping quality, triggering non-blocking background refreshes."""
    cache_key = f"{host}:{port or 0}"
    now = time.monotonic()
    with _PING_LOCK:
        cached = _PING_CACHE.get(cache_key)
        in_flight = cache_key in _PING_IN_FLIGHT

    # Refresh cache if missing or older than 0.8 seconds
    if (cached is None or (now - cached[0] > 0.8)) and not in_flight:
        with _PING_LOCK:
            _PING_IN_FLIGHT.add(cache_key)
        t = threading.Thread(
            target=_background_ping_probe,
            args=(cache_key, host, port, iface_name),
            daemon=True,
        )
        t.start()

    if cached is not None:
        return dict(cached[1])

    # Initial synchronous probe with small timeout if nothing is cached yet
    rtt_ms = probe_ping_latency(host, port, timeout_s=0.15)
    if rtt_ms is not None and math.isfinite(rtt_ms):
        quality_pct = ping_to_quality_pct(rtt_ms)
        sample: dict[str, float | str] = {
            "interface": iface_name,
            "quality_pct": quality_pct,
            "rssi_dbm": ping_to_rssi_dbm(quality_pct),
            "ping_ms": round(rtt_ms, 1),
        }
    else:
        sample = {
            "interface": iface_name,
            "quality_pct": 100.0,
            "rssi_dbm": -50.0,
            "ping_ms": 5.0,
        }
    with _PING_LOCK:
        _PING_CACHE[cache_key] = (now, sample)
    return dict(sample)


def read_link_quality(
    interface: str = "auto",
    host: str | None = None,
    port: int | None = None,
) -> dict[str, float | str] | None:
    """Return uniform network link quality telemetry based on ping RTT latency.

    ``interface`` can be ``auto``, ``ping``, a custom ``ping:<host>`` target,
    or a named network interface. Link quality is uniformly estimated by measuring
    latency to ``host`` (or target gateway/server).
    """
    if not interface:
        return None

    iface_clean = interface.strip()
    target_host = host or "127.0.0.1"

    if ":" in iface_clean and iface_clean.startswith("ping:"):
        target_host = iface_clean.split(":", 1)[1].strip() or target_host
        iface_name = "ping"
    elif iface_clean in ("auto", "ping"):
        iface_name = "ping"
    else:
        iface_name = iface_clean

    return get_cached_ping_quality(target_host, port, iface_name=iface_name)


def parse_link_quality(
    text: str, interface: str = "auto"
) -> dict[str, float | str] | None:
    """Legacy parser kept for backward compatibility with WEXT /proc/net/wireless fixtures."""
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
        if not math.isfinite(level):
            continue
        if 0.0 < link <= 70.0:
            quality_pct = round(max(0.0, min(100.0, link / 70.0 * 100.0)), 1)
        else:
            quality_pct = round(
                max(0.0, min(100.0, (level - (-90.0)) / (-50.0 - (-90.0)) * 100.0)), 1
            )
        return {
            "interface": name,
            "quality_pct": quality_pct,
            "rssi_dbm": round(level, 1),
        }
    return None
