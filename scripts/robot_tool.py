#!/usr/bin/env python3
"""Cortex & SwarmDeck Robot & Fleet Control Tool.

Provides CLI and programmatic tools for Cortex and operators to:
- List connected robots and telemetry (battery, pose, mode, nav_status)
- Drive robots (linear, angular, timed duration)
- Send navigation goals and cancel goals
- Send body actions for legged robots (stand, sit, wave, etc.)
- Stop robot(s) immediately
- Inspect vision / camera frames ("what are you seeing on this robot")
- Capture live camera snapshots to disk
- Inspect image files for multimodal understanding
- Inspect YOLOE live detections and operator proposals
- Diagnose telemetry, camera, RTSP, SSH, and required robot services in one call
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import shlex
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any, Optional


REPO_DIR = Path(__file__).resolve().parents[1]
ROBOT_ALIASES = {
    "asimov": "asimov_0",
    "aslan": "aslan_0",
    "botman": "botman_0",
    "scout": "tars_0",
    "spot": "spot_0",
    "tars": "tars_0",
}
PROFILE_BY_ROBOT_ID = {
    "asimov_0": "asimov",
    "aslan_0": "aslan",
    "botman_0": "botman",
    "spot_0": "spot",
    "tars_0": "scout",
}


def _get_default_server() -> str:
    if "SWARMDECK_SERVER_URL" in os.environ:
        return os.environ["SWARMDECK_SERVER_URL"].rstrip("/")
    return "http://127.0.0.1:8080"


DEFAULT_SERVER_URL = _get_default_server()


def _get_default_rtsp_base() -> str:
    configured = os.environ.get("SWARMDECK_RTSP_BASE_URL")
    if configured:
        return configured.rstrip("/")
    server_host = urllib.parse.urlparse(DEFAULT_SERVER_URL).hostname
    media_host = "mediamtx" if server_host == "server" else "127.0.0.1"
    return f"rtsp://{media_host}:8554"


DEFAULT_RTSP_BASE_URL = _get_default_rtsp_base()


def normalize_robot_id(value: str) -> str:
    cleaned = value.lower().lstrip("@").replace("-", "_")
    return ROBOT_ALIASES.get(cleaned, cleaned)


def _http_get(endpoint: str, server_url: str = DEFAULT_SERVER_URL) -> Any:
    url = f"{server_url}{endpoint}"
    req = urllib.request.Request(url, headers={"Accept": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            data = resp.read()
            if resp.headers.get("content-type", "").startswith("application/json"):
                return json.loads(data.decode())
            return data
    except urllib.error.HTTPError as err:
        try:
            err_body = json.loads(err.read().decode())
            print(f"Error {err.code}: {err_body}", file=sys.stderr)
        except Exception:
            print(f"Error {err.code}: {err.reason}", file=sys.stderr)
        sys.exit(1)
    except Exception as err:
        print(f"Failed to connect to SwarmDeck server at {url}: {err}", file=sys.stderr)
        sys.exit(1)


def _http_post(endpoint: str, payload: dict[str, Any], server_url: str = DEFAULT_SERVER_URL) -> Any:
    url = f"{server_url}{endpoint}"
    data_bytes = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=data_bytes,
        headers={"Content-Type": "application/json", "Accept": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            return json.loads(resp.read().decode())
    except urllib.error.HTTPError as err:
        try:
            err_body = json.loads(err.read().decode())
            print(f"Error {err.code}: {err_body}", file=sys.stderr)
        except Exception:
            print(f"Error {err.code}: {err.reason}", file=sys.stderr)
        sys.exit(1)
    except Exception as err:
        print(f"Failed to connect to SwarmDeck server at {url}: {err}", file=sys.stderr)
        sys.exit(1)


def cmd_list(args: argparse.Namespace) -> None:
    fleet_data = _http_get("/api/fleet", args.server)
    robots = fleet_data.get("robots", [])
    if args.json:
        print(json.dumps(robots, indent=2))
        return

    if not robots:
        print("No robots currently connected to SwarmDeck.")
        return

    print(f"{'ROBOT ID':<15} {'STATE':<9} {'TYPE':<22} {'BATTERY':<10} {'MODE':<10} {'NAV STATUS':<12} {'POSE (X, Y, YAW)'}")
    print("=" * 104)
    for r in robots:
        rid = r.get("robot_id", "unknown")
        rtype = r.get("robot_type", "unknown")
        state = "online" if r.get("online") is True else "OFFLINE"
        batt = f"{int(r.get('battery', 0.0) * 100)}%" if r.get("battery") is not None else "N/A"
        mode = r.get("mode", "idle")
        nav_st = r.get("nav_status", "idle")
        pose = r.get("pose") or {}
        px = f"{pose.get('x', 0.0):.2f}"
        py = f"{pose.get('y', 0.0):.2f}"
        pyaw = f"{pose.get('yaw', 0.0):.2f}"
        pose_str = f"({px}, {py}, {pyaw} rad)"
        print(f"{rid:<15} {state:<9} {rtype:<22} {batt:<10} {mode:<10} {nav_st:<12} {pose_str}")


def cmd_drive(args: argparse.Namespace) -> None:
    payload = {
        "linear": float(args.linear),
        "angular": float(args.angular),
        "duration": float(args.duration) if args.duration else 0.0,
    }
    res = _http_post(f"/api/robot/{args.robot_id}/drive", payload, args.server)
    if args.duration and args.duration > 0:
        print(f"Driving {args.robot_id}: linear={args.linear} m/s, angular={args.angular} rad/s for {args.duration}s... (Auto-stopping afterwards)")
    else:
        print(f"Driving {args.robot_id}: linear={args.linear} m/s, angular={args.angular} rad/s")
    if args.json:
        print(json.dumps(res, indent=2))


def cmd_navigate(args: argparse.Namespace) -> None:
    payload = {
        "x": float(args.x),
        "y": float(args.y),
        "yaw": float(args.yaw) if args.yaw is not None else 0.0,
    }
    res = _http_post(f"/api/robot/{args.robot_id}/goal", payload, args.server)
    print(f"Navigation goal sent to {args.robot_id}: target=({args.x}, {args.y}, yaw={payload['yaw']})")
    if args.json:
        print(json.dumps(res, indent=2))


def cmd_cancel(args: argparse.Namespace) -> None:
    res = _http_post(f"/api/robot/{args.robot_id}/cancel", {}, args.server)
    print(f"Cancelled navigation goal for {args.robot_id}")
    if args.json:
        print(json.dumps(res, indent=2))


def cmd_stop(args: argparse.Namespace) -> None:
    res = _http_post(f"/api/robot/{args.robot_id}/stop", {}, args.server)
    print(f"Stopped {args.robot_id}")
    if args.json:
        print(json.dumps(res, indent=2))


def cmd_body(args: argparse.Namespace) -> None:
    payload: dict[str, Any] = {"action": args.action}
    if args.height is not None:
        payload["height"] = float(args.height)
    res = _http_post(f"/api/robot/{args.robot_id}/body", payload, args.server)
    print(f"Body command '{args.action}' sent to {args.robot_id}")
    if args.json:
        print(json.dumps(res, indent=2))


def cmd_see(args: argparse.Namespace) -> None:
    """Answers 'what are you seeing on this robot'."""
    robot_id = args.robot_id
    fleet_data = _http_get("/api/fleet", args.server)
    robots = {r.get("robot_id"): r for r in fleet_data.get("robots", [])}
    robot = robots.get(robot_id)
    if not robot:
        print(f"Robot '{robot_id}' not found in connected fleet.", file=sys.stderr)
        print(f"Available robots: {list(robots.keys())}", file=sys.stderr)
        sys.exit(1)

    # Fetch detections
    detections_data = _http_get("/api/detections", args.server)
    live_tracks = detections_data.get("tracks", [])
    proposals = detections_data.get("proposals", [])
    entities = detections_data.get("entities", [])

    # Filter for this robot
    robot_tracks = [d for d in live_tracks if d.get("robot_id") == robot_id]
    robot_proposals = [
        p for p in proposals
        if p.get("robot_id") == robot_id
        or robot_id in p.get("robot_ids", [])
        or (isinstance(p.get("sightings"), list) and robot_id in p.get("sightings"))
    ]

    # Check camera frame info
    vision_info = _http_get(f"/api/robot/{robot_id}/vision", args.server)

    pose = robot.get("pose") or {}
    px = pose.get("x", 0.0)
    py = pose.get("y", 0.0)
    pyaw = pose.get("yaw", 0.0)

    summary = {
        "robot_id": robot_id,
        "robot_type": robot.get("robot_type"),
        "pose": {"x": px, "y": py, "yaw": pyaw},
        "camera_streaming": vision_info.get("camera_streaming", False),
        "frame_age_ms": vision_info.get("frame_age_ms"),
        "visible_objects": [],
        "pending_proposals": [],
        "fleet_confirmed_entities_nearby": [],
    }

    for track in robot_tracks:
        summary["visible_objects"].append({
            "class": track.get("class_name") or track.get("label", "object"),
            "confidence": track.get("confidence", 0.0),
            "bbox": track.get("bbox"),
            "map_pos": track.get("position"),
        })

    for prop in robot_proposals:
        summary["pending_proposals"].append({
            "proposal_id": prop.get("proposal_id") or prop.get("id"),
            "class": prop.get("class_name") or prop.get("class"),
            "confidence": prop.get("confidence") or prop.get("best_score"),
            "position": prop.get("position"),
        })

    # Find entities within 5 meters
    for ent in entities:
        pos = ent.get("position") or {}
        ex, ey = pos.get("x", 0.0), pos.get("y", 0.0)
        dist = ((ex - px) ** 2 + (ey - py) ** 2) ** 0.5
        if dist <= 5.0:
            summary["fleet_confirmed_entities_nearby"].append({
                "entity_id": ent.get("entity_id") or ent.get("id"),
                "class": ent.get("class_name") or ent.get("class"),
                "distance_m": round(dist, 2),
                "position": pos,
            })

    if args.json:
        print(json.dumps(summary, indent=2))
        return

    print(f"=== Vision Report for Robot '{robot_id}' ===")
    print(f"Type: {robot.get('robot_type')} | Pose: ({px:.2f}, {py:.2f}, yaw={pyaw:.2f})")
    cam_status = "Active & Streaming" if summary["camera_streaming"] else "Standby / No recent frame"
    print(f"Camera Feed: {cam_status}")
    print()

    if summary["visible_objects"]:
        print(f"Currently Detected Objects in Camera View ({len(summary['visible_objects'])}):")
        for obj in summary["visible_objects"]:
            conf = f"{int(obj['confidence'] * 100)}%" if obj.get("confidence") else ""
            pos = obj.get("map_pos")
            pos_str = f" at map ({pos['x']:.2f}, {pos['y']:.2f})" if pos else ""
            print(f"  - {obj['class']} {conf}{pos_str}")
    else:
        print("No live object detections currently in camera field of view.")

    if summary["pending_proposals"]:
        print(f"\nPending Operator Review Proposals ({len(summary['pending_proposals'])}):")
        for prop in summary["pending_proposals"]:
            conf_str = f" (conf {int(prop['confidence']*100)}%)" if prop.get("confidence") is not None else ""
            pos = prop.get("position")
            pos_str = f" at (x={pos['x']:.2f}, y={pos['y']:.2f})" if pos and "x" in pos else ""
            print(f"  - [{prop['proposal_id']}] {prop['class']}{conf_str}{pos_str}")

    if summary["fleet_confirmed_entities_nearby"]:
        print(f"\nNearby Confirmed Map Objects (<= 5m):")
        for ent in summary["fleet_confirmed_entities_nearby"]:
            pos = ent.get("position")
            pos_str = f" at (x={pos['x']:.2f}, y={pos['y']:.2f})" if pos and "x" in pos else ""
            print(f"  - [{ent['entity_id']}] {ent['class']}{pos_str} ({ent['distance_m']}m away)")


def cmd_detections(args: argparse.Namespace) -> None:
    detections_data = _http_get("/api/detections", args.server)
    if args.json:
        print(json.dumps(detections_data, indent=2))
        return

    tracks = detections_data.get("tracks", [])
    proposals = detections_data.get("proposals", [])
    entities = detections_data.get("entities", [])

    print(f"=== Fleet Detections Overview ===")
    print(f"Live Camera Tracks: {len(tracks)}")
    for t in tracks:
        cls_name = t.get("class_name") or t.get("class") or t.get("label", "object")
        print(f"  - [{t.get('robot_id')}] {cls_name} (conf {int(t.get('confidence',0)*100)}%)")

    print(f"\nProposals Awaiting Operator Review: {len(proposals)}")
    for p in proposals:
        cls_name = p.get("class_name") or p.get("class")
        p_id = p.get("proposal_id") or p.get("id")
        r_id = p.get("robot_id") or (p.get("robot_ids")[0] if p.get("robot_ids") else "unknown")
        print(f"  - [{p_id}] {cls_name} from {r_id}")

    print(f"\nConfirmed Map Entities: {len(entities)}")
    for e in entities:
        pos = e.get("position") or {}
        e_id = e.get("entity_id") or e.get("id")
        cls_name = e.get("class_name") or e.get("class")
        print(f"  - [{e_id}] {cls_name} at ({pos.get('x',0):.2f}, {pos.get('y',0):.2f})")


def _try_http_get(
    endpoint: str, server_url: str, timeout: float = 5.0
) -> tuple[Optional[Any], Optional[str]]:
    url = f"{server_url}{endpoint}"
    request = urllib.request.Request(url, headers={"Accept": "application/json"})
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.loads(response.read().decode()), None
    except urllib.error.HTTPError as exc:
        return None, f"HTTP {exc.code} {exc.reason}"
    except Exception as exc:
        return None, str(exc)


def probe_rtsp_stream(robot_id: str, rtsp_base_url: str) -> dict[str, Any]:
    """Check whether MediaMTX advertises a robot stream.

    DESCRIBE deliberately is not called a decoded-frame check.  The distinction
    matters: a publishing session can expose valid SDP while its RTP payload is
    stalled.
    """
    url = f"{rtsp_base_url.rstrip('/')}/{urllib.parse.quote(robot_id)}"
    parsed = urllib.parse.urlparse(url)
    host = parsed.hostname
    port = parsed.port or 554
    if not host:
        return {"ok": False, "url": url, "error": "RTSP URL has no host"}

    request = (
        f"DESCRIBE {url} RTSP/1.0\r\n"
        "CSeq: 1\r\n"
        "Accept: application/sdp\r\n"
        "User-Agent: SwarmDeckDoctor/1.0\r\n\r\n"
    ).encode()
    try:
        with socket.create_connection((host, port), timeout=3.0) as connection:
            connection.settimeout(3.0)
            connection.sendall(request)
            response = connection.recv(4096).decode(errors="replace")
    except Exception as exc:
        return {"ok": False, "url": url, "error": str(exc)}

    status_line = response.splitlines()[0] if response else "no response"
    parts = status_line.split(maxsplit=2)
    try:
        status_code = int(parts[1])
    except (IndexError, ValueError):
        status_code = None
    return {
        "ok": status_code == 200,
        "url": url,
        "status": status_code,
        "status_line": status_line,
        "evidence": "RTSP publication metadata only; not a decoded-frame check",
    }


def _read_rtsp_response(
    connection: socket.socket, buffered: bytes = b""
) -> tuple[int, dict[str, str], bytes, bytes]:
    while b"\r\n\r\n" not in buffered:
        chunk = connection.recv(4096)
        if not chunk:
            raise RuntimeError("RTSP server closed the connection")
        buffered += chunk
    header_block, buffered = buffered.split(b"\r\n\r\n", 1)
    lines = header_block.decode(errors="replace").split("\r\n")
    status_parts = lines[0].split(maxsplit=2)
    if len(status_parts) < 2 or not status_parts[1].isdigit():
        raise RuntimeError(f"invalid RTSP response: {lines[0]}")
    headers: dict[str, str] = {}
    for line in lines[1:]:
        if ":" in line:
            key, value = line.split(":", 1)
            headers[key.strip().lower()] = value.strip()
    content_length = int(headers.get("content-length", "0"))
    while len(buffered) < content_length:
        chunk = connection.recv(4096)
        if not chunk:
            raise RuntimeError("RTSP body ended early")
        buffered += chunk
    return (
        int(status_parts[1]),
        headers,
        buffered[:content_length],
        buffered[content_length:],
    )


def probe_video_packets(robot_id: str, rtsp_base_url: str) -> dict[str, Any]:
    """PLAY the RTSP stream and require progressing interleaved RTP packets."""
    url = f"{rtsp_base_url.rstrip('/')}/{urllib.parse.quote(robot_id)}"
    parsed = urllib.parse.urlparse(url)
    if not parsed.hostname:
        return {"ok": False, "url": url, "error": "RTSP URL has no host"}

    cseq = 0

    def send_request(
        connection: socket.socket,
        method: str,
        request_url: str,
        headers: Optional[dict[str, str]] = None,
    ) -> None:
        nonlocal cseq
        cseq += 1
        lines = [
            f"{method} {request_url} RTSP/1.0",
            f"CSeq: {cseq}",
            "User-Agent: SwarmDeckDoctor/1.0",
        ]
        lines.extend(f"{key}: {value}" for key, value in (headers or {}).items())
        connection.sendall(("\r\n".join(lines) + "\r\n\r\n").encode())

    try:
        with socket.create_connection(
            (parsed.hostname, parsed.port or 554), timeout=3.0
        ) as connection:
            connection.settimeout(3.0)
            send_request(connection, "DESCRIBE", url, {"Accept": "application/sdp"})
            status, describe_headers, body, buffered = _read_rtsp_response(connection)
            if status != 200:
                return {"ok": False, "url": url, "error": f"DESCRIBE returned {status}"}

            sdp = body.decode(errors="replace")
            control = None
            in_video = False
            for line in sdp.splitlines():
                if line.startswith("m="):
                    in_video = line.startswith("m=video ")
                elif in_video and line.startswith("a=control:"):
                    candidate = line.split(":", 1)[1].strip()
                    if candidate and candidate != "*":
                        control = candidate
                        break
            if not control:
                return {"ok": False, "url": url, "error": "SDP has no video control track"}
            if control.startswith("rtsp://"):
                track_url = control
            else:
                content_base = describe_headers.get("content-base", f"{url}/")
                track_url = f"{content_base.rstrip('/')}/{control.lstrip('/')}"

            send_request(
                connection,
                "SETUP",
                track_url,
                {"Transport": "RTP/AVP/TCP;unicast;interleaved=0-1"},
            )
            status, setup_headers, _, buffered = _read_rtsp_response(connection, buffered)
            if status != 200:
                return {"ok": False, "url": url, "error": f"SETUP returned {status}"}
            session = setup_headers.get("session", "").split(";", 1)[0]
            if not session:
                return {"ok": False, "url": url, "error": "SETUP returned no session"}

            send_request(connection, "PLAY", url, {"Session": session})
            status, _, _, buffered = _read_rtsp_response(connection, buffered)
            if status != 200:
                return {"ok": False, "url": url, "error": f"PLAY returned {status}"}

            sequences: set[int] = set()
            packet_count = 0
            byte_count = 0
            deadline = time.monotonic() + 3.0
            connection.settimeout(0.5)
            while time.monotonic() < deadline and len(sequences) < 2:
                try:
                    chunk = connection.recv(16384)
                except TimeoutError:
                    continue
                if not chunk:
                    break
                buffered += chunk
                while len(buffered) >= 4:
                    if buffered[0] != 0x24:
                        marker = buffered.find(b"$")
                        buffered = buffered[marker:] if marker >= 0 else b""
                        if len(buffered) < 4:
                            break
                    channel = buffered[1]
                    packet_length = int.from_bytes(buffered[2:4], "big")
                    if len(buffered) < 4 + packet_length:
                        break
                    packet = buffered[4 : 4 + packet_length]
                    buffered = buffered[4 + packet_length :]
                    if channel % 2 == 0 and len(packet) >= 12 and packet[0] >> 6 == 2:
                        packet_count += 1
                        byte_count += len(packet)
                        sequences.add(int.from_bytes(packet[2:4], "big"))

            return {
                "ok": len(sequences) >= 2,
                "url": url,
                "packet_count": packet_count,
                "bytes": byte_count,
                "sequence_count": len(sequences),
                "codec": "H264" if "H264/90000" in sdp.upper() else "unknown",
                "error": None if len(sequences) >= 2 else "no progressing RTP packets observed",
            }
    except Exception as exc:
        return {"ok": False, "url": url, "error": str(exc)}


def _profile_details(robot_id: str) -> tuple[Optional[dict[str, Any]], Optional[str]]:
    profile_name = PROFILE_BY_ROBOT_ID.get(robot_id)
    if not profile_name:
        return None, f"no deployment profile is mapped to {robot_id}"
    fleet_env = REPO_DIR / "deploy" / "fleet.env"
    profile_env = REPO_DIR / "deploy" / "robots" / f"{profile_name}.env"
    shell = (
        'source "$1"; source "$2"; '
        "printf '%s\\0%s\\0%s\\0' \"$DEPLOY_SSH_HOST\" "
        '"${DEPLOY_CHECK_CONTAINERS:-}" "${DEPLOY_ROBOT_ID:-}"'
    )
    try:
        result = subprocess.run(
            ["bash", "--noprofile", "--norc", "-c", shell, "_", str(fleet_env), str(profile_env)],
            check=False,
            capture_output=True,
            timeout=5,
        )
    except Exception as exc:
        return None, str(exc)
    if result.returncode != 0:
        return None, result.stderr.decode(errors="replace").strip() or "profile load failed"
    fields = result.stdout.decode(errors="replace").split("\0")
    if len(fields) < 3:
        return None, "profile returned incomplete deployment details"
    return {
        "profile": profile_name,
        "ssh_host": fields[0],
        "containers": fields[1].split(),
        "robot_id": fields[2],
    }, None


def _classify_ssh_error(stderr: str) -> str:
    lowered = stderr.lower()
    if "permission denied" in lowered or "no identities" in lowered:
        return "SSH authentication failed"
    if any(
        marker in lowered
        for marker in ("timed out", "no route to host", "could not resolve", "connection refused")
    ):
        return "robot host is unreachable"
    return "SSH/service check failed"


def check_remote_services(robot_id: str) -> dict[str, Any]:
    profile, profile_error = _profile_details(robot_id)
    if not profile:
        return {"ok": False, "error": profile_error}
    containers = profile["containers"]
    if not containers:
        return {
            "ok": False,
            "profile": profile["profile"],
            "ssh_host": profile["ssh_host"],
            "error": "deployment profile declares no required containers",
        }

    _ensure_ssh_agent()
    inspect_format = (
        "{{.Name}}|{{.State.Status}}|"
        "{{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}}"
    )
    remote_command = shlex.join(
        ["docker", "inspect", "-f", inspect_format, *containers]
    )
    try:
        result = subprocess.run(
            [
                "ssh",
                "-o",
                "BatchMode=yes",
                "-o",
                "ConnectTimeout=4",
                "-o",
                "StrictHostKeyChecking=accept-new",
                profile["ssh_host"],
                remote_command,
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=12,
        )
    except Exception as exc:
        return {**profile, "ok": False, "error": str(exc)}

    observed: dict[str, dict[str, str]] = {}
    for line in result.stdout.splitlines():
        fields = line.lstrip("/").split("|")
        if len(fields) == 3:
            observed[fields[0]] = {"state": fields[1], "health": fields[2]}
    service_rows = []
    for container in containers:
        state = observed.get(container, {"state": "missing", "health": "unknown"})
        service_rows.append({"container": container, **state})

    services_ok = result.returncode == 0 and all(
        item["state"] == "running" and item["health"] in {"healthy", "none"}
        for item in service_rows
    )
    response = {
        "ok": services_ok,
        "profile": profile["profile"],
        "ssh_host": profile["ssh_host"],
        "containers": service_rows,
    }
    if not services_ok:
        detail = result.stderr.strip()
        response["error"] = f"{_classify_ssh_error(detail)}: {detail or 'required container is not ready'}"
    return response


def collect_doctor_report(args: argparse.Namespace) -> dict[str, Any]:
    fleet_payload, fleet_error = _try_http_get("/api/fleet", args.server)
    if fleet_payload is None:
        return {
            "ok": False,
            "server": {"ok": False, "url": args.server, "error": fleet_error},
            "robots": [],
        }

    fleet_robots = fleet_payload.get("robots", []) if isinstance(fleet_payload, dict) else []
    fleet_by_id = {
        item.get("robot_id"): item
        for item in fleet_robots
        if isinstance(item, dict) and item.get("robot_id")
    }
    if args.robot == "all":
        robot_ids = list(fleet_by_id)
    else:
        robot_ids = [normalize_robot_id(args.robot)]

    first_vision: dict[str, Optional[dict[str, Any]]] = {}
    first_errors: dict[str, Optional[str]] = {}
    for robot_id in robot_ids:
        payload, error = _try_http_get(f"/api/robot/{robot_id}/vision", args.server)
        first_vision[robot_id] = payload if isinstance(payload, dict) else None
        first_errors[robot_id] = error
    if robot_ids and args.sample_seconds > 0:
        time.sleep(args.sample_seconds)

    reports = []
    for robot_id in robot_ids:
        robot = fleet_by_id.get(robot_id)
        second, second_error = _try_http_get(f"/api/robot/{robot_id}/vision", args.server)
        vision = second if isinstance(second, dict) else None
        previous = first_vision.get(robot_id)
        first_seq = previous.get("frame_seq") if previous else None
        second_seq = vision.get("frame_seq") if vision else None
        camera_fresh = bool(vision and vision.get("camera_streaming"))
        camera_progressing = bool(
            camera_fresh
            and second_seq is not None
            and (first_seq is None or second_seq != first_seq)
        )
        camera = {
            "ok": camera_fresh and camera_progressing,
            "fresh": camera_fresh,
            "progressing": camera_progressing,
            "frame_age_ms": vision.get("frame_age_ms") if vision else None,
            "first_seq": first_seq,
            "second_seq": second_seq,
        }
        if not vision:
            camera["error"] = second_error or first_errors.get(robot_id) or "no vision response"

        telemetry = {
            "ok": bool(robot and robot.get("online") is True),
            "registered": robot is not None,
            "online": bool(robot and robot.get("online") is True),
            "battery": robot.get("battery") if robot else None,
            "mode": robot.get("mode") if robot else None,
            "nav_status": robot.get("nav_status") if robot else None,
        }
        rtsp = probe_rtsp_stream(robot_id, args.rtsp_base_url)
        if telemetry["online"] and rtsp["ok"]:
            media = probe_video_packets(robot_id, args.rtsp_base_url)
        else:
            media = {
                "ok": False,
                "error": "skipped because telemetry or RTSP publication is unavailable",
            }
        services = check_remote_services(robot_id) if args.services else {"checked": False}
        frame_evidence_ok = media["ok"] is True
        robot_ok = telemetry["ok"] and rtsp["ok"] and frame_evidence_ok
        if args.services:
            robot_ok = robot_ok and services.get("ok") is True
        reports.append(
            {
                "robot_id": robot_id,
                "ok": robot_ok,
                "telemetry": telemetry,
                "camera": camera,
                "rtsp": rtsp,
                "media": media,
                "services": services,
            }
        )

    return {
        "ok": bool(reports) and all(item["ok"] for item in reports),
        "server": {"ok": True, "url": args.server},
        "robots": reports,
    }


def cmd_doctor(args: argparse.Namespace) -> None:
    report = collect_doctor_report(args)
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        heading = "HEALTHY" if report["ok"] else "DEGRADED"
        print(f"=== SwarmDeck Doctor: {heading} ===")
        if not report["server"]["ok"]:
            print(f"Server: FAIL — {report['server'].get('error', 'unavailable')}")
        for robot in report["robots"]:
            label = "HEALTHY" if robot["ok"] else "DEGRADED"
            print(f"\n{robot['robot_id']}: {label}")
            telemetry = robot["telemetry"]
            if telemetry["registered"]:
                state = "online" if telemetry["online"] else "OFFLINE (stored telemetry only)"
                print(f"  telemetry: {state}")
            else:
                print("  telemetry: not registered")
            camera = robot["camera"]
            if camera["ok"]:
                print(f"  camera frames: fresh and progressing ({camera['frame_age_ms']} ms old)")
            elif camera["fresh"]:
                print("  camera frames: fresh but no new frame observed during sample")
            else:
                detail = camera.get("error") or "no recent frame"
                print(f"  camera preview: unavailable — {detail} (separate from RTSP video)")
            rtsp = robot["rtsp"]
            if rtsp["ok"]:
                print("  MediaMTX RTSP: published (DESCRIBE 200)")
            else:
                detail = rtsp.get("status_line") or rtsp.get("error") or "unavailable"
                print(f"  MediaMTX RTSP: FAIL — {detail}")
            media = robot["media"]
            if media.get("ok") is True:
                print(
                    f"  video packets: progressing {media.get('codec', 'RTP')} "
                    f"({media.get('packet_count', 0)} packets sampled)"
                )
            else:
                print(f"  video packets: FAIL — {media.get('error', 'no packets observed')}")
            services = robot["services"]
            if services.get("checked") is not False:
                if services.get("ok"):
                    print(f"  remote services: {len(services['containers'])}/{len(services['containers'])} ready")
                else:
                    print(f"  remote services: FAIL — {services.get('error', 'not ready')}")
                    for item in services.get("containers", []):
                        print(
                            f"    {item['container']}: {item['state']} "
                            f"(health {item['health']})"
                        )
        if report["robots"]:
            print("\nEvidence note: RTSP publication alone is not healthy; progressing media packets are required.")
    if not report["ok"]:
        raise SystemExit(1)


def _ensure_ssh_agent() -> None:
    current_sock = os.environ.get("SSH_AUTH_SOCK")
    needs_update = True
    if current_sock and os.path.exists(current_sock):
        try:
            with socket.socket(socket.AF_UNIX) as s:
                s.settimeout(0.5)
                s.connect(current_sock)
                needs_update = False
        except Exception:
            needs_update = True
    if needs_update:
        agent_socks = sorted(glob.glob("/root/.ssh_host/agent/s.*"), key=os.path.getmtime)
        for sock in reversed(agent_socks):
            try:
                with socket.socket(socket.AF_UNIX) as s:
                    s.settimeout(0.5)
                    s.connect(sock)
                    os.environ["SSH_AUTH_SOCK"] = sock
                    break
            except Exception:
                continue


def cmd_deploy(args: argparse.Namespace) -> None:
    _ensure_ssh_agent()
    raw_name = args.robot.lower().replace("@", "")
    base_robot = raw_name.split("_")[0] if "_" in raw_name else raw_name
    if base_robot == "tars":
        base_robot = "scout"
    print(f"Deploying robot profile '{base_robot}' via 'make deploy ROBOT={base_robot}'...")
    try:
        ret = subprocess.run(["make", "deploy", f"ROBOT={base_robot}"], capture_output=True, text=True, timeout=300)
        if ret.returncode == 0:
            print(f"✓ Successfully deployed and started '{base_robot}'.")
            if ret.stdout:
                lines = [l for l in ret.stdout.strip().split("\n") if l.strip()]
                print("\n".join(lines[-3:]))
        else:
            detail = (ret.stderr + ret.stdout).strip()
            print(f"⚠️ Deployment failed:\n{detail}", file=sys.stderr)
            raise SystemExit(ret.returncode or 1)
    except subprocess.TimeoutExpired:
        print("Deployment failed: timed out after 300 seconds", file=sys.stderr)
        raise SystemExit(1)
    except SystemExit:
        raise
    except Exception as exc:
        print(f"Deployment command error: {exc}", file=sys.stderr)
        raise SystemExit(1)


def main() -> None:
    parser = argparse.ArgumentParser(description="SwarmDeck Robot & Fleet Control Tool")
    parser.add_argument("--server", default=DEFAULT_SERVER_URL, help="SwarmDeck server base URL")
    parser.add_argument("--json", action="store_true", help="Output raw JSON format")

    subparsers = parser.add_subparsers(dest="subcommand", required=True)

    # deploy
    p_deploy = subparsers.add_parser("deploy", help="Deploy/start/restart robot profiles (e.g. make deploy ROBOT=spot)")
    p_deploy.add_argument("robot", default="all", nargs="?", help="Robot profile name (spot, aslan, botman, tars, or all)")
    p_deploy.set_defaults(func=cmd_deploy)

    # list
    p_list = subparsers.add_parser("list", help="List all connected robots and telemetry")
    p_list.set_defaults(func=cmd_list)

    # doctor
    p_doctor = subparsers.add_parser(
        "doctor",
        help="Check telemetry, progressing camera frames, RTSP, and optional remote services",
    )
    p_doctor.add_argument(
        "robot",
        default="all",
        nargs="?",
        help="Robot profile/id or 'all' (aliases such as scout and tars are accepted)",
    )
    p_doctor.add_argument(
        "--services",
        action="store_true",
        help="Also verify required containers over non-interactive SSH",
    )
    p_doctor.add_argument(
        "--rtsp-base-url",
        default=DEFAULT_RTSP_BASE_URL,
        help="MediaMTX RTSP base URL",
    )
    p_doctor.add_argument(
        "--sample-seconds",
        type=float,
        default=1.0,
        help="Delay between camera sequence samples (default: 1.0)",
    )
    p_doctor.set_defaults(func=cmd_doctor)

    # drive
    p_drive = subparsers.add_parser("drive", help="Send velocity commands to a robot")
    p_drive.add_argument("robot_id", help="Target robot ID (e.g. aslan_0, botman_0)")
    p_drive.add_argument("--linear", type=float, default=0.0, help="Linear velocity in m/s (e.g. 0.3 for forward, -0.3 for backward)")
    p_drive.add_argument("--angular", type=float, default=0.0, help="Angular velocity in rad/s (e.g. 0.5 for left turn, -0.5 for right)")
    p_drive.add_argument("--duration", type=float, default=0.0, help="Duration in seconds to drive before auto-stopping (e.g. 2.0)")
    p_drive.set_defaults(func=cmd_drive)

    # navigate
    p_nav = subparsers.add_parser("navigate", help="Send a navigation goal to a robot")
    p_nav.add_argument("robot_id", help="Target robot ID")
    p_nav.add_argument("--x", type=float, required=True, help="Target X coordinate")
    p_nav.add_argument("--y", type=float, required=True, help="Target Y coordinate")
    p_nav.add_argument("--yaw", type=float, default=0.0, help="Target yaw orientation in radians")
    p_nav.set_defaults(func=cmd_navigate)

    # cancel
    p_cancel = subparsers.add_parser("cancel", help="Cancel current navigation goal")
    p_cancel.add_argument("robot_id", help="Target robot ID")
    p_cancel.set_defaults(func=cmd_cancel)

    # stop
    p_stop = subparsers.add_parser("stop", help="Immediately stop robot movement")
    p_stop.add_argument("robot_id", default="all", nargs="?", help="Target robot ID or 'all'")
    p_stop.set_defaults(func=cmd_stop)

    # body
    p_body = subparsers.add_parser("body", help="Send body command for legged robots")
    p_body.add_argument("robot_id", help="Target robot ID")
    p_body.add_argument("--action", required=True, choices=["claim", "release", "stand", "sit", "damping", "lie_to_stand", "lock_stand", "walk_mode", "run_mode", "wave", "set_height"], help="Body posture action")
    p_body.add_argument("--height", type=float, default=None, help="Body height in meters")
    p_body.set_defaults(func=cmd_body)

    # snap / snapshot
    p_snap = subparsers.add_parser("snap", help="Capture a live camera snapshot from a robot")
    p_snap.add_argument("robot_id", help="Target robot ID")
    p_snap.add_argument("--save", default=None, help="File path to save the JPEG snapshot")
    def _run_snap(args):
        try:
            from agent.tools.vision import cmd_snapshot
            cmd_snapshot(args)
        except Exception:
            raw_jpeg = _http_get(f"/api/camera/{args.robot_id}", args.server)
            out_path = args.save or f"/app/agent/captures/snapshot_{args.robot_id}_{int(time.time())}.jpg"
            os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
            with open(out_path, "wb") as f:
                f.write(raw_jpeg)
            print(f"Captured snapshot from {args.robot_id} -> {out_path}")
    p_snap.set_defaults(func=_run_snap)

    p_snapshot = subparsers.add_parser("snapshot", help="Alias for snap")
    p_snapshot.add_argument("robot_id", help="Target robot ID")
    p_snapshot.add_argument("--save", default=None, help="File path to save the JPEG snapshot")
    p_snapshot.set_defaults(func=_run_snap)

    # inspect
    p_inspect = subparsers.add_parser("inspect", help="Inspect and analyze an image file")
    p_inspect.add_argument("image_path", help="Path to image file")
    def _run_inspect(args):
        from agent.tools.vision import cmd_inspect
        cmd_inspect(args)
    p_inspect.set_defaults(func=_run_inspect)

    # see
    p_see = subparsers.add_parser("see", help="Inspect what a robot is currently seeing")
    p_see.add_argument("robot_id", help="Target robot ID")
    p_see.set_defaults(func=cmd_see)

    # detections
    p_det = subparsers.add_parser("detections", help="List live detections, proposals, and entities")
    p_det.set_defaults(func=cmd_detections)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
