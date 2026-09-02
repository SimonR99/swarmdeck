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
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Any, Optional

DEFAULT_SERVER_URL = os.environ.get("SWARMDECK_SERVER_URL", "http://server:8080").rstrip("/")


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

    print(f"{'ROBOT ID':<15} {'TYPE':<16} {'BATTERY':<10} {'MODE':<10} {'NAV STATUS':<12} {'POSE (X, Y, YAW)'}")
    print("=" * 80)
    for r in robots:
        rid = r.get("robot_id", "unknown")
        rtype = r.get("robot_type", "unknown")
        batt = f"{int(r.get('battery', 0.0) * 100)}%" if r.get("battery") is not None else "N/A"
        mode = r.get("mode", "idle")
        nav_st = r.get("nav_status", "idle")
        pose = r.get("pose") or {}
        px = f"{pose.get('x', 0.0):.2f}"
        py = f"{pose.get('y', 0.0):.2f}"
        pyaw = f"{pose.get('yaw', 0.0):.2f}"
        pose_str = f"({px}, {py}, {pyaw} rad)"
        print(f"{rid:<15} {rtype:<16} {batt:<10} {mode:<10} {nav_st:<12} {pose_str}")


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


def cmd_deploy(args: argparse.Namespace) -> None:
    raw_name = args.robot.lower().replace("@", "")
    base_robot = raw_name.split("_")[0] if "_" in raw_name else raw_name
    print(f"Deploying robot profile '{base_robot}' via 'make deploy ROBOT={base_robot}'...")
    try:
        ret = subprocess.run(["make", "deploy", f"ROBOT={base_robot}"], capture_output=True, text=True, timeout=120)
        if ret.returncode == 0:
            print(f"✓ Successfully deployed and started '{base_robot}'.")
            if ret.stdout:
                lines = [l for l in ret.stdout.strip().split("\n") if l.strip()]
                print("\n".join(lines[-3:]))
        else:
            print(f"⚠️ Deployment encountered an issue:\n{ret.stderr or ret.stdout}")
    except Exception as exc:
        print(f"Deployment command error: {exc}")


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
