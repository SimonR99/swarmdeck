"""Cortex Fleet Client for SwarmDeck Server Interop."""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from typing import Any, Dict, List, Optional

DEFAULT_SERVER_URL = os.environ.get("SWARMDECK_SERVER_URL", "http://server:8080").rstrip("/")


def query_fleet(server_url: str = DEFAULT_SERVER_URL) -> List[Dict[str, Any]]:
    url = f"{server_url}/api/fleet"
    try:
        req = urllib.request.Request(url, headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=5) as resp:
            data = json.loads(resp.read().decode())
            return data.get("robots", [])
    except Exception as exc:
        return []


def query_detections(server_url: str = DEFAULT_SERVER_URL) -> Dict[str, Any]:
    url = f"{server_url}/api/detections"
    try:
        req = urllib.request.Request(url, headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=5) as resp:
            return json.loads(resp.read().decode())
    except Exception:
        return {"tracks": [], "proposals": [], "entities": [], "ignored": []}


def send_drive(robot_id: str, linear: float, angular: float, duration: float = 0.0, server_url: str = DEFAULT_SERVER_URL) -> Dict[str, Any]:
    url = f"{server_url}/api/robot/{robot_id}/drive"
    payload = json.dumps({"linear": linear, "angular": angular, "duration": duration}).encode()
    req = urllib.request.Request(url, data=payload, headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            return json.loads(resp.read().decode())
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


def send_nav_goal(robot_id: str, x: float, y: float, yaw: float = 0.0, server_url: str = DEFAULT_SERVER_URL) -> Dict[str, Any]:
    url = f"{server_url}/api/robot/{robot_id}/goal"
    payload = json.dumps({"x": x, "y": y, "yaw": yaw}).encode()
    req = urllib.request.Request(url, data=payload, headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            return json.loads(resp.read().decode())
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


def send_stop(robot_id: str = "all", server_url: str = DEFAULT_SERVER_URL) -> Dict[str, Any]:
    url = f"{server_url}/api/robot/{robot_id}/stop"
    req = urllib.request.Request(url, data=b"{}", headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            return json.loads(resp.read().decode())
    except Exception as exc:
        return {"ok": False, "error": str(exc)}
