"""Persistent operator settings with conservative validation."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from .detection import (
    DETECTION_CLASS_FLOORS,
    DETECTION_CLASS_NAMES,
    capture_floors,
    validate_class_floors,
    validate_classes,
    validate_robot_floors,
)
from ..detect.review import DEFAULT_ASK_RADIUS_M, DEFAULT_SAME_RADIUS_M

MAX_ROBOTS = 16
# How the dashboard draws its manual-drive control.  "arrows" is a four-button
# pad, one direction per button, which is what a gloved finger on a tablet can
# actually hit; "joystick" is the analogue thumbstick.  Keyboard WASD/arrow keys
# work in both, so this only chooses what is on screen.
DRIVE_CONTROL_MODES = ("arrows", "joystick")
DEFAULT_DRIVE_CONTROL_MODE = "arrows"
DEFAULT_COLORS = [
    "#007aff",
    "#8944ab",
    "#008f87",
    "#c93400",
    "#d30f72",
    "#b26a00",
    "#5865f2",
    "#2d8a3f",
]
# Named hardware robots get a stable DEFAULT colour regardless of list index,
# so `aslan_0` is the same orange on every deployment without anyone choosing
# it. This is a default and nothing more: an operator who picks a colour in the
# dialog gets that colour. It used to win over the operator's choice, which made
# the picker silently inert for exactly the three real robots in the fleet.
IDENTITY_COLORS = {
    "spot": "#c9a000",
    "botman": "#007aff",
    "aslan": "#e07000",
    "chris": "#5865f2",
}


def _identity_stem(robot_id: str) -> str:
    if "_" in robot_id and robot_id.rsplit("_", 1)[-1].isdigit():
        return robot_id.rsplit("_", 1)[0]
    return robot_id


def default_color(robot_id: str, index: int) -> str:
    stem = _identity_stem(robot_id)
    if stem in IDENTITY_COLORS:
        return IDENTITY_COLORS[stem]
    return DEFAULT_COLORS[index % len(DEFAULT_COLORS)]


def is_robot_enabled(settings: dict[str, Any], robot_id: str) -> bool:
    """True unless this robot is explicitly switched off in operator settings.

    A robot with no settings row stays enabled: hardware adapters are not
    required to appear in the connections list before they can be driven.
    """
    for robot in settings.get("robots") or []:
        if isinstance(robot, dict) and robot.get("id") == robot_id:
            return bool(robot.get("enabled", True))
    return True


def disabled_robot_ids(settings: dict[str, Any]) -> set[str]:
    return {
        str(robot["id"])
        for robot in settings.get("robots") or []
        if isinstance(robot, dict)
        and robot.get("id")
        and not robot.get("enabled", True)
    }


def defaults() -> dict[str, Any]:
    return {
        "unattended_threshold_s": 45,
        "alert_suppress_s": 30,
        "robot_count": 4,
        "drive_control_mode": DEFAULT_DRIVE_CONTROL_MODE,
        "detection_enabled": True,
        "detection_sensitivity": 0.55,
        "detection_classes": list(DETECTION_CLASS_NAMES),
        # Detection review radii, in metres. Inside `same`, a sighting is the
        # object we already have and is folded in without asking; between the
        # two it is the ambiguous case and the operator is offered the merge.
        # See detect/review.py.
        "detection_same_radius_m": DEFAULT_SAME_RADIUS_M,
        "detection_ask_radius_m": DEFAULT_ASK_RADIUS_M,
        # Fleet-wide display floors, and the sparse per-robot overrides of them.
        # Both are enforced by the backend against stored detections; neither is
        # something a robot obeys.  See config/detection.py.
        "detection_class_floors": dict(DETECTION_CLASS_FLOORS),
        "detection_robot_floors": {},
        # Derived from the two above on every validate(); what the robots are
        # actually asked to capture at.  Never read back from the source dict,
        # so a hand-edited settings.json cannot desynchronise it.
        "detection_capture_floors": dict(DETECTION_CLASS_FLOORS),
        "detection_single_mode": False,
        "detection_single_name": "Target",
        "detection_single_color": "#fbbf24",
        "detection_cross_class_merge": True,
        "robots": [
            {
                "id": f"robot_{index}",
                "enabled": True,
                "color": default_color(f"robot_{index}", index),
            }
            for index in range(4)
        ],
    }


class SettingsStore:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.value = defaults()

    def load(self) -> dict[str, Any]:
        try:
            raw = json.loads(self.path.read_text())
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            raw = {}
        self.value = self.validate(raw)
        return self.value

    def validate(self, raw: Any) -> dict[str, Any]:
        source = raw if isinstance(raw, dict) else {}
        base = defaults()
        try:
            base["unattended_threshold_s"] = int(
                max(10, min(3600, float(source.get("unattended_threshold_s", 45))))
            )
        except (TypeError, ValueError):
            pass
        try:
            base["alert_suppress_s"] = int(
                max(0, min(3600, float(source.get("alert_suppress_s", 30))))
            )
        except (TypeError, ValueError):
            pass
        try:
            base["robot_count"] = int(
                max(1, min(MAX_ROBOTS, int(source.get("robot_count", 4))))
            )
        except (TypeError, ValueError):
            pass
        mode = str(source.get("drive_control_mode", DEFAULT_DRIVE_CONTROL_MODE)).strip()
        if mode in DRIVE_CONTROL_MODES:
            base["drive_control_mode"] = mode
        base["detection_enabled"] = bool(source.get("detection_enabled", True))
        try:
            base["detection_sensitivity"] = round(
                max(0.1, min(1.0, float(source.get("detection_sensitivity", 0.55)))), 2
            )
        except (TypeError, ValueError):
            pass
        # `ask` must stay at or above `same`, or the ambiguous band inverts and
        # every sighting past the fold-in radius looks like a brand new object.
        try:
            same = max(
                0.05,
                min(
                    5.0,
                    float(source.get("detection_same_radius_m", DEFAULT_SAME_RADIUS_M)),
                ),
            )
            ask = max(
                0.05,
                min(
                    10.0,
                    float(source.get("detection_ask_radius_m", DEFAULT_ASK_RADIUS_M)),
                ),
            )
            base["detection_same_radius_m"] = round(same, 2)
            base["detection_ask_radius_m"] = round(max(same, ask), 2)
        except (TypeError, ValueError):
            pass
        base["detection_classes"] = validate_classes(source.get("detection_classes"))
        base["detection_class_floors"] = validate_class_floors(
            source.get("detection_class_floors")
        )
        base["detection_robot_floors"] = validate_robot_floors(
            source.get("detection_robot_floors")
        )
        base["detection_capture_floors"] = capture_floors(base)
        base["detection_single_mode"] = bool(source.get("detection_single_mode", False))
        single_name = str(source.get("detection_single_name", "Target")).strip()[:48]
        base["detection_single_name"] = single_name if single_name else "Target"
        single_color = str(source.get("detection_single_color", "#fbbf24")).strip()[:32]
        base["detection_single_color"] = single_color if single_color else "#fbbf24"
        base["detection_cross_class_merge"] = bool(
            source.get("detection_cross_class_merge", True)
        )

        robots = source.get("robots")
        if isinstance(robots, list):
            clean = []
            seen = set()
            for index, item in enumerate(robots[:MAX_ROBOTS]):
                if not isinstance(item, dict):
                    continue
                robot_id = str(item.get("id", f"robot_{index}")).strip()[:48]
                if not robot_id or robot_id in seen:
                    continue
                seen.add(robot_id)
                color = str(item.get("color") or "").strip()[:32]
                if not color:
                    color = default_color(robot_id, len(clean))
                # `type` and `endpoint` used to be stored here and read by
                # nothing. Adapters dial the backend and declare what they are
                # at `hello`, so both were a second source of truth that could
                # only ever disagree with the robot. Dropped rather than wired.
                clean.append(
                    {
                        "id": robot_id,
                        "enabled": bool(item.get("enabled", True)),
                        "color": color,
                    }
                )
            if clean:
                base["robots"] = clean
                base["robot_count"] = min(base["robot_count"], len(clean))

        # Grow sequential defaults when the requested count exceeds the list.
        while len(base["robots"]) < base["robot_count"]:
            index = len(base["robots"])
            base["robots"].append(
                {
                    "id": f"robot_{index}",
                    "enabled": True,
                    "color": default_color(f"robot_{index}", index),
                }
            )
        return base

    def save(self, raw: Any) -> dict[str, Any]:
        self.value = self.validate(raw)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(json.dumps(self.value, indent=2) + "\n")
        temporary.replace(self.path)
        return self.value
