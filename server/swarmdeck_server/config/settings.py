"""Persistent operator settings with conservative validation."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


DEFAULT_COLORS = ["#007aff", "#8944ab", "#008f87", "#c93400", "#d30f72"]


def defaults() -> dict[str, Any]:
    return {
        "unattended_threshold_s": 45,
        "robot_count": 4,
        "detection_enabled": True,
        "detection_sensitivity": 0.55,
        "robots": [
            {
                "id": f"robot_{index}",
                "enabled": True,
                "type": "ros2",
                "endpoint": "ws://localhost:8080/adapter",
                "color": DEFAULT_COLORS[index % len(DEFAULT_COLORS)],
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
            base["robot_count"] = int(max(1, min(5, int(source.get("robot_count", 4)))))
        except (TypeError, ValueError):
            pass
        base["detection_enabled"] = bool(source.get("detection_enabled", True))
        try:
            base["detection_sensitivity"] = round(
                max(0.1, min(1.0, float(source.get("detection_sensitivity", 0.55)))), 2
            )
        except (TypeError, ValueError):
            pass

        robots = source.get("robots")
        if isinstance(robots, list):
            clean = []
            seen = set()
            for index, item in enumerate(robots[:5]):
                if not isinstance(item, dict):
                    continue
                robot_id = str(item.get("id", f"robot_{index}")).strip()[:48]
                if not robot_id or robot_id in seen:
                    continue
                seen.add(robot_id)
                clean.append({
                    "id": robot_id,
                    "enabled": bool(item.get("enabled", True)),
                    "type": str(item.get("type", "ros2")).strip()[:32] or "ros2",
                    "endpoint": str(item.get("endpoint", "ws://localhost:8080/adapter")).strip()[:256],
                    "color": str(item.get("color", DEFAULT_COLORS[len(clean) % len(DEFAULT_COLORS)])).strip()[:32],
                })
            if clean:
                base["robots"] = clean
                base["robot_count"] = min(base["robot_count"], len(clean))

        # Grow sequential defaults when the requested count exceeds the list.
        while len(base["robots"]) < base["robot_count"]:
            index = len(base["robots"])
            base["robots"].append({
                "id": f"robot_{index}",
                "enabled": True,
                "type": "ros2",
                "endpoint": "ws://localhost:8080/adapter",
                "color": DEFAULT_COLORS[index % len(DEFAULT_COLORS)],
            })
        return base

    def save(self, raw: Any) -> dict[str, Any]:
        self.value = self.validate(raw)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(json.dumps(self.value, indent=2) + "\n")
        temporary.replace(self.path)
        return self.value
