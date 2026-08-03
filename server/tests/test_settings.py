from __future__ import annotations

import json

from swarmdeck_server.config.settings import SettingsStore


def test_settings_round_trip_and_validation(tmp_path):
    path = tmp_path / "settings.json"
    store = SettingsStore(path)

    saved = store.save({
        "unattended_threshold_s": 2,
        "alert_suppress_s": -5,
        "robot_count": 9,
        "detection_enabled": False,
        "detection_sensitivity": 1.5,
        "robots": [
            {"id": "alpha", "enabled": True, "type": "ros2", "endpoint": "ws://host/adapter", "color": "#ff0000"},
            {"id": "alpha", "enabled": True, "type": "ros2", "endpoint": "duplicate"},
            {"id": "beta", "enabled": False, "type": "spot", "endpoint": "ws://spot/adapter"},
        ],
    })

    assert saved["unattended_threshold_s"] == 10
    assert saved["alert_suppress_s"] == 0
    assert saved["robot_count"] == 2
    assert saved["detection_enabled"] is False
    assert saved["detection_sensitivity"] == 1.0
    assert [robot["id"] for robot in saved["robots"]] == ["alpha", "beta"]
    assert saved["robots"][0]["color"] == "#ff0000"
    assert saved["robots"][1]["color"] == "#8944ab"
    assert json.loads(path.read_text()) == saved
    assert SettingsStore(path).load() == saved


def test_invalid_settings_fall_back_without_throwing(tmp_path):
    path = tmp_path / "settings.json"
    path.write_text("not json")
    loaded = SettingsStore(path).load()
    assert loaded["robot_count"] == 4
    assert loaded["detection_enabled"] is True
    assert len(loaded["robots"]) == 4
