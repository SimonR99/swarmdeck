from __future__ import annotations

import json

from swarmdeck_server.config.detection import DETECTION_CLASS_FLOORS, DETECTION_CLASS_NAMES
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


def test_detection_classes_are_filtered_to_the_catalog(tmp_path):
    saved = SettingsStore(tmp_path / "settings.json").save({
        "detection_classes": ["pool_noodle", "not_a_class", "rubber_duck"],
    })

    # Catalog order, not the order they were sent in: the adapters and the
    # dashboard both render this list, and neither should reorder on a save.
    assert saved["detection_classes"] == ["rubber_duck", "pool_noodle"]


def test_an_empty_class_selection_means_every_class(tmp_path):
    """Detection is switched off with detection_enabled, not by emptying this.

    An empty list surviving to the adapters would be a second off switch that
    nothing in the dashboard displays.
    """
    store = SettingsStore(tmp_path / "settings.json")

    assert store.save({"detection_classes": []})["detection_classes"] == DETECTION_CLASS_NAMES
    assert store.save({"detection_classes": "duck"})["detection_classes"] == DETECTION_CLASS_NAMES
    assert store.save({})["detection_classes"] == DETECTION_CLASS_NAMES


def test_class_floors_default_to_the_catalog_and_clamp(tmp_path):
    store = SettingsStore(tmp_path / "settings.json")

    assert store.save({})["detection_class_floors"] == DETECTION_CLASS_FLOORS

    saved = store.save({
        "detection_class_floors": {
            "rubber_duck": 0.40,
            "wooden_block": 1.5,
            "not_a_class": 0.90,
            "disc_cone": "nope",
        }
    })
    floors = saved["detection_class_floors"]

    assert floors["rubber_duck"] == 0.40
    assert floors["wooden_block"] == 0.95
    assert floors["disc_cone"] == DETECTION_CLASS_FLOORS["disc_cone"]
    assert "not_a_class" not in floors
    assert set(floors) == set(DETECTION_CLASS_FLOORS)


def test_settings_accept_seven_robots_for_mixed_hardware_fleet(tmp_path):
    robots = [
        {"id": f"robot_{index}", "type": "sim", "endpoint": "ws://host/adapter"}
        for index in range(4)
    ] + [
        {"id": "tars_0", "type": "ros1", "endpoint": "ws://host/adapter"},
        {"id": "botman_0", "type": "ros2", "endpoint": "ws://host/adapter"},
        {"id": "aslan_0", "type": "ros2", "endpoint": "ws://host/adapter"},
    ]

    saved = SettingsStore(tmp_path / "settings.json").save({
        "robot_count": 7,
        "robots": robots,
    })

    assert saved["robot_count"] == 7
    assert [robot["id"] for robot in saved["robots"]] == [
        "robot_0", "robot_1", "robot_2", "robot_3", "tars_0", "botman_0",
        "aslan_0",
    ]
    assert saved["robots"][5]["color"] == "#007aff"
    assert saved["robots"][6]["color"] == "#e07000"


def test_named_hardware_robots_keep_identity_colors(tmp_path):
    saved = SettingsStore(tmp_path / "settings.json").save({
        "robot_count": 3,
        "robots": [
            {"id": "spot_0", "type": "spot", "endpoint": "ws://host/adapter", "color": "#ff0000"},
            {"id": "botman_0", "type": "ros2", "endpoint": "ws://host/adapter"},
            {"id": "aslan_0", "type": "ros2", "endpoint": "ws://host/adapter"},
        ],
    })

    assert [robot["color"] for robot in saved["robots"]] == [
        "#c9a000",
        "#007aff",
        "#e07000",
    ]


def test_named_robots_are_enabled_unless_explicitly_switched_off():
    from swarmdeck_server.config.settings import disabled_robot_ids, is_robot_enabled

    settings = {
        "robots": [
            {"id": "botman_0", "enabled": False},
            {"id": "aslan_0", "enabled": True},
        ]
    }
    assert is_robot_enabled(settings, "botman_0") is False
    assert is_robot_enabled(settings, "aslan_0") is True
    assert is_robot_enabled(settings, "spot_0") is True
    assert disabled_robot_ids(settings) == {"botman_0"}
