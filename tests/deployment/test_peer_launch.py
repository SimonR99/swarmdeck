"""Exercise peer camera configuration without requiring a ROS installation."""

import importlib.util
import json
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace


def _action(*args, **kwargs):
    return SimpleNamespace(args=args, **kwargs)


def _load(monkeypatch, relative="deploy/autonomy/peer.launch.py"):
    for name, attributes in {
        "launch": {"LaunchDescription": list},
        "launch.actions": {
            "EmitEvent": _action,
            "RegisterEventHandler": _action,
            "TimerAction": _action,
            "DeclareLaunchArgument": _action,
            "OpaqueFunction": _action,
        },
        "launch.event_handlers": {"OnProcessExit": _action},
        "launch.events": {"Shutdown": _action},
        "launch.substitutions": {
            "LaunchConfiguration": lambda name: SimpleNamespace(
                perform=lambda context: context[name]
            ),
            "PathJoinSubstitution": _action,
        },
        "launch_ros": {},
        "launch_ros.actions": {"Node": _action},
        "launch_ros.substitutions": {"FindPackageShare": _action},
    }.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).parents[2] / relative
    spec = importlib.util.spec_from_file_location("peer_launch_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_blank_compose_camera_values_use_namespace_defaults(monkeypatch, tmp_path):
    monkeypatch.setenv("SWARMDECK_MISSION_ID", "6f6afc5c-9a34-4eb4-8243-731629872d25")
    monkeypatch.setenv("SWARMDECK_PEER_NAMES", '["alpha"]')
    monkeypatch.setenv("SWARMDECK_PEER_INDEX", "0")
    monkeypatch.setenv("ROS_DOMAIN_ID", "173")
    monkeypatch.setenv("SWARMDECK_MAP_STORE", str(tmp_path))
    monkeypatch.delenv("SWARMDECK_SIM_RESET_DIR", raising=False)
    # The optional robot compose file renders unset values as empty strings;
    # launch must still derive the robot namespace and its camera defaults.
    monkeypatch.setenv("SWARMDECK_SENSOR_NAMESPACE", "")
    for key in (
        "SWARMDECK_COLOR_TOPIC",
        "SWARMDECK_DEPTH_TOPIC",
        "SWARMDECK_COLOR_INFO_TOPIC",
        "SWARMDECK_COLOR_FRAME_CONVENTION",
    ):
        monkeypatch.setenv(key, "")

    nodes = _load(monkeypatch).generate_launch_description()
    bridge = next(
        node for node in nodes if getattr(node, "name", "") == "onboard_mapper"
    )
    params = bridge.parameters[0]

    assert params["color_topic"] == "/alpha/camera/image"
    assert params["depth_topic"] == "/alpha/camera/depth_image"
    assert params["color_info_topic"] == "/alpha/camera/camera_info"
    assert params["color_frame_convention"] == "optical"


def test_each_peer_start_claims_fresh_persisted_run(monkeypatch, tmp_path):
    mission = "6f6afc5c-9a34-4eb4-8243-731629872d25"
    monkeypatch.setenv("SWARMDECK_MISSION_ID", mission)
    monkeypatch.setenv("SWARMDECK_PEER_NAMES", '["alpha"]')
    monkeypatch.setenv("SWARMDECK_PEER_INDEX", "0")
    monkeypatch.setenv("ROS_DOMAIN_ID", "173")
    monkeypatch.setenv("SWARMDECK_MAP_STORE", str(tmp_path / "maps"))
    reset = tmp_path / "reset"
    monkeypatch.setenv("SWARMDECK_SIM_RESET_DIR", str(reset))
    module = _load(monkeypatch)

    def start():
        nodes = module.generate_launch_description()
        return next(
            node for node in nodes if getattr(node, "name", "") == "onboard_mapper"
        ).parameters[0]

    first, second = start(), start()
    assert first["map_epoch"] == 0
    assert second["map_epoch"] == 1
    assert first["run_id"] != second["run_id"]
    request = reset / "robots" / "alpha" / "request.json"
    request.parent.mkdir(parents=True)
    request.write_text(
        json.dumps({"mission_id": mission, "robot_id": "alpha", "map_epoch": 7})
    )
    third = start()
    assert third["map_epoch"] == 7
    assert third["mission_id"] == first["mission_id"] == mission
    assert third["run_id"] not in {first["run_id"], second["run_id"]}
    # Replaying an earlier reset request cannot make a crashed frontend reuse
    # its keyframe namespace.
    assert start()["map_epoch"] == 8


