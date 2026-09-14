"""Exercise peer camera configuration without requiring a ROS installation."""

import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace


def _action(*args, **kwargs):
    return SimpleNamespace(args=args, **kwargs)


def _load(monkeypatch):
    for name, attributes in {
        "launch": {"LaunchDescription": list},
        "launch.actions": {
            "EmitEvent": _action,
            "RegisterEventHandler": _action,
            "TimerAction": _action,
        },
        "launch.event_handlers": {"OnProcessExit": _action},
        "launch.events": {"Shutdown": _action},
        "launch_ros": {},
        "launch_ros.actions": {"Node": _action},
    }.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).parents[2] / "deploy/autonomy/peer.launch.py"
    spec = importlib.util.spec_from_file_location("peer_launch_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_blank_compose_camera_values_use_namespace_defaults(monkeypatch):
    monkeypatch.setenv("SWARMDECK_MISSION_ID", "6f6afc5c-9a34-4eb4-8243-731629872d25")
    monkeypatch.setenv("SWARMDECK_PEER_NAMES", '["alpha"]')
    monkeypatch.setenv("SWARMDECK_PEER_INDEX", "0")
    monkeypatch.setenv("ROS_DOMAIN_ID", "173")
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
    bridge = next(node for node in nodes if getattr(node, "name", "") == "onboard_mapper")
    params = bridge.parameters[0]

    assert params["color_topic"] == "/alpha/camera/image"
    assert params["depth_topic"] == "/alpha/camera/depth_image"
    assert params["color_info_topic"] == "/alpha/camera/camera_info"
    assert params["color_frame_convention"] == "optical"


def test_simulation_camera_body_convention_is_explicit():
    compose = Path("deploy/compose/docker-compose.peers.yml").read_text()
    assert "SWARMDECK_COLOR_FRAME_CONVENTION: body" in compose
