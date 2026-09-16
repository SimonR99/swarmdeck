"""adapter_sim must not inherit a hardware dashboard's robot_count."""

import io
import json
from types import SimpleNamespace as NS
from unittest.mock import AsyncMock, Mock

import pytest

from adapters.exploration import planner_path


def test_cli_wins_over_env_and_yaml(sim_module):
    assert sim_module.resolve_sim_robot_count(2, "4", 5) == 2


def test_env_wins_over_yaml_when_cli_is_unset(sim_module):
    assert sim_module.resolve_sim_robot_count(None, "2", 7) == 2


def test_yaml_fleet_size_is_used_when_nothing_overrides_it(sim_module):
    assert sim_module.resolve_sim_robot_count(None, "", 2) == 2


def test_dashboard_seven_cannot_leak_in_as_the_default(sim_module):
    """settings.json robot_count=7 is a hardware session; sim defaults to 4."""
    assert sim_module.resolve_sim_robot_count(None, "", None) == 4


def test_count_is_clamped_to_the_spawnable_fleet(sim_module):
    assert sim_module.resolve_sim_robot_count(9, "", None) == 5
    assert sim_module.resolve_sim_robot_count(0, "", None) == 1


def test_hello_matches_the_hardware_protocol_envelope(sim_module):
    from adapters.runtime import PROTOCOL_VERSION, TRANSPORT_DEFAULTS

    bridge = sim_module.RobotBridge.__new__(sim_module.RobotBridge)
    bridge.id = "robot_0"
    bridge.cfg = sim_module.deep_merge(
        TRANSPORT_DEFAULTS,
        {
            "robot_type": "agilex_bunker",
            "ros_distro": "jazzy",
            "footprint_radius": 0.643,
        },
    )
    msg = bridge.hello()
    assert msg["protocol"] == PROTOCOL_VERSION
    assert msg["adapter"] == "adapter_sim/0.1.0"
    assert "reset" in msg["capabilities"]
    assert "battery" not in msg["capabilities"]


def test_objective_capability_requires_a_configured_planner(sim_module):
    bridge = sim_module.RobotBridge.__new__(sim_module.RobotBridge)
    bridge.exploration = None
    bridge.objective_planner = None
    assert "plan_objective" not in bridge.capabilities()
    bridge.objective_planner = object()
    assert "plan_objective" in bridge.capabilities()


def test_sim_controller_accepts_platform_steps_for_explore_and_navigation(
    sim_module, monkeypatch
):
    platforms = ["bunker", "scout_mini", "spot"]
    fleet = {
        "robot_count": len(platforms),
        "robot_types": {f"robot_{i}": value for i, value in enumerate(platforms)},
    }
    monkeypatch.setattr(sim_module.sys, "argv", ["adapter_sim"])
    monkeypatch.delenv("SWARMDECK_ROBOT_COUNT", raising=False)
    monkeypatch.setattr(
        sim_module.urllib.request,
        "urlopen",
        lambda *args, **kwargs: io.BytesIO(
            json.dumps({"config": {"fleet": fleet}}).encode()
        ),
    )
    factory = Mock()
    monkeypatch.setattr(sim_module, "RobotBridge", factory)
    monkeypatch.setattr(sim_module, "main_async", AsyncMock())
    monkeypatch.setattr(sim_module.threading, "Thread", Mock())
    sim_module.main()

    for call, platform in zip(factory.call_args_list, platforms, strict=True):
        step = sim_module.robot_spec(platform).max_step_height
        for key in ("exploration_config", "planning_config"):
            config = call.kwargs[key]
            limits = {
                key: config[key]
                for key in ("planar_tolerance_m", "max_inclination_rad")
            }
            points = [
                NS(
                    pose=NS(
                        position=NS(x=x, y=0.0, z=z),
                        orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0),
                    )
                )
                for x, z in ((0.0, 0.0), (0.125, step - 0.008))
            ]
            route = NS(
                header=NS(frame_id="map", stamp=NS(sec=1, nanosec=0)),
                poses=points,
            )
            assert len(planner_path(route, "map", **limits).poses) == 2
            points[-1].pose.position.z = step + 0.01
            with pytest.raises(ValueError, match="cannot traverse"):
                planner_path(route, "map", **limits)
