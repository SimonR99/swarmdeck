"""Evaluate launch descriptions without ROS or starting any simulation."""

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock

ROOT = Path(__file__).resolve().parents[2]


class _Action:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.__dict__.update(kwargs)


class _Config:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


def _launch_module(monkeypatch, package, filename):
    # Only the ROS launch objects are stand-ins; execute the real builders.
    modules = {
        "launch": {"LaunchDescription": _Action},
        "launch.actions": dict.fromkeys(
            [
                "DeclareLaunchArgument",
                "ExecuteProcess",
                "IncludeLaunchDescription",
                "OpaqueFunction",
                "TimerAction",
            ],
            _Action,
        ),
        "launch.conditions": dict.fromkeys(["IfCondition", "UnlessCondition"], _Action),
        "launch.substitutions": {
            "LaunchConfiguration": _Config,
            "PathJoinSubstitution": _Action,
        },
        "launch.launch_description_sources": {"PythonLaunchDescriptionSource": _Action},
        "launch_ros": {},
        "launch_ros.actions": dict.fromkeys(
            ["ComposableNodeContainer", "Node"], _Action
        ),
        "launch_ros.descriptions": dict.fromkeys(
            ["ComposableNode", "ParameterFile"], _Action
        ),
        "launch_ros.substitutions": {"FindPackageShare": _Action},
        "nav2_common": {},
        "nav2_common.launch": dict.fromkeys(
            ["ReplaceString", "RewrittenYaml"], _Action
        ),
    }
    for name, attrs in modules.items():
        monkeypatch.setitem(sys.modules, name, SimpleNamespace(**attrs))
    path = ROOT / f"swarmdeck_ros/src/{package}/launch/{filename}.launch.py"
    spec = importlib.util.spec_from_file_location(f"velocity_{filename}", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_session_routes_smoothed_velocity_through_each_adapter(
    sim_module, monkeypatch, tmp_path
):
    session = _launch_module(monkeypatch, "swarmdeck_bringup", "session")
    # Do not generate a world, write session data, or run any launch action.
    monkeypatch.setattr(session, "argos_actions", lambda *args: [])
    monkeypatch.setenv("SWARMDECK_ROBOT_COUNT", "2")
    monkeypatch.syspath_prepend(str(ROOT / "swarmdeck_ros/src/swarmdeck_sim/scenario"))
    cfg = tmp_path / "fleet.yaml"
    cfg.write_text("fleet:\n  robot_count: 2\n  robot_prefix: test_\n")
    context = {
        "config": str(cfg),
        "headless": "true",
        "runtime_dir": str(tmp_path),
        "launch_argos": "false",
        "targets": "0",
        "odometry": "fast_livo2",
    }
    actions = session.setup(context)
    includes = [action.actions[0] for action in actions if hasattr(action, "actions")]
    assert len(includes) == 2
    monkeypatch.setattr(
        "adapters.exploration.configure_exploration", lambda bridge: None
    )
    monkeypatch.setattr(
        "adapters.objective_planning.configure_objective_planning", lambda bridge: None
    )
    monkeypatch.delenv("SWARMDECK_DETECTOR_URL", raising=False)
    for index, include in enumerate(includes):
        arguments = dict(include.launch_arguments)
        ns = f"test_{index}"
        assert arguments["namespace"] == ns
        assert arguments["output_cmd_vel_topic"] == "cmd_vel_adapter"
        node = MagicMock()
        bridge = sim_module.RobotBridge(node, ns, "http://backend")
        subscriptions = {
            call.args[1]: call.args[2]
            for call in node.create_subscription.call_args_list
        }
        assert subscriptions[f"/{ns}/cmd_vel_adapter"] == bridge._on_nav_cmd_vel
        assert node.create_publisher.call_args.args[1] == f"/{ns}/cmd_vel"
        assert f"/{ns}/cmd_vel" not in subscriptions
        assert bridge._nav_execution_enabled is False


def test_nav_remaps_both_composed_and_standalone_smoothers_without_changing_defaults(
    monkeypatch,
):
    nav = _launch_module(monkeypatch, "swarmdeck_nav", "nav")
    entities = nav.generate_launch_description().args[0]
    defaults = {
        action.args[0]: action.default_value
        for action in entities
        if hasattr(action, "default_value")
    }
    assert defaults["output_cmd_vel_topic"] == "cmd_vel"
    assert defaults["controller_cmd_vel_topic"] == "cmd_vel_nav"
    nodes = [action for action in entities if hasattr(action, "name")]
    container = next(node for node in nodes if node.name == "nav_container")
    for collection in (nodes, container.composable_node_descriptions):
        smoother = next(node for node in collection if node.name == "velocity_smoother")
        remaps = dict(smoother.remappings)
        assert (
            remaps["cmd_vel"].perform({"controller_cmd_vel_topic": "cmd_vel_nav"})
            == "cmd_vel_nav"
        )
        assert (
            remaps["cmd_vel_smoothed"].perform(
                {"output_cmd_vel_topic": "cmd_vel_adapter"}
            )
            == "cmd_vel_adapter"
        )
