"""Exercise deployment map selection without requiring a ROS installation."""

import importlib.util
import re
import runpy
import subprocess
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest


def test_mgg_docker_patches_are_well_formed(tmp_path):
    """Catch malformed patch artifacts before the expensive native image build."""
    root = Path(__file__).parents[2]
    dockerfile = (root / "deploy/docker/Dockerfile.mgg").read_text()
    patches = re.findall(r"git apply /tmp/([^\s/]+\.patch)", dockerfile)
    assert patches
    for name in patches:
        result = subprocess.run(
            ["git", "apply", "--numstat", str(root / "deploy/patches" / name)],
            cwd=tmp_path,
            capture_output=True,
            text=True,
            timeout=5,
        )
        assert result.returncode == 0, f"{name}: {result.stderr}"


@pytest.fixture
def launch_module(monkeypatch):
    def action(**kwargs):
        return SimpleNamespace(**kwargs)

    for name, attributes in {
        "launch": {"LaunchDescription": list},
        "launch.actions": {
            "DeclareLaunchArgument": action,
            "OpaqueFunction": action,
            "ExecuteProcess": action,
        },
        "launch.substitutions": {"LaunchConfiguration": action},
        "launch_ros": {},
        "launch_ros.actions": {"Node": action},
    }.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    for key in (
        "SWARMDECK_MGG_MAP_BACKEND",
        "SWARMDECK_MISSION_ID",
        "SWARMDECK_MAPS_ROOT",
        "SWARMDECK_PLANNER_MAP_PROVIDER",
        "SWARMDECK_INDEXED_MAP_QUERY",
        "SWARMDECK_PLANNING_FRAME_TEMPLATE",
    ):
        monkeypatch.delenv(key, raising=False)
    monkeypatch.setenv("SWARMDECK_PLANNER_MAP_PROVIDER", "mola")
    path = Path(__file__).parents[2] / "deploy/mgg/robot.launch.py"
    spec = importlib.util.spec_from_file_location("mgg_launch_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_cloud_backend_needs_no_onboard_mission(launch_module):
    assert launch_module.map_backend_parameters("robot_0") == {
        "map.backend": "cloud_octomap"
    }


def test_mola_launch_binds_exact_robot_and_mission(launch_module, monkeypatch):
    mission = "6f6afc5c-9a34-4eb4-8243-731629872d25"
    monkeypatch.setenv("SWARMDECK_MGG_MAP_BACKEND", "mola_snapshot")
    monkeypatch.setenv("SWARMDECK_MISSION_ID", mission)
    monkeypatch.setenv("SWARMDECK_MAPS_ROOT", "/custom/maps")
    nodes = launch_module.robot_nodes(
        "robot_2",
        "robot_2/map",
        "/odom",
        "/cloud",
        "/tf",
        "/tf_static",
        "params.yaml",
        True,
        sim_depth=True,
    )
    planner = next(node for node in nodes if getattr(node, "package", "") == "mgg_ros")
    overrides = planner.parameters[-1]
    assert overrides["map.backend"] == "mola_snapshot"
    assert overrides["map.mola.peer_root"] == f"/custom/maps/{mission}/robot_2"
    assert overrides["map.resolution"] == 0.20
    assert overrides["PlanningParams.global_frame_id"] == "robot_2/map"
    assert overrides["indexed_map_query_service"] == "/robot_2/mapping/query_batch"
    relay = next(node for node in nodes if hasattr(node, "cmd"))
    assert "cloud_enabled:=false" in relay.cmd
    assert "depth_enabled:=false" in relay.cmd
    # The lightweight static camera transform remains available to peer
    # capture; cloud/depth subscriptions and their processing stay disabled.
    assert "sim_depth:=true" in relay.cmd


@pytest.mark.parametrize(
    "mission", ["", "../old-mission", "6F6AFC5C-9A34-4EB4-8243-731629872D25"]
)
def test_mola_rejects_ambiguous_mission(launch_module, monkeypatch, mission):
    monkeypatch.setenv("SWARMDECK_MGG_MAP_BACKEND", "mola_snapshot")
    monkeypatch.setenv("SWARMDECK_MISSION_ID", mission)
    with pytest.raises(ValueError):
        launch_module.map_backend_parameters("robot_0")


@pytest.mark.parametrize(
    "robot,root", [("../robot_0", "/maps"), ("robot/0", "/maps"), ("robot_0", "maps")]
)
def test_mola_rejects_paths_outside_robot_scope(
    launch_module, monkeypatch, robot, root
):
    monkeypatch.setenv("SWARMDECK_MGG_MAP_BACKEND", "mola_snapshot")
    monkeypatch.setenv("SWARMDECK_MISSION_ID", "6f6afc5c-9a34-4eb4-8243-731629872d25")
    monkeypatch.setenv("SWARMDECK_MAPS_ROOT", root)
    with pytest.raises(ValueError):
        launch_module.map_backend_parameters(robot)


def test_misspelled_backend_does_not_fall_back(launch_module, monkeypatch):
    monkeypatch.setenv("SWARMDECK_MGG_MAP_BACKEND", "mola")
    with pytest.raises(ValueError, match="SWARMDECK_MGG_MAP_BACKEND"):
        launch_module.map_backend_parameters("robot_0")


def test_mola_requires_matching_exact_terrain_provider(launch_module, monkeypatch):
    monkeypatch.setenv("SWARMDECK_MGG_MAP_BACKEND", "mola_snapshot")
    monkeypatch.setenv("SWARMDECK_PLANNER_MAP_PROVIDER", "indexed")
    with pytest.raises(ValueError, match="SWARMDECK_PLANNER_MAP_PROVIDER=mola"):
        launch_module.map_backend_parameters("robot_0")


def test_sim_fleet_models_the_selected_lidar_fov(launch_module, monkeypatch, tmp_path):
    repo = Path(__file__).parents[2]
    monkeypatch.syspath_prepend(str(repo / "adapters/protocol"))
    monkeypatch.syspath_prepend(str(repo / "swarmdeck_ros/src"))
    config = tmp_path / "fleet.yaml"
    config.write_text("""fleet:
  robot_count: 3
  robot_type: bunker
  robot_types:
    robot_1: scout_mini
    robot_2: spot
  lidar:
    profile: vlp16
""")
    monkeypatch.setenv("SWARMDECK_CONFIG", str(config))
    path = repo / "deploy/mgg/fleet.launch.py"
    spec = importlib.util.spec_from_file_location("mgg_fleet_launch_under_test", path)
    fleet_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fleet_module)

    nodes = fleet_module.generate_launch_description()
    planners = [node for node in nodes if getattr(node, "package", "") == "mgg_ros"]
    overrides = [planner.parameters[-1] for planner in planners]
    assert len(overrides) == 3
    for params in overrides:
        assert params["SensorParams.VLP16.fov"] == pytest.approx(
            [2.0 * 3.141592653589793, 2.0 * 0.2618]
        )
        assert params["SensorParams.VLP16.rotations"] == [0.0, 0.0, 0.0]
        assert params["grid_refinement_resolution_m"] == 0.5
        assert params["objective_grid_timeout_ms"] == 4000
        assert params["objective_grid_max_margin_m"] == 8.0
    assert [params["PlanningParams.max_step_height"] for params in overrides] == [
        0.15,
        0.15,
        0.30,
    ]
    physics = runpy.run_path(str(repo / "deploy/patches/argos/apply_steps.py"))[
        "STEP_LIMITS"
    ]
    assert [params["PlanningParams.max_step_height"] for params in overrides] == [
        physics[platform] for platform in ("bunker", "scout-mini", "spot")
    ]


def test_sim_fleet_uses_configured_robot_prefix_for_every_mgg_boundary(
    launch_module, monkeypatch, tmp_path
):
    repo = Path(__file__).parents[2]
    monkeypatch.syspath_prepend(str(repo / "adapters/protocol"))
    monkeypatch.syspath_prepend(str(repo / "swarmdeck_ros/src"))
    config = tmp_path / "fleet.yaml"
    config.write_text("""fleet:
  robot_count: 2
  robot_prefix: rover_
  robot_type: bunker
  robot_types:
    rover_1: spot
  lidar:
    profile: vlp16
""")
    monkeypatch.setenv("SWARMDECK_CONFIG", str(config))
    path = repo / "deploy/mgg/fleet.launch.py"
    spec = importlib.util.spec_from_file_location("mgg_custom_prefix_launch", path)
    fleet_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fleet_module)

    nodes = fleet_module.generate_launch_description()
    planners = [node for node in nodes if getattr(node, "package", "") == "mgg_ros"]
    controllers = [node for node in nodes if getattr(node, "package", "") == "mgg_pci"]
    relays = [node for node in nodes if hasattr(node, "cmd")]

    assert [node.namespace for node in planners] == ["rover_0/mgg", "rover_1/mgg"]
    assert [node.namespace for node in controllers] == [
        "rover_0/mgg",
        "rover_1/mgg",
    ]
    assert [
        node.parameters[-1]["PlanningParams.global_frame_id"] for node in planners
    ] == [
        "rover_0/map_frame",
        "rover_1/map_frame",
    ]
    assert planners[1].parameters[-1]["PlanningParams.max_step_height"] == 0.30
    assert "__ns:=/rover_0/mgg" in relays[0].cmd
    assert "input_odometry:=/rover_0/odom" in relays[0].cmd
    assert "input_cloud:=/rover_0/scan/points" in relays[0].cmd
    assert "__ns:=/rover_1/mgg" in relays[1].cmd
    assert "input_odometry:=/rover_1/odom" in relays[1].cmd
    assert "input_cloud:=/rover_1/scan/points" in relays[1].cmd
    assert "robot_" not in " ".join(str(item) for relay in relays for item in relay.cmd)


def test_sim_fleet_rejects_robot_prefix_that_is_not_a_ros_namespace(
    launch_module, monkeypatch, tmp_path
):
    repo = Path(__file__).parents[2]
    monkeypatch.syspath_prepend(str(repo / "adapters/protocol"))
    monkeypatch.syspath_prepend(str(repo / "swarmdeck_ros/src"))
    config = tmp_path / "fleet.yaml"
    config.write_text("""fleet:
  robot_count: 1
  robot_prefix: bad-name-
  robot_type: bunker
  lidar:
    profile: vlp16
""")
    monkeypatch.setenv("SWARMDECK_CONFIG", str(config))
    path = repo / "deploy/mgg/fleet.launch.py"
    spec = importlib.util.spec_from_file_location("mgg_invalid_prefix_launch", path)
    fleet_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fleet_module)

    with pytest.raises(ValueError, match="fleet.robot_prefix"):
        fleet_module.generate_launch_description()


def test_sim_objective_budget_does_not_change_hardware_defaults(
    launch_module, monkeypatch
):
    repo = Path(__file__).parents[2]
    path = repo / "deploy/mgg/hardware.launch.py"
    spec = importlib.util.spec_from_file_location(
        "mgg_hardware_launch_under_test", path
    )
    hardware = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(hardware)
    settings = hardware.hardware_settings(
        {
            "footprint": [[-0.4, -0.3], [0.4, 0.3]],
            "exploration": {
                "planner": {
                    "body_height_m": 0.5,
                    "bounds_min": [-10.0, -10.0, -2.0],
                    "bounds_max": [10.0, 10.0, 2.0],
                }
            },
        }
    )
    assert "objective_grid_timeout_ms" not in settings
    assert "grid_refinement_resolution_m" not in settings


@pytest.mark.parametrize("profile", ["generic_2d", "legacy_360"])
def test_sim_mgg_rejects_planar_lidar_gain_models(
    launch_module, monkeypatch, tmp_path, profile
):
    repo = Path(__file__).parents[2]
    monkeypatch.syspath_prepend(str(repo / "adapters/protocol"))
    monkeypatch.syspath_prepend(str(repo / "swarmdeck_ros/src"))
    config = tmp_path / "fleet.yaml"
    config.write_text(f"""fleet:
  robot_count: 1
  robot_type: bunker
  lidar:
    profile: {profile}
""")
    monkeypatch.setenv("SWARMDECK_CONFIG", str(config))
    path = repo / "deploy/mgg/fleet.launch.py"
    spec = importlib.util.spec_from_file_location(
        "mgg_planar_fleet_launch_under_test", path
    )
    fleet_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fleet_module)

    with pytest.raises(ValueError, match="requires a multi-ring"):
        fleet_module.generate_launch_description()


def test_stable_frame_is_shared_by_cloud_odometry_planner_and_controller(
    launch_module, monkeypatch
):
    monkeypatch.setenv("SWARMDECK_PLANNING_FRAME_TEMPLATE", "{robot}/odom")
    nodes = launch_module.robot_nodes(
        "robot_2",
        "robot_2/map",
        "/odom",
        "/cloud",
        "/tf",
        "/tf_static",
        "params.yaml",
        True,
        sim_depth=True,
    )
    relay = next(node for node in nodes if hasattr(node, "cmd"))
    assert "map_frame:=robot_2/odom" in relay.cmd
    assert "input_cloud:=/cloud" in relay.cmd
    planner = next(node for node in nodes if getattr(node, "package", "") == "mgg_ros")
    controller = next(
        node for node in nodes if getattr(node, "package", "") == "mgg_pci"
    )
    assert planner.parameters[-1]["PlanningParams.global_frame_id"] == "robot_2/odom"
    assert controller.parameters[-1]["world_frame"] == "robot_2/odom"
    assert planner.parameters[-1]["map.max_range"] == 20.0
    assert "mapping_max_range:=20.0" in relay.cmd


@pytest.mark.parametrize("template", ["", "odom", "{other}/odom", "{robot}/bad frame"])
def test_invalid_planning_frame_fails_at_launch(launch_module, monkeypatch, template):
    monkeypatch.setenv("SWARMDECK_PLANNING_FRAME_TEMPLATE", template)
    with pytest.raises(ValueError, match="PLANNING_FRAME_TEMPLATE"):
        launch_module.robot_nodes(
            "robot_2",
            "robot_2/map",
            "/odom",
            "/cloud",
            "/tf",
            "/tf_static",
            "params",
            True,
        )


@pytest.mark.parametrize(
    "sim_depth,backend,expected",
    [
        (True, "cloud_octomap", "observed_ground"),
        (False, "cloud_octomap", "strict_volume"),
        (True, "mola_snapshot", "observed_ground"),
        (False, "mola_snapshot", "strict_volume"),
    ],
)
def test_ground_evidence_policy_requires_simulation_map(
    launch_module, monkeypatch, sim_depth, backend, expected
):
    monkeypatch.setenv("SWARMDECK_MGG_MAP_BACKEND", backend)
    monkeypatch.setenv("SWARMDECK_MISSION_ID", "6f6afc5c-9a34-4eb4-8243-731629872d25")
    nodes = launch_module.robot_nodes(
        "robot_2",
        "robot_2/map",
        "/odom",
        "/cloud",
        "/tf",
        "/tf_static",
        "params.yaml",
        True,
        sim_depth=sim_depth,
        planner_overrides={"objective_body_evidence_policy": "observed_ground"},
    )
    planner = next(node for node in nodes if getattr(node, "package", "") == "mgg_ros")
    assert planner.parameters[-1]["objective_body_evidence_policy"] == expected
    assert planner.parameters[-1]["objective_ground_evidence_policy"] == (
        "provisional_unknown" if sim_depth else "observed_ground"
    )
