"""Exercise deployment map selection without requiring a ROS installation."""

import importlib.util
import runpy
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest


@pytest.fixture
def launch_module(monkeypatch):
    def action(*args, **kwargs):
        return SimpleNamespace(args=args, **kwargs)

    for name, attributes in {
        "launch": {"LaunchDescription": list},
        "launch.actions": {
            "DeclareLaunchArgument": action,
            "OpaqueFunction": action,
            "ExecuteProcess": action,
            "EmitEvent": action,
            "RegisterEventHandler": action,
        },
        "launch.substitutions": {"LaunchConfiguration": action},
        "launch.event_handlers": {"OnProcessExit": action},
        "launch.events": {"Shutdown": action},
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
        "SWARMDECK_PLANNING_FRAME_TEMPLATE",
        "SWARMDECK_MGG_ROBOT",
        "SWARMDECK_ROBOT_POSES",
        "SWARMDECK_PEER_NAMES",
    ):
        monkeypatch.delenv(key, raising=False)
    monkeypatch.setenv("SWARMDECK_MISSION_ID", "6f6afc5c-9a34-4eb4-8243-731629872d25")
    path = Path(__file__).parents[2] / "deploy/mgg/robot.launch.py"
    spec = importlib.util.spec_from_file_location("mgg_launch_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_mola_launch_binds_exact_robot_and_mission(launch_module, monkeypatch):
    mission = "6f6afc5c-9a34-4eb4-8243-731629872d25"
    monkeypatch.setenv("SWARMDECK_MGG_MAP_BACKEND", "mola_snapshot")
    monkeypatch.setenv("SWARMDECK_MISSION_ID", mission)
    monkeypatch.setenv("SWARMDECK_MAPS_ROOT", "/custom/maps")
    nodes = launch_module.robot_nodes(
        "robot_2",
        "robot_2/odom",
        "/robot_2/odom",
        "/tf",
        "/tf_static",
        "params.yaml",
        True,
    )
    planner = next(node for node in nodes if getattr(node, "package", "") == "mgg_ros")
    overrides = planner.parameters[-1]
    assert overrides["map.backend"] == "mola_snapshot"
    assert overrides["map.mola.peer_root"] == f"/custom/maps/{mission}/robot_2"
    assert overrides["map.resolution"] == 0.20
    assert overrides["PlanningParams.global_frame_id"] == "robot_2/odom"
    assert "indexed_map_query_service" not in overrides
    assert "objective_body_evidence_policy" not in overrides
    assert "objective_ground_evidence_policy" not in overrides
    assert overrides["auto_global_planner_low_gain_rounds"] == 3
    controller = next(
        node for node in nodes if getattr(node, "package", "") == "mgg_pci"
    )
    assert controller.parameters[-1]["world_frame"] == "robot_2/odom"


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
    assert [params["PlanningParams.edge_length_max"] for params in overrides] == [
        3.0,
        2.0,
        4.0,
    ]
    physics = runpy.run_path(str(repo / "deploy/patches/argos/apply_steps.py"))[
        "STEP_LIMITS"
    ]
    assert [params["PlanningParams.max_step_height"] for params in overrides] == [
        physics[platform] for platform in ("bunker", "scout-mini", "spot")
    ]


def test_mola_initial_edges_reach_the_selected_lidars_first_ground_ring(
    launch_module, monkeypatch, tmp_path
):
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
    monkeypatch.setenv("SWARMDECK_MGG_MAP_BACKEND", "mola_snapshot")
    monkeypatch.setenv("SWARMDECK_MISSION_ID", "6f6afc5c-9a34-4eb4-8243-731629872d25")
    path = repo / "deploy/mgg/fleet.launch.py"
    spec = importlib.util.spec_from_file_location("mgg_mola_reach_launch", path)
    fleet_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fleet_module)

    nodes = fleet_module.generate_launch_description()
    planners = [node for node in nodes if getattr(node, "package", "") == "mgg_ros"]
    overrides = [planner.parameters[-1] for planner in planners]
    expected = [3.0, 2.0, 4.0]
    assert [
        params["PlanningParams.edge_length_max"] for params in overrides
    ] == expected
    pathological_robot = SimpleNamespace(base_height=2.0, lidar_x=0.0, lidar_z=2.0)
    with pytest.raises(ValueError, match="exceeds bounded reach"):
        fleet_module.mola_initial_ground_reach(
            pathological_robot, SimpleNamespace(vfov=0.1)
        )


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
    assert [
        node.parameters[-1]["PlanningParams.global_frame_id"] for node in planners
    ] == ["rover_0/odom", "rover_1/odom"]
    assert planners[1].parameters[-1]["PlanningParams.max_step_height"] == 0.30
    monkeypatch.setenv("SWARMDECK_MGG_ROBOT", "rover_1")
    selected = fleet_module.generate_launch_description()
    assert [
        node.namespace for node in selected if getattr(node, "package", "") == "mgg_ros"
    ] == ["rover_1/mgg"]
    assert [
        node.namespace for node in selected if getattr(node, "package", "") == "mgg_pci"
    ] == ["rover_1/mgg"]


def load_fleet_launch(monkeypatch, tmp_path, name):
    repo = Path(__file__).parents[2]
    monkeypatch.syspath_prepend(str(repo / "adapters/protocol"))
    monkeypatch.syspath_prepend(str(repo / "swarmdeck_ros/src"))
    config = tmp_path / "fleet.yaml"
    config.write_text("""fleet:
  robot_count: 2
  robot_type: bunker
  robot_types:
    robot_1: spot
  lidar:
    profile: vlp16
""")
    monkeypatch.setenv("SWARMDECK_CONFIG", str(config))
    spec = importlib.util.spec_from_file_location(
        name, repo / "deploy/mgg/fleet.launch.py"
    )
    fleet_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fleet_module)
    return fleet_module


def robot_pose_processes(nodes):
    return [node for node in nodes if getattr(node, "name", "").endswith("_poses")]


def test_sim_planners_share_roadmaps_on_one_topic(launch_module, monkeypatch, tmp_path):
    fleet_module = load_fleet_launch(monkeypatch, tmp_path, "mgg_share_launch")
    nodes = fleet_module.generate_launch_description()
    planners = [node for node in nodes if getattr(node, "package", "") == "mgg_ros"]
    for planner in planners:
        assert ("neighbour_graph_out", "/mgg/graphs") in planner.remappings
        assert ("neighbour_graph_in", "/mgg/graphs") in planner.remappings
        assert planner.parameters[-1]["neighbour_pose_source"] == "topic"
        # No static offset names a real (1-based) robot id.
        assert planner.parameters[-1]["neighbour_offsets"] == [0.0, 0.0, 0.0, 0.0]
    # One transform publisher per robot, naming the other robots; cslam is
    # the default source.
    processes = robot_pose_processes(nodes)
    assert [p.name for p in processes] == ["robot_0_robot_poses", "robot_1_robot_poses"]
    command = processes[0].cmd
    assert command[1].endswith("deploy/mgg/robot_poses.py")
    assert command[2:] == [
        "--robot",
        "robot_0",
        "--peers",
        '["robot_1"]',
        "--robot-poses",
        "cslam",
    ]


def test_sim_robot_poses_source_is_selectable(launch_module, monkeypatch, tmp_path):
    fleet_module = load_fleet_launch(monkeypatch, tmp_path, "mgg_truth_launch")
    monkeypatch.setenv("SWARMDECK_ROBOT_POSES", "ground_truth")
    processes = robot_pose_processes(fleet_module.generate_launch_description())
    assert all(p.cmd[-1] == "ground_truth" for p in processes)
    monkeypatch.setenv("SWARMDECK_ROBOT_POSES", "odometry")
    with pytest.raises(ValueError, match="SWARMDECK_ROBOT_POSES"):
        fleet_module.generate_launch_description()


def test_hardware_roadmap_sharing_accepts_only_cslam(launch_module, monkeypatch):
    path = Path(__file__).parents[2] / "deploy/mgg/hardware.launch.py"
    spec = importlib.util.spec_from_file_location("mgg_hardware_peers", path)
    hardware = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(hardware)
    assert hardware.hardware_fleet("botman") == (1, [])
    monkeypatch.setenv("SWARMDECK_PEER_NAMES", '["botman", "aslan"]')
    # Distinct 1-based ids, or the merge drops the other robot as itself.
    assert hardware.hardware_fleet("botman") == (1, ["aslan"])
    assert hardware.hardware_fleet("aslan") == (2, ["botman"])
    monkeypatch.setenv("SWARMDECK_ROBOT_POSES", "ground_truth")
    with pytest.raises(ValueError, match="only cslam"):
        hardware.hardware_fleet("botman")


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

    with pytest.raises(ValueError, match="qualified multi-ring"):
        fleet_module.generate_launch_description()


@pytest.mark.parametrize("template", ["", "odom", "{other}/odom", "{robot}/bad frame"])
def test_invalid_planning_frame_fails_at_launch(launch_module, monkeypatch, template):
    monkeypatch.setenv("SWARMDECK_PLANNING_FRAME_TEMPLATE", template)
    with pytest.raises(ValueError, match="PLANNING_FRAME_TEMPLATE"):
        launch_module.robot_nodes(
            "robot_2",
            "robot_2/odom",
            "/robot_2/odom",
            "/tf",
            "/tf_static",
            "params",
            True,
        )


def test_sim_global_bounds_cover_the_whole_mesh_world_in_odom(
    launch_module, monkeypatch
):
    repo = Path(__file__).parents[2]
    monkeypatch.syspath_prepend(str(repo / "adapters/protocol"))
    monkeypatch.syspath_prepend(str(repo / "swarmdeck_ros/src"))
    path = repo / "deploy/mgg/fleet.launch.py"
    spec = importlib.util.spec_from_file_location("mgg_bounds_launch", path)
    fleet_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fleet_module)

    # The procedural building keeps the fixed box.
    assert fleet_module.global_bounds({"fleet": {}}, "robot_0") == (
        fleet_module.DEFAULT_GLOBAL_BOUNDS
    )
    # SubT: arena 453.5 x 213.5 x 36.5 m centred at (191.9, 10, -1.7), seen
    # from robot_0's spawn at (-14.5, 1.0, 0.15): from the hangar's back wall
    # to the network's far end at x = 411 m, and down to z = -15 m.
    config = {
        "world": "subt_finals",
        "fleet": {},
        "map": {"start_poses": {"robot_0": {"x": -14.5, "y": 1.0, "z": 0.15}}},
    }
    low, high = fleet_module.global_bounds(config, "robot_0")
    # Arena x [-34.85, 418.65], y [-96.75, 116.75], z [-19.95, 16.55].
    assert low == pytest.approx([-34.85 + 14.5, -96.75 - 1.0, -19.95 - 0.15])
    assert high == pytest.approx([418.65 + 14.5, 116.75 - 1.0, 16.55 - 0.15])
    assert high[0] > 411.0 + 14.5 and low[2] < -15.0 - 0.15
