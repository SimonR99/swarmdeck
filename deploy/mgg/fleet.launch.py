"""MGG sidecar for the existing ARGoS fleet; never starts a second simulator."""

import importlib.util
import math
import os
import re
import sys
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

MOLA_MAP_RESOLUTION_M = 0.2
GRID_REFINEMENT_RESOLUTION_M = 0.5
MAX_INITIAL_GROUND_REACH_M = 5.0


def simulation_sensor_overrides(lidar):
    """MGG gain model geometry for the selected simulated lidar profile."""
    if lidar.rings <= 1 or not math.isfinite(lidar.vfov) or lidar.vfov <= 0.0:
        raise ValueError(
            "MGG 3D exploration requires a multi-ring simulated lidar with "
            "positive vertical FOV; select a qualified 3D lidar profile"
        )
    return {
        # SensorParams.fov contains total symmetric widths; LidarSpec.vfov is
        # the half-angle passed to the ARGoS photorealistic lidar.
        "SensorParams.VLP16.fov": [2.0 * math.pi, 2.0 * lidar.vfov],
        "SensorParams.VLP16.rotations": [0.0, 0.0, 0.0],
    }


def mola_initial_ground_reach(robot, lidar):
    """Bound the first graph edge so it can reach a measured lidar floor."""

    values = (
        robot.base_height,
        robot.lidar_x,
        robot.lidar_z,
        lidar.vfov,
        MOLA_MAP_RESOLUTION_M,
        GRID_REFINEMENT_RESOLUTION_M,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("MOLA initial ground reach requires finite sensor geometry")
    sensor_height = robot.base_height + robot.lidar_z
    if sensor_height <= 0.0 or not 0.0 < lidar.vfov < math.pi / 2.0:
        raise ValueError("MOLA initial ground reach requires a downward 3D lidar")

    horizontal_floor_range = sensor_height / math.tan(lidar.vfov)
    nearest_floor_radius = abs(horizontal_floor_range - abs(robot.lidar_x))
    required = nearest_floor_radius + MOLA_MAP_RESOLUTION_M
    if not math.isfinite(required) or required > MAX_INITIAL_GROUND_REACH_M:
        raise ValueError(
            "MOLA lidar ground blind radius exceeds the bounded initial reach"
        )
    reach = (
        math.ceil(required / GRID_REFINEMENT_RESOLUTION_M)
        * GRID_REFINEMENT_RESOLUTION_M
    )
    if reach > MAX_INITIAL_GROUND_REACH_M:
        raise ValueError(
            "MOLA lidar ground blind radius exceeds the bounded initial reach"
        )
    return reach


def simulation_robot_prefix(fleet):
    prefix = fleet.get("robot_prefix", "robot_")
    if (
        not isinstance(prefix, str)
        or re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", prefix) is None
    ):
        raise ValueError(
            "fleet.robot_prefix must contain only ROS namespace letters, digits "
            "and underscores, and may not start with a digit"
        )
    return prefix


def generate_launch_description():
    here = Path(__file__).parent
    module_spec = importlib.util.spec_from_file_location(
        "mgg_robot_launch", here / "robot.launch.py"
    )
    module = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(module)
    sys.path.insert(0, "/app/swarmdeck_ros/src")
    sys.path.insert(0, "/app/adapters/protocol")
    from swarmdeck_sim.scenario.spawn_fleet import lidar_spec, robot_types, robot_spec

    with open(os.environ.get("SWARMDECK_CONFIG", "/app/configs/4robot.yaml")) as stream:
        fleet = yaml.safe_load(stream)["fleet"]
    count = int(os.environ.get("SWARMDECK_ROBOT_COUNT") or fleet.get("robot_count", 4))
    prefix = simulation_robot_prefix(fleet)
    platforms = robot_types(fleet, count, prefix)
    lidar = lidar_spec(fleet)
    sensor_overrides = simulation_sensor_overrides(lidar)
    mola_snapshot = (
        os.environ.get("SWARMDECK_MGG_MAP_BACKEND", "cloud_octomap") == "mola_snapshot"
    )
    nodes = []
    params = "/opt/mgg/ros2/src/mgg_argos/config/bistro.yaml"
    for i, platform in enumerate(platforms):
        robot = f"{prefix}{i}"
        selected = os.environ.get("SWARMDECK_MGG_ROBOT")
        if selected and robot != selected:
            continue
        spec = robot_spec(platform)
        step_height = spec.max_step_height
        legacy_edge_length_max = 2.5 if platform == "spot" else 1.2
        initial_ground_reach = (
            mola_initial_ground_reach(spec, lidar)
            if mola_snapshot
            else legacy_edge_length_max
        )
        initial_ground_overrides = (
            {"objective_start_support_max_distance_m": initial_ground_reach}
            if mola_snapshot
            else {}
        )
        nodes += module.robot_nodes(
            robot,
            f"{robot}/map_frame",
            f"/{robot}/odom",
            f"/{robot}/scan/points",
            f"/{robot}/tf",
            f"/{robot}/tf_static",
            params,
            True,
            i + 1,
            [spec.length, spec.width, 2 * spec.base_height],
            {
                "SensorParams.VLP16.center_offset": [spec.lidar_x, 0.0, spec.lidar_z],
                **sensor_overrides,
                # Keep the extended body box above climbable terrain and the
                # occupied floor voxel; max_ground_height is a box offset.
                "PlanningParams.max_ground_height": spec.base_height
                + step_height
                + 0.15
                + 0.025,
                "PlanningParams.max_step_height": step_height,
                # MOLA has only qualified lidar geometry. Its initial edge must
                # cross the sensor's ground blind radius to measured support.
                # Cloud mode retains the camera-aware legacy reach.
                "PlanningParams.edge_length_max": initial_ground_reach,
                **initial_ground_overrides,
                # Half-metre search nodes keep long road detours tractable.
                # Terrain projection/sweeps and Nav2 retain their own finer
                # resolution; this does not relax obstacle or step checks.
                "grid_refinement_resolution_m": GRID_REFINEMENT_RESOLUTION_M,
                # Navigate/Home have a separate deadline from exploration.
                # Leave headroom for live map queries on long Bistro detours.
                "objective_grid_timeout_ms": 4000,
                # A real diagonal Bistro kerb seals the old +/-4 m objective
                # window. Permit the native planner to use a wider window
                # when its existing cell budget can represent it.
                "objective_grid_max_margin_m": 8.0,
                "BoundedSpaceParams.Global.min_val": [-60.0, -60.0, -3.0],
                "BoundedSpaceParams.Global.max_val": [60.0, 60.0, 3.0],
                "PlanningParams.max_inclination": math.radians(30),
            },
            sim_depth=True,
            camera_offset=[spec.camera_x, spec.camera_z],
        )
    if not nodes:
        raise ValueError("SWARMDECK_MGG_ROBOT is not in the simulation fleet")
    exits = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=[
                    EmitEvent(event=Shutdown(reason="robot planner child exited"))
                ],
            )
        )
        for node in nodes
    ]
    return LaunchDescription([*exits, *nodes])
