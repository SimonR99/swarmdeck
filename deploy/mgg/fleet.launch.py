"""MGG sidecar for the existing ARGoS fleet; never starts a second simulator."""

import importlib.util
import math
import os
import sys
from pathlib import Path
import yaml
from launch import LaunchDescription


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
    platforms = robot_types(fleet, count, "robot_")
    lidar = lidar_spec(fleet)
    sensor_overrides = simulation_sensor_overrides(lidar)
    nodes = []
    params = "/opt/mgg/ros2/src/mgg_argos/config/bistro.yaml"
    for i, platform in enumerate(platforms):
        robot = f"robot_{i}"
        spec = robot_spec(platform)
        step_height = spec.max_step_height
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
                # Spot’s elevated camera first sees floor beyond the default
                # 1.2 m edge cap; allow the initial graph to reach it.
                "PlanningParams.edge_length_max": 2.5 if platform == "spot" else 1.2,
                # Explicit Navigate/Home may search a long known-road corridor.
                # Keep the short exploration graph budget unchanged; the
                # objective planner owns this separate bounded deadline.
                "objective_grid_timeout_ms": 2000,
                "BoundedSpaceParams.Global.min_val": [-60.0, -60.0, -3.0],
                "BoundedSpaceParams.Global.max_val": [60.0, 60.0, 3.0],
                "PlanningParams.max_inclination": math.radians(30),
            },
            sim_depth=True,
            camera_offset=[spec.camera_x, spec.camera_z],
        )
    return LaunchDescription(nodes)
