"""MGG sidecar for the existing ARGoS fleet; never starts a second simulator."""

import importlib.util
import math
import os
import sys
from pathlib import Path
import yaml
from launch import LaunchDescription


def generate_launch_description():
    here = Path(__file__).parent
    module_spec = importlib.util.spec_from_file_location(
        "mgg_robot_launch", here / "robot.launch.py"
    )
    module = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(module)
    sys.path.insert(0, "/app/swarmdeck_ros/src")
    sys.path.insert(0, "/app/adapters/protocol")
    from swarmdeck_sim.scenario.spawn_fleet import robot_types, robot_spec

    with open(os.environ.get("SWARMDECK_CONFIG", "/app/configs/4robot.yaml")) as stream:
        fleet = yaml.safe_load(stream)["fleet"]
    count = int(os.environ.get("SWARMDECK_ROBOT_COUNT") or fleet.get("robot_count", 4))
    platforms = robot_types(fleet, count, "robot_")
    nodes = []
    params = "/opt/mgg/ros2/src/mgg_argos/config/bistro.yaml"
    for i, platform in enumerate(platforms):
        robot = f"robot_{i}"
        spec = robot_spec(platform)
        step_height = 0.30 if platform == "spot" else 0.10
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
                "BoundedSpaceParams.Global.min_val": [-60.0, -60.0, -3.0],
                "BoundedSpaceParams.Global.max_val": [60.0, 60.0, 3.0],
                "PlanningParams.max_inclination": math.radians(30),
            },
            sim_depth=True,
            camera_offset=[spec.camera_x, spec.camera_z],
        )
    return LaunchDescription(nodes)
