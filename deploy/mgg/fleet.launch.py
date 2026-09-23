"""MGG sidecar for the existing ARGoS fleet; never starts a simulator."""

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
# GridGraphLocal resolution in mgg_argos/config/bistro.yaml.
LATTICE_RESOLUTION_M = 0.5
MAX_INITIAL_GROUND_REACH_M = 5.0
# MGG's global space, in each robot's odom frame. It keeps exploration from
# wandering off into open space; it does not cost planning time. The
# procedural building is 26 m across; a mesh world is bounded by its arena.
DEFAULT_GLOBAL_BOUNDS = ([-60.0, -60.0, -3.0], [60.0, 60.0, 3.0])


def simulation_sensor_overrides(lidar):
    if lidar.rings <= 1 or not math.isfinite(lidar.vfov) or lidar.vfov <= 0.0:
        raise ValueError("MGG 3D exploration requires a qualified multi-ring lidar")
    return {
        "SensorParams.VLP16.fov": [2.0 * math.pi, 2.0 * lidar.vfov],
        "SensorParams.VLP16.rotations": [0.0, 0.0, 0.0],
    }


def mola_initial_ground_reach(robot, lidar):
    sensor_height = robot.base_height + robot.lidar_z
    if not all(math.isfinite(v) for v in (sensor_height, lidar.vfov)):
        raise ValueError("MOLA initial ground reach requires finite sensor geometry")
    if sensor_height <= 0.0 or not 0.0 < lidar.vfov < math.pi / 2.0:
        raise ValueError("MOLA initial ground reach requires a downward 3D lidar")
    required = (
        abs(sensor_height / math.tan(lidar.vfov) - abs(robot.lidar_x))
        + MOLA_MAP_RESOLUTION_M
    )
    if required > MAX_INITIAL_GROUND_REACH_M:
        raise ValueError("MOLA lidar ground blind radius exceeds bounded reach")
    return math.ceil(required / LATTICE_RESOLUTION_M) * LATTICE_RESOLUTION_M


def simulation_robot_prefix(fleet):
    prefix = fleet.get("robot_prefix", "robot_")
    if (
        not isinstance(prefix, str)
        or re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", prefix) is None
    ):
        raise ValueError(
            "fleet.robot_prefix must contain only ROS namespace characters"
        )
    return prefix


def global_bounds(config, robot):
    """MGG's global space for `robot`: its world's whole arena, in the robot's
    odom frame (the planner's world frame), which starts at the robot's spawn
    pose. The arena box is moved into that frame and re-boxed around its
    rotated corners."""
    from swarmdeck_sim.scenario.worlds import mesh_world

    world = mesh_world(config)
    if world is None:
        return DEFAULT_GLOBAL_BOUNDS
    size = [float(v) for v in world.arena_size.split(",")]
    center = [float(v) for v in world.arena_center.split(",")]
    starts = (config.get("map") or {}).get("start_poses") or {}
    start = starts.get(robot) or world.default_start_poses.get(robot) or {}
    sx, sy, sz = (float(start.get(k, 0.0)) for k in ("x", "y", "z"))
    yaw = float(start.get("yaw", 0.0))
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    corners = []
    for dx in (-0.5, 0.5):
        for dy in (-0.5, 0.5):
            x = center[0] + dx * size[0] - sx
            y = center[1] + dy * size[1] - sy
            corners.append((cos_y * x + sin_y * y, -sin_y * x + cos_y * y))
    low_z = center[2] - size[2] / 2.0 - sz
    high_z = center[2] + size[2] / 2.0 - sz
    return (
        [min(c[0] for c in corners), min(c[1] for c in corners), low_z],
        [max(c[0] for c in corners), max(c[1] for c in corners), high_z],
    )


def generate_launch_description():
    here = Path(__file__).parent
    spec = importlib.util.spec_from_file_location(
        "mgg_robot_launch", here / "robot.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    sys.path.insert(0, "/app/swarmdeck_ros/src")
    sys.path.insert(0, "/app/adapters/protocol")
    from swarmdeck_sim.scenario.spawn_fleet import lidar_spec, robot_types, robot_spec

    with open(os.environ.get("SWARMDECK_CONFIG", "/app/configs/4robot.yaml")) as stream:
        config = yaml.safe_load(stream)
    fleet = config["fleet"]
    count = int(os.environ.get("SWARMDECK_ROBOT_COUNT") or fleet.get("robot_count", 4))
    prefix = simulation_robot_prefix(fleet)
    platforms = robot_types(fleet, count, prefix)
    lidar = lidar_spec(fleet)
    sensor_overrides = simulation_sensor_overrides(lidar)
    nodes = []
    params = "/opt/mgg/ros2/src/mgg_argos/config/bistro.yaml"
    for i, platform in enumerate(platforms):
        robot = f"{prefix}{i}"
        selected = os.environ.get("SWARMDECK_MGG_ROBOT")
        if selected and robot != selected:
            continue
        spec = robot_spec(platform)
        initial_ground_reach = mola_initial_ground_reach(spec, lidar)
        bounds_min, bounds_max = global_bounds(config, robot)
        nodes += module.robot_nodes(
            robot,
            f"{robot}/odom",
            f"/{robot}/odom",
            f"/{robot}/tf",
            f"/{robot}/tf_static",
            params,
            True,
            i + 1,
            [spec.length, spec.width, 2 * spec.base_height],
            {
                "SensorParams.VLP16.center_offset": [spec.lidar_x, 0.0, spec.lidar_z],
                **sensor_overrides,
                "PlanningParams.max_ground_height": spec.base_height
                + spec.max_step_height
                + 0.175,
                "PlanningParams.max_step_height": spec.max_step_height,
                "PlanningParams.edge_length_max": initial_ground_reach,
                # The lidar's ground blind radius: the root may hang that far
                # from the first supported vertex at a standing start.
                "hanging_root_edge_length_max": initial_ground_reach,
                "BoundedSpaceParams.Global.min_val": bounds_min,
                "BoundedSpaceParams.Global.max_val": bounds_max,
                "PlanningParams.max_inclination": math.radians(30),
            },
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
