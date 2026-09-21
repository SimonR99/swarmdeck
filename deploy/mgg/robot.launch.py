"""Run one MGG planner/PCI against a continuous robot navigation frame."""

from pathlib import Path
import os
import re
import uuid

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def map_backend_parameters(robot):
    backend = os.environ.get("SWARMDECK_MGG_MAP_BACKEND", "mola_snapshot")
    if backend != "mola_snapshot":
        raise ValueError("SWARMDECK_MGG_MAP_BACKEND must be mola_snapshot")
    if os.environ.get("SWARMDECK_PLANNER_MAP_PROVIDER", "mola") != "mola":
        raise ValueError(
            "MOLA graph planning requires SWARMDECK_PLANNER_MAP_PROVIDER=mola"
        )
    mission = os.environ.get("SWARMDECK_MISSION_ID", "")
    if str(uuid.UUID(mission)) != mission:
        raise ValueError("MOLA graph planning requires a canonical mission UUID")
    if not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_-]*", robot):
        raise ValueError("MOLA graph planning requires a simple robot ID")
    root = Path(os.environ.get("SWARMDECK_MAPS_ROOT", "/maps"))
    if not root.is_absolute():
        raise ValueError("SWARMDECK_MAPS_ROOT must be absolute")
    return {
        "map.backend": backend,
        "map.mola.peer_root": str(root / mission / robot),
        "map.resolution": 0.20,
    }


def robot_nodes(
    robot,
    frame,
    odom,
    tf,
    tf_static,
    params,
    sim_time,
    robot_index=1,
    size=None,
    planner_overrides=None,
):
    template = os.environ.get("SWARMDECK_PLANNING_FRAME_TEMPLATE")
    if template is not None:
        if template.count("{robot}") != 1:
            raise ValueError("Invalid SWARMDECK_PLANNING_FRAME_TEMPLATE")
        frame = template.replace("{robot}", robot).lstrip("/")
        if not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_/-]*", frame):
            raise ValueError("Invalid SWARMDECK_PLANNING_FRAME_TEMPLATE")
    ns = f"{robot}/mgg"
    common = {"use_sim_time": sim_time}
    tf_remaps = [("/tf", tf), ("/tf_static", tf_static)]
    overrides = {
        "PlanningParams.global_frame_id": frame,
        "PlanningParams.robot_id": robot_index,
        "mission_id": os.environ.get("SWARMDECK_MISSION_ID", ""),
        "objective_body_evidence_policy": "observed_ground",
        "objective_ground_evidence_policy": "provisional_unknown",
        "indexed_map_query_service": f"/{robot}/mapping/query_batch",
    }
    overrides.update(planner_overrides or {})
    overrides.update(map_backend_parameters(robot))
    if size:
        overrides["RobotParams.size"] = size
    return [
        Node(
            package="mgg_ros",
            executable="mggplanner_node",
            namespace=ns,
            parameters=[params, common, overrides],
            remappings=tf_remaps + [("odometry", odom)],
        ),
        Node(
            package="mgg_pci",
            executable="mgg_pci_node",
            namespace=ns,
            parameters=[
                common,
                {
                    "world_frame": frame,
                    "bootstrap_distance": 0.0,
                    "external_path_execution": True,
                },
            ],
            remappings=tf_remaps + [("odometry", odom)],
        ),
    ]


def setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    return robot_nodes(
        get("robot"),
        get("navigation_frame"),
        get("odom"),
        get("tf"),
        get("tf_static"),
        get("params"),
        get("use_sim_time") == "true",
    )


def generate_launch_description():
    defaults = {
        "robot": "robot_0",
        "navigation_frame": "odom",
        "odom": "/odom",
        "tf": "/tf",
        "tf_static": "/tf_static",
        "use_sim_time": "false",
    }
    return LaunchDescription(
        [DeclareLaunchArgument("params")]
        + [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
        + [OpaqueFunction(function=setup)]
    )
