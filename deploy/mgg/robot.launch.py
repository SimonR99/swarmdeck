"""One MGG planner/PCI connected to an existing SwarmDeck robot's ROS topics."""

from pathlib import Path
import os
import re
import uuid
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SIM_MAPPING_MAX_RANGE_M = 20.0


def map_backend_parameters(robot):
    """Select one map owner; MOLA never silently falls back to raw clouds."""
    backend = os.environ.get("SWARMDECK_MGG_MAP_BACKEND", "cloud_octomap")
    if backend not in {"cloud_octomap", "mola_snapshot"}:
        raise ValueError(
            "SWARMDECK_MGG_MAP_BACKEND must be cloud_octomap or mola_snapshot"
        )
    result = {"map.backend": backend}
    if backend == "mola_snapshot":
        if os.environ.get("SWARMDECK_PLANNER_MAP_PROVIDER", "indexed") != "mola":
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
        result["map.mola.peer_root"] = str(root / mission / robot)
        # Native PlannerGridLimits currently publishes 0.20 m cells. Override
        # the legacy Bistro cloud map's 0.15 m setting; the loader verifies it.
        result["map.resolution"] = 0.20
    return result


def robot_nodes(
    robot,
    frame,
    odom,
    cloud,
    tf,
    tf_static,
    params,
    sim_time,
    robot_index=1,
    size=None,
    planner_overrides=None,
    sim_depth=False,
    camera_offset=(0.0, 0.0),
    base_frame="",
    depth_topic="",
    info_topic="",
):
    # Map/UI coordinates can move when local SLAM updates map -> odom. The
    # accumulated cloud and controller route must use the same stable frame.
    template = os.environ.get("SWARMDECK_PLANNING_FRAME_TEMPLATE")
    if template is not None:
        if template.count("{robot}") != 1:
            raise ValueError("Invalid SWARMDECK_PLANNING_FRAME_TEMPLATE")
        frame = template.replace("{robot}", robot).lstrip("/")
        if not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_/-]*", frame):
            raise ValueError("Invalid SWARMDECK_PLANNING_FRAME_TEMPLATE")
    ns = f"{robot}/mgg"
    base_frame = base_frame or (f"{robot}/base_link" if sim_depth else "")
    base_frame_arg = base_frame or "''"
    common = {"use_sim_time": sim_time}
    tf_remaps = [("/tf", tf), ("/tf_static", tf_static)]
    overrides = {
        "PlanningParams.global_frame_id": frame,
        "PlanningParams.robot_id": robot_index,
        "mission_id": os.environ.get("SWARMDECK_MISSION_ID", ""),
    }
    overrides.update(planner_overrides or {})
    overrides.update(map_backend_parameters(robot))
    # Sparse simulated lidar plus a forward camera supplies terrain evidence,
    # not complete free-air coverage around the chassis. Select that contract
    # explicitly; hardware and MOLA keep the strict volumetric policy.
    overrides["objective_body_evidence_policy"] = (
        "observed_ground"
        if sim_depth and overrides["map.backend"] == "cloud_octomap"
        else "strict_volume"
    )
    overrides["objective_ground_evidence_policy"] = (
        "provisional_unknown"
        if sim_depth and overrides["map.backend"] == "cloud_octomap"
        else "observed_ground"
    )
    if sim_depth:
        # ARGoS depth uses a finite 40 m no-return sentinel. Keep native
        # truncation below it so those samples never become occupied endpoints.
        overrides["map.max_range"] = SIM_MAPPING_MAX_RANGE_M
    if overrides["map.backend"] == "mola_snapshot" or os.environ.get(
        "SWARMDECK_INDEXED_MAP_QUERY", "0"
    ).lower() in (
        "1",
        "true",
        "yes",
    ):
        overrides["indexed_map_query_service"] = f"/{robot}/mapping/query_batch"
    if size:
        overrides["RobotParams.size"] = size
    return [
        ExecuteProcess(
            cmd=[
                "python3",
                str(Path(__file__).with_name("inputs.py")),
                "--ros-args",
                "-r",
                f"__ns:=/{ns}",
                "-p",
                f"map_frame:={frame}",
                "-p",
                f"use_sim_time:={str(sim_time).lower()}",
                "-p",
                f"sim_depth:={str(sim_depth).lower()}",
                "-p",
                f"cloud_enabled:={str(overrides['map.backend'] == 'cloud_octomap').lower()}",
                "-p",
                f"depth_enabled:={str(sim_depth or bool(depth_topic and info_topic)).lower()}",
                "-p",
                f"camera_x:={camera_offset[0]}",
                "-p",
                f"camera_z:={camera_offset[1]}",
                "-p",
                f"mapping_max_range:={SIM_MAPPING_MAX_RANGE_M}",
                "-p",
                f"base_frame:={base_frame_arg}",
                "-r",
                f"depth:={depth_topic or f'/{robot}/camera/depth_image'}",
                "-r",
                f"camera_info:={info_topic or f'/{robot}/camera/camera_info'}",
                "-r",
                f"input_odometry:={odom}",
                "-r",
                f"input_cloud:={cloud}",
                "-r",
                f"/tf:={tf}",
                "-r",
                f"/tf_static:={tf_static}",
            ],
            output="screen",
        ),
        Node(
            package="mgg_ros",
            executable="mggplanner_node",
            namespace=ns,
            parameters=[params, common, overrides],
            remappings=tf_remaps
            + [("odometry", "map_odometry"), ("pointcloud", "mapping_cloud")],
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
                    # Nav2 FollowPath owns arrival and controller recovery.
                    "external_path_execution": True,
                },
            ],
            remappings=[("odometry", "map_odometry")],
        ),
    ]


def setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    return robot_nodes(
        get("robot"),
        get("map_frame"),
        get("odom"),
        get("cloud"),
        get("tf"),
        get("tf_static"),
        get("params"),
        get("use_sim_time") == "true",
        base_frame=get("base_frame"),
        depth_topic=get("depth"),
        info_topic=get("camera_info"),
    )


def generate_launch_description():
    defaults = {
        "robot": "robot_0",
        "map_frame": "map",
        "base_frame": "",
        "depth": "",
        "camera_info": "",
        "odom": "/odom",
        "cloud": "/points",
        "tf": "/tf",
        "tf_static": "/tf_static",
        "use_sim_time": "false",
    }
    return LaunchDescription(
        [DeclareLaunchArgument("params")]
        + [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
        + [OpaqueFunction(function=setup)]
    )
