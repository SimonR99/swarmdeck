"""Run one MGG planner/PCI against a continuous robot navigation frame."""

from pathlib import Path
import json
import os
import re
import uuid

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Every planner publishes its global roadmap to one shared topic and reads the
# others' from it, dropping its own (upstream's mgg_argos swarm launch).
ROADMAP_TOPIC = "/mgg/graphs"
ROADMAP_REMAPS = [
    ("neighbour_graph_out", ROADMAP_TOPIC),
    ("neighbour_graph_in", ROADMAP_TOPIC),
]


def map_backend_parameters(robot):
    backend = os.environ.get("SWARMDECK_MGG_MAP_BACKEND", "mola_snapshot")
    if backend != "mola_snapshot":
        raise ValueError("SWARMDECK_MGG_MAP_BACKEND must be mola_snapshot")
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
    peers=(),
    robot_poses="cslam",
):
    """One robot's MGG planner and PCI. With `peers`, the planner shares its
    roadmap with theirs on ROADMAP_TOPIC, placed by robot_poses.py from the
    `robot_poses` source (robot_poses.SOURCES)."""
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
        # Upstream waits 15 cycles without a frontier before it repositions
        # over the global graph, replanning at 10 Hz. PCI's external mode
        # retries an empty cycle with a backoff, so three cycles reach the
        # same decision in seconds instead of a minute of standing still.
        "auto_global_planner_low_gain_rounds": 3,
        # The keyframe map carves free space at keyframes only, so a parked
        # robot's body volume is mostly unobserved until it moves; admit
        # lattice cells over mapped ground whose body volume is partly
        # unknown (known occupied volume still rejects), as the qualified
        # simulation policy did before the ros2 port.
        "allow_unknown_lattice_body": sim_time,
        # Other robots' roadmaps are placed with live inter-robot transforms
        # from robot_poses.py, never the parameter file's static offsets
        # (all zero in bistro.yaml). The offsets name only robot id 0, which
        # no planner has, so an MGG without the topic source merges nothing.
        "neighbour_pose_source": "topic",
        "neighbour_offsets": [0.0, 0.0, 0.0, 0.0],
    }
    overrides.update(planner_overrides or {})
    overrides.update(map_backend_parameters(robot))
    if size:
        overrides["RobotParams.size"] = size
    nodes = [
        Node(
            package="mgg_ros",
            executable="mggplanner_node",
            namespace=ns,
            parameters=[params, common, overrides],
            remappings=tf_remaps + [("odometry", odom)] + ROADMAP_REMAPS,
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
    if peers:
        nodes.append(
            ExecuteProcess(
                cmd=[
                    "python3",
                    str(Path(__file__).with_name("robot_poses.py")),
                    "--robot",
                    robot,
                    "--peers",
                    json.dumps(list(peers)),
                    "--robot-poses",
                    robot_poses,
                ],
                name=f"{robot}_robot_poses",
                output="screen",
            )
        )
    return nodes


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
