"""Swarm-SLAM (MISTLab `cslam`) for one SwarmDeck robot.

    ros2 launch swarmdeck_cslam cslam.launch.py namespace:=robot_0 robot_id:=0 \\
         max_nb_robots:=4

**Status: experimental.** The image builds on Jazzy against apt GTSAM 4.2 and
the Gazebo fleet has produced geometrically verified inter-robot closures.
RTAB-Map grids and cslam trajectories still disagree, so cslam transforms are
not ready for physical navigation. See docs/archive/collaborative-slam.md.

What this is for. Everything upstream of it — SLAM Toolbox, RTAB-Map, the grid
merge — leaves each robot with a private pose graph, so no robot's drift is ever
corrected by another's observations. cslam adds the
missing capability: robot A recognises a place robot B has been, that becomes a
constraint between the two graphs, and optimising them jointly corrects BOTH
robots while yielding the relative transform as a by-product. Grid registration
only ever recovered the transform, and only after each robot had finished being
wrong.

Two SwarmDeck properties make the integration cheap:

* cslam wants **time-synchronised odometry and a point cloud** per robot, plus
  integer robot ids from 0. After `slam_backend:=rtabmap` the fleet publishes
  exactly that, and robots are already named `robot_0 ... robot_N`.
* The backend must import no ROS (architecture principle 1). cslam runs entirely
  in the ROS 2 domain — on the robot itself in a real deployment — and the
  adapter then declares `coordinate_frame: merged`, which the protocol already
  supports. `mapsvc` switches to `merge_mode: cslam` and becomes bookkeeping.

Inter-robot communication is Zenoh (`ros-jazzy-rmw-zenoh-cpp`), matching
upstream's recommendation. On hardware, set RMW_IMPLEMENTATION=rmw_zenoh_cpp on
every robot and run a router reachable by all of them; a shared DDS domain works
on one machine but does not survive a real network.
"""

import os
from pathlib import Path
from uuid import UUID

from autonomy.map_epochs import claim_map_epoch
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    robot_id = int(get("robot_id"))
    count = int(get("max_nb_robots"))
    namespace = get("namespace")
    if not 0 <= robot_id < count or namespace != f"r{robot_id}":
        raise ValueError(
            "cslam requires namespace r<robot_id> and contiguous fleet indices"
        )
    robot = get("sensor_namespace").strip("/")
    mission = get("mission_id")
    if str(UUID(mission)) != mission:
        raise ValueError("mission_id must be the canonical fleet mission UUID")
    root = Path(get("map_store"))
    epoch = claim_map_epoch(root, mission, robot)
    config = PathJoinSubstitution(
        [FindPackageShare("swarmdeck_cslam"), "config", "cslam_lidar.yaml"]
    )
    common = [
        config,
        {
            "use_sim_time": get("use_sim_time") == "true",
            "robot_id": robot_id,
            "max_nb_robots": count,
            "swarmdeck.mission_id": mission,
            "swarmdeck.map_epoch": epoch,
            "swarmdeck.epoch_state_path": str(root / mission / robot / "peer-epochs"),
            "frontend.odom_topic": f"/{robot}/odom_icp",
            "frontend.pointcloud_topic": f"/{robot}/scan/points",
        },
    ]
    # Native inter-robot topic names hard-code r<index>; sensor names remain
    # absolute robot names. A frontend exit terminates the launch, never respawns
    # an individual producer with a reused keyframe sequence.
    nodes = [
        Node(
            package="cslam",
            executable=executable,
            name=name,
            namespace=namespace,
            parameters=common,
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            output="screen",
        )
        for executable, name in (
            ("lidar_handler_node.py", "cslam_map_manager"),
            ("loop_closure_detection_node.py", "cslam_loop_closure_detection"),
            ("pose_graph_manager", "cslam_pose_graph_manager"),
        )
    ]
    exits = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason="frontend exited; fresh map epoch required"
                        )
                    )
                ],
            )
        )
        for node in nodes
    ]
    return [*exits, *nodes]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="r0"),
            DeclareLaunchArgument("sensor_namespace", default_value="robot_0"),
            DeclareLaunchArgument("robot_id", default_value="0"),
            DeclareLaunchArgument("max_nb_robots", default_value="4"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "mission_id", default_value=os.environ.get("SWARMDECK_MISSION_ID", "")
            ),
            DeclareLaunchArgument(
                "map_store",
                default_value=os.environ.get("SWARMDECK_MAP_STORE", "/maps"),
            ),
            OpaqueFunction(function=setup),
        ]
    )
