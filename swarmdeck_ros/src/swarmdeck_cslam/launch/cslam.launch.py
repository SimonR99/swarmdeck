"""Swarm-SLAM (MISTLab `cslam`) for one SwarmDeck robot.

    ros2 launch swarmdeck_cslam cslam.launch.py namespace:=robot_0 robot_id:=0 \\
         max_nb_robots:=4

**Status: experimental.** The image builds on Jazzy against apt GTSAM 4.2 and
the Gazebo fleet has produced geometrically verified inter-robot closures.
RTAB-Map grids and cslam trajectories still disagree, so cslam transforms are
not ready for physical navigation. See docs/architecture/collaborative-slam.md.

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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # cslam's OWN namespace, which must be `r<robot_id>`. This is not a
    # preference: the C++ pose_graph_manager builds its inter-robot topic names
    # as `/r{id}/cslam/...` from `robot_id` alone, while the Python front-end
    # nodes inherit the launch namespace. Put the Python half under
    # `/robot_0` and you get two complete sets of topics — `/robot_0/cslam/...`
    # and `/r0/cslam/...` — that look healthy, log nothing, and never meet.
    # Every node initialises, no descriptor is ever exchanged.
    ns = LaunchConfiguration("namespace")
    # Where the FLEET publishes, which is a different namespace entirely
    # (`robot_0`). The two are bridged by giving cslam absolute input topics.
    sensor_ns = LaunchConfiguration("sensor_namespace")
    robot_id = LaunchConfiguration("robot_id")
    max_robots = LaunchConfiguration("max_nb_robots")
    use_sim = LaunchConfiguration("use_sim_time")

    config = PathJoinSubstitution(
        [FindPackageShare("swarmdeck_cslam"), "config", "cslam_lidar.yaml"]
    )
    common = [
        config,
        {
            "use_sim_time": use_sim,
            "robot_id": robot_id,
            "max_nb_robots": max_robots,
            # Absolute, so they resolve to the fleet's namespace rather than
            # cslam's. Overrides the relative names in cslam_lidar.yaml.
            "frontend.odom_topic": ["/", sensor_ns, "/odom_icp"],
            "frontend.pointcloud_topic": ["/", sensor_ns, "/scan/points"],
        },
    ]
    tf_remap = [("/tf", "tf"), ("/tf_static", "tf_static")]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="r0",
                description="cslam's own namespace. MUST be r<robot_id> — the C++ "
                "back end hardcodes that pattern for its inter-robot "
                "topics and will silently never meet the Python nodes "
                "otherwise.",
            ),
            DeclareLaunchArgument(
                "sensor_namespace",
                default_value="robot_0",
                description="Where the fleet publishes odom_icp and scan/points.",
            ),
            DeclareLaunchArgument(
                "robot_id",
                default_value="0",
                description="cslam requires integer ids starting at 0 and below "
                "max_nb_robots. SwarmDeck's robot_N naming maps "
                "straight onto it.",
            ),
            DeclareLaunchArgument("max_nb_robots", default_value="4"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            GroupAction(
                [
                    PushRosNamespace(ns),
                    # Keyframe selection and lidar descriptors: what gets
                    # exchanged between robots. Sparse by design — the paper's
                    # contribution is sending few descriptors, not many clouds.
                    #
                    # `lidar_handler_node.py`, NOT `map_manager`: the C++
                    # `map_manager` is the stereo/RGB-D front end. Both exist in
                    # the package and both start cleanly, so picking the wrong
                    # one gives a healthy-looking stack that never produces a
                    # lidar descriptor. Confirmed against upstream's own
                    # cslam_experiments/launch/cslam/cslam_lidar.launch.py.
                    Node(
                        package="cslam",
                        executable="lidar_handler_node.py",
                        name="cslam_map_manager",
                        parameters=common,
                        remappings=tf_remap,
                        output="screen",
                    ),
                    # Decides which candidate inter-robot matches are worth
                    # verifying, and verifies them geometrically (TEASER++).
                    Node(
                        package="cslam",
                        executable="loop_closure_detection_node.py",
                        name="cslam_loop_closure_detection",
                        parameters=common,
                        remappings=tf_remap,
                        output="screen",
                    ),
                    # The joint GTSAM back end. This is the part that makes one
                    # robot's observations correct another's drift.
                    Node(
                        package="cslam",
                        executable="pose_graph_manager",
                        name="cslam_pose_graph_manager",
                        parameters=common,
                        remappings=tf_remap,
                        output="screen",
                    ),
                ]
            ),
        ]
    )
