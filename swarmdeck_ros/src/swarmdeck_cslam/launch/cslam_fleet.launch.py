"""Swarm-SLAM for a whole simulated fleet, one node set per robot.

    ros2 launch swarmdeck_cslam cslam_fleet.launch.py robots:=4

On hardware each robot runs `cslam.launch.py` itself and they meet over the
network — that is the decentralised architecture Swarm-SLAM exists for. In
simulation every robot already lives in one Gazebo process, so running the node
sets side by side in one container is the faithful equivalent: they are still
separate nodes with separate `robot_id`s exchanging the same messages over the
same graph, and none of them can see another's internal state.

Requires the RTAB-Map backend, because cslam's motion prior here is `odom_icp` —
the lidar odometry. Pointing it at the wheel topic would hand the pose graph the
one channel that cannot observe slip.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def setup(context, *args, **kwargs):
    count = int(LaunchConfiguration("robots").perform(context))
    prefix = LaunchConfiguration("prefix").perform(context)
    use_sim = LaunchConfiguration("use_sim_time").perform(context)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("swarmdeck_cslam"), "/launch/cslam.launch.py"]
            ),
            launch_arguments={
                # cslam's namespace is r<id>; the fleet's is robot_<id>. They are
                # deliberately different — see cslam.launch.py.
                "namespace": f"r{i}",
                "sensor_namespace": f"{prefix}{i}",
                # cslam requires contiguous integer ids from 0, which is exactly
                # what SwarmDeck's robot_N naming already provides.
                "robot_id": str(i),
                "max_nb_robots": str(count),
                "use_sim_time": use_sim,
            }.items(),
        )
        for i in range(count)
    ] + [
        # Summarises the joint graph onto /swarmdeck/slam_graph for the adapter.
        # One instance for the whole fleet, not one per robot.
        # The cslam-native occupancy grid: keyframe clouds rendered at the
        # joint optimiser's own poses, so geometry and transform come from ONE
        # system. See cslam_grid.py for why mixing two was the bug.
        Node(
            package="swarmdeck_cslam",
            executable="cslam_grid.py",
            name="cslam_grid",
            parameters=[
                {
                    "robots": count,
                    "use_sim_time": use_sim == "true",
                    "resolution": 0.05,
                    "size_m": 30.0,
                }
            ],
            output="screen",
        ),
        Node(
            package="swarmdeck_cslam",
            executable="graph_reporter.py",
            name="swarmdeck_graph_reporter",
            parameters=[{"robots": count, "use_sim_time": use_sim == "true"}],
            output="screen",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("robots", default_value="4"),
            DeclareLaunchArgument("prefix", default_value="robot_"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            OpaqueFunction(function=setup),
        ]
    )
