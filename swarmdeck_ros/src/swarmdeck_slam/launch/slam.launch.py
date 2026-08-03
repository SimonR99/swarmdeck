"""Per-robot 2D SLAM: planar lidar -> SLAM Toolbox -> OccupancyGrid.

Three things here are non-obvious and were each found the hard way, because all
three fail *silently* — no error, no log line, node looks healthy:

1. `async_slam_toolbox_node` is a LIFECYCLE node. On Jazzy it sits in
   `unconfigured` with zero subscribers and logs nothing at all, so it presents
   as a hang. `nav2_lifecycle_manager` with autostart drives it to `active`.
2. The scan topic must be REMAPPED (`scan:=...`). Setting the `scan_topic`
   parameter has no effect and leaves subscription count at 0.
3. Everything needs `use_sim_time:=true` and a bridged `/clock`. Gazebo stamps
   sensors with sim time; without it TF lookups never resolve and SLAM stalls.

With `lidar_rings:=1` (the default) Gazebo publishes a usable LaserScan directly
and nothing converts anything. With an odd `lidar_rings` > 1 the lidar also
publishes a 3D cloud, and `pointcloud_to_laserscan` slices the planar scan back
out of it — see the node below for why the band is as tight as it is, and
robot.sdf.jinja for why the ring count must be odd.

This is still 2D SLAM either way. It cannot use the 3D structure, correct z/roll/
pitch drift, or close loops visually; see docs/collaborative-slam.md for what a
3D-capable per-robot SLAM would change.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ns = LaunchConfiguration("namespace")
    use_sim = LaunchConfiguration("use_sim_time")
    rings = LaunchConfiguration("lidar_rings")
    multi_ring = IfCondition(PythonExpression(['"', rings, '" != "1"']))

    slam_params = PathJoinSubstitution(
        [FindPackageShare("swarmdeck_slam"), "config", "slam_toolbox.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="robot_0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("lidar_rings", default_value="1"),
            # Lidar mount transform. Gazebo names the sensor frame
            # <model>/<link>/<sensor>, which nothing else publishes.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="lidar_tf",
                namespace=ns,
                arguments=[
                    "--x", "-0.07", "--z", "0.402",
                    "--frame-id", [ns, "/base_link"],
                    "--child-frame-id", [ns, "/base_link/lidar"],
                ],
                parameters=[{"use_sim_time": use_sim}],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="proximity_lidar_tf",
                namespace=ns,
                arguments=[
                    "--x", "0.24", "--z", "0.05",
                    "--frame-id", [ns, "/base_link"],
                    "--child-frame-id", [ns, "/base_link/proximity_lidar"],
                ],
                parameters=[{"use_sim_time": use_sim}],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ),
            # A multi-ring lidar's LaserScan is not a planar slice, so derive one.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("swarmdeck_slam"), "/launch/cloud_to_scan.launch.py"]
                ),
                condition=multi_ring,
                launch_arguments={"namespace": ns, "use_sim_time": use_sim}.items(),
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                namespace=ns,
                # Remap, do NOT use the scan_topic parameter — see docstring.
                remappings=[
                    ("/scan", "scan"),
                    ("/map", "map"),
                    ("/map_metadata", "map_metadata"),
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                ],
                parameters=[
                    slam_params,
                    {
                        "use_sim_time": use_sim,
                        "odom_frame": [ns, "/odom"],
                        "base_frame": [ns, "/base_link"],
                        "map_frame": [ns, "/map_frame"],
                    },
                ],
                output="screen",
            ),
            # Drives slam_toolbox unconfigured -> configure -> activate.
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_slam",
                namespace=ns,
                parameters=[
                    {
                        "use_sim_time": use_sim,
                        "autostart": True,
                        "node_names": ["slam_toolbox"],
                        "bond_timeout": 0.0,
                    }
                ],
                output="screen",
            ),
        ]
    )
