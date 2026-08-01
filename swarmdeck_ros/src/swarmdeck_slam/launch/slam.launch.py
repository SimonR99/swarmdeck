"""Per-robot 2D SLAM.

3D lidar pointcloud -> height-band 2D scan -> SLAM Toolbox -> OccupancyGrid.
Launched once per robot namespace by swarmdeck_bringup.

Two things here are non-obvious and were found the hard way:

1. `async_slam_toolbox_node` is a LIFECYCLE node. On Jazzy it stays in the
   `unconfigured` state with zero subscribers and logs nothing at all, so it
   looks like a silent hang. `nav2_lifecycle_manager` with autostart drives it
   to `active`.
2. The scan topic must be REMAPPED (`scan:=...`). Setting the `scan_topic`
   parameter does not take effect, and again fails silently.

Everything must run with `use_sim_time:=true` — Gazebo stamps sensors with sim
time, and without `/clock` bridged the TF lookups never resolve.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ns = LaunchConfiguration("namespace")
    zmin = LaunchConfiguration("height_min")
    zmax = LaunchConfiguration("height_max")
    use_sim = LaunchConfiguration("use_sim_time")

    slam_params = PathJoinSubstitution(
        [FindPackageShare("swarmdeck_slam"), "config", "slam_toolbox.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="robot_0"),
            DeclareLaunchArgument("height_min", default_value="0.10"),
            DeclareLaunchArgument("height_max", default_value="1.80"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            # Lidar mount transform. Gazebo names the sensor frame
            # <model>/<link>/<sensor>, which nothing else publishes.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="lidar_tf",
                arguments=[
                    "--x", "0.10", "--z", "0.16",
                    "--frame-id", [ns, "/base_link"],
                    "--child-frame-id", [ns, "/base_link/lidar"],
                ],
                parameters=[{"use_sim_time": use_sim}],
            ),
            # FR-M2: reduce the 3D lidar to a 2D scan by height band.
            # angle_increment must match the lidar's 360 horizontal samples
            # (2*pi/360) or every other bin comes back as inf.
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pc_to_scan",
                remappings=[
                    ("cloud_in", [ns, "/scan/points"]),
                    ("scan", [ns, "/scan"]),
                ],
                parameters=[
                    {
                        "target_frame": "",
                        "transform_tolerance": 0.01,
                        "min_height": zmin,
                        "max_height": zmax,
                        "angle_min": -3.14159,
                        "angle_max": 3.14159,
                        "angle_increment": 0.017453,
                        "scan_time": 0.1,
                        "range_min": 0.15,
                        "range_max": 16.0,
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                        "use_sim_time": use_sim,
                    }
                ],
                output="screen",
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                # Remap, do NOT use the scan_topic parameter — see module docstring.
                remappings=[("scan", [ns, "/scan"]), ("/map", [ns, "/map"])],
                parameters=[
                    slam_params,
                    {
                        "use_sim_time": use_sim,
                        "odom_frame": [ns, "/odom"],
                        "base_frame": [ns, "/base_link"],
                        "map_frame": "map",
                    },
                ],
                output="screen",
            ),
            # Drives slam_toolbox unconfigured -> configure -> activate.
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_slam",
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
