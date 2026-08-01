"""Per-robot 2D SLAM.

3D lidar pointcloud -> height-band 2D scan -> SLAM Toolbox -> OccupancyGrid.
Launched once per robot namespace by swarmdeck_bringup.
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

    slam_params = PathJoinSubstitution(
        [FindPackageShare("swarmdeck_slam"), "config", "slam_toolbox.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="robot_0"),
            DeclareLaunchArgument("height_min", default_value="0.10"),
            DeclareLaunchArgument("height_max", default_value="1.80"),
            # FR-M2: reduce the 3D lidar to a 2D scan by height band.
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pc_to_scan",
                namespace=ns,
                remappings=[("cloud_in", "scan/points"), ("scan", "scan")],
                parameters=[
                    {
                        "target_frame": "",
                        "transform_tolerance": 0.01,
                        "min_height": zmin,
                        "max_height": zmax,
                        "angle_min": -3.14159,
                        "angle_max": 3.14159,
                        "angle_increment": 0.0087,
                        "scan_time": 0.1,
                        "range_min": 0.15,
                        "range_max": 16.0,
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                    }
                ],
                output="screen",
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                namespace=ns,
                parameters=[
                    slam_params,
                    {
                        "odom_frame": [ns, "/odom"],
                        "map_frame": [ns, "/map"],
                        "base_frame": [ns, "/base_link"],
                    },
                ],
                output="screen",
            ),
        ]
    )
