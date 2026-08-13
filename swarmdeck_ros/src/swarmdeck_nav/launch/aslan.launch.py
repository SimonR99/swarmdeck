"""Run the repository's Nav2 stack against Aslan's live ROS 2 interfaces."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Keep in step with botman.launch.py — both Bunkers share this Nav2 YAML, and
# nav.launch.py overwrites robot_radius/footprint unless these are passed.
_BUNKER_HALF_L = 1.023 / 2.0
_BUNKER_HALF_W = 0.778 / 2.0
_LIDAR_X = -0.150
_FRONT = _BUNKER_HALF_L - _LIDAR_X
_REAR = -_BUNKER_HALF_L - _LIDAR_X
_BUNKER_FOOTPRINT = (
    f"[[{_FRONT:.3f},{_BUNKER_HALF_W:.3f}],[{_FRONT:.3f},{-_BUNKER_HALF_W:.3f}],"
    f"[{_REAR:.3f},{-_BUNKER_HALF_W:.3f}],[{_REAR:.3f},{_BUNKER_HALF_W:.3f}]]"
)
_BUNKER_RADIUS = f"{(_FRONT ** 2 + _BUNKER_HALF_W ** 2) ** 0.5:.3f}"


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    package_share = FindPackageShare("swarmdeck_nav")

    odometry_tf = Node(
        package="swarmdeck_nav",
        executable="odom_to_tf",
        name="aslan_odom_to_tf",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "odom_topic": "/laser_odometry",
                "parent_frame": "map",
                "child_frame": "os_lidar",
                "planar": True,
                "use_receive_time": True,
            }
        ],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "nav.launch.py"])
        ),
        launch_arguments={
            "namespace": "aslan_0",
            "use_sim_time": use_sim_time,
            # The two Bunkers expose the same SuperOdometry/Ouster interfaces
            # and share the same conservative physical limits.
            "params_file": PathJoinSubstitution(
                [package_share, "config", "botman_nav2_params.yaml"]
            ),
            "tf_topic": "/tf",
            "tf_static_topic": "/tf_static",
            "robot_radius": _BUNKER_RADIUS,
            "inflation_radius": "0.90",
            "footprint": _BUNKER_FOOTPRINT,
            "controller_cmd_vel_topic": "cmd_vel_nav_raw",
            "output_cmd_vel_topic": "cmd_vel_nav",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            odometry_tf,
            nav2,
        ]
    )
