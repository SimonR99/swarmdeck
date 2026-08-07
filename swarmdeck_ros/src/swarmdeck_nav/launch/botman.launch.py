"""Run the repository's Nav2 stack against Botman's live ROS 2 interfaces."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    package_share = FindPackageShare("swarmdeck_nav")

    odometry_tf = Node(
        package="swarmdeck_nav",
        executable="odom_to_tf",
        name="botman_odom_to_tf",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "odom_topic": "/laser_odometry",
                "parent_frame": "map",
                "child_frame": "os_lidar",
                "planar": True,
                # SuperOdometry arrives about 0.7 s behind wall time while the
                # Ouster scan arrives about 0.1 s behind. Stamp the relayed TF
                # on receipt so the scan-only costmaps can transform live data.
                "use_receive_time": True,
            }
        ],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "nav.launch.py"])
        ),
        launch_arguments={
            "namespace": "botman_0",
            "use_sim_time": use_sim_time,
            "params_file": PathJoinSubstitution(
                [package_share, "config", "botman_nav2_params.yaml"]
            ),
            # Botman's drivers are not namespaced and publish the global TF
            # graph. The Nav2 nodes remain namespaced to avoid name collisions.
            "tf_topic": "/tf",
            "tf_static_topic": "/tf_static",
            # Nav2 never writes the Bunker driver's /cmd_vel directly. The
            # hardware adapter relays this final output only for an active goal.
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
