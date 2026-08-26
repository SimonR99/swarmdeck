"""Run the repository's Nav2 stack against Botman's live ROS 2 interfaces."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Chassis in the *lidar* frame. robot_base_frame is os_lidar, and the live TF
# tree has no os_lidar -> base_link edge, so a circle around the lidar is the
# wrong shape: the Ouster is mounted at the FRONT of the Bunker, 0.15 m ahead of
# the chassis centre, so the rear bumper is 0.66 m behind it — outside the 0.65 m
# radius that circumscribes the chassis *centre*. Those deck returns become
# obstacles, DWB cannot plan, and the BT's only remaining motion is BackUp.
#
# _LIDAR_X was -0.150 until 2026-08-21, copied from the *sim* bunker model, which
# put the footprint exactly backwards: it claimed 0.66 m of robot ahead where
# there is 0.36 m, and 0.36 m behind where there is really 0.66 m. That left a
# quarter-metre of real chassis OUTSIDE the collision polygon while reversing.
# Confirmed on the physical robot: the lidar is at the front, +0.15 m from centre.
#
# nav.launch.py's RewrittenYaml always overwrites robot_radius / footprint /
# inflation_radius. Omitting them here silently installs the Scout-sized
# 0.422 m default, which is how this stack ended up reverse-only.
_BUNKER_HALF_L = 1.023 / 2.0
_BUNKER_HALF_W = 0.778 / 2.0
_LIDAR_X = 0.150
_FRONT = _BUNKER_HALF_L - _LIDAR_X
_REAR = -_BUNKER_HALF_L - _LIDAR_X
_BUNKER_FOOTPRINT = (
    f"[[{_FRONT:.3f},{_BUNKER_HALF_W:.3f}],[{_FRONT:.3f},{-_BUNKER_HALF_W:.3f}],"
    f"[{_REAR:.3f},{-_BUNKER_HALF_W:.3f}],[{_REAR:.3f},{_BUNKER_HALF_W:.3f}]]"
)
# Circumscribed from the lidar, used only if the polygon fails to parse. Take
# whichever end is FURTHER from the sensor: with the lidar forward of centre that
# is the rear, and using _FRONT alone would understate the radius by 0.24 m.
_BUNKER_RADIUS = (
    f"{(max(abs(_FRONT), abs(_REAR)) ** 2 + _BUNKER_HALF_W ** 2) ** 0.5:.3f}"
)


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    package_share = FindPackageShare("swarmdeck_nav")

    odometry_tf = Node(
        package="swarmdeck_nav",
        executable="odom_to_tf",
        name="botman_odom_to_tf",
        output="screen",
        condition=IfCondition(publish_odom_tf),
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
            "robot_radius": _BUNKER_RADIUS,
            # Keep the physical Bunker footprint, but use the requested
            # 0.50 m obstacle-inflation margin instead of the old 0.90 m.
            "inflation_radius": "0.50",
            "footprint": _BUNKER_FOOTPRINT,
            # Nav2 never writes the Bunker driver's /cmd_vel directly. The
            # hardware adapter relays this final output only for an active goal.
            "controller_cmd_vel_topic": "cmd_vel_nav_raw",
            "output_cmd_vel_topic": "cmd_vel_nav",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            # The physical Compose deployment has a separate odom_tf service so
            # pose survives a Nav2 restart. It passes false here to avoid two
            # broadcasters publishing the same map -> os_lidar transform.
            DeclareLaunchArgument("publish_odom_tf", default_value="true"),
            odometry_tf,
            nav2,
        ]
    )
