"""Run the repository's Nav2 stack against Aslan's live ROS 2 interfaces."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Keep in step with botman.launch.py — both physical robots are the same Bunker
# chassis, and nav.launch.py overwrites robot_radius/footprint/inflation_radius
# unless these are passed.
#
# The chassis is expressed in the *lidar* frame because robot_base_frame is
# os_lidar and the live TF tree has no os_lidar -> base_link edge. The Ouster is
# mounted at the front, 0.15 m ahead of the chassis centre. Using the simulated
# -0.15 m value mirrors the footprint and leaves the real rear deck outside the
# collision polygon, which makes Nav2 treat the robot as its own obstacle.
_BUNKER_HALF_L = 1.023 / 2.0
_BUNKER_HALF_W = 0.778 / 2.0
_LIDAR_X = 0.150
_FRONT = _BUNKER_HALF_L - _LIDAR_X
_REAR = -_BUNKER_HALF_L - _LIDAR_X
_BUNKER_FOOTPRINT = (
    f"[[{_FRONT:.3f},{_BUNKER_HALF_W:.3f}],[{_FRONT:.3f},{-_BUNKER_HALF_W:.3f}],"
    f"[{_REAR:.3f},{-_BUNKER_HALF_W:.3f}],[{_REAR:.3f},{_BUNKER_HALF_W:.3f}]]"
)
# Use the end furthest from the lidar for the fallback circle. With the sensor
# ahead of centre that is the rear, not the front.
_BUNKER_RADIUS = f"{(max(abs(_FRONT), abs(_REAR)) ** 2 + _BUNKER_HALF_W ** 2) ** 0.5:.3f}"


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    package_share = FindPackageShare("swarmdeck_nav")

    odometry_tf = Node(
        package="swarmdeck_nav",
        executable="odom_to_tf",
        name="aslan_odom_to_tf",
        output="screen",
        condition=IfCondition(publish_odom_tf),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "odom_topic": "/laser_odometry",
                "parent_frame": "map",
                "child_frame": "os_lidar",
                "planar": True,
                # SuperOdometry arrives about 0.3 s behind wall time while
                # the Ouster scan arrives about 0.2 s behind. Stamp the
                # relayed TF on receipt so the scan-only costmaps can
                # transform live data.
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
            # and share the same physical footprint and navigation limits.
            "params_file": PathJoinSubstitution(
                [package_share, "config", "botman_nav2_params.yaml"]
            ),
            "tf_topic": "/tf",
            "tf_static_topic": "/tf_static",
            "robot_radius": _BUNKER_RADIUS,
            "inflation_radius": "0.50",
            "footprint": _BUNKER_FOOTPRINT,
            "controller_cmd_vel_topic": "cmd_vel_nav_raw",
            "output_cmd_vel_topic": "cmd_vel_nav",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            # The physical Compose deployment has a separate odom_tf service
            # so pose survives a Nav2 restart. It passes false here to avoid
            # two broadcasters publishing the same map -> os_lidar transform.
            #
            # Leaving both running is not benign: measured on 2026-08-25 with
            # both alive, 49% of map -> os_lidar broadcasts were an identical
            # pose re-sent under a second timestamp up to 26 ms later, and
            # 7.8% of stamps arrived out of order, so a later stamp could
            # carry an earlier pose. Parked that is invisible -- duplicating
            # an unchanging pose costs nothing -- which is why this survived
            # standstill testing while every lookup made in motion
            # interpolated across an inverted pair.
            DeclareLaunchArgument("publish_odom_tf", default_value="true"),
            odometry_tf,
            nav2,
        ]
    )
