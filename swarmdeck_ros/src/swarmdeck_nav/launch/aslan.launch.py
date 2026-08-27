"""Run the repository's Nav2 stack against Aslan's live ROS 2 interfaces."""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_BASE_FRAME = "aslan_base_link"
_SCAN_FRAME = "aslan_lidar_scan"
_SCAN_TOPIC = "/aslan_nav_scan"

# Keep in step with botman.launch.py — both physical robots are the same Bunker
# chassis, and nav.launch.py overwrites robot_radius/footprint/inflation_radius
# unless these are passed.
#
# The chassis is expressed around the lidar origin in a physical-forward base
# frame. SuperOdometry labels /laser_odometry's child os_lidar, but live
# raw-cloud/registered-scan comparison proves its pose is actually this base
# frame: raw points need a pi-yaw before that pose places them in map. Keep the
# raw scan coordinates and publish them under an unambiguous corrected frame.
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
_BUNKER_RADIUS = (
    f"{(max(abs(_FRONT), abs(_REAR)) ** 2 + _BUNKER_HALF_W ** 2) ** 0.5:.3f}"
)

# Project the full useful vertical part of the Ouster cloud, measured from the
# floor, instead of one horizontal ring. The raw cloud's Z origin is the lidar.
_LIDAR_HEIGHT = 0.520
_OBSTACLE_MIN_HEIGHT = 0.150 - _LIDAR_HEIGHT
_OBSTACLE_MAX_HEIGHT = 1.800 - _LIDAR_HEIGHT
_SELF_FILTER_PADDING = 0.050


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
                "child_frame": _BASE_FRAME,
                "planar": True,
                # SuperOdometry arrives about 0.3 s behind wall time while
                # the Ouster scan arrives about 0.2 s behind. Stamp the
                # relayed TF on receipt so the scan-only costmaps can
                # transform live data.
                "use_receive_time": True,
            }
        ],
    )

    # SuperOdometry also broadcasts its mislabeled map -> os_lidar edge itself,
    # so os_lidar cannot safely receive a second parent. The projected scan is
    # published directly in a dedicated raw-coordinate frame instead.
    lidar_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aslan_base_to_scan",
        output="screen",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "3.141592653589793",
            "--frame-id",
            _BASE_FRAME,
            "--child-frame-id",
            _SCAN_FRAME,
        ],
    )
    obstacle_scan = Node(
        package="swarmdeck_nav",
        executable="footprint_cloud_to_scan",
        name="aslan_footprint_cloud_to_scan",
        output="screen",
        parameters=[
            {
                "input_topic": "/ouster/points",
                "output_topic": _SCAN_TOPIC,
                "output_frame": _SCAN_FRAME,
                "min_height": _OBSTACLE_MIN_HEIGHT,
                "max_height": _OBSTACLE_MAX_HEIGHT,
                "range_min": 0.05,
                "range_max": 10.0,
                "angle_min": -math.pi,
                "angle_increment": 2.0 * math.pi / 512.0,
                "scan_time": 0.1,
                "use_inf": True,
                # Raw Ouster axes are pi from the physical-forward base axes.
                "sensor_yaw_in_base": math.pi,
                "footprint_front": _FRONT,
                "footprint_rear": _REAR,
                "footprint_half_width": _BUNKER_HALF_W,
                "footprint_padding": _SELF_FILTER_PADDING,
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
            "robot_base_frame": _BASE_FRAME,
            "obstacle_scan_topic": _SCAN_TOPIC,
            "obstacle_sensor_frame": _SCAN_FRAME,
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
            # two broadcasters publishing the same map -> base transform.
            #
            # Leaving both running is not benign: measured on 2026-08-25 with
            # both alive, 49% of map -> base broadcasts were an identical
            # pose re-sent under a second timestamp up to 26 ms later, and
            # 7.8% of stamps arrived out of order, so a later stamp could
            # carry an earlier pose. Parked that is invisible -- duplicating
            # an unchanging pose costs nothing -- which is why this survived
            # standstill testing while every lookup made in motion
            # interpolated across an inverted pair.
            DeclareLaunchArgument("publish_odom_tf", default_value="true"),
            odometry_tf,
            lidar_frame,
            obstacle_scan,
            nav2,
        ]
    )
