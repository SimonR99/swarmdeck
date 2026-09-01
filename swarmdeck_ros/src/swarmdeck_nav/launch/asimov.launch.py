"""Run the repository's Nav2 stack against Asimov's live ROS 2 interfaces.

Simpler than botman.launch.py/aslan.launch.py: those bridge SuperOdometry's
/laser_odometry into a TF because SuperOdometry publishes no map -> base edge
of its own, and add a static lidar-frame correction because SuperOdometry
mislabels its pose's child frame. Asimov needs neither. The onboard
Unitree/Livox localization already publishes a live, correctly oriented
world -> base_link (and world -> livox_frame) TF -- adapter_ros2.py already
relies on that same TF today for keyframe/SLAM alignment -- so this file only
has to project the Mid-360 cloud into an obstacle scan and start Nav2.
"""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_SCAN_TOPIC = "/asimov_nav_scan"
# The projector's output frame is just a label copied onto the LaserScan
# header; reuse the real, live lidar frame instead of inventing a synthetic
# one, since -- unlike the Bunkers -- nothing here needs to correct a
# mislabeled TF child frame.
_SCAN_FRAME = "livox_frame"

# G1 torso/pelvis bounding footprint, matching
# adapters/adapter_ros2/config/unitree_g1.yaml's footprint/footprint_radius so
# Nav2 and the adapter's own reported chassis size cannot drift apart.
_FRONT = 0.18
_REAR = -0.18
_HALF_WIDTH = 0.22
_FOOTPRINT_RADIUS = 0.30
_G1_FOOTPRINT = (
    f"[[{_FRONT:.3f},{_HALF_WIDTH:.3f}],[{_FRONT:.3f},{-_HALF_WIDTH:.3f}],"
    f"[{_REAR:.3f},{-_HALF_WIDTH:.3f}],[{_REAR:.3f},{_HALF_WIDTH:.3f}]]"
)

# The Mid-360 cloud's Z origin is the lidar, 1.17 m above the floor (see
# unitree_g1.yaml's lidar_height_m). Project the same physical 0.15..1.80 m
# band adapter_ros2.py already uses for keyframes (map_cloud_height_band).
_LIDAR_HEIGHT = 1.17
_OBSTACLE_MIN_HEIGHT = 0.15 - _LIDAR_HEIGHT
_OBSTACLE_MAX_HEIGHT = 1.80 - _LIDAR_HEIGHT
# Larger than the Bunkers' 0.05 m: a walking gait swings arms and legs beyond
# the torso's static bounding box, unlike a rigid wheeled chassis.
_SELF_FILTER_PADDING = 0.10
# UNVERIFIED. The Bunkers' Ouster is confirmed mounted pi-yaw from the
# physical-forward base frame; nothing has yet measured whether the Mid-360's
# livox_frame agrees with base_link's forward axis. Confirm on first
# bring-up: watch whether the projected obstacle scan lines up with the
# visible world while walking forward. Wrong sign here rejects the wrong side
# of the footprint and can make G1 walk into what look like clear obstacles.
_SENSOR_YAW_IN_BASE = 0.0


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    package_share = FindPackageShare("swarmdeck_nav")

    obstacle_scan = Node(
        package="swarmdeck_nav",
        executable="footprint_cloud_to_scan",
        name="asimov_footprint_cloud_to_scan",
        output="screen",
        parameters=[
            {
                "input_topic": "/utlidar/cloud_livox_mid360",
                "output_topic": _SCAN_TOPIC,
                "output_frame": _SCAN_FRAME,
                "min_height": _OBSTACLE_MIN_HEIGHT,
                "max_height": _OBSTACLE_MAX_HEIGHT,
                "range_min": 0.05,
                "range_max": 12.0,
                "angle_min": -math.pi,
                "angle_increment": 2.0 * math.pi / 512.0,
                "scan_time": 0.1,
                "use_inf": True,
                "sensor_yaw_in_base": _SENSOR_YAW_IN_BASE,
                "footprint_front": _FRONT,
                "footprint_rear": _REAR,
                "footprint_half_width": _HALF_WIDTH,
                "footprint_padding": _SELF_FILTER_PADDING,
            }
        ],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "nav.launch.py"])
        ),
        launch_arguments={
            "namespace": "asimov_0",
            "use_sim_time": use_sim_time,
            "params_file": PathJoinSubstitution(
                [package_share, "config", "asimov_nav2_params.yaml"]
            ),
            # Asimov's TF is published globally, not namespaced.
            "tf_topic": "/tf",
            "tf_static_topic": "/tf_static",
            "robot_base_frame": "base_link",
            "robot_radius": f"{_FOOTPRINT_RADIUS:.3f}",
            "footprint": _G1_FOOTPRINT,
            "inflation_radius": "0.45",
            "controller_cmd_vel_topic": "cmd_vel_nav_raw",
            "output_cmd_vel_topic": "cmd_vel_nav",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            obstacle_scan,
            nav2,
        ]
    )
