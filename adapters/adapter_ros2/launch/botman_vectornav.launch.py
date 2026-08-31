"""VectorNav VN-100 driver for Botman, owned by SwarmDeck.

WHY THIS EXISTS

  Nothing in Botman's deployment started this IMU. `bunker_gnm.launch.py` in
  the read-only MIST workspace defines vectornav_node and
  vectornav_sensor_msgs_node but never adds them to the LaunchDescription:
  line 204 there is a commented-out

      # declare_args + [superodom_launch, ouster_launch, vectornav_node,
      #                 vectornav_sensor_msgs_node, bunker_launch, ...]

  and the live list is `declare_args + [bunker_launch]`. The lidar, SLAM and
  OAK entries in that same list are already owned by this repo's Compose file
  for exactly this reason; the IMU was the one that was missed.

  It went unnoticed because the node happens to be easy to start by hand, so
  /vectornav/imu can be live during a session and gone after the next deploy.
  SuperOdometry then logs "no IMU data, running LiDAR Odometry only" and keeps
  going, which is a quiet degradation rather than a failure.

  Two nodes are needed, not one: `vectornav` talks to the serial port and
  publishes the raw binary registers, `vn_sensor_msgs` turns those into
  sensor_msgs/Imu on /vectornav/imu. SuperOdometry subscribes to the latter,
  so starting only the driver produces a graph that looks healthy and no IMU
  messages at all.

Usage:
    ros2 launch adapters/adapter_ros2/launch/botman_vectornav.launch.py
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_ADAPTER_CONFIG = Path(__file__).resolve().parents[1] / "config"


def generate_launch_description() -> LaunchDescription:
    params = LaunchConfiguration("params_file")
    start_imu = LaunchConfiguration("start_imu")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(_ADAPTER_CONFIG / "botman_vectornav.yaml"),
                description="VN-100 driver parameters, including the serial port and rate.",
            ),
            DeclareLaunchArgument("start_imu", default_value="true"),
            # Serial driver. Publishes /vectornav/raw/* only.
            Node(
                package="vectornav",
                executable="vectornav",
                name="vectornav",
                output="screen",
                parameters=[params],
                condition=IfCondition(start_imu),
                respawn=True,
                respawn_delay=2.0,
            ),
            # Converts the raw registers to sensor_msgs/Imu on /vectornav/imu.
            # Without this the driver runs and nothing subscribable appears.
            Node(
                package="vectornav",
                executable="vn_sensor_msgs",
                name="vn_sensor_msgs",
                output="screen",
                parameters=[params],
                condition=IfCondition(start_imu),
                respawn=True,
                respawn_delay=2.0,
            ),
        ]
    )
