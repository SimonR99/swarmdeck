"""Minimal Spot hardware launch owned by SwarmDeck.

The upstream `rover_launch/spot.launch.py` always starts `spot_driver`, which
claims a lease on the robot. This launch splits lidar, SuperOdometry and the
driver so mapping can come up without touching the gait. All MIST
source/configuration remains mounted read-only.

`spot_driver` is resolved only when `start_driver:=true`, so a lidar/SLAM
bring-up still works in an image that has not installed that package.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def _spot_driver(context, *args, **kwargs):
    if LaunchConfiguration("start_driver").perform(context) != "true":
        return []
    mist_config = Path(
        LaunchConfiguration("mist_config").perform(context)
    )
    spot_share = Path(get_package_share_directory("spot_driver"))
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(spot_share / "launch/spot_driver.launch.py")
            ),
            launch_arguments={
                "config_file": str(mist_config / "spot.yaml"),
            }.items(),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    mist_config = LaunchConfiguration("mist_config")
    start_lidar = LaunchConfiguration("start_lidar")
    start_slam = LaunchConfiguration("start_slam")

    ouster_share = Path(get_package_share_directory("ouster_ros"))
    superodom_share = Path(get_package_share_directory("super_odometry"))

    ouster = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ouster_share / "launch/driver.launch.py")),
        launch_arguments={
            "params_file": PathJoinSubstitution(
                [mist_config, "drivers/driver_ouster.yaml"]
            ),
            "ouster_ns": "ouster",
            "viz": "false",
            "os_driver_name": "os_driver",
            "throttle_rate": "0.1",
            "input_topic": "/ouster/points",
            "output_topic": "/ouster/points_throttled",
        }.items(),
        condition=IfCondition(start_lidar),
    )

    superodom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(superodom_share / "launch/os1_128.launch.py")
        ),
        launch_arguments={
            "config_file": PathJoinSubstitution(
                [mist_config, "superodom/os1_128.yaml"]
            ),
            "calibration_file": PathJoinSubstitution(
                [mist_config, "superodom/os1_128_calibration.yaml"]
            ),
        }.items(),
        condition=IfCondition(start_slam),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mist_config",
                default_value="/workspace/src/control/rover_launch/config",
            ),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_slam", default_value="true"),
            DeclareLaunchArgument("start_driver", default_value="false"),
            ouster,
            superodom,
            OpaqueFunction(function=_spot_driver),
        ]
    )
