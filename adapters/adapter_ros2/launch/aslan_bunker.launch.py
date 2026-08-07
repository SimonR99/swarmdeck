"""Minimal Aslan hardware launch owned by SwarmDeck.

The upstream full-stack launch imports camera and VectorNav packages that are
not installed on Aslan and hardcodes the base interface. This launch starts
only the interfaces SwarmDeck needs: Ouster, SuperOdometry, and the Bunker
base. All MIST source/configuration remains mounted read-only.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    mist_config = Path("/workspace/src/control/rover_launch/config")

    start_base = LaunchConfiguration("start_base")
    start_lidar = LaunchConfiguration("start_lidar")
    start_slam = LaunchConfiguration("start_slam")
    can_interface = LaunchConfiguration("can_interface")

    ouster_share = Path(get_package_share_directory("ouster_ros"))
    superodom_share = Path(get_package_share_directory("super_odometry"))
    bunker_share = Path(get_package_share_directory("bunker_base"))

    ouster = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ouster_share / "launch/driver.launch.py")),
        launch_arguments={
            "params_file": str(mist_config / "drivers/driver_ouster.yaml"),
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
            "config_file": str(mist_config / "superodom/os1_128.yaml"),
            "calibration_file": str(
                mist_config / "superodom/os1_128_calibration.yaml"
            ),
        }.items(),
        condition=IfCondition(start_slam),
    )

    bunker = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bunker_share / "launch/bunker_base.launch.py")
        ),
        launch_arguments={
            "port_name": can_interface,
            "use_sim_time": "false",
        }.items(),
        condition=IfCondition(start_base),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_base", default_value="true"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_slam", default_value="true"),
            # Aslan's upstream launch expects a USB-CAN adapter named can2.
            # Keep it explicit and overridable rather than silently selecting
            # one of the Jetson's currently-down native CAN controllers.
            DeclareLaunchArgument("can_interface", default_value="can2"),
            ouster,
            superodom,
            bunker,
        ]
    )
