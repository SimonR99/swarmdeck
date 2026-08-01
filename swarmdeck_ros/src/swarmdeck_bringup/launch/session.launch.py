"""One-command launch for the whole simulated stack (acceptance criterion 13).

    ros2 launch swarmdeck_bringup session.launch.py config:=study/4robot.yaml

Brings up: Gazebo world -> fleet spawn -> ros_gz bridges -> per-robot SLAM.
The backend and UI run separately (`make server`, `make ui`) because they are
ROS-free by design.
"""

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

REPO = Path(__file__).resolve().parents[4]


def bridge_args(ns: str) -> list[str]:
    """gz <-> ROS 2 topic bridges for one robot."""
    return [
        f"/{ns}/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        f"/{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        f"/{ns}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        f"/{ns}/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
        f"/{ns}/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        f"/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
    ]


def setup(context, *args, **kwargs):
    cfg_arg = LaunchConfiguration("config").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"

    cfg_path = Path(cfg_arg)
    if not cfg_path.is_absolute():
        cfg_path = REPO / cfg_arg
    cfg = yaml.safe_load(cfg_path.read_text())

    count = int(cfg.get("fleet", {}).get("robot_count", 4))
    prefix = cfg.get("fleet", {}).get("robot_prefix", "robot_")
    band = cfg.get("map", {}).get("height_band", [0.10, 1.80])
    seed = cfg.get("seed", 20260801)

    sim_share = REPO / "swarmdeck_ros" / "src" / "swarmdeck_sim"
    world = sim_share / "worlds" / "indoor.sdf"
    scenario = sim_share / "scenario"

    actions = [
        # Regenerate the world from the seed so the run is reproducible.
        ExecuteProcess(
            cmd=["python3", str(scenario / "generate_world.py"),
                 "--seed", str(seed), "-o", str(world)],
            output="screen",
        ),
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=["gz", "sim", "-s" if headless else "", "-r",
                         "--headless-rendering" if headless else "", str(world)],
                    output="screen",
                )
            ],
        ),
        TimerAction(
            period=12.0,
            actions=[
                ExecuteProcess(
                    cmd=["python3", str(scenario / "spawn_fleet.py"),
                         "--config", str(cfg_path)],
                    output="screen",
                )
            ],
        ),
    ]

    delayed = []
    for i in range(min(count, 5)):
        ns = f"{prefix}{i}"
        delayed.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name=f"bridge_{ns}",
                arguments=bridge_args(ns),
                output="screen",
            )
        )
        delayed.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("swarmdeck_slam"), "/launch/slam.launch.py"]
                ),
                launch_arguments={
                    "namespace": ns,
                    "height_min": str(band[0]),
                    "height_max": str(band[1]),
                }.items(),
            )
        )

    actions.append(TimerAction(period=20.0, actions=delayed))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value="study/4robot.yaml"),
            DeclareLaunchArgument("headless", default_value="true"),
            OpaqueFunction(function=setup),
        ]
    )
