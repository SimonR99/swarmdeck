"""Minimal Spot hardware launch owned by SwarmDeck.

The upstream `rover_launch/launch/full_stack/spot_lio_sam.launch.py` always
starts `spot_driver` (lease), TARE, and the high-level controller. This launch
splits lidar, LIO-SAM (plus VectorNav) and the driver so mapping can come up
without touching the gait. All MIST source/configuration remains mounted
read-only.

`spot_driver` is resolved only when `start_driver:=true`, so a lidar/SLAM
bring-up still works in an image that has not installed that package.

SuperOdom is present in the workspace but COLCON_IGNORE'd; LIO-SAM is what
is actually built and launched on this payload.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _spot_driver(context, *args, **kwargs):
    if LaunchConfiguration("start_driver").perform(context) != "true":
        return []
    mist_config = Path(LaunchConfiguration("mist_config").perform(context))
    # Their full spot_driver.launch.py also starts image publishers, RViz, and
    # a xacro robot_state_publisher. Claim/Stand only need spot_ros2.
    # MIST spot.yaml keeps start_estop: False (tablet holds the estop). With
    # nobody on a tablet, claim waits forever; the ROS driver must be the
    # estop endpoint for the GUI buttons to work.
    keepalive = Path(__file__).resolve().parent.parent / "spot_clear_keepalive.py"
    return [
        Node(
            package="spot_driver",
            executable="spot_ros2",
            name="spot_ros2",
            output="screen",
            parameters=[
                str(mist_config / "spot.yaml"),
                {"start_estop": True},
            ],
        ),
        # Drops the tablet's `tablet-stop` keepalive so Claim/Stand can power
        # the motors. Credentials stay in MIST's spot.yaml, not this repo.
        ExecuteProcess(
            cmd=[
                "python3",
                str(keepalive),
                "--params",
                str(mist_config / "spot.yaml"),
            ],
            output="screen",
        ),
    ]


def _spot_camera(context, *args, **kwargs):
    if LaunchConfiguration("start_camera").perform(context) != "true":
        return []
    overlay = Path("/app/swarmdeck/adapters/adapter_ros2/config/spot_d435.yaml")
    # Payload D435i color via usb_cam. Not Spot's body cameras — those stay
    # off so this container can be recreated without dropping the lease.
    return [
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="usb_cam",
            namespace="d435",
            output="screen",
            parameters=[str(overlay)],
            remappings=[
                ("image_raw", "color/image_raw"),
                ("image_raw/compressed", "color/image_raw/compressed"),
                ("camera_info", "color/camera_info"),
            ],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    mist_config = LaunchConfiguration("mist_config")
    start_lidar = LaunchConfiguration("start_lidar")
    start_slam = LaunchConfiguration("start_slam")

    ouster_share = Path(get_package_share_directory("ouster_ros"))
    rover_share = Path(get_package_share_directory("rover_launch"))

    ouster = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ouster_share / "launch/driver.launch.py")),
        launch_arguments={
            "params_file": PathJoinSubstitution(
                [mist_config, "drivers/driver_ouster.yaml"]
            ),
            "ouster_ns": "ouster",
            "viz": "false",
            "os_driver_name": "os_driver",
        }.items(),
        condition=IfCondition(start_lidar),
    )

    vectornav = Node(
        package="vectornav",
        executable="vectornav",
        output="screen",
        parameters=[PathJoinSubstitution([mist_config, "drivers/driver_imu.yaml"])],
        condition=IfCondition(start_slam),
    )
    vn_sensor_msgs = Node(
        package="vectornav",
        executable="vn_sensor_msgs",
        output="screen",
        parameters=[PathJoinSubstitution([mist_config, "drivers/driver_imu.yaml"])],
        condition=IfCondition(start_slam),
    )

    lio_sam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(rover_share / "launch/auxilary/lio_sam.launch.py")
        ),
        launch_arguments={
            "lio_params_file": PathJoinSubstitution(
                [mist_config, "lio_sam/lio_sam.yaml"]
            ),
        }.items(),
        condition=IfCondition(start_slam),
    )

    # Same extrinsics as spot_lio_sam.launch.py. lidar_link is LIO-SAM's
    # lidar frame; os_lidar is the Ouster driver's.
    tf_body_oslidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_body_oslidar",
        arguments=["-0.082", "0", "-0.398", "0", "0", "0", "body", "os_lidar"],
        condition=IfCondition(start_slam),
    )
    tf_lidar_body = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_lidar_body",
        arguments=["0.0", "0", "0", "1.5708", "0", "0", "lidar_link", "body"],
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
            DeclareLaunchArgument("start_camera", default_value="false"),
            ouster,
            vectornav,
            vn_sensor_msgs,
            lio_sam,
            tf_body_oslidar,
            tf_lidar_body,
            OpaqueFunction(function=_spot_driver),
            OpaqueFunction(function=_spot_camera),
        ]
    )
