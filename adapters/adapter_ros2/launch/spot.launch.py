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
        Node(
            package="spot_driver",
            executable="state_publisher_node",
            name="state_publisher_node",
            output="screen",
            parameters=[
                str(mist_config / "spot.yaml"),
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

    lio_params = PathJoinSubstitution([mist_config, "lio_sam/lio_sam.yaml"])
    tf_map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_map_odom",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "map", "odom_link"],
        parameters=[lio_params],
        output="screen",
        condition=IfCondition(start_slam),
    )
    lio_sam_imu = Node(
        package="lio_sam",
        executable="lio_sam_imuPreintegration",
        name="lio_sam_imuPreintegration",
        parameters=[lio_params],
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(start_slam),
    )
    lio_sam_img = Node(
        package="lio_sam",
        executable="lio_sam_imageProjection",
        name="lio_sam_imageProjection",
        parameters=[lio_params],
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(start_slam),
    )
    lio_sam_feat = Node(
        package="lio_sam",
        executable="lio_sam_featureExtraction",
        name="lio_sam_featureExtraction",
        parameters=[lio_params],
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(start_slam),
    )
    lio_sam_map = Node(
        package="lio_sam",
        executable="lio_sam_mapOptimization",
        name="lio_sam_mapOptimization",
        parameters=[lio_params],
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(start_slam),
    )

    # Same extrinsics as spot_lio_sam.launch.py. lidar_link is LIO-SAM's
    # lidar frame; os_lidar is the Ouster driver's.
    tf_body_oslidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_body_oslidar",
        arguments=[
            "--x", "-0.082",
            "--y", "0.0",
            "--z", "-0.398",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "body",
            "--child-frame-id", "os_lidar",
        ],
        condition=IfCondition(start_slam),
    )
    tf_lidar_body = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_lidar_body",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "1.5708",
            "--frame-id", "lidar_link",
            "--child-frame-id", "body",
        ],
        condition=IfCondition(start_slam),
    )
    tf_lidar_imubody = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_lidar_imubody",
        arguments=[
            "--x", "0.062",
            "--y", "-0.104",
            "--z", "-0.04",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "-1.5708",
            "--frame-id", "lidar_link",
            "--child-frame-id", "imu_body",
        ],
        condition=IfCondition(start_slam),
    )
    tf_body_d435 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_body_d435",
        arguments=[
            "--x", "0.35",
            "--y", "0.0",
            "--z", "0.20",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "body",
            "--child-frame-id", "d435_link",
        ],
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
            tf_map_odom,
            lio_sam_imu,
            lio_sam_img,
            lio_sam_feat,
            lio_sam_map,
            tf_body_oslidar,
            tf_lidar_body,
            tf_lidar_imubody,
            tf_body_d435,
            OpaqueFunction(function=_spot_driver),
            OpaqueFunction(function=_spot_camera),
        ]
    )
