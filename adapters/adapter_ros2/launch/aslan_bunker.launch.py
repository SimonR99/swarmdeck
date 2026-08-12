"""Minimal Aslan hardware launch owned by SwarmDeck.

The upstream full-stack launch imports camera and VectorNav packages that are
not installed on Aslan and hardcodes the base interface. This launch starts
only the interfaces SwarmDeck needs: Ouster, SuperOdometry, and the Bunker
base. All MIST source/configuration remains mounted read-only.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description() -> LaunchDescription:
    mist_config = Path("/workspace/src/control/rover_launch/config")
    superodom_config = str(mist_config / "superodom/os1_128.yaml")
    superodom_calib = str(mist_config / "superodom/os1_128_calibration.yaml")

    start_base = LaunchConfiguration("start_base")
    start_lidar = LaunchConfiguration("start_lidar")
    start_slam = LaunchConfiguration("start_slam")
    start_imu = LaunchConfiguration("start_imu")
    can_interface = LaunchConfiguration("can_interface")
    imu_topic = LaunchConfiguration("imu_topic")

    ouster_share = Path(get_package_share_directory("ouster_ros"))
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

    # SuperOdom's stock os1_128.launch.py always starts imu_preintegration and
    # always subscribes to /ouster/imu. Aslan's Ouster IMU is 100 Hz but the
    # lidar-IMU extrinsics are identity, preintegration resets on "Large bias"
    # every ~0.5 s, and laser mapping then treats that failed predictor as a
    # large motion — which re-runs a broken IMU init and makes the map pose
    # jump. Launch the lidar nodes ourselves and keep IMU off until someone
    # measures a real extrinsic.
    feature_extraction = Node(
        package="super_odometry",
        executable="feature_extraction_node",
        output="screen",
        parameters=[
            superodom_config,
            {
                "calibration_file": superodom_calib,
                "imu_topic": imu_topic,
            },
        ],
    )
    laser_mapping = Node(
        package="super_odometry",
        executable="laser_mapping_node",
        output="screen",
        parameters=[
            superodom_config,
            {
                "calibration_file": superodom_calib,
                "imu_topic": imu_topic,
                "map_dir": "/tmp/aslan_superodom.pcd",
            },
        ],
        remappings=[
            ("laser_odom_to_init", "integrated_to_init"),
        ],
    )
    imu_preintegration = Node(
        package="super_odometry",
        executable="imu_preintegration_node",
        output="screen",
        parameters=[
            superodom_config,
            {
                "calibration_file": superodom_calib,
                "imu_topic": imu_topic,
            },
        ],
        condition=IfCondition(start_imu),
    )
    superodom = GroupAction(
        [
            SetParameter(name="use_sim_time", value=False),
            feature_extraction,
            laser_mapping,
            imu_preintegration,
        ],
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
            DeclareLaunchArgument("start_imu", default_value="false"),
            # Unpublished topic: feature extraction then runs lidar-only.
            # Pass imu_topic:=/ouster/imu with start_imu:=true after calibration.
            DeclareLaunchArgument("imu_topic", default_value="/aslan/imu_disabled"),
            # Aslan's upstream launch expects a USB-CAN adapter named can2.
            # Keep it explicit and overridable rather than silently selecting
            # one of the Jetson's currently-down native CAN controllers.
            DeclareLaunchArgument("can_interface", default_value="can2"),
            ouster,
            superodom,
            bunker,
        ]
    )
