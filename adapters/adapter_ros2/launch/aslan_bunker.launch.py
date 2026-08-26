"""Minimal Aslan hardware launch owned by SwarmDeck.

The upstream full-stack launch imports camera packages that are not installed
on Aslan and hardcodes the base interface. This launch starts the interfaces
SwarmDeck needs: Ouster, VectorNav VN-100, SuperOdometry, and the Bunker
base. All MIST source remains mounted read-only; IMU driver and lidar-IMU
calibration live in this package because the mist copies target /ouster/imu.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

_ADAPTER_CONFIG = Path(__file__).resolve().parents[1] / "config"


def generate_launch_description() -> LaunchDescription:
    mist_config = Path("/workspace/src/control/rover_launch/config")
    superodom_config = str(mist_config / "superodom/os1_128.yaml")
    superodom_calib = str(_ADAPTER_CONFIG / "aslan_superodom_calibration.yaml")
    vectornav_params = str(_ADAPTER_CONFIG / "aslan_vectornav.yaml")

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

    # VN-100T-CR on /dev/vectornav. The Ouster IMU stays published but is not
    # SuperOdom's imu_topic: identity lidar-IMU extrinsics against that sensor
    # reset preintegration on "Large bias" every ~0.5 s and jumped the map.
    vectornav = Node(
        package="vectornav",
        executable="vectornav",
        output="screen",
        parameters=[vectornav_params],
        condition=IfCondition(start_imu),
    )
    vn_sensor_msgs = Node(
        package="vectornav",
        executable="vn_sensor_msgs",
        output="screen",
        parameters=[vectornav_params],
        condition=IfCondition(start_imu),
    )

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
            vectornav,
            vn_sensor_msgs,
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
            DeclareLaunchArgument("start_imu", default_value="true"),
            DeclareLaunchArgument("imu_topic", default_value="/vectornav/imu"),
            # Aslan's upstream launch expects a USB-CAN adapter named can2.
            # Keep it explicit and overridable rather than silently selecting
            # one of the Jetson's currently-down native CAN controllers.
            DeclareLaunchArgument("can_interface", default_value="can2"),
            ouster,
            superodom,
            bunker,
        ]
    )
