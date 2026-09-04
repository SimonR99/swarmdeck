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
    superodom_config = LaunchConfiguration(
        "config_file",
        default=str(_ADAPTER_CONFIG / "aslan_superodom_ouster.yaml"),
    )
    superodom_calib = LaunchConfiguration(
        "calibration_file",
        default=str(_ADAPTER_CONFIG / "aslan_superodom_ouster_calibration.yaml"),
    )
    vectornav_params = str(_ADAPTER_CONFIG / "aslan_vectornav.yaml")

    start_base = LaunchConfiguration("start_base")
    start_lidar = LaunchConfiguration("start_lidar")
    start_slam = LaunchConfiguration("start_slam")
    start_imu = LaunchConfiguration("start_imu")
    start_vectornav = LaunchConfiguration("start_vectornav")
    can_interface = LaunchConfiguration("can_interface")
    imu_topic = LaunchConfiguration("imu_topic")

    ouster_share = Path(get_package_share_directory("ouster_ros"))
    bunker_share = Path(get_package_share_directory("bunker_base"))

    ouster = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ouster_share / "launch/driver.launch.py")),
        launch_arguments={
            # Repo-owned copy of the robot's own MIST driver_ouster.yaml, which
            # differs from it in one line: proc_mask drops IMG|RAW|TLM. Those
            # eight publishers had no subscribers and shared the point cloud's
            # FastDDS shared-memory segment. The MIST workspace is read-only and
            # outside this repo, so keeping the override here is what stops it
            # being lost to the next workspace re-sync.
            "params_file": str(_ADAPTER_CONFIG / "aslan_driver_ouster.yaml"),
            "ouster_ns": "ouster",
            "viz": "false",
            "os_driver_name": "os_driver",
        }.items(),
        condition=IfCondition(start_lidar),
    )

    # VN-100T-CR on /dev/vectornav, started only when start_vectornav is true.
    vectornav = Node(
        package="vectornav",
        executable="vectornav",
        output="screen",
        parameters=[vectornav_params],
        condition=IfCondition(start_vectornav),
    )
    vn_sensor_msgs = Node(
        package="vectornav",
        executable="vn_sensor_msgs",
        output="screen",
        parameters=[vectornav_params],
        condition=IfCondition(start_vectornav),
    )

    # The VN-100 stamps its messages "vectornav" and nothing published a TF for
    # that frame. Sensor-to-sensor extrinsic, independent of the lidar-origin
    # base-frame convention; it only ADDS A CHILD to os_lidar, giving it no
    # second parent, which is why it is safe where base_link -> os_sensor is not.
    #
    # Rotation MEASURED 2026-09-01 by scripts/calibration/run_aslan_calibration.py
    # and recorded in aslan_superodom_calibration.yaml: os_lidar -> vectornav is
    # roll -0.03, pitch -0.85, yaw -90.65 deg (residual RMS 0.0257 rad/s over
    # 3416 samples).
    #
    # TRANSLATION IS NOT MEASURED. The calibration solved rotation only and
    # aslan_superodom_calibration.yaml carries a zero translation, so this
    # publishes zero too rather than inventing a lever arm. Measure the VN-100's
    # position relative to the lidar before trusting it for anything but
    # orientation.
    vectornav_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aslan_vectornav_tf",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "-0.000524",
            "--pitch", "-0.014835",
            "--yaw", "-1.582152",
            "--frame-id", "os_lidar",
            "--child-frame-id", "vectornav",
        ],
        condition=IfCondition(start_vectornav),
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
            vectornav_tf,
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
            DeclareLaunchArgument("start_vectornav", default_value="false"),
            DeclareLaunchArgument("imu_topic", default_value="/ouster/imu"),
            DeclareLaunchArgument(
                "config_file",
                default_value=str(_ADAPTER_CONFIG / "aslan_superodom_ouster.yaml"),
            ),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=str(
                    _ADAPTER_CONFIG / "aslan_superodom_ouster_calibration.yaml"
                ),
            ),
            # Aslan's upstream launch expects a USB-CAN adapter named can2.
            # Keep it explicit and overridable rather than silently selecting
            # one of the Jetson's currently-down native CAN controllers.
            DeclareLaunchArgument("can_interface", default_value="can2"),
            ouster,
            superodom,
            bunker,
        ]
    )
