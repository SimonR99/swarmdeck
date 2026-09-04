"""Minimal Botman hardware launch owned by SwarmDeck.

Starts the interfaces SwarmDeck needs on Botman: Ouster OS-0-64, VectorNav VN-100,
SuperOdometry, and the Bunker base. All MIST source remains mounted read-only.
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
        default=str(_ADAPTER_CONFIG / "botman_superodom.yaml"),
    )
    superodom_calib = LaunchConfiguration(
        "calibration_file",
        default=str(_ADAPTER_CONFIG / "botman_superodom_calibration.yaml"),
    )
    vectornav_params = str(_ADAPTER_CONFIG / "botman_vectornav.yaml")

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
            "params_file": str(_ADAPTER_CONFIG / "botman_driver_ouster.yaml"),
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
    # that frame, so the IMU could not be related to anything. This edge is a
    # sensor-to-sensor extrinsic, independent of the lidar-origin base-frame
    # convention, and it only ADDS A CHILD to os_lidar -- it gives os_lidar no
    # second parent, which is why it is safe where a base_link -> os_sensor edge
    # is not.
    #
    # MEASURED 2026-08-31 by scripts/calibration/calibrate_imu_to_imu.py: the
    # gyro pair gives the rotation, the accelerometer pair the x/y offset, over
    # a 0.6 rad/s spin, composed with the Ouster factory os_lidar -> os_imu.
    # The quaternion is a -92.52 deg yaw.
    #
    # Z IS A CALIBRATION RESULT TOO, not a tape reading. calibrate_imu_to_imu.py
    # refuses a yaw spin for exactly the reason it would leave z unobservable,
    # and asks instead for rotation about non-vertical axes: press a front
    # corner and release ~5 times, press a side corner ~5 times, lift a side.
    # Run that way across the three-IMU set (OAK-D, Ouster, VN-100), the
    # accelerometer pair does resolve the vertical component. The camera-lidar
    # leg of the same session used the ChArUco fiducial panel.
    #
    # Z is still the least-constrained of the three: the recorded excitation for
    # this robot was singular values [16.75, 10.58, 0.77], so the third axis was
    # driven far less hard than the first two. Treat it as measured but with the
    # widest error bar, and re-run with brisker rocking if it matters.
    vectornav_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="botman_vectornav_tf",
        output="screen",
        arguments=[
            "--x", "0.0010",
            "--y", "0.0608",
            "--z", "-0.284",
            "--qx", "-0.008202",
            "--qy", "0.002840",
            "--qz", "-0.722461",
            "--qw", "0.691357",
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
                "map_dir": "/tmp/botman_superodom.pcd",
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

    bunker_launch = bunker_share / "launch/bunker_base.launch.py"
    if not bunker_launch.exists():
        bunker_launch = bunker_share / "launch/bunker_gnm.launch.py"

    bunker = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bunker_launch)),
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
            DeclareLaunchArgument("imu_topic", default_value="/vectornav/imu"),
            DeclareLaunchArgument(
                "config_file",
                default_value=str(_ADAPTER_CONFIG / "botman_superodom.yaml"),
            ),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=str(
                    _ADAPTER_CONFIG / "botman_superodom_calibration.yaml"
                ),
            ),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            ouster,
            superodom,
            bunker,
        ]
    )
