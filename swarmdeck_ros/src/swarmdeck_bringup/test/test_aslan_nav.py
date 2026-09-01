"""Static safety contract for Aslan's hardware wiring."""

from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[4]
CONFIG = REPO / "adapters/adapter_ros2/config/aslan_bunker.yaml"
COMPOSE = REPO / "deploy/compose/docker-compose.robot-aslan.yml"
LAUNCH = REPO / "swarmdeck_ros/src/swarmdeck_nav/launch/aslan.launch.py"
PARAMS = REPO / "swarmdeck_ros/src/swarmdeck_nav/config/botman_nav2_params.yaml"
PROJECTOR = (
    REPO
    / "swarmdeck_ros/src/swarmdeck_nav/src/footprint_cloud_to_scan.cpp"
)
ROBOT_LAUNCH = REPO / "adapters/adapter_ros2/launch/aslan_bunker.launch.py"


def test_aslan_uses_isolated_navigation_output():
    config = yaml.safe_load(CONFIG.read_text())

    assert config["topics"]["cmd_vel"] == "/cmd_vel"
    assert config["topics"]["nav_cmd_vel"] == "/aslan_0/cmd_vel_nav"
    assert config["topics"]["plan"] == "/aslan_0/plan"
    assert config["topics"]["local_plan"] == "/aslan_0/local_plan"
    assert config["actions"]["navigate_to_pose"] == "/aslan_0/navigate_to_pose"


def test_aslan_services_share_the_robot_ros_domain():
    compose = yaml.safe_load(COMPOSE.read_text())

    for service in ("robot_stack", "lidar", "slam", "nav2", "adapter"):
        assert "ROS_DOMAIN_ID" in compose["services"][service]["environment"]
    adapter_dependencies = compose["services"]["adapter"]["depends_on"]
    assert adapter_dependencies["lidar"]["condition"] == "service_started"
    assert adapter_dependencies["slam"]["condition"] == "service_started"
    assert adapter_dependencies["nav2"]["condition"] == "service_started"
    assert "robot_stack" not in adapter_dependencies


def test_aslan_launch_keeps_mist_workspace_read_only_and_can_explicit():
    compose = yaml.safe_load(COMPOSE.read_text())
    volumes = compose["services"]["robot_stack"]["volumes"]
    command = compose["services"]["robot_stack"]["command"][2]

    assert "${ASLAN_MIST_WS:-/ssd/mist_ws_ros2}:/workspace:ro" in volumes
    assert "${ASLAN_CAN_INTERFACE:-can2}" in command
    assert "start_base:=true" in command
    assert "start_lidar:=false" in command
    assert "start_slam:=false" in command
    assert 'default_value="can2"' in ROBOT_LAUNCH.read_text()
    assert compose["services"]["robot_stack"]["profiles"] == ["base"]
    assert "profiles" not in compose["services"]["lidar"]
    assert "profiles" not in compose["services"]["slam"]
    assert "profiles" not in compose["services"]["adapter"]


def test_aslan_nav_namespace_and_tf_bridge_are_distinct():
    source = LAUNCH.read_text()

    assert '"namespace": "aslan_0"' in source
    assert 'name="aslan_odom_to_tf"' in source
    assert '"use_receive_time": True' in source
    assert '"robot_base_frame": _BASE_FRAME' in source
    assert 'name="aslan_base_to_scan"' in source
    assert '"--frame-id",\n            _BASE_FRAME' in source
    assert '"--child-frame-id",\n            _SCAN_FRAME' in source
    assert '"--yaw",\n            "3.141592653589793"' in source
    assert 'executable="footprint_cloud_to_scan"' in source
    assert '"input_topic": "/ouster/points"' in source
    assert '"min_height": _OBSTACLE_MIN_HEIGHT' in source
    assert '"max_height": _OBSTACLE_MAX_HEIGHT' in source
    assert '"footprint_padding": _SELF_FILTER_PADDING' in source
    assert '"obstacle_scan_topic": _SCAN_TOPIC' in source
    assert '"obstacle_sensor_frame": _SCAN_FRAME' in source
    assert '"robot_radius": _BUNKER_RADIUS' in source
    assert '"footprint": _BUNKER_FOOTPRINT' in source
    assert "_LIDAR_X = 0.150" in source
    assert '"inflation_radius": "0.50"' in source


def test_aslan_filters_its_footprint_before_cloud_projection():
    """A self return must not mask a farther, valid point in the same beam."""
    source = PROJECTOR.read_text()

    reject = source.index("if (rear <= base_x")
    project = source.index("scan.ranges[index] =")
    assert reject < project
    assert "pz < min_height_ || pz > max_height_" in source
    assert "std::numeric_limits<float>::infinity()" in source


def test_aslan_runs_exactly_one_map_to_physical_base_broadcaster():
    """Compose already runs odom_tf; Nav2 must not start a second copy.

    Both bridges relay the same /laser_odometry pose and both stamp it with
    their own receipt time, so leaving both alive publishes every pose twice
    under two timestamps. Measured on the robot 2026-08-25 with both running:
    12.95 Hz of map -> base against a 7.04 Hz odometry source, 49% of
    broadcasts an identical pose re-sent up to 26 ms later, and 7.8% of stamps
    out of order -- a later stamp carrying an earlier pose. Parked, that is
    invisible, because duplicating an unchanging pose changes nothing. In
    motion every tf2 lookup interpolates across an inverted pair.

    Botman has always passed publish_odom_tf:=false; aslan.launch.py simply
    never grew the argument, so aslan's odom_tf service was additive.
    """
    source = LAUNCH.read_text()
    compose = yaml.safe_load(COMPOSE.read_text())
    nav2_command = compose["services"]["nav2"]["command"]

    assert 'DeclareLaunchArgument("publish_odom_tf", default_value="true")' in source
    assert "condition=IfCondition(publish_odom_tf)" in source
    assert "from launch.conditions import IfCondition" in source
    assert "publish_odom_tf:=false" in nav2_command
    # The standalone bridge is the one that must survive a Nav2 restart.
    odom_tf_command = compose["services"]["odom_tf"]["command"]
    assert "__node:=aslan_odom_to_tf_host" in odom_tf_command
    assert "child_frame:=aslan_base_link" in odom_tf_command


def test_aslan_uses_the_same_bunker_footprint_radius():
    config = yaml.safe_load(CONFIG.read_text())

    assert config["footprint_radius"] == 0.77
    assert config["base_frame"] == "aslan_base_link"
    compose = yaml.safe_load(COMPOSE.read_text())
    assert compose["services"]["oak_mount_tf"]["command"][-3:] == [
        "aslan_base_link",
        "--child-frame-id",
        "oak-d-base-frame",
    ]


def test_aslan_uses_the_same_forward_or_reverse_nav_limits():
    params = yaml.safe_load(PARAMS.read_text())
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]

    assert follow_path["min_vel_x"] == -0.2
    assert follow_path["max_vel_x"] == 0.4
    assert "PreferForward" in follow_path["critics"]


def test_aslan_slam_uses_ouster_imu():
    source = ROBOT_LAUNCH.read_text()
    compose = yaml.safe_load(COMPOSE.read_text())
    slam_command = compose["services"]["slam"]["command"][2]

    assert 'DeclareLaunchArgument("start_imu", default_value="true")' in source
    assert (
        'DeclareLaunchArgument("imu_topic", default_value="/ouster/imu")' in source
    )
    assert (
        'DeclareLaunchArgument("start_vectornav", default_value="false")' in source
    )
    assert "imu_preintegration_node" in source
    assert "launch/os1_128.launch.py" not in source
    assert "aslan_superodom_ouster_calibration.yaml" in source
    assert ". /workspace/install/setup.bash" in slam_command
    assert "start_imu:=true" in slam_command
    assert "imu_topic:=$${ASLAN_IMU_TOPIC:-/ouster/imu}" in slam_command
    assert "aslan_superodom_ouster_calibration.yaml" in slam_command
