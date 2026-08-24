"""Static safety contract for Aslan's hardware wiring."""

from pathlib import Path

import yaml


REPO = Path(__file__).resolve().parents[4]
CONFIG = REPO / "adapters/adapter_ros2/config/aslan_bunker.yaml"
COMPOSE = REPO / "deploy/compose/docker-compose.robot-aslan.yml"
LAUNCH = REPO / "swarmdeck_ros/src/swarmdeck_nav/launch/aslan.launch.py"
PARAMS = REPO / "swarmdeck_ros/src/swarmdeck_nav/config/botman_nav2_params.yaml"
ROBOT_LAUNCH = REPO / "adapters/adapter_ros2/launch/aslan_bunker.launch.py"


def test_aslan_uses_isolated_navigation_output():
    config = yaml.safe_load(CONFIG.read_text())

    assert config["topics"]["cmd_vel"] == "/cmd_vel"
    assert config["topics"]["nav_cmd_vel"] == "/aslan_0/cmd_vel_nav"
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
    assert '"robot_radius": _BUNKER_RADIUS' in source
    assert '"footprint": _BUNKER_FOOTPRINT' in source
    assert "_LIDAR_X = 0.150" in source
    assert '"inflation_radius": "0.50"' in source


def test_aslan_uses_the_same_bunker_footprint_radius():
    config = yaml.safe_load(CONFIG.read_text())

    assert config["footprint_radius"] == 0.77


def test_aslan_uses_the_same_forward_or_reverse_nav_limits():
    params = yaml.safe_load(PARAMS.read_text())
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]

    assert follow_path["min_vel_x"] == -0.2
    assert follow_path["max_vel_x"] == 0.4
    assert "PreferForward" in follow_path["critics"]


def test_aslan_slam_uses_vectornav_imu():
    source = ROBOT_LAUNCH.read_text()
    compose = yaml.safe_load(COMPOSE.read_text())
    slam_command = compose["services"]["slam"]["command"][2]
    vn = yaml.safe_load(
        (REPO / "adapters/adapter_ros2/config/aslan_vectornav.yaml").read_text()
    )

    assert 'DeclareLaunchArgument("start_imu", default_value="true")' in source
    assert 'DeclareLaunchArgument("imu_topic", default_value="/vectornav/imu")' in source
    assert 'executable="vectornav"' in source
    assert 'executable="vn_sensor_msgs"' in source
    assert "imu_preintegration_node" in source
    assert "launch/os1_128.launch.py" not in source
    assert "aslan_superodom_calibration.yaml" in source
    assert ". /workspace/install/setup.bash" in slam_command
    assert "start_imu:=true" in slam_command
    assert "imu_topic:=/vectornav/imu" in slam_command
    assert vn["vectornav"]["ros__parameters"]["port"] == "/dev/vectornav"
    assert vn["vectornav"]["ros__parameters"]["baud"] == 115200
