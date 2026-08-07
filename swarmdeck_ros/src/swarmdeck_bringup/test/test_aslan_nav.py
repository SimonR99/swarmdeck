"""Static safety contract for Aslan's hardware wiring."""

from pathlib import Path

import yaml


REPO = Path(__file__).resolve().parents[4]
CONFIG = REPO / "adapters/adapter_ros2/config/aslan_bunker.yaml"
COMPOSE = REPO / "docker-compose.robot-aslan.yml"
LAUNCH = REPO / "swarmdeck_ros/src/swarmdeck_nav/launch/aslan.launch.py"
ROBOT_LAUNCH = REPO / "adapters/adapter_ros2/launch/aslan_bunker.launch.py"


def test_aslan_uses_isolated_navigation_output():
    config = yaml.safe_load(CONFIG.read_text())

    assert config["topics"]["cmd_vel"] == "/cmd_vel"
    assert config["topics"]["nav_cmd_vel"] == "/aslan_0/cmd_vel_nav"
    assert config["actions"]["navigate_to_pose"] == "/aslan_0/navigate_to_pose"


def test_aslan_services_share_the_robot_ros_domain():
    compose = yaml.safe_load(COMPOSE.read_text())

    for service in ("robot_stack", "nav2", "adapter"):
        assert "ROS_DOMAIN_ID" in compose["services"][service]["environment"]
    assert (
        compose["services"]["adapter"]["depends_on"]["nav2"]["condition"]
        == "service_healthy"
    )


def test_aslan_launch_keeps_mist_workspace_read_only_and_can_explicit():
    compose = yaml.safe_load(COMPOSE.read_text())
    volumes = compose["services"]["robot_stack"]["volumes"]

    assert "/ssd/mist_ws_ros2:/workspace:ro" in volumes
    assert "${ASLAN_CAN_INTERFACE:-can2}" in compose["services"]["robot_stack"][
        "command"
    ][2]
    assert 'default_value="can2"' in ROBOT_LAUNCH.read_text()


def test_aslan_nav_namespace_and_tf_bridge_are_distinct():
    source = LAUNCH.read_text()

    assert '"namespace": "aslan_0"' in source
    assert 'name="aslan_odom_to_tf"' in source
    assert '"use_receive_time": True' in source
