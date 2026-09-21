"""Platform footprint and inflation contracts shared by every robot launch."""

from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[4]
CONFIGS = {
    "botman": (
        REPO / "adapters/adapter_ros2/config/bunker.yaml",
        REPO / "swarmdeck_ros/src/swarmdeck_nav/launch/botman.launch.py",
        0.77,
        0.50,
        "_BUNKER_FOOTPRINT",
    ),
    "aslan": (
        REPO / "adapters/adapter_ros2/config/aslan_bunker.yaml",
        REPO / "swarmdeck_ros/src/swarmdeck_nav/launch/aslan.launch.py",
        0.77,
        0.50,
        "_BUNKER_FOOTPRINT",
    ),
    "asimov": (
        REPO / "adapters/adapter_ros2/config/unitree_g1.yaml",
        REPO / "swarmdeck_ros/src/swarmdeck_nav/launch/asimov.launch.py",
        0.30,
        0.45,
        "_G1_FOOTPRINT",
    ),
}


@pytest.mark.parametrize("robot", CONFIGS)
def test_platform_footprint_and_inflation_are_passed_to_nav2(robot):
    config_path, launch_path, radius, inflation, footprint_symbol = CONFIGS[robot]
    config = yaml.safe_load(config_path.read_text())
    source = launch_path.read_text()

    assert config["footprint_radius"] == radius
    assert config["footprint"]
    assert f'"inflation_radius": "{inflation:.2f}"' in source
    assert f'"footprint": {footprint_symbol}' in source
    assert '"robot_radius":' in source
