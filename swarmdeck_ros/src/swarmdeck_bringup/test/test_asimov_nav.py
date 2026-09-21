"""Startup ownership contracts for simulation and hardware environments."""

from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[4]
NAV = REPO / "swarmdeck_ros/src/swarmdeck_nav/launch/nav.launch.py"
SESSION = REPO / "swarmdeck_ros/src/swarmdeck_bringup/launch/session.launch.py"
HARDWARE_LAUNCHES = {
    "botman": REPO / "swarmdeck_ros/src/swarmdeck_nav/launch/botman.launch.py",
    "aslan": REPO / "swarmdeck_ros/src/swarmdeck_nav/launch/aslan.launch.py",
    "asimov": REPO / "swarmdeck_ros/src/swarmdeck_nav/launch/asimov.launch.py",
}


@pytest.mark.parametrize("robot, launch_path", HARDWARE_LAUNCHES.items())
def test_bounded_startup_is_enabled_for_simulation_only(robot, launch_path):
    nav_source = NAV.read_text()
    session_source = SESSION.read_text()
    hardware_source = launch_path.read_text()

    assert (
        'DeclareLaunchArgument("bounded_startup", default_value="false")' in nav_source
    )
    assert '"bounded_startup": "true"' in session_source
    assert '"bounded_startup"' not in hardware_source, robot
