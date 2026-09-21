"""Velocity limits shared by the controller and each platform adapter."""

from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[4]
PARAMS = {
    "botman": REPO / "swarmdeck_ros/src/swarmdeck_nav/config/botman_nav2_params.yaml",
    "aslan": REPO / "swarmdeck_ros/src/swarmdeck_nav/config/botman_nav2_params.yaml",
    "asimov": REPO / "swarmdeck_ros/src/swarmdeck_nav/config/asimov_nav2_params.yaml",
}

# Platform limits are the adapter/driver safety envelopes. Nav2 and its
# smoother must stay within those limits rather than relying on a downstream
# clamp to make an unsafe command valid.
PLATFORM_LIMITS = {
    "botman": (0.40, 0.60),
    "aslan": (0.40, 0.60),
    "asimov": (0.25, 0.30),
}


@pytest.mark.parametrize("robot", PARAMS)
def test_platform_velocity_limits_stay_inside_driver_limits(robot):
    params = yaml.safe_load(PARAMS[robot].read_text())
    controller = params["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = params["velocity_smoother"]["ros__parameters"]
    max_linear, max_angular = PLATFORM_LIMITS[robot]

    assert -max_linear <= controller["min_vel_x"] <= 0.0
    assert 0.0 < controller["max_vel_x"] <= max_linear
    assert 0.0 < controller["max_vel_theta"] <= max_angular
    assert -max_linear <= smoother["min_velocity"][0] <= controller["min_vel_x"] + 1e-9
    assert smoother["max_velocity"][0] <= max_linear
    assert -max_angular <= smoother["min_velocity"][2] <= 0.0
    assert 0.0 <= smoother["max_velocity"][2] <= max_angular
