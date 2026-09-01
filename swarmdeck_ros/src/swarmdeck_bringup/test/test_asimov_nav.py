"""Static safety contract for Asimov's hardware Nav2 wiring.

These checks deliberately avoid launching hardware. They make the important
seams reviewable in CI: the live world -> base_link TF is used instead of a
synthesized one (unlike the Bunkers, Asimov's launch adds no odom_to_tf or
static lidar-frame node), Humble-compatible BT plugins, and the adapter as the
only path from autonomous velocity to the real driver.
"""

from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[4]
PARAMS = REPO / "swarmdeck_ros/src/swarmdeck_nav/config/asimov_nav2_params.yaml"
G1 = REPO / "adapters/adapter_ros2/config/unitree_g1.yaml"
COMPOSE = REPO / "deploy/compose/docker-compose.robot-asimov.yml"
LAUNCH = REPO / "swarmdeck_ros/src/swarmdeck_nav/launch/asimov.launch.py"
PROJECTOR = REPO / "swarmdeck_ros/src/swarmdeck_nav/src/footprint_cloud_to_scan.cpp"


def test_asimov_uses_the_live_world_frame_not_map():
    """global_frame is not rewritten by nav.launch.py; it must be right here.

    Asimov's onboard Unitree/Livox localization publishes world -> base_link
    directly -- adapter_ros2.py already depends on this same TF today for
    keyframe/SLAM alignment. Nav2 has to agree on the frame's real name.
    """
    params = yaml.safe_load(PARAMS.read_text())
    for section, path in (
        ("bt_navigator", ("bt_navigator", "ros__parameters")),
        ("local_costmap", ("local_costmap", "local_costmap", "ros__parameters")),
        ("global_costmap", ("global_costmap", "global_costmap", "ros__parameters")),
        ("behavior_server", ("behavior_server", "ros__parameters")),
    ):
        node = params
        for key in path:
            node = node[key]
        assert node["global_frame"] == "world", section
        assert node["robot_base_frame"] == "base_link", section


def test_asimov_launch_adds_no_synthetic_tf_bridge():
    """Unlike botman/aslan, nothing here should fabricate map -> base TF.

    The Bunkers need odom_to_tf and a static lidar-frame node because
    SuperOdometry publishes no map -> base edge and mislabels its pose's
    child frame. Asimov's onboard localization already publishes a live,
    correctly oriented world -> base_link (and world -> livox_frame) TF, so
    reintroducing either node here would fight it.
    """
    source = LAUNCH.read_text()
    assert "odom_to_tf" not in source
    assert "static_transform_publisher" not in source
    assert 'executable="footprint_cloud_to_scan"' in source
    assert '"input_topic": "/utlidar/cloud_livox_mid360"' in source
    assert '"output_frame": _SCAN_FRAME' in source
    assert '_SCAN_FRAME = "livox_frame"' in source
    assert '"robot_base_frame": "base_link"' in source
    assert '"tf_topic": "/tf"' in source
    assert '"tf_static_topic": "/tf_static"' in source


def test_asimov_sensor_yaw_is_flagged_unverified():
    """The Bunkers' pi-yaw mount offset was measured; Asimov's is not yet."""
    source = LAUNCH.read_text()
    assert "_SENSOR_YAW_IN_BASE = 0.0" in source
    assert "UNVERIFIED" in source


def test_asimov_passes_its_own_g1_footprint_not_a_bunker_default():
    """nav.launch.py rewrites robot_radius/footprint from launch args.

    A robot launched without an override plans as a 0.422 m Scout Mini disc.
    G1's torso is smaller and its own shape; both must come from the same
    numbers adapter_ros2's config uses so the adapter and Nav2 cannot report
    different chassis sizes.
    """
    source = LAUNCH.read_text()
    g1 = yaml.safe_load(G1.read_text())

    assert '"robot_radius": f"{_FOOTPRINT_RADIUS:.3f}"' in source
    assert '"footprint": _G1_FOOTPRINT' in source
    assert g1["footprint_radius"] == 0.30
    assert g1["footprint"] == [[0.18, 0.22], [0.18, -0.22], [-0.18, -0.22], [-0.18, 0.22]]


def test_asimov_uses_isolated_navigation_output():
    config = yaml.safe_load(G1.read_text())

    assert config["topics"]["cmd_vel"] == "/cmd_vel"
    assert config["topics"]["nav_cmd_vel"] == "/asimov_0/cmd_vel_nav"
    assert config["topics"]["plan"] == "/asimov_0/plan"
    assert config["topics"]["local_plan"] == "/asimov_0/local_plan"
    assert config["actions"]["navigate_to_pose"] == "/asimov_0/navigate_to_pose"


def test_asimov_velocity_limits_stay_inside_the_bridge_clamps():
    """asimov_odom_bridge.py hard-clamps on_cmd_vel to vx<=1.0, wz<=1.5.

    Nav2's own limits must sit comfortably inside those, not rely on them:
    the clamp is a last-resort safety net, not a place to discover that Nav2
    asked for more than the robot can honour.
    """
    params = yaml.safe_load(PARAMS.read_text())
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = params["velocity_smoother"]["ros__parameters"]

    assert 0.0 < follow_path["max_vel_x"] < 1.0
    assert -1.0 < follow_path["min_vel_x"] < 0.0
    assert 0.0 < follow_path["max_vel_theta"] < 1.5
    assert smoother["max_velocity"][0] <= follow_path["max_vel_x"]
    assert smoother["min_velocity"][0] >= follow_path["min_vel_x"]


def test_asimov_point_goals_do_not_require_a_final_heading():
    params = yaml.safe_load(PARAMS.read_text())
    goal_checker = params["controller_server"]["ros__parameters"]["goal_checker"]

    assert goal_checker["yaw_goal_tolerance"] > 3.141592653589793


def test_asimov_self_filter_accounts_for_limb_swing():
    """A walking gait swings limbs beyond the torso's static bounding box."""
    source = LAUNCH.read_text()

    assert "_SELF_FILTER_PADDING = 0.10" in source
    check_order = PROJECTOR.read_text()
    assert check_order.index("if (rear <= base_x") < check_order.index(
        "scan.ranges[index] ="
    )


def test_autonomous_velocity_can_only_reach_driver_through_adapter():
    compose = yaml.safe_load(COMPOSE.read_text())

    healthcheck = compose["services"]["nav2"]["healthcheck"]
    assert ". /opt/ros/humble/setup.sh" in healthcheck["test"][1]
    assert "/asimov_0/navigate_to_pose" in healthcheck["test"][1]
    assert "grep -q '^active'" in healthcheck["test"][1]

    adapter_dependencies = compose["services"]["adapter"]["depends_on"]
    assert adapter_dependencies["nav2"]["condition"] == "service_started"
    assert compose["services"]["nav2"]["network_mode"] == "host"
    # No odom_tf service: unlike Botman/Aslan, nothing here needs to survive a
    # Nav2 restart independently, since the localization producing world TF is
    # entirely outside this Compose file (the host-side native G1 bridge).
    assert "odom_tf" not in compose["services"]
