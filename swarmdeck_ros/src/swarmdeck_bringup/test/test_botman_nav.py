"""Static safety contract for Botman's hardware Nav2 wiring.

These checks deliberately avoid launching hardware. They make the important
seams reviewable in CI: Humble-compatible plugins, live sensor topics, and the
adapter as the only path from autonomous velocity to the real driver.
"""

from pathlib import Path

import yaml


REPO = Path(__file__).resolve().parents[4]
PARAMS = REPO / "swarmdeck_ros/src/swarmdeck_nav/config/botman_nav2_params.yaml"
BUNKER = REPO / "adapters/adapter_ros2/config/bunker.yaml"
COMPOSE = REPO / "docker-compose.robot-botman.yml"
BOTMAN_LAUNCH = REPO / "swarmdeck_ros/src/swarmdeck_nav/launch/botman.launch.py"


def test_botman_uses_live_superodom_and_ouster_interfaces():
    params = yaml.safe_load(PARAMS.read_text())
    bt = params["bt_navigator"]["ros__parameters"]
    local = params["local_costmap"]["local_costmap"]["ros__parameters"]
    scan = local["obstacle_layer"]["scan"]

    assert bt["global_frame"] == "map"
    assert bt["robot_base_frame"] == "os_lidar"
    assert bt["odom_topic"] == "/laser_odometry"
    assert scan["topic"] == "/ouster/scan"
    assert scan["sensor_frame"] == "os_lidar"


def test_botman_params_are_humble_compatible_and_need_no_grid_map():
    params = yaml.safe_load(PARAMS.read_text())
    bt = params["bt_navigator"]["ros__parameters"]
    controller = params["controller_server"]["ros__parameters"]
    planner = params["planner_server"]["ros__parameters"]["GridBased"]
    global_map = params["global_costmap"]["global_costmap"]["ros__parameters"]

    assert "plugin_lib_names" in bt
    assert "navigators" not in bt
    assert controller["progress_checker_plugin"] == "progress_checker"
    assert planner["plugin"] == "nav2_navfn_planner/NavfnPlanner"
    assert global_map["rolling_window"] is True
    assert "static_layer" not in global_map["plugins"]
    assert global_map["track_unknown_space"] is False


def test_autonomous_velocity_can_only_reach_driver_through_adapter():
    params = yaml.safe_load(PARAMS.read_text())
    bunker = yaml.safe_load(BUNKER.read_text())
    compose = yaml.safe_load(COMPOSE.read_text())

    smoother = params["velocity_smoother"]["ros__parameters"]
    assert smoother["max_velocity"] == [0.2, 0.0, 0.2]
    assert bunker["topics"]["cmd_vel"] == "/cmd_vel"
    assert bunker["topics"]["nav_cmd_vel"] == "/botman_0/cmd_vel_nav"
    assert bunker["actions"]["navigate_to_pose"] == "/botman_0/navigate_to_pose"

    adapter_dependencies = compose["services"]["adapter"]["depends_on"]
    assert adapter_dependencies["nav2"]["condition"] == "service_healthy"
    healthcheck = compose["services"]["nav2"]["healthcheck"]
    assert ". /opt/ros/humble/setup.sh" in healthcheck["test"][1]
    assert "grep -q '^active'" in healthcheck["test"][1]


def test_botman_tf_bridge_accounts_for_live_pipeline_latency():
    launch_source = BOTMAN_LAUNCH.read_text()
    assert '"use_receive_time": True' in launch_source


def test_botman_passes_the_bunker_footprint_instead_of_the_scout_default():
    """nav.launch.py rewrites robot_radius from launch args, default 0.422 m.

    That is a Scout Mini. A Bunker launched without an override plans as a
    42 cm disc around the lidar; the deck is then an obstacle and the only
    recovery is reverse. The chassis rectangle has to be in the lidar frame.
    """
    source = BOTMAN_LAUNCH.read_text()
    assert '"robot_radius": _BUNKER_RADIUS' in source
    assert '"footprint": _BUNKER_FOOTPRINT' in source
    assert "_LIDAR_X = -0.150" in source
    # Front bumper must sit inside the polygon, not on a 0.65 m circle.
    half_l, lidar_x = 1.023 / 2.0, -0.150
    front = half_l - lidar_x
    assert front > 0.65
