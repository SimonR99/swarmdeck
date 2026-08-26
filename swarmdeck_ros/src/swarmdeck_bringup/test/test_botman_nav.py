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
COMPOSE = REPO / "deploy/compose/docker-compose.robot-botman.yml"
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


def test_botman_params_are_humble_compatible_and_keep_live_local_costmap():
    params = yaml.safe_load(PARAMS.read_text())
    bt = params["bt_navigator"]["ros__parameters"]
    controller = params["controller_server"]["ros__parameters"]
    planner = params["planner_server"]["ros__parameters"]["GridBased"]
    global_map = params["global_costmap"]["global_costmap"]["ros__parameters"]
    local_map = params["local_costmap"]["local_costmap"]["ros__parameters"]

    assert "plugin_lib_names" in bt
    assert "navigators" not in bt
    assert controller["progress_checker_plugin"] == "progress_checker"
    assert planner["plugin"] == "nav2_navfn_planner/NavfnPlanner"
    # NOT rolling. This asserted True until 2026-08-25, on the grounds that a
    # rolling window "keeps Nav2 usable before a collaborative map exists".
    # That is handled on the server instead: MapService.nav_grid serves a
    # robot's OWN raytraced grid while it is unmerged, so the static layer has
    # a map from the first upload. Verified live -- spot_0, online and in no
    # component, got HTTP 200 from /api/map/nav/spot_0.
    #
    # Rolling cost full-map planning: the window follows the robot, so the
    # planner saw 40 x 40 m of maps measuring 66.0 x 58.4 m (botman_0) and
    # 56.9 x 45.6 m (aslan_0). See test_costmap_sources.py.
    assert global_map["rolling_window"] is False
    assert "static_layer" in global_map["plugins"]
    assert global_map["static_layer"]["map_topic"] == "/global_map"
    # Collision authority stays on live sensors. A foreign map here is a crash.
    assert "static_layer" not in local_map["plugins"]
    assert global_map["track_unknown_space"] is True


def test_bunker_point_goals_do_not_require_a_final_heading():
    params = yaml.safe_load(PARAMS.read_text())
    goal_checker = params["controller_server"]["ros__parameters"]["goal_checker"]

    # The adapter must provide a quaternion for NavigateToPose, but dashboard
    # goals contain only x/y. A tolerance greater than pi makes every planar
    # heading valid, so reaching XY ends navigation without a corrective turn.
    assert goal_checker["yaw_goal_tolerance"] > 3.141592653589793


def test_bunker_point_goals_can_use_forward_or_reverse_velocity():
    params = yaml.safe_load(PARAMS.read_text())
    controller = params["controller_server"]["ros__parameters"]
    follow_path = controller["FollowPath"]
    smoother = params["velocity_smoother"]["ros__parameters"]

    assert follow_path["publish_local_plan"] is True
    assert follow_path["max_vel_x"] > 0.0
    assert follow_path["min_vel_x"] < 0.0
    assert smoother["min_velocity"][0] <= follow_path["min_vel_x"]
    assert "PreferForward" in follow_path["critics"]
    assert follow_path["PreferForward.penalty"] > 0.0


def test_autonomous_velocity_can_only_reach_driver_through_adapter():
    params = yaml.safe_load(PARAMS.read_text())
    bunker = yaml.safe_load(BUNKER.read_text())
    compose = yaml.safe_load(COMPOSE.read_text())

    smoother = params["velocity_smoother"]["ros__parameters"]
    assert smoother["max_velocity"] == [0.4, 0.0, 0.6]
    assert bunker["topics"]["cmd_vel"] == "/cmd_vel"
    assert bunker["topics"]["nav_cmd_vel"] == "/botman_0/cmd_vel_nav"
    assert bunker["topics"]["plan"] == "/botman_0/plan"
    assert bunker["topics"]["local_plan"] == "/botman_0/local_plan"
    assert bunker["actions"]["navigate_to_pose"] == "/botman_0/navigate_to_pose"

    adapter_dependencies = compose["services"]["adapter"]["depends_on"]
    assert adapter_dependencies["nav2"]["condition"] == "service_started"
    healthcheck = compose["services"]["nav2"]["healthcheck"]
    assert ". /opt/ros/humble/setup.sh" in healthcheck["test"][1]
    assert "grep -q '^active'" in healthcheck["test"][1]


def test_botman_tf_bridge_accounts_for_live_pipeline_latency():
    launch_source = BOTMAN_LAUNCH.read_text()
    assert '"use_receive_time": True' in launch_source


def test_botman_passes_the_bunker_footprint_instead_of_the_scout_default():
    """nav.launch.py rewrites robot_radius from launch args, default 0.422 m.

    That is a Scout Mini. A Bunker launched without an override plans as a
    42 cm disc around the lidar; the deck is then an obstacle and forward
    planning can fail. The chassis rectangle has to be in the lidar frame.
    """
    source = BOTMAN_LAUNCH.read_text()
    assert '"robot_radius": _BUNKER_RADIUS' in source
    assert '"footprint": _BUNKER_FOOTPRINT' in source
    assert "_LIDAR_X = 0.150" in source
    # Both ends of the chassis must sit inside the polygon in the lidar frame.
    half_l, lidar_x = 1.023 / 2.0, 0.150
    front = half_l - lidar_x
    rear = -half_l - lidar_x
    assert front > 0.0
    assert abs(rear) > 0.65
