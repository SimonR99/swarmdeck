"""Capture-time peer-body masking: geometry, timestamp joins and defaults."""

import importlib.util
import json
import math
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

from autonomy.peer_mask import (
    MAX_PEER_POSE_TOLERANCE_S,
    PEER_BODY_PROFILES,
    PeerBody,
    PeerBodyMask,
    PeerPoseHistory,
    idle_mask_counters,
    mask_peer_bodies,
    peer_body,
    points_inside_body,
    relative_transform,
)

REPO = Path(__file__).resolve().parents[2]
BUNKER = PEER_BODY_PROFILES["bunker"]


def pose(x=0.0, y=0.0, z=0.0, yaw=0.0):
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [math.cos(yaw), -math.sin(yaw), 0.0],
        [math.sin(yaw), math.cos(yaw), 0.0],
        [0.0, 0.0, 1.0],
    ]
    matrix[:3, 3] = (x, y, z)
    return matrix


def ns(seconds):
    return int(seconds * 1e9)


# -- body geometry ---------------------------------------------------------


def test_point_inside_a_peer_body_is_dropped():
    # A Bunker two metres ahead. Its chassis centre is squarely inside the box.
    peers = {"robot_1": (pose(x=2.0), BUNKER)}
    points = np.array([[2.0, 0.0, 0.0], [2.0, 0.2, 0.3]])

    kept, dropped = mask_peer_bodies(points, peers, margin_m=0.0)

    assert len(kept) == 0
    assert dropped == {"robot_1": 2}


def test_point_just_outside_the_margin_is_kept():
    peers = {"robot_1": (pose(x=2.0), BUNKER)}
    margin = 0.15
    edge = 2.0 + BUNKER.length / 2.0 + margin
    inside = np.array([[edge - 1e-6, 0.0, 0.0]])
    outside = np.array([[edge + 1e-3, 0.0, 0.0]])

    assert len(mask_peer_bodies(inside, peers, margin)[0]) == 0
    kept, dropped = mask_peer_bodies(outside, peers, margin)
    assert kept.tolist() == outside.tolist()
    assert dropped == {"robot_1": 0}


def test_the_box_spans_floor_to_mast_and_no_further():
    peers = {"robot_1": (pose(x=2.0), BUNKER)}
    # Wheels on the floor, mast top, and a point above the mast.
    points = np.array(
        [
            [2.0, 0.0, -BUNKER.base_height],
            [2.0, 0.0, BUNKER.top_height],
            [2.0, 0.0, BUNKER.top_height + 0.5],
        ]
    )

    kept, dropped = mask_peer_bodies(points, peers, margin_m=0.0)

    assert dropped == {"robot_1": 2}
    assert kept.tolist() == [[2.0, 0.0, BUNKER.top_height + 0.5]]


def test_a_rotated_peer_masks_its_own_rectangle_not_an_axis_aligned_one():
    # Turned ninety degrees, a Bunker is 0.778 m long and 1.023 m wide as seen
    # from here. A point 0.45 m to its side is inside; 0.45 m ahead is not.
    peers = {"robot_1": (pose(x=3.0, yaw=math.pi / 2.0), BUNKER)}
    across = np.array([[3.0, 0.45, 0.0]])
    along = np.array([[3.45, 0.0, 0.0]])

    assert len(mask_peer_bodies(across, peers, margin_m=0.0)[0]) == 0
    assert len(mask_peer_bodies(along, peers, margin_m=0.0)[0]) == 1


def test_each_platform_masks_its_own_box():
    spot, scout = PEER_BODY_PROFILES["spot"], PEER_BODY_PROFILES["scout_mini"]
    # 0.30 m off the axis clears a Spot (0.50 m wide) but not a Scout Mini
    # (0.58 m wide), so one platform's box cannot stand in for another's.
    point = np.array([[2.0, 0.28, 0.0]])

    assert len(mask_peer_bodies(point, {"p": (pose(x=2.0), spot)}, 0.0)[0]) == 1
    assert len(mask_peer_bodies(point, {"p": (pose(x=2.0), scout)}, 0.0)[0]) == 0


def test_per_peer_counts_are_disjoint_and_sum_to_the_points_removed():
    # Two overlapping boxes must not both claim the same point.
    peers = {
        "robot_1": (pose(x=2.0), BUNKER),
        "robot_2": (pose(x=2.2), BUNKER),
    }
    points = np.array([[2.1, 0.0, 0.0], [20.0, 0.0, 0.0]])

    kept, dropped = mask_peer_bodies(points, peers, margin_m=0.0)

    assert sum(dropped.values()) == len(points) - len(kept) == 1
    assert dropped == {"robot_1": 1, "robot_2": 0}


def test_masking_without_peers_returns_the_capture_unchanged():
    points = np.array([[1.0, 2.0, 3.0]])
    kept, dropped = mask_peer_bodies(points, {}, margin_m=0.15)
    assert kept.tolist() == points.tolist()
    assert dropped == {}


def test_a_negative_margin_is_refused():
    with pytest.raises(ValueError):
        points_inside_body(np.zeros((1, 3)), np.eye(4), BUNKER, -0.01)


def test_body_dimensions_must_be_positive():
    with pytest.raises(ValueError):
        PeerBody(length=0.0, width=1.0, base_height=0.2, top_height=0.5)


def test_an_unknown_platform_is_refused_rather_than_guessed():
    with pytest.raises(ValueError, match="unknown peer platform"):
        peer_body("forklift")


# -- frames and timestamps -------------------------------------------------


def test_a_peer_that_moved_is_masked_at_its_pose_at_the_capture_stamp():
    """The join must use the historical pose, never the newest one."""
    history = PeerPoseHistory()
    history.add(ns(10.0), pose(x=2.0))  # where it was when the scan fired
    history.add(ns(10.5), pose(x=6.0))  # where it is now
    self_pose = pose()
    point_at_capture = np.array([[2.0, 0.0, 0.0]])
    point_at_latest = np.array([[6.0, 0.0, 0.0]])

    sample = history.sample_at(ns(10.0), ns(0.05))
    peers = {"robot_1": (relative_transform(self_pose, sample), BUNKER)}

    # The return that landed on the peer at capture time is removed.
    assert len(mask_peer_bodies(point_at_capture, peers, 0.0)[0]) == 0
    # Geometry at the peer's *current* pose is untouched: masking there would
    # delete whatever the scan really saw at that spot.
    assert len(mask_peer_bodies(point_at_latest, peers, 0.0)[0]) == 1


def test_a_stale_pose_yields_no_sample_so_the_peer_is_not_masked():
    history = PeerPoseHistory()
    history.add(ns(10.0), pose(x=2.0))

    assert history.sample_at(ns(10.02), ns(0.05)) is not None
    assert history.sample_at(ns(10.5), ns(0.05)) is None
    assert history.sample_at(ns(9.5), ns(0.05)) is None


def test_the_nearest_sample_wins_inside_the_tolerance():
    history = PeerPoseHistory()
    history.add(ns(10.00), pose(x=1.0))
    history.add(ns(10.04), pose(x=2.0))

    sample = history.sample_at(ns(10.03), ns(0.05))

    assert sample[0, 3] == pytest.approx(2.0)


def test_an_empty_history_never_produces_a_pose():
    assert PeerPoseHistory().sample_at(ns(1.0), ns(1.0)) is None


def test_the_tolerance_is_capped_however_large_a_caller_asks():
    history = PeerPoseHistory()
    history.add(ns(10.0), pose(x=2.0))

    assert (
        history.sample_at(ns(10.0 + MAX_PEER_POSE_TOLERANCE_S + 0.1), ns(60.0)) is None
    )


def test_out_of_order_samples_are_still_joined_by_stamp():
    history = PeerPoseHistory()
    history.add(ns(10.04), pose(x=2.0))
    history.add(ns(10.00), pose(x=1.0))

    assert history.sample_at(ns(10.00), ns(0.005))[0, 3] == pytest.approx(1.0)
    assert history.sample_at(ns(10.04), ns(0.005))[0, 3] == pytest.approx(2.0)


def test_history_is_bounded_by_its_horizon():
    history = PeerPoseHistory(horizon_s=0.1)
    for index in range(50):
        history.add(ns(10.0 + index * 0.01), pose(x=float(index)))

    assert len(history) <= 12
    assert history.sample_at(ns(10.0), ns(0.05)) is None


def test_a_pose_that_is_not_a_finite_transform_is_refused():
    history = PeerPoseHistory()
    with pytest.raises(ValueError):
        history.add(ns(1.0), np.full((4, 4), np.nan))
    with pytest.raises(ValueError):
        history.add(ns(1.0), np.eye(3))


def test_relative_transform_places_a_peer_in_the_capturing_robot_frame():
    # Capturing robot at (1, 1) facing +y; peer one metre further along +y.
    own = pose(x=1.0, y=1.0, yaw=math.pi / 2.0)
    peer = pose(x=1.0, y=2.0, yaw=math.pi / 2.0)

    T_base_peer = relative_transform(own, peer)

    # In the capturing robot's own frame that is one metre straight ahead.
    assert T_base_peer[:3, 3] == pytest.approx([1.0, 0.0, 0.0])
    assert T_base_peer[:3, :3] == pytest.approx(np.eye(3))


def test_relative_transform_round_trips_a_point():
    own, peer = pose(x=3.0, y=-2.0, yaw=0.7), pose(x=5.0, y=1.0, yaw=-0.4)
    T_base_peer = relative_transform(own, peer)
    # A point at the peer's origin, expressed in the shared frame, lands on the
    # peer's origin once carried into the capturing robot's frame.
    in_reference = peer[:3, 3]
    in_base = np.linalg.inv(own) @ np.append(in_reference, 1.0)

    assert in_base[:3] == pytest.approx(T_base_peer[:3, 3])


# -- the stale-pose policy and its counters --------------------------------


def two_robot_mask(margin_m=0.0, tolerance_s=0.05):
    return PeerBodyMask(
        "robot_0",
        {"robot_0": BUNKER, "robot_1": BUNKER},
        tolerance_ns=ns(tolerance_s),
        margin_m=margin_m,
    )


def test_a_fresh_peer_pose_masks_and_is_counted_per_peer():
    mask = two_robot_mask()
    mask.add_pose("robot_0", ns(10.0), pose())
    mask.add_pose("robot_1", ns(10.0), pose(x=2.0))
    points = np.array([[2.0, 0.0, 0.0], [12.0, 0.0, 0.0]])

    kept = mask.apply(points, ns(10.0))

    assert kept.tolist() == [[12.0, 0.0, 0.0]]
    counters = mask.counters()
    assert counters["peer_body_mask_points_dropped"] == 1
    assert counters["peer_body_mask_points_dropped_by_peer"] == {"robot_1": 1}
    assert counters["peer_body_mask_peers_skipped_stale"] == 0


def test_a_stale_peer_pose_disables_masking_for_that_peer_and_counts_it():
    mask = two_robot_mask()
    mask.add_pose("robot_0", ns(10.0), pose())
    # The peer's only sample is half a second from the capture: far outside
    # the tolerance, so its body cannot be placed for this capture.
    mask.add_pose("robot_1", ns(9.5), pose(x=2.0))
    points = np.array([[2.0, 0.0, 0.0]])

    kept = mask.apply(points, ns(10.0))

    assert kept.tolist() == points.tolist()  # conservative: nothing deleted
    counters = mask.counters()
    assert counters["peer_body_mask_peers_skipped_stale"] == 1
    assert counters["peer_body_mask_points_dropped"] == 0
    assert counters["peer_body_mask_points_dropped_by_peer"] == {}


def test_a_peer_that_never_reported_a_pose_is_skipped_and_counted():
    mask = two_robot_mask()
    mask.add_pose("robot_0", ns(10.0), pose())
    points = np.array([[2.0, 0.0, 0.0]])

    assert mask.apply(points, ns(10.0)).tolist() == points.tolist()
    assert mask.counters()["peer_body_mask_peers_skipped_stale"] == 1


def test_the_bridge_can_ask_whether_a_capture_can_be_masked_at_all():
    mask = two_robot_mask()
    mask.add_pose("robot_1", ns(10.0), pose(x=2.0))
    assert not mask.can_place_self(ns(10.0))
    mask.add_pose("robot_0", ns(10.0), pose())
    assert mask.can_place_self(ns(10.0))
    assert not mask.can_place_self(ns(12.0))


def test_without_our_own_pose_the_whole_capture_is_left_alone():
    mask = two_robot_mask()
    mask.add_pose("robot_1", ns(10.0), pose(x=2.0))
    points = np.array([[2.0, 0.0, 0.0]])

    assert mask.apply(points, ns(10.0)).tolist() == points.tolist()
    counters = mask.counters()
    assert counters["peer_body_mask_captures_without_self_pose"] == 1
    assert counters["peer_body_mask_peers_skipped_stale"] == 1
    assert counters["peer_body_mask_points_dropped"] == 0


def test_the_mask_joins_each_capture_to_its_own_stamp():
    """Two captures, one moving peer, each masked where the peer then was."""
    mask = two_robot_mask()
    for stamp, x in ((10.0, 0.0), (10.5, 0.0)):
        mask.add_pose("robot_0", ns(stamp), pose(x=x))
    mask.add_pose("robot_1", ns(10.0), pose(x=2.0))
    mask.add_pose("robot_1", ns(10.5), pose(x=6.0))
    early_hit = np.array([[2.0, 0.0, 0.0]])
    late_hit = np.array([[6.0, 0.0, 0.0]])

    # At the first stamp only the peer's pose there masks anything.
    assert len(mask.apply(early_hit, ns(10.0))) == 0
    assert len(mask.apply(late_hit, ns(10.0))) == 1
    # At the second stamp the situation is exactly reversed.
    assert len(mask.apply(late_hit, ns(10.5))) == 0
    assert len(mask.apply(early_hit, ns(10.5))) == 1
    assert mask.counters()["peer_body_mask_points_dropped_by_peer"] == {"robot_1": 2}


def test_a_moving_capturing_robot_places_the_peer_in_its_own_frame():
    mask = two_robot_mask()
    # We are at (5, 0) turned ninety degrees; the peer is at (5, 2), which is
    # two metres straight ahead of us in our own base frame.
    mask.add_pose("robot_0", ns(10.0), pose(x=5.0, yaw=math.pi / 2.0))
    mask.add_pose("robot_1", ns(10.0), pose(x=5.0, y=2.0))

    assert len(mask.apply(np.array([[2.0, 0.0, 0.0]]), ns(10.0))) == 0
    assert len(mask.apply(np.array([[0.0, 2.0, 0.0]]), ns(10.0))) == 1


def test_poses_for_a_robot_outside_the_fleet_are_rejected_and_counted():
    mask = two_robot_mask()
    assert mask.add_pose("robot_9", ns(10.0), pose()) is False
    assert mask.add_pose("robot_0", ns(10.0), np.eye(3)) is False
    mask.reject_pose()

    assert mask.counters()["peer_body_mask_pose_rejections"] == 3


def test_idle_counters_match_a_live_mask_that_has_done_nothing():
    assert idle_mask_counters() == two_robot_mask().counters()


def test_a_mask_needs_a_body_for_the_capturing_robot():
    with pytest.raises(ValueError, match="capturing robot"):
        PeerBodyMask("robot_0", {"robot_1": BUNKER}, ns(0.05), 0.0)


def test_a_nonpositive_tolerance_is_refused():
    with pytest.raises(ValueError, match="tolerance"):
        PeerBodyMask("robot_0", {"robot_0": BUNKER}, 0, 0.0)


# -- the table the simulation actually spawns ------------------------------


def test_body_profiles_match_the_simulation_platform_table():
    """The peer container cannot import the sim package, so guard the copy."""
    sys.path.insert(0, str(REPO / "swarmdeck_ros/src/swarmdeck_sim"))
    try:
        from scenario.spawn_fleet import ROBOT_PROFILES
    finally:
        sys.path.remove(str(REPO / "swarmdeck_ros/src/swarmdeck_sim"))

    assert set(PEER_BODY_PROFILES) == set(ROBOT_PROFILES)
    for name, spec in ROBOT_PROFILES.items():
        body = PEER_BODY_PROFILES[name]
        assert body.length == spec.length
        assert body.width == spec.width
        assert body.base_height == spec.base_height
        # Tall enough to cover the mapping lidar, which is what another
        # robot's rings hit first on all three platforms.
        assert body.top_height == max(spec.deck_top, spec.lidar_z)


def test_the_launcher_resolves_the_platforms_the_simulator_spawns():
    """A mask sized from the wrong platform masks the wrong volume.

    The launcher parses fleet.robot_type/robot_types itself to avoid a host
    YAML dependency, so it has to agree with the simulation package's own
    resolver on every scenario the fleet actually runs.
    """
    import yaml

    sys.path[:0] = [str(REPO / "deploy"), str(REPO / "swarmdeck_ros/src/swarmdeck_sim")]
    try:
        from scenario.spawn_fleet import robot_types
        from simulation_launch import fleet_from_config, platforms_from_config
    finally:
        del sys.path[:2]

    scenarios = sorted(REPO.glob("configs/*robot*.yaml")) + [
        REPO / "configs/bistro.yaml"
    ]
    checked = 0
    for path in scenarios:
        try:
            count, prefix = fleet_from_config(path)
        except ValueError:
            continue  # a hardware profile configures no simulated fleet
        names = [f"{prefix}{index}" for index in range(count)]
        fleet = (yaml.safe_load(path.read_text()) or {}).get("fleet")
        expected = dict(zip(names, robot_types(fleet, count, prefix)))
        assert platforms_from_config(path, names) == expected, path
        # Every platform the scenarios use must have a masking body.
        for platform in expected.values():
            assert peer_body(platform)
        checked += 1
    assert checked >= 8


def test_the_launcher_refuses_overrides_for_robots_outside_the_fleet(tmp_path):
    sys.path.insert(0, str(REPO / "deploy"))
    try:
        from simulation_launch import platforms_from_config
    finally:
        sys.path.pop(0)
    config = tmp_path / "typo.yaml"
    config.write_text(
        "fleet:\n"
        "  robot_count: 2\n"
        "  robot_prefix: robot_\n"
        "  robot_type: bunker\n"
        "  robot_types:\n"
        "    robot_7: spot\n"
    )

    with pytest.raises(ValueError, match="outside this fleet"):
        platforms_from_config(config, ["robot_0", "robot_1"])


# -- configuration ---------------------------------------------------------


def _load_peer_launch(monkeypatch):
    for name, attributes in {
        "launch": {"LaunchDescription": list},
        "launch.actions": {
            "EmitEvent": lambda *a, **k: SimpleNamespace(args=a, **k),
            "RegisterEventHandler": lambda *a, **k: SimpleNamespace(args=a, **k),
            "TimerAction": lambda *a, **k: SimpleNamespace(args=a, **k),
        },
        "launch.event_handlers": {"OnProcessExit": lambda *a, **k: None},
        "launch.events": {"Shutdown": lambda *a, **k: None},
        "launch_ros": {},
        "launch_ros.actions": {"Node": lambda *a, **k: SimpleNamespace(args=a, **k)},
    }.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    path = REPO / "deploy/autonomy/peer.launch.py"
    spec = importlib.util.spec_from_file_location("peer_launch_peer_mask", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _bridge_parameters(monkeypatch, **environment):
    monkeypatch.setenv("SWARMDECK_MISSION_ID", "6f6afc5c-9a34-4eb4-8243-731629872d25")
    monkeypatch.setenv("SWARMDECK_PEER_NAMES", '["robot_0","robot_1"]')
    monkeypatch.setenv("SWARMDECK_PEER_INDEX", "0")
    monkeypatch.setenv("ROS_DOMAIN_ID", "173")
    for key in (
        "SWARMDECK_PEER_BODY_MASK",
        "SWARMDECK_PEER_PLATFORMS",
        "SWARMDECK_PEER_POSE_TOPIC_TEMPLATE",
        "SWARMDECK_PEER_POSE_FRAME",
        "SWARMDECK_PEER_MASK_MARGIN_M",
        "SWARMDECK_PEER_MASK_POSE_TOLERANCE_S",
    ):
        monkeypatch.delenv(key, raising=False)
    for key, value in environment.items():
        monkeypatch.setenv(key, value)
    nodes = _load_peer_launch(monkeypatch).generate_launch_description()
    bridge = next(
        node for node in nodes if getattr(node, "name", "") == "onboard_mapper"
    )
    return bridge.parameters[0]


def test_hardware_default_leaves_the_peer_mask_off(monkeypatch):
    """No env, no mask: hardware has no guaranteed shared-frame peer pose."""
    params = _bridge_parameters(monkeypatch)

    assert params["peer_body_mask"] is False
    assert params["peer_platforms"] == "{}"


def test_no_hardware_peer_overlay_enables_the_mask():
    for overlay in ("docker-compose.robot-peer.yml", "docker-compose.cslam.yml"):
        text = (REPO / "deploy/compose" / overlay).read_text()
        assert "SWARMDECK_PEER_BODY_MASK" not in text, overlay


def test_the_simulation_launcher_enables_the_mask_with_a_platform_map():
    launch = REPO / "deploy/simulation_launch.py"
    text = launch.read_text()
    assert 'SWARMDECK_PEER_BODY_MASK="true" if platforms else "false"' in text
    assert "SWARMDECK_PEER_PLATFORMS" in text


def test_launch_enables_the_mask_when_the_simulation_asks(monkeypatch):
    params = _bridge_parameters(
        monkeypatch,
        SWARMDECK_PEER_BODY_MASK="true",
        SWARMDECK_PEER_PLATFORMS='{"robot_0":"bunker","robot_1":"spot"}',
    )

    assert params["peer_body_mask"] is True
    assert json.loads(params["peer_platforms"]) == {
        "robot_0": "bunker",
        "robot_1": "spot",
    }
    # Defaults documented in the launch docstring: the ARGoS bridge's shared
    # `world` pose stream, and a tolerance on the order of the pose period.
    assert params["peer_pose_topic_template"] == "/{robot}/ground_truth"
    assert params["peer_pose_frame"] == "world"
    assert params["peer_mask_pose_tolerance_s"] == pytest.approx(0.05)
    assert params["peer_mask_margin_m"] == pytest.approx(0.15)


def test_blank_compose_values_fall_back_to_the_mask_defaults(monkeypatch):
    # The optional robot compose file renders unset values as empty strings.
    params = _bridge_parameters(
        monkeypatch,
        SWARMDECK_PEER_BODY_MASK="",
        SWARMDECK_PEER_PLATFORMS="",
        SWARMDECK_PEER_POSE_TOPIC_TEMPLATE="",
        SWARMDECK_PEER_POSE_FRAME="",
        SWARMDECK_PEER_MASK_MARGIN_M="",
        SWARMDECK_PEER_MASK_POSE_TOLERANCE_S="",
    )

    assert params["peer_body_mask"] is False
    assert params["peer_platforms"] == "{}"
    assert params["peer_pose_topic_template"] == "/{robot}/ground_truth"
    assert params["peer_pose_frame"] == "world"
    assert params["peer_mask_margin_m"] == pytest.approx(0.15)
