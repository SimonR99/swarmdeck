import json
import math
import sys
from types import SimpleNamespace as NS
import uuid
import numpy as np
import pytest
from adapters.peer_coordination import PeerCoordinator
from adapters.exploration import PlannerPath, PlannerPose
from autonomy.map_epochs import robot_run_id


def test_stable_reservation_ignores_map_gauge_and_rejects_planning_shift(
    monkeypatch,
):
    strings, exclusions = [], []
    monkeypatch.setenv("SWARMDECK_PLANNING_FRAME_TEMPLATE", "{robot}/odom")
    monkeypatch.delenv("SWARMDECK_MISSION_ID", raising=False)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", NS(String=lambda **kw: NS(**kw)))
    monkeypatch.setitem(
        sys.modules,
        "geometry_msgs.msg",
        NS(
            Pose=lambda: NS(position=NS(x=0, y=0, z=0), orientation=NS(w=1)),
            PoseArray=lambda: NS(header=NS(frame_id=""), poses=[]),
        ),
    )

    def publisher(message_type, *_args):
        return NS(
            publish=lambda msg: (
                strings.append(json.loads(msg.data))
                if hasattr(msg, "data")
                else exclusions.append(msg)
            )
        )

    node = NS(create_publisher=publisher, create_subscription=lambda *args: None)
    coordinator = PeerCoordinator(
        NS(node=node, id="r0", navigation_frame="r0/navigation_frame"), {}
    )
    now = [0.0]
    coordinator.clock = lambda: now[0]
    mission = str(uuid.uuid4())

    def authority(order, map_x, planning_x):
        return {
            "robot_id": "r0",
            "mission_id": mission,
            "robot_map_epoch": 0,
            "run_id": robot_run_id(mission, "r0", 0),
            "participants": ["r0"],
            "component_id": "component:test",
            "solution_order": [order, 0],
            "correction_revision": order,
            "navigation_frame": "r0/navigation_frame",
            "T_component_navigation": [
                [1, 0, 0, map_x],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ],
            "planning_frame": "r0/odom",
            "T_component_planning": [
                [1, 0, 0, planning_x],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ],
        }

    coordinator.on_authority(NS(data=json.dumps(authority(1, 0.0, 0.0))))
    plan = PlannerPath(
        "r0/odom",
        100,
        (
            PlannerPose(0, 0, 0, 0, 0, 0, 1),
            PlannerPose(5, 0, 0, 0, 0, 0, 1),
        ),
    )
    assert coordinator.reserve(plan, 1) == "pending"
    now[0] = 1.0
    assert coordinator.reserve(plan, 1) == "granted"
    token = coordinator.token
    assert strings[-1]["target"] == [5, 0, 0]

    coordinator.on_authority(NS(data=json.dumps(authority(2, 0.27, 0.0))))
    assert coordinator.token == token
    assert coordinator.reserve(plan, 1) == "granted"
    coordinator.publish_exclusions()
    assert exclusions[-1].header.frame_id == "r0/odom"

    coordinator.on_authority(NS(data=json.dumps(authority(3, 0.27, 0.27))))
    assert coordinator.token is None
    assert coordinator.invalid_token == token
    assert coordinator.reserve(plan, 1) == "rejected"
    assert coordinator.last_decision_reason == (
        "reservation invalidated by material map correction"
    )


def test_authority_freshness_path_generation_and_correction(monkeypatch):
    messages = []
    monkeypatch.delenv("SWARMDECK_MISSION_ID", raising=False)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", NS(String=lambda **kw: NS(**kw)))
    monkeypatch.setitem(
        sys.modules,
        "geometry_msgs.msg",
        NS(
            Pose=lambda: NS(position=NS(x=0, y=0, z=0), orientation=NS(w=1)),
            PoseArray=lambda: NS(header=NS(frame_id=""), poses=[]),
        ),
    )
    node = NS(
        create_publisher=lambda *args: NS(
            publish=lambda msg: (
                messages.append(json.loads(msg.data)) if hasattr(msg, "data") else None
            )
        ),
        create_subscription=lambda *args: None,
    )
    coordinator = PeerCoordinator(NS(node=node, id="r0", navigation_frame="map"), {})
    now = [0.0]
    coordinator.clock = lambda: now[0]
    plan = PlannerPath(
        "map", 100, (PlannerPose(0, 0, 0, 0, 0, 0, 1), PlannerPose(5, 0, 0, 0, 0, 0, 1))
    )
    assert coordinator.reserve(plan, 1) == "pending"
    mission = str(uuid.uuid4())
    authority = {
        "robot_id": "r0",
        "mission_id": mission,
        "robot_map_epoch": 0,
        "run_id": robot_run_id(mission, "r0", 0),
        "participants": ["r0", "r1"],
        "component_id": "component:test",
        "solution_order": [1, 0],
        "navigation_frame": "map",
        "T_component_navigation": np.eye(4).tolist(),
    }
    coordinator.on_authority(NS(data=json.dumps(authority)))
    assert coordinator.reserve(plan, 1) == "pending"
    now[0] = 1
    assert coordinator.reserve(plan, 1) == "granted"
    assert messages[-1]["target"] == [5, 0, 0]
    peer = {
        "robot_id": "r1",
        "session_id": authority["mission_id"],
        "sequence": 1,
        "component_id": authority["component_id"],
        "target": [5, 0, 0],
        "radius_m": 2.0,
        "cost": 0.0,
        "lease_s": 3.0,
        "active": True,
    }
    coordinator.receive(NS(data=json.dumps(peer)))
    assert coordinator.reserve(plan, 1) == "rejected"
    assert coordinator.last_decision_reason == "conflict won by r1"
    coordinator.receive(NS(data=json.dumps({**peer, "sequence": 2, "active": False})))
    assert coordinator.reserve(plan, 1) == "granted"
    authority["solution_order"] = [2, 0]
    coordinator.on_authority(NS(data=json.dumps(authority)))
    # A causal optimizer advance with the same component transform is a fresh
    # authority heartbeat, not a reason to abandon an executing path.
    assert coordinator.reserve(plan, 1) == "granted"
    assert messages[-1]["active"]
    newer = PlannerPath("map", 101, plan.poses)
    assert coordinator.reserve(newer, 1) == "pending"
    now[0] = 2
    assert coordinator.reserve(newer, 1) == "granted"
    now[0] = 5
    assert coordinator.reserve(newer, 1) == "pending"
    assert coordinator.last_decision_reason == "map authority unavailable or stale"
    assert not messages[-1]["active"]
    assert coordinator.reserve(plan, 0) == "rejected"


def test_completion_requires_current_run_all_peers_and_one_verified_component(
    monkeypatch,
):
    sent = []
    monkeypatch.setitem(sys.modules, "std_msgs.msg", NS(String=lambda **kw: NS(**kw)))
    monkeypatch.setitem(
        sys.modules, "geometry_msgs.msg", NS(Pose=object, PoseArray=object)
    )
    node = NS(
        create_subscription=lambda *args: None,
        create_publisher=lambda *args: NS(
            publish=lambda msg: sent.append(json.loads(msg.data))
        ),
    )
    bridge = NS(
        node=node,
        id="r0",
        navigation_frame="map",
        exploration=NS(status="locally_exhausted", active=False),
    )
    coordinator = PeerCoordinator(bridge, {})
    clock = [10.0]
    coordinator.clock = lambda: clock[0]
    coordinator.authority = {"mission_id": str(uuid.uuid4()), "component_id": "same"}
    coordinator.received_at = clock[0]
    run = str(uuid.uuid4())
    coordinator.begin_run(run, ["r0", "r1"])
    coordinator.report_progress()
    report = sent[-1]
    assert coordinator.completion_state == "unknown"
    peer = {**report, "robot_id": "r1", "component_id": "unrelated"}
    coordinator.receive_report(NS(data=json.dumps(peer)))
    assert coordinator.completion_state == "incomplete"

    peer.update(component_id="same", sequence=2)
    coordinator.receive_report(NS(data=json.dumps(peer)))
    assert coordinator.completion_state == "complete"
    coordinator.authority["correction_revision"] = 1
    assert coordinator.completion_state == "incomplete"
    coordinator.report_progress()
    assert sent[-1]["state"] == "blocked"
    coordinator.authority["correction_revision"] = 0
    # Old/duplicate reports cannot renew a lease or overwrite current evidence.
    clock[0] = 14.0
    coordinator.received_at = clock[0]
    coordinator.report_progress()
    coordinator.receive_report(NS(data=json.dumps(peer)))
    clock[0] = 16.0
    coordinator.received_at = clock[0]
    assert coordinator.completion_state == "unknown"
    coordinator.begin_run(str(uuid.uuid4()), ["r0", "r1"])
    coordinator.receive_report(NS(data=json.dumps(peer)))
    assert not coordinator.completion.reports
    assert coordinator.completion_state == "unknown"
    # Operator Stop overrides previously exhausted evidence.
    bridge.exploration.status = "stopped"
    coordinator.report_progress()
    assert sent[-1]["state"] == "blocked" and not sent[-1]["coverage_met"]


def test_reordered_authority_preserves_newer_reservation_and_freshness(monkeypatch):
    sent = []
    monkeypatch.delenv("SWARMDECK_MISSION_ID", raising=False)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", NS(String=lambda **kw: NS(**kw)))
    monkeypatch.setitem(
        sys.modules,
        "geometry_msgs.msg",
        NS(
            Pose=lambda: NS(position=NS(x=0, y=0, z=0), orientation=NS(w=1)),
            PoseArray=lambda: NS(header=NS(frame_id=""), poses=[]),
        ),
    )
    node = NS(
        create_subscription=lambda *args: None,
        create_publisher=lambda *args: NS(
            publish=lambda msg: (
                sent.append(json.loads(msg.data)) if hasattr(msg, "data") else None
            )
        ),
    )
    coordinator = PeerCoordinator(NS(node=node, id="r0", navigation_frame="map"), {})
    now = [0.0]
    coordinator.clock = lambda: now[0]
    mission = str(uuid.uuid4())

    def authority(order, revision):
        return {
            "robot_id": "r0",
            "mission_id": mission,
            "robot_map_epoch": 0,
            "run_id": robot_run_id(mission, "r0", 0),
            "participants": ["r0", "r1"],
            "component_id": "component:test",
            "solution_order": list(order),
            "correction_revision": revision,
            "map_epoch": 0,
            "mapping_graph_revision": revision,
            "geometry_revision": "a" * 64,
            "map_source_stamp": {"sec": revision, "nanosec": 0},
            "navigation_frame": "map",
            "T_component_navigation": np.eye(4).tolist(),
        }

    older = authority((2, 0), 2)
    coordinator.on_authority(NS(data=json.dumps(older)))
    plan = PlannerPath(
        "map",
        100,
        (
            PlannerPose(0, 0, 0, 0, 0, 0, 1),
            PlannerPose(5, 0, 0, 0, 0, 0, 1),
        ),
    )
    assert coordinator.reserve(plan, 1) == "pending"
    now[0] = 1.0
    assert coordinator.reserve(plan, 1) == "granted"

    newer = authority((3, 0), 3)
    coordinator.on_authority(NS(data=json.dumps(newer)))
    assert coordinator.token == (1, plan.revision_ns)
    assert coordinator.reserve(plan, 1) == "granted"
    next_plan = PlannerPath("map", 101, plan.poses)
    assert coordinator.reserve(next_plan, 2) == "pending"
    now[0] = 2.0
    assert coordinator.reserve(next_plan, 2) == "granted"
    active_token = coordinator.token
    messages = len(sent)
    accepted_at = coordinator.received_at

    now[0] = 2.5
    coordinator.on_authority(NS(data=json.dumps(older)))
    assert coordinator.authority["solution_order"] == [3, 0]
    assert coordinator.token == active_token
    assert coordinator.received_at == accepted_at
    assert len(sent) == messages

    for invalid in (
        {**authority((4, 0), 4), "geometry_revision": "not-a-digest"},
        {**authority((4, 0), 4), "map_source_stamp": {"sec": 4, "nanosec": -1}},
    ):
        coordinator.on_authority(NS(data=json.dumps(invalid)))
        assert coordinator.authority["solution_order"] == [3, 0]
        assert coordinator.token == active_token
        assert coordinator.received_at == accepted_at
        assert len(sent) == messages

    corrected = {**newer, "T_component_navigation": np.eye(4).tolist()}
    corrected["T_component_navigation"][0][3] = 0.06
    now[0] = 2.6
    coordinator.on_authority(NS(data=json.dumps(corrected)))
    assert coordinator.token == active_token
    assert coordinator.received_at == 2.6
    assert len(sent) == messages

    # Compare against the transform that created the reservation. Small
    # incremental corrections cannot evade the material-change threshold.
    material = {**newer, "T_component_navigation": np.eye(4).tolist()}
    material["T_component_navigation"][0][3] = 0.11
    now[0] = 3.0
    coordinator.on_authority(NS(data=json.dumps(material)))
    assert coordinator.authority["T_component_navigation"][0][3] == 0.11
    assert coordinator.invalid_token == active_token
    assert coordinator.token is None
    assert coordinator.received_at == 3.0
    assert len(sent) == messages + 1
    assert coordinator.reserve(next_plan, 2) == "rejected"
    assert coordinator.last_decision_reason == (
        "reservation invalidated by material map correction"
    )

    now[0] = 6.1
    coordinator.on_authority(NS(data=json.dumps(older)))
    assert coordinator.received_at == 3.0
    final_plan = PlannerPath("map", 102, plan.poses)
    assert coordinator.reserve(final_plan, 3) == "pending"
    coordinator.on_authority(NS(data=json.dumps(authority((4, 0), 4))))
    assert coordinator.received_at == 6.1


def test_stale_authority_and_report_do_not_create_fleet_completion(monkeypatch):
    sent = []
    monkeypatch.delenv("SWARMDECK_MISSION_ID", raising=False)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", NS(String=lambda **kw: NS(**kw)))
    monkeypatch.setitem(
        sys.modules, "geometry_msgs.msg", NS(Pose=object, PoseArray=object)
    )
    node = NS(
        create_subscription=lambda *args: None,
        create_publisher=lambda *args: NS(
            publish=lambda msg: sent.append(json.loads(msg.data))
        ),
    )
    mission = str(uuid.uuid4())
    bridge = NS(
        node=node,
        id="r0",
        navigation_frame="map",
        exploration=NS(status="locally_exhausted", active=False),
    )
    coordinator = PeerCoordinator(bridge, {})
    coordinator.clock = lambda: 10.0

    def authority(order, component):
        return {
            "robot_id": "r0",
            "mission_id": mission,
            "robot_map_epoch": 0,
            "run_id": robot_run_id(mission, "r0", 0),
            "participants": ["r0", "r1"],
            "component_id": component,
            "solution_order": list(order),
            "correction_revision": order[0],
            "navigation_frame": "map",
            "T_component_navigation": np.eye(4).tolist(),
        }

    current = authority((5, 0), "component:new")
    coordinator.on_authority(NS(data=json.dumps(current)))
    run = str(uuid.uuid4())
    coordinator.begin_run(run, ["r0", "r1"])
    coordinator.report_progress()
    local = sent[-1]
    peer = {
        **local,
        "robot_id": "r1",
        "sequence": 10,
        "state": "exploring",
        "coverage_met": False,
    }
    coordinator.receive_report(NS(data=json.dumps(peer)))
    assert coordinator.completion_state == "incomplete"

    coordinator.on_authority(NS(data=json.dumps(authority((4, 9), "component:old"))))
    stale_report = {
        **peer,
        "sequence": 9,
        "component_id": "component:old",
        "state": "locally_exhausted",
        "coverage_met": True,
    }
    coordinator.receive_report(NS(data=json.dumps(stale_report)))
    assert coordinator.authority["component_id"] == "component:new"
    assert coordinator.report_heads["r1"] == 10
    assert coordinator.completion_state == "incomplete"


def test_stale_lease_retains_and_validates_one_plan_authority_binding(monkeypatch):
    sent = []
    monkeypatch.delenv("SWARMDECK_MISSION_ID", raising=False)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", NS(String=lambda **kw: NS(**kw)))
    monkeypatch.setitem(
        sys.modules,
        "geometry_msgs.msg",
        NS(
            Pose=lambda: NS(position=NS(x=0, y=0, z=0), orientation=NS(w=1)),
            PoseArray=lambda: NS(header=NS(frame_id=""), poses=[]),
        ),
    )
    node = NS(
        create_subscription=lambda *args: None,
        create_publisher=lambda *args: NS(
            publish=lambda msg: (
                sent.append(json.loads(msg.data)) if hasattr(msg, "data") else None
            )
        ),
    )
    coordinator = PeerCoordinator(NS(node=node, id="r0", navigation_frame="map"), {})
    now = [0.0]
    coordinator.clock = lambda: now[0]
    mission = str(uuid.uuid4())

    def authority(order, *, component="component:a", x=0.0):
        transform = np.eye(4)
        transform[0, 3] = x
        return {
            "robot_id": "r0",
            "mission_id": mission,
            "robot_map_epoch": 0,
            "run_id": robot_run_id(mission, "r0", 0),
            "participants": ["r0"],
            "component_id": component,
            "solution_order": [order, 0],
            "correction_revision": order,
            "navigation_frame": "map",
            "T_component_navigation": transform.tolist(),
        }

    plan = PlannerPath(
        "map",
        100,
        (
            PlannerPose(0, 0, 0, 0, 0, 0, 1),
            PlannerPose(5, 0, 0, 0, 0, 0, 1),
        ),
    )
    coordinator.on_authority(NS(data=json.dumps(authority(1))))
    assert coordinator.reserve(plan, 1) == "pending"
    now[0] = 1.0
    assert coordinator.reserve(plan, 1) == "granted"
    bound_token = coordinator.token

    now[0] = 4.1
    assert coordinator.reserve(plan, 1) == "pending"
    assert coordinator.token == bound_token
    assert coordinator.arbiter.local is None
    assert not sent[-1]["active"]

    # The same authority may renew the withdrawn lease for the retained plan.
    coordinator.on_authority(NS(data=json.dumps(authority(2))))
    assert coordinator.reserve(plan, 1) == "pending"
    assert coordinator.token == bound_token
    assert coordinator.arbiter.local is not None
    now[0] = 5.1
    assert coordinator.reserve(plan, 1) == "granted"

    # A component correction is compared with the retained binding even after
    # another stale withdrawal, so the old plan cannot silently rebind.
    now[0] = 8.2
    assert coordinator.reserve(plan, 1) == "pending"
    coordinator.on_authority(NS(data=json.dumps(authority(3, component="component:b"))))
    assert coordinator.invalid_token == bound_token
    assert coordinator.token is None
    assert coordinator.reserve(plan, 1) == "rejected"

    newer = PlannerPath("map", 101, plan.poses)
    assert coordinator.reserve(newer, 2) == "pending"
    new_token = coordinator.token
    now[0] = 9.3
    assert coordinator.reserve(newer, 2) == "granted"
    now[0] = 12.4
    assert coordinator.reserve(newer, 2) == "pending"
    coordinator.on_authority(
        NS(data=json.dumps(authority(4, component="component:b", x=0.11)))
    )
    assert coordinator.invalid_token == new_token
    assert coordinator.token is None
    assert coordinator.reserve(newer, 2) == "rejected"

    final = PlannerPath("map", 102, plan.poses)
    assert coordinator.reserve(final, 3) == "pending"
    assert coordinator.reservation_signature == (mission, "component:b")
    coordinator.release(3)  # Operator Stop/ordinary completion uses this path.
    assert coordinator.token is None
    assert coordinator.reservation_transform is None
    assert coordinator.reservation_signature is None

    # A new exploration generation cannot inherit an older plan binding while
    # authority is stale.
    coordinator.on_authority(
        NS(data=json.dumps(authority(5, component="component:b", x=0.11)))
    )
    assert coordinator.reserve(final, 3) == "pending"
    now[0] = 16.5
    replacement = PlannerPath("map", 103, plan.poses)
    assert coordinator.reserve(replacement, 4) == "pending"
    assert coordinator.generation == 4
    assert coordinator.token is None
    assert coordinator.reservation_signature is None


def test_surveyed_start_poses_arbitrate_between_separate_components(monkeypatch):
    monkeypatch.delenv("SWARMDECK_MISSION_ID", raising=False)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", NS(String=lambda **kw: NS(**kw)))
    monkeypatch.setitem(
        sys.modules,
        "geometry_msgs.msg",
        NS(
            Pose=lambda: NS(position=NS(x=0, y=0, z=0), orientation=NS(w=1)),
            PoseArray=lambda: NS(header=NS(frame_id=""), poses=[]),
        ),
    )
    mission = str(uuid.uuid4())
    wire, now = [], [0.0]

    def coordinator(robot, start):
        sent = []
        node = NS(
            create_publisher=lambda *args: NS(
                publish=lambda msg: (
                    (sent.append(msg), wire.append(msg))
                    if hasattr(msg, "data")
                    else sent.append(msg)
                )
            ),
            create_subscription=lambda *args: None,
        )
        value = PeerCoordinator(
            NS(node=node, id=robot, navigation_frame="map"),
            {"deployment_start_pose": start},
        )
        value.clock = lambda: now[0]
        value.on_authority(
            NS(
                data=json.dumps(
                    {
                        "robot_id": robot,
                        "mission_id": mission,
                        "robot_map_epoch": 0,
                        "run_id": robot_run_id(mission, robot, 0),
                        "participants": ["r0", "r1"],
                        # Each robot holds its own map component.
                        "component_id": f"component:{robot}",
                        "solution_order": [0, -1],
                        "navigation_frame": "map",
                        "T_component_navigation": np.eye(4).tolist(),
                    }
                )
            )
        )
        return value, sent

    # Both start facing -y, 2 m apart; each plans 5 m straight ahead in its own
    # map frame, which is the same place on the ground give or take 2 m.
    r0, _ = coordinator("r0", {"x": -11.2, "y": 4.0, "yaw": -math.pi / 2})
    r1, r1_sent = coordinator("r1", {"x": -13.2, "y": 4.0, "yaw": -math.pi / 2})
    ahead = PlannerPath(
        "map", 100, (PlannerPose(0, 0, 0, 0, 0, 0, 1), PlannerPose(5, 0, 0, 0, 0, 0, 1))
    )
    assert r0.reserve(ahead, 1) == "pending"
    claim = json.loads(wire[-1].data)
    assert claim["component_id"] == f"deployment:{mission}"
    assert claim["target"] == pytest.approx([-11.2, -1.0, 0.0])

    longer = PlannerPath(
        "map",
        100,
        (
            PlannerPose(0, 0, 0, 0, 0, 0, 1),
            PlannerPose(2, 2, 0, 0, 0, 0, 1),
            PlannerPose(5, 0, 0, 0, 0, 0, 1),
        ),
    )
    assert r1.reserve(longer, 1) == "pending"
    r1.receive(NS(data=json.dumps(claim)))
    now[0] = 1.0
    assert r1.reserve(longer, 1) == "rejected"
    assert r1.last_decision_reason == "conflict won by r0"
    # MGG receives the winner's target in r1's own planning frame: 2 m to its
    # left (+y) and 5 m ahead.
    r1.publish_exclusions()
    excluded = [m for m in r1_sent if hasattr(m, "poses") and m.poses][-1].poses[0]
    assert (excluded.position.x, excluded.position.y) == pytest.approx((5.0, 2.0))

    # A granted reservation is guarded by the robot's own component transform.
    # The deployment transform also carries odometry drift, which only moves
    # the target and must not revoke a path.
    assert r0.reserve(ahead, 1) == "granted"
    assert np.allclose(r0.authority["reservation_guard"], np.eye(4))
    assert not np.allclose(r0.authority["T_component_navigation"], np.eye(4))
    assert np.allclose(r0.reservation_transform, np.eye(4))

    # Each robot reports where it stands; the other's planner receives it as
    # a body in its own planning frame. r0 has driven 1 m forward and stands
    # 2 m to r1's left and 1 m ahead of it.
    r0.bridge.map_pose = lambda: {"x": 1.0, "y": 0.0, "yaw": 0.0}
    r1.bridge.map_pose = lambda: {"x": 0.0, "y": 0.0, "yaw": 0.0}
    r0.publish_peer_bodies()
    report = json.loads(wire[-1].data)
    assert (report["robot_id"], report["x"], report["y"]) == (
        "r0",
        pytest.approx(-11.2),
        pytest.approx(3.0),
    )
    r1.receive_peer_pose(wire[-1])
    r1.receive_peer_pose(NS(data=json.dumps({**report, "robot_id": "r1"})))
    r1.receive_peer_pose(NS(data=json.dumps({**report, "session_id": "other"})))
    r1.publish_peer_bodies()
    bodies = [m for m in r1_sent if hasattr(m, "poses")][-1]
    assert bodies.header.frame_id == r1.frame
    assert [(b.position.x, b.position.y) for b in bodies.poses] == [
        pytest.approx((1.0, 2.0))
    ]
    # Reports that stop arriving stop blocking the planner.
    now[0] += 4.0
    r1.on_authority(NS(data=json.dumps(r1.raw_authority | {"solution_order": [1, 0]})))
    r1.publish_peer_bodies()
    assert [m for m in r1_sent if hasattr(m, "poses")][-1].poses == []
    now[0] = 1.0

    # A frontier on the far side of the street is nobody else's.
    elsewhere = PlannerPath(
        "map",
        101,
        (PlannerPose(0, 0, 0, 0, 0, 0, 1), PlannerPose(-9, 0, 0, 0, 0, 0, 1)),
    )
    assert r1.reserve(elsewhere, 2) == "pending"
    now[0] = 2.0
    assert r1.reserve(elsewhere, 2) == "granted"


def test_settle_remaining_counts_down_to_the_grant(monkeypatch):
    monkeypatch.delenv("SWARMDECK_MISSION_ID", raising=False)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", NS(String=lambda **kw: NS(**kw)))
    monkeypatch.setitem(
        sys.modules,
        "geometry_msgs.msg",
        NS(
            Pose=lambda: NS(position=NS(x=0, y=0, z=0), orientation=NS(w=1)),
            PoseArray=lambda: NS(header=NS(frame_id=""), poses=[]),
        ),
    )
    node = NS(
        create_publisher=lambda *args: NS(publish=lambda msg: None),
        create_subscription=lambda *args: None,
    )
    coordinator = PeerCoordinator(NS(node=node, id="r0", navigation_frame="map"), {})
    now = [0.0]
    coordinator.clock = lambda: now[0]
    assert coordinator.settle_remaining_s() is None
    mission = str(uuid.uuid4())
    identity = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    coordinator.on_authority(
        NS(
            data=json.dumps(
                {
                    "robot_id": "r0",
                    "mission_id": mission,
                    "robot_map_epoch": 0,
                    "run_id": robot_run_id(mission, "r0", 0),
                    "participants": ["r0"],
                    "component_id": "component:test",
                    "solution_order": [1, 0],
                    "navigation_frame": "map",
                    "T_component_navigation": identity,
                }
            )
        )
    )
    plan = PlannerPath(
        "map",
        100,
        (PlannerPose(0, 0, 0, 0, 0, 0, 1), PlannerPose(5, 0, 0, 0, 0, 0, 1)),
    )
    assert coordinator.reserve(plan, 1) == "pending"
    assert coordinator.settle_remaining_s() == pytest.approx(0.5)
    now[0] = 0.4
    assert coordinator.reserve(plan, 1) == "pending"
    assert coordinator.settle_remaining_s() == pytest.approx(0.1)
    now[0] = 0.5
    assert coordinator.settle_remaining_s() is None
    assert coordinator.reserve(plan, 1) == "granted"
