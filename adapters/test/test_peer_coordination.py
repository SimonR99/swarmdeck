import json
import sys
from types import SimpleNamespace as NS
import uuid
import numpy as np
from adapters.peer_coordination import PeerCoordinator
from adapters.exploration import PlannerPath, PlannerPose


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
    coordinator = PeerCoordinator(NS(node=node, id="r0", map_frame="map"), {})
    now = [0.0]
    coordinator.clock = lambda: now[0]
    plan = PlannerPath(
        "map", 100, (PlannerPose(0, 0, 0, 0, 0, 0, 1), PlannerPose(5, 0, 0, 0, 0, 0, 1))
    )
    assert coordinator.reserve(plan, 1) == "pending"
    authority = {
        "robot_id": "r0",
        "mission_id": str(uuid.uuid4()),
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
    authority["solution_order"] = [2, 0]
    coordinator.on_authority(NS(data=json.dumps(authority)))
    assert coordinator.reserve(plan, 1) == "rejected"
    assert not messages[-1]["active"]
    newer = PlannerPath("map", 101, plan.poses)
    assert coordinator.reserve(newer, 1) == "pending"
    now[0] = 2
    assert coordinator.reserve(newer, 1) == "granted"
    now[0] = 5
    assert coordinator.reserve(newer, 1) == "pending"
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
        map_frame="map",
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
    coordinator = PeerCoordinator(NS(node=node, id="r0", map_frame="map"), {})
    now = [0.0]
    coordinator.clock = lambda: now[0]
    mission = str(uuid.uuid4())

    def authority(order, revision):
        return {
            "robot_id": "r0",
            "mission_id": mission,
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
    corrected["T_component_navigation"][0][3] = 0.05
    now[0] = 2.6
    coordinator.on_authority(NS(data=json.dumps(corrected)))
    assert coordinator.token == active_token
    assert coordinator.received_at == 2.6
    assert len(sent) == messages

    now[0] = 2.7
    coordinator.on_authority(NS(data=json.dumps(corrected)))
    assert coordinator.token == active_token
    assert coordinator.received_at == 2.7
    assert len(sent) == messages

    material = {**newer, "T_component_navigation": np.eye(4).tolist()}
    material["T_component_navigation"][0][3] = 0.2
    now[0] = 3.0
    coordinator.on_authority(NS(data=json.dumps(material)))
    assert coordinator.authority["T_component_navigation"][0][3] == 0.2
    assert coordinator.invalid_token == active_token
    assert coordinator.token is None
    assert coordinator.received_at == 3.0
    assert len(sent) == messages + 1

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
        map_frame="map",
        exploration=NS(status="locally_exhausted", active=False),
    )
    coordinator = PeerCoordinator(bridge, {})
    coordinator.clock = lambda: 10.0

    def authority(order, component):
        return {
            "robot_id": "r0",
            "mission_id": mission,
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
