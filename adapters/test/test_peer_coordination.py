import json
import sys
from types import SimpleNamespace as NS
import uuid
import numpy as np
from adapters.peer_coordination import PeerCoordinator
from adapters.exploration import PlannerPath, PlannerPose


def test_authority_freshness_path_generation_and_correction(monkeypatch):
    messages = []
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
