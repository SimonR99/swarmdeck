import json
import sys
from types import SimpleNamespace as NS
from uuid import uuid4

import numpy as np
import pytest

from adapters.mapping_authority import (
    MappingAuthority,
    accepts_authority_update,
    authority_for_frame,
    expected_mission_id,
    planning_frame,
    snapshot_values,
)
from autonomy.cslam import pose_matrix
from autonomy.map_epochs import robot_run_id


def authority():
    mission = str(uuid4())
    return dict(
        robot_id="r0",
        mission_id=mission,
        robot_map_epoch=0,
        run_id=robot_run_id(mission, "r0", 0),
        navigation_frame="map",
        component_id="component:test",
        map_epoch=1,
        mapping_graph_revision=2,
        geometry_revision="a" * 64,
        map_source_stamp={"sec": 3, "nanosec": 4},
        T_component_navigation=np.eye(4).tolist(),
    )


def test_planning_frame_defaults_to_map_and_expands_robot(monkeypatch):
    bridge = NS(id="r0", map_frame="/r0/map_frame")
    monkeypatch.delenv("SWARMDECK_PLANNING_FRAME_TEMPLATE", raising=False)
    assert planning_frame(bridge) == "r0/map_frame"
    monkeypatch.setenv("SWARMDECK_PLANNING_FRAME_TEMPLATE", "/{robot}/odom")
    assert planning_frame(bridge) == "r0/odom"
    monkeypatch.setenv("SWARMDECK_PLANNING_FRAME_TEMPLATE", "{unknown}/odom")
    with pytest.raises(ValueError, match="template"):
        planning_frame(bridge)
    for invalid in (" {robot}/odom", "{robot}//odom", "{robot}/odom/"):
        monkeypatch.setenv("SWARMDECK_PLANNING_FRAME_TEMPLATE", invalid)
        with pytest.raises(ValueError, match="planning frame"):
            planning_frame(bridge)


def test_authority_for_stable_frame_preserves_component_home():
    value = authority()
    value["navigation_frame"] = "r0/map_frame"
    # Live evidence showed these two translations changing together while
    # component<-odom remained invariant.
    component_from_map = np.eye(4)
    component_from_map[0, 3] = 1.04
    value["T_component_navigation"] = component_from_map.tolist()
    value["planning_frame"] = "r0/odom"
    value["T_component_planning"] = np.eye(4).tolist()
    map_from_home = np.eye(4)
    map_from_home[0, 3] = 2.0
    value["home"] = {
        "keyframe_id": "home:0",
        "T_navigation_home": map_from_home.tolist(),
    }

    selected = authority_for_frame(value, "/r0/odom")
    assert selected["navigation_frame"] == "r0/odom"
    np.testing.assert_allclose(selected["T_component_navigation"], np.eye(4))
    expected_odom_home = component_from_map @ map_from_home
    np.testing.assert_allclose(
        selected["home"]["T_navigation_home"], expected_odom_home
    )
    # Selection is a copy and cannot alter the UI authority.
    assert value["navigation_frame"] == "r0/map_frame"
    np.testing.assert_allclose(value["home"]["T_navigation_home"], map_from_home)


@pytest.mark.parametrize(
    "change",
    [
        {},
        {"planning_frame": "r0/other", "T_component_planning": np.eye(4).tolist()},
        {"planning_frame": "r0/odom", "T_component_planning": [[1.0]]},
    ],
)
def test_authority_for_stable_frame_never_falls_back(change):
    value = {**authority(), **change}
    with pytest.raises(ValueError):
        authority_for_frame(value, "r0/odom")


def test_same_frame_missing_transform_has_bounded_error():
    value = authority()
    del value["T_component_navigation"]
    with pytest.raises(ValueError, match="no navigation transform"):
        authority_for_frame(value, "map")


@pytest.mark.parametrize("axis", range(3))
def test_snapshot_rotation_handles_half_turn(axis):
    value = authority()
    matrix = np.eye(4)
    matrix[:3, :3] = -np.eye(3)
    matrix[axis, axis] = 1
    matrix[:3, 3] = [1, 2, 3]
    value["T_component_navigation"] = matrix.tolist()
    *_, xyz, q = snapshot_values(value)
    recovered = pose_matrix(
        NS(
            position=NS(x=xyz[0], y=xyz[1], z=xyz[2]),
            orientation=NS(x=q[0], y=q[1], z=q[2], w=q[3]),
        )
    )
    np.testing.assert_allclose(recovered, matrix, atol=1e-10)


def test_authority_relays_exact_key_and_expires(monkeypatch):
    published = []
    monkeypatch.delenv("SWARMDECK_MISSION_ID", raising=False)

    def message():
        return NS(
            source_stamp=NS(sec=0, nanosec=0),
            component_from_navigation=NS(
                translation=NS(x=0, y=0, z=0), rotation=NS(x=0, y=0, z=0, w=1)
            ),
        )

    monkeypatch.setitem(sys.modules, "std_msgs.msg", NS(String=object))
    monkeypatch.setitem(sys.modules, "mgg_msgs.msg", NS(MappingSnapshot=message))
    monkeypatch.setitem(
        sys.modules,
        "rclpy.qos",
        NS(QoSProfile=lambda **kw: kw, DurabilityPolicy=NS(TRANSIENT_LOCAL=1)),
    )
    node = NS(
        create_subscription=lambda *args: None,
        create_publisher=lambda *args: NS(publish=published.append),
    )
    reader = MappingAuthority(NS(node=node, id="r0", map_frame="map"))
    clock = [10.0]
    reader.clock = lambda: clock[0]
    value = authority()
    reader.receive(NS(data=json.dumps(value)))
    assert reader.current() == value
    out = published[-1]
    assert (
        out.epoch,
        out.graph_revision,
        out.geometry_revision,
        out.source_stamp.sec,
        out.source_stamp.nanosec,
    ) == (1, 2, "a" * 64, 3, 4)
    broken = {**value, "map_epoch": -1}
    clock[0] = 12
    reader.receive(NS(data=json.dumps(broken)))
    assert len(published) == 1
    reader.receive(NS(data=json.dumps({**value, "mapping_graph_revision": 1})))
    assert len(published) == 1
    assert reader.received_at == 10
    clock[0] = 14
    assert reader.current() is None


def test_snapshot_uses_opted_in_planning_authority_but_current_stays_ui(monkeypatch):
    published = []

    def message():
        return NS(
            source_stamp=NS(sec=0, nanosec=0),
            component_from_navigation=NS(
                translation=NS(x=0, y=0, z=0),
                rotation=NS(x=0, y=0, z=0, w=1),
            ),
        )

    monkeypatch.delenv("SWARMDECK_MISSION_ID", raising=False)
    monkeypatch.setenv("SWARMDECK_PLANNING_FRAME_TEMPLATE", "{robot}/odom")
    monkeypatch.setitem(sys.modules, "std_msgs.msg", NS(String=object))
    monkeypatch.setitem(sys.modules, "mgg_msgs.msg", NS(MappingSnapshot=message))
    monkeypatch.setitem(
        sys.modules,
        "rclpy.qos",
        NS(QoSProfile=lambda **kw: kw, DurabilityPolicy=NS(TRANSIENT_LOCAL=1)),
    )
    node = NS(
        create_subscription=lambda *args: None,
        create_publisher=lambda *args: NS(publish=published.append),
    )
    reader = MappingAuthority(NS(node=node, id="r0", map_frame="map"))
    value = authority()
    value["planning_frame"] = "r0/odom"
    component_from_odom = np.eye(4)
    component_from_odom[0, 3] = 5.0
    value["T_component_planning"] = component_from_odom.tolist()
    reader.receive(NS(data=json.dumps(value)))

    assert reader.current()["navigation_frame"] == "map"
    assert reader.current_for_frame("r0/odom")["navigation_frame"] == "r0/odom"
    assert published[-1].component_from_navigation.translation.x == 5.0


def test_reordered_authority_cannot_rollback_or_renew_freshness(monkeypatch):
    published = []

    def message():
        return NS(
            source_stamp=NS(sec=0, nanosec=0),
            component_from_navigation=NS(
                translation=NS(x=0, y=0, z=0),
                rotation=NS(x=0, y=0, z=0, w=1),
            ),
        )

    monkeypatch.delenv("SWARMDECK_MISSION_ID", raising=False)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", NS(String=object))
    monkeypatch.setitem(sys.modules, "mgg_msgs.msg", NS(MappingSnapshot=message))
    monkeypatch.setitem(
        sys.modules,
        "rclpy.qos",
        NS(QoSProfile=lambda **kw: kw, DurabilityPolicy=NS(TRANSIENT_LOCAL=1)),
    )
    node = NS(
        create_subscription=lambda *args: None,
        create_publisher=lambda *args: NS(publish=published.append),
    )
    reader = MappingAuthority(NS(node=node, id="r0", map_frame="map"))
    clock = [10.0]
    reader.clock = lambda: clock[0]
    current = {**authority(), "solution_order": [2, 0]}
    reader.receive(NS(data=json.dumps(current)))

    clock[0] = 11.0
    reader.receive(NS(data=json.dumps(current)))
    assert reader.received_at == 11.0  # Equal heartbeat renews receipt freshness.
    assert len(published) == 2

    # The snapshot counter is higher so this specifically exercises transport
    # reordering of the optimizer's causal solution key.
    stale = {
        **current,
        "solution_order": [1, 9],
        "mapping_graph_revision": 3,
    }
    clock[0] = 15.0
    reader.receive(NS(data=json.dumps(stale)))
    assert reader.current() is None
    assert reader.received_at == 11.0
    assert len(published) == 2

    newer = {
        **current,
        "solution_order": [3, 0],
        "mapping_graph_revision": 3,
    }
    reader.receive(NS(data=json.dumps(newer)))
    assert reader.current() == newer
    assert reader.received_at == 15.0

    # Once indexed, neither a higher unordered legacy message nor conflicting
    # immutable metadata at the same snapshot key may replace the authority.
    legacy = {
        key: value
        for key, value in {**newer, "solution_order": [4, 0]}.items()
        if key
        not in {
            "map_epoch",
            "mapping_graph_revision",
            "geometry_revision",
            "map_source_stamp",
        }
    }
    reader.receive(NS(data=json.dumps(legacy)))
    for conflict in (
        {"component_id": "component:other"},
        {"geometry_revision": "b" * 64},
        {"map_source_stamp": {"sec": 99, "nanosec": 0}},
    ):
        reader.receive(NS(data=json.dumps({**newer, **conflict})))
    moved = np.eye(4)
    moved[0, 3] = 1e-5
    corrected = {**newer, "T_component_navigation": moved.tolist()}
    reader.receive(NS(data=json.dumps(corrected)))
    # The navigation-frame TF is live and may change between heartbeats without
    # changing the indexed component snapshot.
    assert reader.current() == corrected
    assert len(published) == 4


def test_mission_filter_is_exact_and_unconfigured_readers_pin_first(monkeypatch):
    first = authority()
    first["solution_order"] = [0, -1]
    other = {**first, "mission_id": str(uuid4()), "solution_order": [99, 0]}
    monkeypatch.setenv("SWARMDECK_MISSION_ID", first["mission_id"])
    assert expected_mission_id("r0") == first["mission_id"]
    assert accepts_authority_update(first, None, expected_mission_id("r0"))
    assert not accepts_authority_update(other, None, expected_mission_id("r0"))
    assert not accepts_authority_update(other, first)


def test_reset_notice_cancels_only_its_robot_and_fences_delayed_authority(monkeypatch):
    from adapters.onboard_mapping import map_source_is_current, map_upload_headers

    monkeypatch.delenv("SWARMDECK_MISSION_ID", raising=False)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", NS(String=object))
    monkeypatch.setitem(sys.modules, "mgg_msgs.msg", None)
    node = NS(create_subscription=lambda *_args: None)
    target = NS(
        id="r0", node=node, map_frame="map", onboard_mapping=True, nav_status="active"
    )
    target.cancel_goal = lambda: setattr(target, "nav_status", "cancelled")
    target.drive = lambda *_: None
    reader = target._mapping_authority = MappingAuthority(target)
    old = authority()
    for field in (
        "map_epoch",
        "mapping_graph_revision",
        "geometry_revision",
        "map_source_stamp",
    ):
        old.pop(field)
    reader.receive(NS(data=json.dumps(old)))
    assert reader.current() == old
    notice = {
        "robot_id": "r0",
        "mission_id": old["mission_id"],
        "robot_map_epoch": 1,
        "run_id": robot_run_id(old["mission_id"], "r0", 1),
        "state": "resetting",
    }
    reader.receive(
        NS(
            data=json.dumps(
                {
                    **notice,
                    "robot_id": "r1",
                    "run_id": robot_run_id(old["mission_id"], "r1", 1),
                }
            )
        )
    )
    assert target.nav_status == "active"
    assert reader.current() == old
    reader.receive(NS(data=json.dumps(notice)))
    assert target.nav_status == "cancelled"
    assert reader.current() is None
    assert map_upload_headers(target) is None
    reader.receive(NS(data=json.dumps(old)))
    assert reader.current() is None
    fresh = {**old, **notice, "source_reset_stamp": {"sec": 12, "nanosec": 5}}
    fresh.pop("state")
    reader.receive(NS(data=json.dumps(fresh)))
    assert reader.current() == fresh
    assert map_upload_headers(target)["X-Run-Id"] == notice["run_id"]
    assert not map_source_is_current(target, NS(header=NS(stamp=NS(sec=12, nanosec=5))))
    assert map_source_is_current(target, NS(header=NS(stamp=NS(sec=12, nanosec=6))))
    reader.receive(NS(data=json.dumps(old)))
    assert reader.current() == fresh
