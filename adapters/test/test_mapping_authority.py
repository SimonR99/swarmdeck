import json
import sys
from types import SimpleNamespace as NS
from uuid import uuid4

import numpy as np
import pytest

from adapters.mapping_authority import MappingAuthority, snapshot_values
from autonomy.cslam import pose_matrix


def authority():
    return dict(
        robot_id="r0",
        mission_id=str(uuid4()),
        navigation_frame="map",
        component_id="component:test",
        map_epoch=1,
        mapping_graph_revision=2,
        geometry_revision="a" * 64,
        map_source_stamp={"sec": 3, "nanosec": 4},
        T_component_navigation=np.eye(4).tolist(),
    )


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
