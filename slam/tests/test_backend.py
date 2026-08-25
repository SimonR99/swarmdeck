"""Online back-end: wire packets in, merged occupancy out. No grid registration."""

from __future__ import annotations

import numpy as np
import pytest
import synthetic

from swarmdeck_protocol import encode_keyframe, decode_keyframe
from swarmdeck_slam.backend import (
    CollaborativeBackend,
    majority_component,
    snapshot_update,
)
from swarmdeck_slam.types import quat_xyz_from_se3
from swarmdeck_slam.verify import VerifyConfig


def _ingest_fleet(backend: CollaborativeBackend, fleet) -> None:
    """Stream keyframes in capture order, through the real wire format."""
    keyed = []
    for robot in fleet:
        for kf in robot.keyframes:
            keyed.append(kf)
    keyed.sort(key=lambda kf: (kf.stamp, kf.id.robot_id, kf.id.seq))
    for kf in keyed:
        blob = encode_keyframe(
            robot_id=kf.id.robot_id,
            seq=kf.id.seq,
            stamp=kf.stamp,
            points=kf.points,
            t_odom_base=quat_xyz_from_se3(kf.t_odom_base),
        )
        assert backend.ingest_packet(decode_keyframe(blob))


@pytest.fixture(scope="module")
def shared_snapshot():
    _, fleet = synthetic.two_robot_fleet()
    backend = CollaborativeBackend()
    _ingest_fleet(backend, fleet)
    snapshot = backend.optimize_and_render()
    assert snapshot is not None
    return snapshot, fleet


def test_robots_sharing_a_building_merge_through_the_online_backend(shared_snapshot) -> None:
    snapshot, fleet = shared_snapshot
    components = snapshot.optimized.components
    assert len(components) == 1
    assert components[0].robots == {robot.robot_id for robot in fleet}
    grid = majority_component(snapshot)
    assert grid is not None
    assert grid.robots == components[0].robots
    assert int((grid.cells == 100).sum()) > 0


def test_disjoint_buildings_are_never_merged() -> None:
    _, fleet = synthetic.disjoint_fleet()
    backend = CollaborativeBackend()
    _ingest_fleet(backend, fleet)
    snapshot = backend.optimize_and_render()
    assert snapshot is not None
    assert majority_component(snapshot) is None
    assert all(len(c.robots) == 1 for c in snapshot.optimized.components)
    update = snapshot_update(snapshot)
    assert all(not graph["in_common_frame"] for graph in update["graphs"].values())


def test_duplicate_seq_is_ignored() -> None:
    _, fleet = synthetic.two_robot_fleet()
    backend = CollaborativeBackend()
    kf = fleet[0].keyframes[0]
    blob = encode_keyframe(
        robot_id=kf.id.robot_id,
        seq=kf.id.seq,
        stamp=kf.stamp,
        points=kf.points,
        t_odom_base=quat_xyz_from_se3(kf.t_odom_base),
    )
    packet = decode_keyframe(blob)
    assert backend.ingest_packet(packet)
    assert not backend.ingest_packet(packet)
    assert len(backend) == 1


def test_snapshot_update_only_flags_merged_robots(shared_snapshot) -> None:
    snapshot, fleet = shared_snapshot
    update = snapshot_update(snapshot)
    ids = {robot.robot_id for robot in fleet}
    assert set(update["graphs"]) == ids
    assert all(update["graphs"][rid]["in_common_frame"] for rid in ids)
    assert all(update["origins"][rid]["frame"].startswith("component-") for rid in ids)


def test_isotropic_information_is_the_production_default() -> None:
    backend = CollaborativeBackend()
    assert backend.verify.information == "isotropic"
    # Unit tests keep the Hessian path; production is the documented workaround
    # for the ATE xfail until real Ouster bags exist to calibrate against.
    assert VerifyConfig().information == "hessian"


def _thicken_planar(points: np.ndarray) -> np.ndarray:
    """What adapter_sim does with a 2D lidar: copy the ring at a few heights."""
    xy = np.asarray(points, dtype=np.float32).copy()
    xy[:, 2] = 0.0
    return np.vstack([
        np.column_stack([xy[:, 0], xy[:, 1], np.full(len(xy), z, dtype=np.float32)])
        for z in (0.0, 0.12, 0.24)
    ])


def test_planar_thickened_scans_still_merge_two_robots() -> None:
    """Gazebo's default lidar is one ring. The sim adapter thickens it.

    If this fails, `4robot.yaml` cannot produce a collaborative map until the
    fleet is switched to a multi-ring lidar; do not "fix" it by lowering
    verification gates.
    """
    _, fleet = synthetic.two_robot_fleet()
    backend = CollaborativeBackend()
    keyed = []
    for robot in fleet:
        for kf in robot.keyframes:
            keyed.append(kf)
    keyed.sort(key=lambda kf: (kf.stamp, kf.id.robot_id, kf.id.seq))
    for kf in keyed:
        blob = encode_keyframe(
            robot_id=kf.id.robot_id,
            seq=kf.id.seq,
            stamp=kf.stamp,
            points=_thicken_planar(kf.points),
            t_odom_base=quat_xyz_from_se3(kf.t_odom_base),
        )
        backend.ingest_packet(decode_keyframe(blob))
    snapshot = backend.optimize_and_render()
    assert snapshot is not None
    assert len(snapshot.optimized.components) == 1
    grid = majority_component(snapshot)
    assert grid is not None
    assert int((grid.cells == 100).sum()) > 0
