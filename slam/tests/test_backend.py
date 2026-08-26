"""Online back-end: wire packets in, merged occupancy out. No grid registration."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest
import synthetic

from swarmdeck_protocol import encode_keyframe, decode_keyframe
from swarmdeck_slam.backend import (
    ODOM_INFORMATION,
    CollaborativeBackend,
    majority_component,
    scoped_grids,
    snapshot_update,
)
from swarmdeck_slam.types import EdgeKind, quat_xyz_from_se3, se3_inverse
from swarmdeck_slam.verify import VerifyConfig, verify_candidate


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


def test_hessian_information_is_the_production_default() -> None:
    """Production keeps GICP's conditioned Hessian, and no longer discards it.

    This asserted "isotropic" until 2026-08-25. That default was a workaround
    chosen on the synthetic fixture and explicitly scoped, in this test's own
    comment, to last only until real data existed to calibrate against. It now
    does: sessions/captures/3d-run-01 is a two-robot Gazebo run with ground
    truth, and replaying it (slam/tools/replay.py --ablate) measures hessian
    better on every scope -- optimized yaw RMSE 9.22 -> 7.38 deg for robot_0 and
    1.14 -> 0.74 for robot_1, joint ATE 0.6837 -> 0.6604 m and 6.69 -> 5.30 deg.

    Do not revert this to isotropic on synthetic-fixture evidence alone. The
    fixture is planar and non-repetitive, so it never exercises the degenerate
    corridor-slide geometry the conditioned Hessian exists to express, and it
    will keep preferring isotropic for that reason.
    """
    backend = CollaborativeBackend()
    assert backend.verify.information == "hessian"
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


def test_a_frame_jump_is_down_weighted_not_trusted() -> None:
    """An ODOMETRY edge is a GNC known-inlier, so nothing downstream can reject
    one. That makes a wrong odometry edge the only input with no defense behind
    it -- and ``t_odom_base`` really carries the robot's own SLAM map pose (see
    ``types.py``), which jumps when that SLAM re-optimizes and can switch to an
    odom-frame pose entirely when the adapter's TF lookup fails.

    The hop straddling such a change is not a measurement. It must stay in the
    graph -- dropping it would leave the far side of the chain gauge-free -- but
    it must not be asserted at full confidence.
    """
    _, fleet = synthetic.two_robot_fleet()
    alpha = fleet[0]
    jump = synthetic.yaw_pose(25.0, -13.0, np.deg2rad(70.0))

    backend = CollaborativeBackend()
    for position, kf in enumerate(alpha.keyframes):
        if position >= len(alpha.keyframes) // 2:
            kf = replace(kf, t_odom_base=jump @ kf.t_odom_base)
        backend.ingest_keyframe(kf)

    assert backend.implausible_hops == 1, (
        "exactly the one hop across the injected frame jump should be flagged"
    )
    odometry = [e for e in backend._graph._edges if e.kind is EdgeKind.ODOMETRY]
    flagged = [e for e in odometry if e.information.max() < ODOM_INFORMATION.max()]
    assert len(flagged) == 1
    assert flagged[0].information.max() == pytest.approx(
        ODOM_INFORMATION.max() * backend.implausible_hop_information_scale
    )
    assert len(odometry) == len(alpha.keyframes) - 1, "the edge is kept, never dropped"


def test_ordinary_motion_is_never_flagged_as_a_frame_jump() -> None:
    """The guard must not fire on real driving, including the multi-keyframe
    hops that a service queue drop legitimately produces."""
    _, fleet = synthetic.two_robot_fleet()
    backend = CollaborativeBackend()
    for kf in fleet[0].keyframes[::3]:  # every third keyframe: a 3x stretched hop
        backend.ingest_keyframe(kf)
    assert backend.implausible_hops == 0


def test_published_scopes_match_the_grids_that_get_published(shared_snapshot) -> None:
    """The server drops any scope missing from this list, so it must name
    exactly what ``_publish_snapshot`` actually POSTs -- otherwise the server
    garbage-collects a grid the service just sent, or keeps one forever."""
    snapshot, fleet = shared_snapshot
    scopes = [scope for scope, _grid in scoped_grids(snapshot)]

    assert snapshot_update(snapshot)["scopes"] == scopes
    assert len(set(scopes)) == len(scopes), "scope names must be unique"
    for robot in fleet:
        assert f"robot:{robot.robot_id}" in scopes
    for component in snapshot.optimized.components:
        assert f"component:{component.component_id}" in scopes


def test_hop_guard_falls_back_to_distance_when_the_clock_is_unusable() -> None:
    """Stamps come off a ROS header and are not guaranteed monotonic. With no
    usable ``dt`` the speed test is undefined, so the weaker absolute-distance
    gate stands in rather than the guard silently switching itself off."""
    backend = CollaborativeBackend()
    huge = synthetic.yaw_pose(40.0, 0.0, 0.0)
    ordinary = synthetic.yaw_pose(0.8, 0.0, 0.0)

    assert backend._implausible_hop(huge, dt_s=0.0) is True
    assert backend._implausible_hop(ordinary, dt_s=0.0) is False
    assert backend._implausible_hop(ordinary, dt_s=-5.0) is False


def test_registration_prior_is_withheld_across_a_component_boundary() -> None:
    """Two robots that have not merged have NO known relative transform.

    Seeding GICP by composing their two frames anyway would hand it a
    specific, confident, baseless guess -- and GICP converges near whatever it
    is given. Measured on real data, exactly that seed recovers 0 of 165
    genuinely co-located pairs. Bootstrap must stay on the yaw-only path.
    """
    _, fleet = synthetic.disjoint_fleet()
    backend = CollaborativeBackend(registration_prior="all")
    _ingest_fleet(backend, fleet)
    backend.optimize_and_render()

    solved = backend._last_solved
    assert len(solved.components) == 2, "fixture must leave the robots unmerged"
    alpha = next(kf for kf in backend._keyframes.values() if kf.id.robot_id == "alpha")
    beta = next(kf for kf in backend._keyframes.values() if kf.id.robot_id == "beta")

    assert backend._registration_prior(alpha, beta) is None
    assert backend._registration_prior(beta, alpha) is None
    # ...while a same-robot pair, which shares a frame by construction, is seeded.
    alpha_other = max(
        (kf for kf in backend._keyframes.values() if kf.id.robot_id == "alpha"),
        key=lambda kf: kf.id.seq,
    )
    assert backend._registration_prior(alpha, alpha_other) is not None


def test_registration_prior_scope_controls_inter_robot_seeding() -> None:
    """``intra`` seeds only same-robot pairs; ``all`` also seeds merged robots."""
    _, fleet = synthetic.two_robot_fleet()
    for scope, expect_inter in (("none", False), ("intra", False), ("all", True)):
        backend = CollaborativeBackend(registration_prior=scope)
        _ingest_fleet(backend, fleet)
        backend.optimize_and_render()
        assert len(backend._last_solved.components) == 1, "fixture must merge"

        by_robot = {}
        for kf in backend._keyframes.values():
            by_robot.setdefault(kf.id.robot_id, []).append(kf)
        a, b = (sorted(v, key=lambda kf: kf.id.seq) for v in by_robot.values())

        assert (backend._registration_prior(a[-1], b[0]) is not None) is expect_inter, scope
        same = backend._registration_prior(a[-1], a[0])
        assert (same is not None) is (scope != "none"), scope


def test_registration_prior_places_the_source_where_the_graph_thinks_it_is() -> None:
    """The seed is ``T_target_source``, and the source keyframe is NOT yet in
    the graph -- it is the one being ingested. It has to be placed with its
    robot's ``t_world_map`` correction composed onto its own pose, which is
    precisely what that quantity is documented for."""
    _, fleet = synthetic.two_robot_fleet()
    backend = CollaborativeBackend(registration_prior="all")
    _ingest_fleet(backend, fleet)
    backend.optimize_and_render()
    solved = backend._last_solved

    keyframes = sorted(backend._keyframes.values(), key=lambda kf: (kf.id.robot_id, kf.id.seq))
    source, target = keyframes[0], keyframes[-1]
    prior = backend._registration_prior(source, target)

    expected = se3_inverse(solved.poses[target.id]) @ (
        solved.t_world_map[source.id.robot_id] @ source.t_odom_base
    )
    assert np.allclose(prior, expected)


def test_a_seeded_verification_still_has_to_pass_every_gate() -> None:
    """A seed must not become a way to launder an unverified match.

    Two keyframes from genuinely different buildings, handed the strongest
    possible seed (their true relative transform), must still be rejected --
    the gates measure geometric support, not agreement with the seed.
    """
    _, fleet = synthetic.disjoint_fleet()
    alpha, beta = fleet
    src, dst = alpha.keyframes[0], beta.keyframes[0]
    oracle = se3_inverse(beta.truth[dst.id]) @ alpha.truth[src.id]

    assert verify_candidate(source=src, target=dst, yaw_prior=0.0,
                            t_target_source_prior=oracle) is None
