"""Tests for :mod:`swarmdeck_slam.graph` (``GtsamPoseGraph``).

This module shipped with zero test coverage -- its author reported a "basic
smoke test" passing, but that claim was never independently checked. Every
test here therefore runs the real ``gtsam`` GNC+LM solver end to end; nothing
about PCM, GNC, or the factor graph is mocked.

Two ways of constructing loop-closure edges are used, deliberately:

* **Real GICP** (:func:`_find_real_closures`, via ``swarmdeck_slam.verify``)
  for tests that want a realistic, independently-produced edge -- ground
  truth is used only to *propose* which keyframe pairs are close enough to be
  worth verifying (standing in for a place-recognition front end, which is
  not this module's concern), exactly as ``tests/test_verify.py`` does. Every
  resulting :class:`~swarmdeck_slam.types.Edge` is then as real as production
  code would hand ``graph.py``.
* **Hand-built directly from ground truth** for tests that need one fully
  controlled edge -- most importantly the direction-inversion regression,
  where GICP noise would only obscure the thing being tested.

Odometry edges are always built by :func:`_odometry_edges`, with an
information matrix calibrated (with margin) to ``tests/synthetic.py``'s own
``drift_per_metre``/``yaw_drift_per_metre`` model. An uncalibrated (too
tight) odometry information matrix starves every loop closure of the leverage
needed to correct drift, which would make these tests about their own
miscalibration rather than about ``graph.py`` -- this was verified the hard
way while writing this file (see the ATE test's docstring).
"""

from __future__ import annotations

import time
from dataclasses import replace

import numpy as np
import pytest

from swarmdeck_slam.evaluation import (
    compute_ate,
    inter_robot_transform_error,
    score_components,
)
from swarmdeck_slam.graph import GtsamPoseGraph
from swarmdeck_slam.types import (
    Edge,
    EdgeKind,
    Keyframe,
    KeyframeId,
    OptimizedGraph,
    se3_identity,
    se3_inverse,
    se3_relative,
)
from swarmdeck_slam.verify import verify_candidate

from synthetic import (
    SyntheticRobot,
    disjoint_fleet,
    make_scene,
    simulate_robot,
    truth_groups,
    two_robot_fleet,
    yaw_pose,
)

# --------------------------------------------------------------------------- #
# Shared helpers -- DRY edge construction, reused by every test below
# --------------------------------------------------------------------------- #

# Margin applied on top of synthetic.py's own drift-per-metre constants when
# turning them into an odometry sigma: real odometry noise statistics are
# never known exactly, so a solver calibrated to the textbook number with no
# slack is an unrealistic best case. 2x was measured (see report) to leave
# real leverage for loop closures to correct drift without being so loose
# that odometry stops constraining anything.
_ODOM_SIGMA_MARGIN = 2.0
_DEFAULT_DRIFT_PER_METRE = 0.012
_DEFAULT_YAW_DRIFT_PER_METRE = 0.004


def _closure_information(sigma_t: float, sigma_r: float) -> np.ndarray:
    """A diagonal 6x6 information matrix in TANGENT_ORDER (rotation first)."""
    return np.diag([1.0 / sigma_r**2] * 3 + [1.0 / sigma_t**2] * 3)


def _odometry_edges(
    robot: SyntheticRobot, *, margin: float = _ODOM_SIGMA_MARGIN
) -> list[Edge]:
    """``ODOMETRY`` edges between consecutive keyframes, from the robot's own
    (drifted) ``t_odom_base`` -- exactly what a real front end would report,
    since it never sees ground truth. Sigma is proportional to the step
    length, matching synthetic.py's own drift model (see module docstring).
    """
    edges: list[Edge] = []
    keyframes = robot.keyframes
    for i in range(len(keyframes) - 1):
        t_src_dst = se3_relative(keyframes[i].t_odom_base, keyframes[i + 1].t_odom_base)
        step_m = float(np.linalg.norm(t_src_dst[:3, 3]))
        sigma_t = max(_DEFAULT_DRIFT_PER_METRE * step_m, 0.005) * margin
        sigma_r = max(_DEFAULT_YAW_DRIFT_PER_METRE * step_m, 0.002) * margin
        edges.append(
            Edge(
                EdgeKind.ODOMETRY,
                keyframes[i].id,
                keyframes[i + 1].id,
                t_src_dst,
                _closure_information(sigma_t, sigma_r),
            )
        )
    return edges


def _find_real_closures(
    robots: list[SyntheticRobot], *, radius: float = 3.0, min_index_gap: int = 5
) -> list[Edge]:
    """Real ``verify_candidate`` closures between every keyframe pair (intra-
    or inter-robot) within ``radius`` of each other in ground truth.

    Ground truth is used only to propose candidates, the same way
    ``tests/test_verify.py`` does -- place recognition is a different
    module's job. What each candidate becomes (an :class:`Edge`, or nothing)
    is decided entirely by real GICP registration.
    """
    all_keyframes = [(robot, kf) for robot in robots for kf in robot.keyframes]
    edges: list[Edge] = []
    for i, (robot_a, kf_a) in enumerate(all_keyframes):
        for robot_b, kf_b in all_keyframes[i + 1 :]:
            if robot_a is robot_b and abs(kf_a.id.seq - kf_b.id.seq) < min_index_gap:
                continue
            distance = float(
                np.linalg.norm(
                    robot_a.truth[kf_a.id][:3, 3] - robot_b.truth[kf_b.id][:3, 3]
                )
            )
            if distance >= radius:
                continue
            t_src_dst_true = se3_relative(
                robot_a.truth[kf_a.id], robot_b.truth[kf_b.id]
            )
            t_target_source = se3_inverse(t_src_dst_true)
            yaw_prior = float(np.arctan2(t_target_source[1, 0], t_target_source[0, 0]))
            edge = verify_candidate(kf_a, kf_b, yaw_prior)
            if edge is not None:
                edges.append(edge)
    return edges


def _build_graph(
    robots: list[SyntheticRobot], closures: list[Edge] = (), **kwargs
) -> GtsamPoseGraph:
    pose_graph = GtsamPoseGraph(**kwargs)
    for robot in robots:
        for keyframe in robot.keyframes:
            pose_graph.add_keyframe(keyframe)
        for edge in _odometry_edges(robot):
            pose_graph.add_edge(edge)
    for edge in closures:
        pose_graph.add_edge(edge)
    return pose_graph


def _drifted_poses(robot: SyntheticRobot) -> dict[KeyframeId, np.ndarray]:
    """The pre-optimization estimate: raw odometry composed with the robot's
    true start frame (assumed correctly known, per ``tests/test_render.py``'s
    ``_drifted_poses``, so a component-merge error is never conflated with
    plain trajectory drift).
    """
    return {kf.id: robot.t_world_map_true @ kf.t_odom_base for kf in robot.keyframes}


def _robot_poses(result: OptimizedGraph, robot_id: str) -> dict[KeyframeId, np.ndarray]:
    return {
        kf_id: pose
        for kf_id, pose in result.poses.items()
        if kf_id.robot_id == robot_id
    }


def _synthetic_fleet(
    n_robots: int, n_keyframes: int, *, seed: int = 1
) -> list[SyntheticRobot]:
    """An N-robot fleet sharing one scene, all driving the same rectangle from
    different odometry seeds -- for the scaling test, which needs a robot
    count the shared two/disjoint fixtures don't parametrize.
    """
    scene = make_scene(seed)
    waypoints = [(3.0, 3.0), (9.0, 3.0), (9.0, 20.0), (3.0, 20.0), (3.0, 3.0)]
    return [
        simulate_robot(
            scene,
            f"r{i}",
            waypoints,
            seed=seed + i + 1,
            n_keyframes=n_keyframes,
            drift_per_metre=0.03,
            yaw_drift_per_metre=0.01,
        )
        for i in range(n_robots)
    ]


def _hand_built_loop_closures(
    robots: list[SyntheticRobot], *, rng: np.random.Generator
) -> list[Edge]:
    """Two intra-robot closures per robot, straight from ground truth plus a
    small perturbation -- cheap (no GICP) and gives LM genuine residual to
    work on, which matters for the scaling test: a graph where every closure
    already sits at the optimum converges in zero LM iterations and would
    understate real-world cost.
    """
    edges: list[Edge] = []
    info = _closure_information(sigma_t=0.03, sigma_r=np.deg2rad(2.0))
    for robot in robots:
        keyframes = robot.keyframes
        for i, j in ((0, len(keyframes) - 1), (2, len(keyframes) - 3)):
            src, dst = keyframes[i], keyframes[j]
            t_src_dst = se3_relative(robot.truth[src.id], robot.truth[dst.id])
            perturbation = se3_identity()
            perturbation[:3, 3] = rng.normal(scale=0.02, size=3)
            edges.append(
                Edge(
                    EdgeKind.INTRA_LOOP, src.id, dst.id, t_src_dst @ perturbation, info
                )
            )
    return edges


# --------------------------------------------------------------------------- #
# Shared fixture: two overlapping robots, real closures, one optimize() call
# --------------------------------------------------------------------------- #


@pytest.fixture(scope="module")
def merged_fleet() -> tuple[list[SyntheticRobot], list[Edge], OptimizedGraph]:
    """``two_robot_fleet``, wired up with real odometry and real GICP closures
    and optimized once. Several tests below only *read* this result, so it is
    built once per module rather than once per test.
    """
    _scene, robots = two_robot_fleet(seed=3)
    closures = _find_real_closures(robots)
    assert len(closures) >= 10, (
        f"only {len(closures)} real closures found for seed=3 -- fixture assumptions "
        "below (component merges, PCM has corroborating edges to check against) need it"
    )
    result = _build_graph(robots, closures).optimize()
    return robots, closures, result


# --------------------------------------------------------------------------- #
# 1. Optimizing a drifting loop measurably reduces ATE
# --------------------------------------------------------------------------- #


def test_optimize_reduces_ate_on_drifting_loop(merged_fleet) -> None:
    """Both robots' optimized trajectories are closer to ground truth than
    their raw drifted odometry.

    This is not automatic in pose-graph SLAM: an early version of this test
    used a hand-picked, too-tight odometry information matrix and saw the
    OPPOSITE result (ATE getting *worse* after "optimization") purely because
    the solver had no reason to trust a real, correct loop closure over
    odometry it was told to treat as near-exact. Calibrating the odometry
    sigma to synthetic.py's own injected drift statistics (see
    ``_odometry_edges``) fixed that -- a lesson about the test, not a defect
    found in ``graph.py``.
    """
    robots, _closures, result = merged_fleet
    for robot in robots:
        pre = compute_ate(_drifted_poses(robot), robot.truth)
        post = compute_ate(_robot_poses(result, robot.robot_id), robot.truth)
        assert post.translation_m.rmse < pre.translation_m.rmse, (
            f"{robot.robot_id}: optimized ATE rmse {post.translation_m.rmse:.4f} did not "
            f"improve on pre-optimization ATE rmse {pre.translation_m.rmse:.4f}"
        )


# --------------------------------------------------------------------------- #
# 2. A confident outlier closure is rejected and does not corrupt the solution
# --------------------------------------------------------------------------- #


def test_inter_robot_outlier_rejected_by_pcm_without_corrupting_solution(
    merged_fleet,
) -> None:
    """PCM is the only defense that can see an inter-robot outlier (GNC judges
    by residual against the rest of the graph, and a self-consistent outlier
    has a small residual by definition -- see graph.py's module docstring).
    One fabricated, confidently-wrong inter-robot closure is added alongside
    the fleet's ~10+ genuine ones; PCM has no corroborating partner for it, so
    it should be rejected outright and the solution should come out
    identical to the closure-free run.
    """
    robots, closures, clean_result = merged_fleet
    bad_edge = Edge(
        kind=EdgeKind.INTER_LOOP,
        src=KeyframeId("alpha", 3),
        dst=KeyframeId("beta", 12),
        t_src_dst=yaw_pose(5.0, -5.0, 1.2),
        information=_closure_information(sigma_t=0.02, sigma_r=np.deg2rad(1.0)),
    )
    result = _build_graph(robots, [*closures, bad_edge]).optimize()

    assert bad_edge in result.rejected_edges
    assert len(result.components) == 1
    assert result.components[0].robots == frozenset({"alpha", "beta"})
    for robot in robots:
        clean_ate = compute_ate(_robot_poses(clean_result, robot.robot_id), robot.truth)
        outlier_ate = compute_ate(_robot_poses(result, robot.robot_id), robot.truth)
        assert outlier_ate.translation_m.rmse == pytest.approx(
            clean_ate.translation_m.rmse, abs=1e-9
        ), f"{robot.robot_id}: solution changed after a PCM-rejected outlier was added"


def test_intra_robot_outlier_rejected_by_gnc_without_corrupting_solution(
    merged_fleet,
) -> None:
    """Intra-robot closures never reach PCM (there is no second robot to
    cross-check against -- see module docstring), so GNC is the only defense
    available for this case. A confidently-wrong intra-robot closure (claims
    two far-apart keyframes on ``alpha`` are the same place) should be
    down-weighted below the GNC threshold and excluded from the final refit.
    """
    robots, closures, clean_result = merged_fleet
    alpha = next(robot for robot in robots if robot.robot_id == "alpha")
    bad_edge = Edge(
        kind=EdgeKind.INTRA_LOOP,
        src=KeyframeId("alpha", 2),
        dst=KeyframeId("alpha", 18),
        t_src_dst=yaw_pose(0.3, -0.2, 0.05),
        information=_closure_information(sigma_t=0.02, sigma_r=np.deg2rad(1.0)),
    )
    result = _build_graph(robots, [*closures, bad_edge]).optimize()

    assert bad_edge in result.rejected_edges
    clean_ate = compute_ate(_robot_poses(clean_result, "alpha"), alpha.truth)
    outlier_ate = compute_ate(_robot_poses(result, "alpha"), alpha.truth)
    # A looser bound than the PCM case: unlike PCM (which excludes the outlier
    # before GNC ever runs), GNC's own mu-graduation sees the outlier before
    # rejecting it, so it can perturb the LM refit's starting point very
    # slightly even once excluded from the final refit graph.
    assert outlier_ate.translation_m.rmse < clean_ate.translation_m.rmse * 1.1


# --------------------------------------------------------------------------- #
# 3. Robots with no inter-robot closure stay in separate components
# --------------------------------------------------------------------------- #


def test_disjoint_fleet_without_closures_stays_in_separate_components() -> None:
    """Two robots in two different buildings (``disjoint_fleet``), given only
    their own odometry and no inter-robot closure at all, must never be
    placed in one frame -- this is the most important property this package
    has (see graph.py and types.py module docstrings on false merges)."""
    _scenes, robots = disjoint_fleet(seed=3)
    result = _build_graph(robots).optimize()

    assert len(result.components) == 2
    assert {component.robots for component in result.components} == {
        frozenset({"alpha"}),
        frozenset({"beta"}),
    }
    assert not result.share_frame("alpha", "beta")
    assert score_components(result.components, truth_groups(robots)).is_perfect


def test_single_uncorroborated_cross_building_closure_is_rejected() -> None:
    """Even a single, confidently-wrong closure fabricated between the two
    disjoint buildings must not merge them: the default
    ``min_pcm_clique_size=2`` means an inter-robot closure needs an
    independent corroborating closure between the same two robots before PCM
    will accept it -- a lone closure, however confident, is exactly the
    failure mode PCM exists to catch (see ``GtsamPoseGraph.__init__``)."""
    _scenes, robots = disjoint_fleet(seed=3)
    bad_edge = Edge(
        kind=EdgeKind.INTER_LOOP,
        src=KeyframeId("alpha", 5),
        dst=KeyframeId("beta", 5),
        t_src_dst=yaw_pose(1.0, 2.0, 0.3),
        information=_closure_information(sigma_t=0.02, sigma_r=np.deg2rad(1.0)),
    )
    result = _build_graph(robots, [bad_edge]).optimize()

    assert bad_edge in result.rejected_edges
    assert not result.share_frame("alpha", "beta")


def test_unmerged_robot_still_gets_its_own_component() -> None:
    """Answers the required question directly: does the optimizer emit a
    ``Component`` for a robot with keyframes but no verified inter-robot
    closure? **Yes.** ``render.py`` assumes exactly this (it renders such a
    robot alone in its own singleton component rather than dropping its map
    -- see ``render.py``'s ``_partition_robots`` docstring); this test pins
    that ``graph.py`` actually delivers it, so a future change can't make an
    unmerged robot silently vanish from the operator's map.
    """
    _scenes, robots = disjoint_fleet(seed=3)
    result = _build_graph(robots).optimize()

    for robot in robots:
        component = result.component_of(robot.robot_id)
        assert (
            component is not None
        ), f"{robot.robot_id} has keyframes but no Component at all"
        assert component.robots == frozenset({robot.robot_id})


# --------------------------------------------------------------------------- #
# 4. Two robots WITH verified closures merge and recover the true transform
# --------------------------------------------------------------------------- #


def test_two_robots_merge_and_recover_relative_transform(merged_fleet) -> None:
    robots, _closures, result = merged_fleet

    assert len(result.components) == 1
    assert result.components[0].robots == frozenset({"alpha", "beta"})
    assert result.share_frame("alpha", "beta")
    assert score_components(result.components, truth_groups(robots)).is_perfect

    truth_t_world_map = {robot.robot_id: robot.t_world_map_true for robot in robots}
    errors = inter_robot_transform_error(result.t_world_map, truth_t_world_map)
    assert set(errors) == {"alpha", "beta"}
    for robot_id, error in errors.items():
        # Generous relative to the measured recovery (~0.07-0.27 m, ~0.3-0.4
        # deg on this fixture) -- tight enough to catch a real mismerge or a
        # sign error, loose enough not to be sensitive to fixture-seed noise.
        assert (
            error.translation_m < 1.0
        ), f"{robot_id}: recovered T_world_map off by {error.translation_m:.3f} m"
        assert (
            error.rotation_deg < 5.0
        ), f"{robot_id}: recovered T_world_map off by {error.rotation_deg:.3f} deg"


# --------------------------------------------------------------------------- #
# 5. Direction-inversion regression: T_dst_src fed where T_src_dst is expected
# --------------------------------------------------------------------------- #


def test_inverted_edge_direction_is_wrong_or_rejected() -> None:
    """The single most dangerous silent bug this package can have: an edge
    built as ``T_dst_src`` where ``Edge.t_src_dst`` (i.e. ``T_src_dst``) is
    expected. ``_between_factor`` uses ``edge.t_src_dst`` directly as the
    ``BetweenFactorPose3`` measurement between ``edge.src`` and ``edge.dst``
    -- get the direction backwards and the graph still optimizes cleanly, it
    just silently converges to (or, if caught, rejects) the wrong relative
    pose. No other test in this file would catch a regression here if this
    one didn't exist.

    Builds the same one-robot loop twice, differing only in whether the one
    injected closure is ``T_src_dst`` (correct) or its inverse (the bug this
    test targets), and requires the correct version to actually improve ATE
    (so the comparison is meaningful) and the inverted version to either be
    rejected outright or measurably corrupt the solution -- both are
    acceptable, since either proves direction is not silently ignored.
    """
    _scene, robots = two_robot_fleet(seed=3)
    alpha = next(robot for robot in robots if robot.robot_id == "alpha")
    src, dst = alpha.keyframes[0], alpha.keyframes[-1]
    t_src_dst_true = se3_relative(alpha.truth[src.id], alpha.truth[dst.id])
    info = _closure_information(sigma_t=0.03, sigma_r=np.deg2rad(2.0))

    def _optimize_with(t_src_dst: np.ndarray) -> tuple[Edge, OptimizedGraph]:
        edge = Edge(EdgeKind.INTRA_LOOP, src.id, dst.id, t_src_dst, info)
        pose_graph = _build_graph([alpha], [edge])
        return edge, pose_graph.optimize()

    _good_edge, good_result = _optimize_with(t_src_dst_true)
    bad_edge, bad_result = _optimize_with(se3_inverse(t_src_dst_true))

    ate_pre = compute_ate(_drifted_poses(alpha), alpha.truth)
    ate_good = compute_ate(good_result.poses, alpha.truth)
    assert (
        ate_good.translation_m.rmse < ate_pre.translation_m.rmse
    ), "positive control failed: the correctly-oriented closure should improve ATE"

    if bad_edge in bad_result.rejected_edges:
        return  # acceptable: the inverted edge was caught and never influenced the solution
    ate_bad = compute_ate(bad_result.poses, alpha.truth)
    assert ate_bad.translation_m.rmse > ate_good.translation_m.rmse * 2.0, (
        "an edge fed backwards (T_dst_src where T_src_dst was expected) was neither "
        "rejected nor did it measurably corrupt the solution -- direction may be silently ignored"
    )


# --------------------------------------------------------------------------- #
# 6. Measured scaling, with an asserted (not guessed) bound
# --------------------------------------------------------------------------- #

# Measured on this environment (Python 3.12, gtsam 4.2.2), three repeated runs,
# stable to +/-2%: an 8-robot x 100-keyframe fleet (800 keyframes, ~792
# odometry edges, 16 intra-robot loop closures with real residual to resolve --
# see _hand_built_loop_closures) optimizes in ~0.12-0.13s. Doubling to 1600
# keyframes measured ~0.34s (~2.7x), consistent with the module docstring's
# note that GNC reruns LM from scratch every outer mu-step, so growth faster
# than linear is expected. These bounds carry >8x margin over the measured
# numbers for slower/loaded hardware; they are real measurements, not guesses.
_SCALING_N_ROBOTS = 8
_SCALING_N_KEYFRAMES = 100
_SCALING_BOUND_S = 1.0
_SCALING_GROWTH_FACTOR_BOUND = 6.0


def _timed_optimize(
    robots: list[SyntheticRobot], *, rng_seed: int
) -> tuple[float, OptimizedGraph]:
    rng = np.random.default_rng(rng_seed)
    pose_graph = _build_graph(robots, _hand_built_loop_closures(robots, rng=rng))
    start = time.perf_counter()
    result = pose_graph.optimize()
    return time.perf_counter() - start, result


def test_optimize_scaling_has_a_measured_bound() -> None:
    robots = _synthetic_fleet(_SCALING_N_ROBOTS, _SCALING_N_KEYFRAMES)
    elapsed, result = _timed_optimize(robots, rng_seed=0)
    assert len(result.poses) == _SCALING_N_ROBOTS * _SCALING_N_KEYFRAMES
    assert (
        elapsed < _SCALING_BOUND_S
    ), f"optimize() took {elapsed:.3f}s, expected < {_SCALING_BOUND_S}s"


def test_optimize_scaling_growth_is_bounded() -> None:
    """Doubling the keyframe count should not blow up wall time by more than a
    generous multiple. Batch GNC+LM (the documented, deliberate limit -- see
    module docstring's Scalability section) is expected to grow faster than
    linear; growth far past this bound would mean it can no longer keep up
    with anything resembling real-time keyframe arrival even at small scale.
    """
    robots_small = _synthetic_fleet(_SCALING_N_ROBOTS, _SCALING_N_KEYFRAMES)
    robots_large = _synthetic_fleet(_SCALING_N_ROBOTS, _SCALING_N_KEYFRAMES * 2)
    elapsed_small, _ = _timed_optimize(robots_small, rng_seed=0)
    elapsed_large, _ = _timed_optimize(robots_large, rng_seed=0)
    assert elapsed_large < elapsed_small * _SCALING_GROWTH_FACTOR_BOUND, (
        f"2x keyframes took {elapsed_large / elapsed_small:.2f}x as long "
        f"(small={elapsed_small:.3f}s, large={elapsed_large:.3f}s), expected "
        f"< {_SCALING_GROWTH_FACTOR_BOUND}x"
    )


# --------------------------------------------------------------------------- #
# 7. PCM/GNC never reject structural edges (ODOMETRY, anchor priors)
# --------------------------------------------------------------------------- #


def test_structural_edges_are_never_rejectable() -> None:
    """``ODOMETRY`` edges (and, internally, each component's anchor prior --
    never exposed as an ``Edge``; see ``_build_factors``) are registered as
    GNC known-inliers and are never even part of the loop-closure candidate
    pool PCM/GNC screen (see module docstring's 'known inliers' paragraph).
    Proven by setting an impossible GNC weight threshold (> 1.0, which no GNC
    weight can ever exceed): every loop closure gets excluded from the refit,
    but the graph still solves for every keyframe and no odometry edge ever
    appears in ``rejected_edges``.
    """
    _scene, robots = two_robot_fleet(seed=3)
    alpha = next(robot for robot in robots if robot.robot_id == "alpha")
    src, dst = alpha.keyframes[0], alpha.keyframes[-1]
    loop_edge = Edge(
        EdgeKind.INTRA_LOOP,
        src.id,
        dst.id,
        se3_relative(alpha.truth[src.id], alpha.truth[dst.id]),
        _closure_information(sigma_t=0.03, sigma_r=np.deg2rad(2.0)),
    )
    all_odometry = [edge for robot in robots for edge in _odometry_edges(robot)]

    pose_graph = GtsamPoseGraph(gnc_weight_threshold=1.5)
    for robot in robots:
        for keyframe in robot.keyframes:
            pose_graph.add_keyframe(keyframe)
    for edge in all_odometry:
        pose_graph.add_edge(edge)
    pose_graph.add_edge(loop_edge)

    result = pose_graph.optimize()

    assert (
        loop_edge in result.rejected_edges
    )  # the impossible threshold rejects every closure
    assert not any(edge in result.rejected_edges for edge in all_odometry)
    total_keyframes = sum(len(robot.keyframes) for robot in robots)
    assert (
        len(result.poses) == total_keyframes
    )  # every keyframe still solved: graph stayed connected


# --------------------------------------------------------------------------- #
# Supplementary: determinism and basic input handling
# --------------------------------------------------------------------------- #


def test_optimize_is_deterministic(merged_fleet) -> None:
    """``GtsamPoseGraph`` claims (module docstring) to use no RNG anywhere, so
    the exact same input must produce the exact same output. Rebuilds the
    fixture's graph from scratch (a fresh solver instance) rather than
    calling ``optimize()`` twice on the same one, since the interesting claim
    is about the class as a whole, not about re-solving idempotency.
    """
    robots, closures, result_a = merged_fleet
    result_b = _build_graph(robots, closures).optimize()

    assert set(result_a.poses) == set(result_b.poses)
    for keyframe_id, pose_a in result_a.poses.items():
        np.testing.assert_allclose(pose_a, result_b.poses[keyframe_id], atol=1e-9)
    assert result_a.final_error == pytest.approx(result_b.final_error, abs=1e-9)
    assert result_a.components == result_b.components


def test_optimize_on_empty_graph_returns_empty_result() -> None:
    result = GtsamPoseGraph().optimize()
    assert result.poses == {}
    assert result.components == []
    assert result.rejected_edges == []


def test_edge_referencing_unknown_keyframe_raises() -> None:
    """A caller bug (an edge naming a keyframe that was never added) must fail
    loudly at ``optimize()`` rather than silently dropping the edge or
    crashing deep inside gtsam with an opaque key-not-found error.
    """
    _scene, robots = two_robot_fleet(seed=3)
    alpha = next(robot for robot in robots if robot.robot_id == "alpha")
    pose_graph = GtsamPoseGraph()
    for keyframe in alpha.keyframes:
        pose_graph.add_keyframe(keyframe)
    for edge in _odometry_edges(alpha):
        pose_graph.add_edge(edge)

    ghost = KeyframeId("nonexistent", 0)
    pose_graph.add_edge(
        Edge(
            EdgeKind.INTRA_LOOP,
            alpha.keyframes[0].id,
            ghost,
            se3_identity(),
            _closure_information(sigma_t=0.05, sigma_r=np.deg2rad(3.0)),
        )
    )
    with pytest.raises(ValueError):
        pose_graph.optimize()


# --------------------------------------------------------------------------- #
# 8. T_world_map is fitted over the trajectory, not sampled from one keyframe
# --------------------------------------------------------------------------- #


def _t_world_map_residual(
    result: OptimizedGraph, robot: SyntheticRobot, t_world_map: np.ndarray
) -> float:
    """Translation RMSE of a candidate ``T_world_map`` against the optimized
    poses it claims to explain -- the same measure ``graph.py`` selects on."""
    errors = [
        np.linalg.norm(
            (t_world_map @ kf.t_odom_base)[:3, 3] - result.poses[kf.id][:3, 3]
        )
        for kf in robot.keyframes
    ]
    return float(np.sqrt(np.mean(np.square(errors))))


def _snapshot_t_world_map(result: OptimizedGraph, robot: SyntheticRobot) -> np.ndarray:
    """What ``_t_world_map`` used to publish: read off the latest keyframe alone."""
    latest = max(robot.keyframes, key=lambda kf: kf.id.seq)
    return result.poses[latest.id] @ se3_inverse(latest.t_odom_base)


def _rebase_from(
    robot: SyntheticRobot, index: int, shift: np.ndarray
) -> SyntheticRobot:
    """Move every keyframe from ``index`` onward into a shifted source frame.

    Stands in for the robot's own SLAM node re-optimizing and moving its map
    frame mid-run, which is what really arrives in ``t_odom_base`` (see that
    field's docstring). The keyframes still describe the same physical
    trajectory; only the frame they are expressed in jumps.
    """
    moved = []
    for position, keyframe in enumerate(robot.keyframes):
        if position < index:
            moved.append(keyframe)
            continue
        moved.append(
            Keyframe(
                id=keyframe.id,
                stamp=keyframe.stamp,
                t_odom_base=shift @ keyframe.t_odom_base,
                points=keyframe.points,
                descriptor=keyframe.descriptor,
                descriptor_kind=keyframe.descriptor_kind,
            )
        )
    return replace(robot, keyframes=moved)


def test_t_world_map_beats_the_single_keyframe_snapshot_on_a_drifting_frame() -> None:
    """The regression that shipped a 6.5 m error into the operator's fleet view.

    ``t_odom_base`` carries the robot's own SLAM ``T_map_base``, and a SLAM
    node moves its map frame every time it re-optimizes. Reading
    ``T_world_map`` off the newest keyframe therefore samples wherever that
    frame happened to sit at one instant. Measured on
    ``sessions/captures/3d-run-01``, that put robot_0's published frame 6.57 m
    and 15.4 deg from the whole-trajectory fit, on a run whose joint ATE was
    0.66 m -- two numbers that cannot both be true.

    Here the frame jump is injected deliberately so the fixture has a known
    answer: the published transform must explain the WHOLE trajectory
    better than the last keyframe alone does.
    """
    _scene, robots = two_robot_fleet(seed=3)
    shift = yaw_pose(1.5, -0.8, np.deg2rad(9.0))
    drifting = [
        _rebase_from(robot, len(robot.keyframes) // 2, shift) for robot in robots
    ]

    # Odometry rebuilt from the same rebased poses is exactly self-consistent:
    # without an independent geometric observation, both the trajectory fit
    # and newest-frame snapshot have zero residual and the intended regression
    # is unobservable. Real closures preserve the physical trajectory while
    # the saved map-frame poses jump, which is the production failure this
    # fixture is meant to exercise.
    result = _build_graph(drifting, _find_real_closures(drifting)).optimize()

    for robot in drifting:
        published = result.t_world_map[robot.robot_id]
        snapshot = _snapshot_t_world_map(result, robot)
        fitted_rmse = _t_world_map_residual(result, robot, published)
        snapshot_rmse = _t_world_map_residual(result, robot, snapshot)
        assert fitted_rmse < snapshot_rmse * 0.75, (
            f"{robot.robot_id}: published T_world_map explains the trajectory with "
            f"rmse {fitted_rmse:.3f} m vs {snapshot_rmse:.3f} m for the single-keyframe "
            "snapshot -- the fit is not being used"
        )


def test_t_world_map_never_loses_to_the_snapshot_it_replaced() -> None:
    """The fit is accepted only when it actually explains the poses better.

    A trajectory-wide fit is the right default but not unconditionally
    better: a robot that has barely moved, or driven straight down a
    corridor, leaves the fit free to rotate about the travel axis. Selecting
    on residual means this change can never be a regression for any fixture,
    which is what makes it safe to apply to every robot unconditionally.
    """
    _scene, robots = two_robot_fleet(seed=3)
    result = _build_graph(robots, _find_real_closures(robots)).optimize()

    for robot in robots:
        published = _t_world_map_residual(
            result, robot, result.t_world_map[robot.robot_id]
        )
        snapshot = _t_world_map_residual(
            result, robot, _snapshot_t_world_map(result, robot)
        )
        assert published <= snapshot + 1e-9, (
            f"{robot.robot_id}: published frame is worse ({published:.4f} m) than the "
            f"single-keyframe snapshot ({snapshot:.4f} m) it replaced"
        )


def test_t_world_map_falls_back_for_a_robot_with_too_few_keyframes() -> None:
    """Two keyframes fit a rigid transform exactly and prove nothing.

    A 2-point Kabsch fit always has zero residual while leaving rotation
    about the line between the points completely free, so the residual guard
    is vacuous there -- the keyframe-count floor is what stops an arbitrary
    rotation being published as a robot's frame.
    """
    _scene, robots = two_robot_fleet(seed=3)
    short = replace(
        robots[0],
        robot_id="stub",
        keyframes=[
            Keyframe(
                id=KeyframeId("stub", kf.id.seq),
                stamp=kf.stamp,
                t_odom_base=kf.t_odom_base,
                points=kf.points,
            )
            for kf in robots[0].keyframes[:2]
        ],
    )

    result = _build_graph([short], []).optimize()

    published = result.t_world_map["stub"]
    snapshot = _snapshot_t_world_map(result, short)
    assert np.allclose(published, snapshot), (
        "a 2-keyframe robot must fall back to the single-keyframe read rather than "
        "publish an under-determined fit"
    )


def test_pose_priors_keep_a_working_slam_trajectory_from_folding() -> None:
    """Onboard SLAM poses are a suggestion: a loop may refine them, not fan them.

    A 5 m lie between the first and last keyframe of one robot is the kind of
    confident-wrong closure that previously smeared occupancy into double
    walls. Pose priors, registered as GNC known-inliers, cap that motion.
    """
    _scene, robots = two_robot_fleet(seed=4)
    alpha = next(robot for robot in robots if robot.robot_id == "alpha")
    lie = np.eye(4, dtype=np.float64)
    lie[0, 3] = 5.0
    loop = Edge(
        EdgeKind.INTRA_LOOP,
        alpha.keyframes[0].id,
        alpha.keyframes[-1].id,
        lie,
        _closure_information(sigma_t=0.02, sigma_r=np.deg2rad(1.0)),
    )

    def max_move(result: OptimizedGraph) -> float:
        trajectory = alpha.keyframes[0].id.trajectory
        frame = result.t_world_trajectory[trajectory]
        return max(
            float(
                np.linalg.norm(
                    result.poses[kf.id][:3, 3] - (frame @ kf.t_odom_base)[:3, 3]
                )
            )
            for kf in alpha.keyframes
        )

    free = _build_graph([alpha], [loop]).optimize()
    pinned = _build_graph(
        [alpha],
        [loop],
        pose_prior_sigmas=np.array([0.05, 0.05, 0.05, 0.10, 0.10, 0.15]),
    ).optimize()
    assert max_move(pinned) < 0.5
    assert max_move(pinned) <= max_move(free) + 1e-6
