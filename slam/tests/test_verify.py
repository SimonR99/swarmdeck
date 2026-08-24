"""Tests for geometric loop-closure verification.

Every test here exercises real GICP registration against the shared
synthetic fixture (``tests/synthetic.py``) -- nothing is mocked. That
matters more here than almost anywhere else in the package: this module's
entire job is deciding, from real geometry, whether a proposed loop closure
is trustworthy, and a mocked GICP result cannot tell you whether the
decision logic actually works against a real (if synthetic) point cloud.

Per the project brief, the single most important test in this file is
``test_disjoint_building_pairs_are_rejected``: accepting a false loop
closure destroys a map, so false-positive avoidance is checked far more
thoroughly here than any other property.
"""

from __future__ import annotations

import asyncio
import math

import numpy as np
import pytest
import small_gicp

from swarmdeck_slam.types import (
    EdgeKind,
    Keyframe,
    KeyframeId,
    se3_distance,
    se3_identity,
    se3_inverse,
    se3_relative,
)
from swarmdeck_slam.verify import (
    Candidate,
    VerifyConfig,
    verify_candidate,
    verify_candidates,
)

from synthetic import SyntheticRobot, disjoint_fleet, two_robot_fleet

# Generous relative to measured recovery error on the synthetic fixture
# (1.4-2.1 cm translation, 0.10-0.16 deg rotation for genuine matches) --
# tight enough to catch a broken registration or a bad information matrix,
# loose enough not to be sensitive to fixture-seed noise.
_TRANSLATION_TOLERANCE_M = 0.08
_ROTATION_TOLERANCE_RAD = math.radians(1.5)


def _true_relative_pose(source_robot: SyntheticRobot, source_kf: Keyframe,
                         target_robot: SyntheticRobot, target_kf: Keyframe) -> np.ndarray:
    """Exact ``T_source_target`` from ground truth, in this module's own convention."""
    return se3_relative(source_robot.truth[source_kf.id], target_robot.truth[target_kf.id])


def _exact_yaw_prior(t_src_dst_truth: np.ndarray) -> float:
    """The yaw ``verify_candidate`` actually wants, from a ground-truth ``t_src_dst``.

    ``yaw_prior`` is documented as the yaw of ``T_target_source`` in
    small_gicp's own naming -- the *opposite* direction from this module's
    ``t_src_dst = T_source_target`` return value -- so it is the yaw of
    ``se3_inverse(t_src_dst_truth)``, not of ``t_src_dst_truth`` itself.
    Getting this backwards in a test would silently feed GICP a yaw prior
    off by roughly twice the true heading difference, which is exactly the
    kind of direction mixup this file exists to catch elsewhere -- so it is
    centralised here once rather than risked at every call site.
    """
    rotation = se3_inverse(t_src_dst_truth)[:3, :3]
    return math.atan2(rotation[1, 0], rotation[0, 0])


def _closest_pair(
    a: SyntheticRobot, b: SyntheticRobot, *, min_index_gap: int = 0
) -> tuple[Keyframe, Keyframe, float]:
    """The (a_keyframe, b_keyframe) pair whose ground-truth positions are nearest.

    ``min_index_gap`` excludes trivially-adjacent keyframes from the same
    robot, so an "intra-robot loop closure" test can't accidentally degrade
    into "consecutive odometry edge".
    """
    best: tuple[float, Keyframe, Keyframe] | None = None
    for i, kf_a in enumerate(a.keyframes):
        for j, kf_b in enumerate(b.keyframes):
            if a is b and abs(i - j) < min_index_gap:
                continue
            distance = float(np.linalg.norm(a.truth[kf_a.id][:3, 3] - b.truth[kf_b.id][:3, 3]))
            if best is None or distance < best[0]:
                best = (distance, kf_a, kf_b)
    assert best is not None
    distance, kf_a, kf_b = best
    return kf_a, kf_b, distance


def _make_wall_keyframe(
    robot_id: str, seq: int, rng: np.random.Generator, *,
    n: int = 600, length: float = 20.0, height: float = 2.4, noise: float = 0.01,
) -> Keyframe:
    """A single flat wall scan: the textbook degenerate GICP geometry.

    Well constrained perpendicular to the wall; essentially unconstrained
    sliding along it or shifting its vertical extent -- like a featureless
    corridor, but built directly rather than mined out of the shared
    building fixture, since we need to *guarantee* the degeneracy.
    """
    x = rng.uniform(-length / 2.0, length / 2.0, n)
    z = rng.uniform(0.0, height, n)
    y = np.zeros(n)
    points = np.stack([x, y, z], axis=1) + rng.normal(scale=noise, size=(n, 3))
    return Keyframe(
        id=KeyframeId(robot_id, seq), stamp=float(seq),
        t_odom_base=se3_identity(), points=points.astype(np.float32),
    )


# --------------------------------------------------------------------------- #
# H block ordering -- permanent regression guard
# --------------------------------------------------------------------------- #

def test_hessian_block_ordering_is_rotation_first() -> None:
    """small_gicp's ``.H`` is rotation-first, matching gtsam's TANGENT_ORDER.

    Not documented anywhere in small_gicp, so this pins the behaviour down
    directly rather than trusting the docstring in verify.py to stay true
    forever. Uses a flat disk of points centred at the origin, which has a
    distinctive, ordering-independent information fingerprint:

    * tilting the disk (rotating about an in-plane axis) moves its edge
      points a lot -- large lever arm -- so those two rotation directions
      carry strong information;
    * spinning the disk about its own normal moves no point at all for an
      isotropic disk -- weak information;
    * sliding the disk sideways (translating within its own plane) barely
      changes the fit -- weak information;
    * lifting the disk off its plane (translating along the normal)
      changes every point's fit -- strong information.

    So whichever 3x3 diagonal block has the "2 strong + 1 weak" pattern is
    rotation, and whichever has "1 strong + 2 weak" is translation. This
    lets the test assert the ordering from the *shape* of the result, with
    no dependency on which literal indices small_gicp happens to use.
    """
    rng = np.random.default_rng(7)

    def make_disk(n: int = 4000, radius: float = 8.0, z_noise: float = 0.01) -> np.ndarray:
        r = radius * np.sqrt(rng.uniform(0.0, 1.0, n))
        theta = rng.uniform(0.0, 2 * np.pi, n)
        return np.stack(
            [r * np.cos(theta), r * np.sin(theta), rng.normal(scale=z_noise, size=n)], axis=1
        )

    target = make_disk()
    source = make_disk()  # independent sample of the same disk, centred at the origin

    result = small_gicp.align(
        target, source, init_T_target_source=se3_identity(), registration_type="GICP",
        downsampling_resolution=0.3, max_correspondence_distance=1.0,
        num_threads=1, max_iterations=50,
    )
    assert result.converged

    hessian = 0.5 * (result.H + result.H.T)
    block_a = np.linalg.eigvalsh(hessian[0:3, 0:3])  # candidate blocks, order unknown yet
    block_b = np.linalg.eigvalsh(hessian[3:6, 3:6])

    def is_two_strong_one_weak(eigenvalues: np.ndarray) -> bool:
        smallest, middle, largest = np.sort(eigenvalues)
        # A wide, unambiguous margin: strong directions many orders of
        # magnitude above the weak one, middle solidly with the strong pair.
        return middle > 50 * smallest and largest > 50 * smallest

    def is_one_strong_two_weak(eigenvalues: np.ndarray) -> bool:
        smallest, middle, largest = np.sort(eigenvalues)
        return largest > 50 * middle and largest > 50 * smallest

    rotation_like_a, translation_like_a = is_two_strong_one_weak(block_a), is_one_strong_two_weak(block_a)
    rotation_like_b, translation_like_b = is_two_strong_one_weak(block_b), is_one_strong_two_weak(block_b)

    # Exactly one block must look like rotation and the other like
    # translation -- if both or neither match, the disk fixture itself
    # isn't discriminating and the test should fail loudly, not guess.
    assert rotation_like_a != rotation_like_b
    assert translation_like_a != translation_like_b
    assert rotation_like_a == translation_like_b

    # The actual pin: rotation is first (indices 0:3), translation second
    # (indices 3:6) -- gtsam's own TANGENT_ORDER, so Edge.information from
    # verify.py can be handed to a gtsam solver with no reordering.
    assert rotation_like_a, (
        "small_gicp's H is no longer rotation-first -- verify.py's block "
        "slicing (and this docstring's claim) need to be revisited"
    )


# --------------------------------------------------------------------------- #
# True pairs: accepted, and accurate
# --------------------------------------------------------------------------- #

def test_true_intra_robot_pair_is_accepted_and_accurate() -> None:
    """A genuine same-robot revisit is accepted with an accurate relative pose."""
    _, robots = two_robot_fleet(seed=0)
    alpha = next(r for r in robots if r.robot_id == "alpha")
    # alpha's path is a closed rectangle -- keyframe 0 and the last keyframe
    # both sit at its start, a genuine revisit far apart in trajectory time.
    source_kf, target_kf, distance = _closest_pair(alpha, alpha, min_index_gap=5)
    assert distance < 0.5  # sanity: this really is the same physical spot

    truth = _true_relative_pose(alpha, source_kf, alpha, target_kf)
    yaw_prior = _exact_yaw_prior(truth) + math.radians(-11.0)  # coarse, not exact

    edge = verify_candidate(source_kf, target_kf, yaw_prior)

    assert edge is not None
    assert edge.kind is EdgeKind.INTRA_LOOP
    assert not edge.is_inter_robot
    assert edge.inlier_ratio > 0.9
    assert 0.0 < edge.fitness <= 1.0

    translation_error, rotation_error = se3_distance(edge.t_src_dst, truth)
    assert translation_error < _TRANSLATION_TOLERANCE_M
    assert rotation_error < _ROTATION_TOLERANCE_RAD


def test_true_inter_robot_pair_is_accepted_and_accurate() -> None:
    """Cross-robot verification: alpha and beta observing the same spot."""
    _, robots = two_robot_fleet(seed=0)
    alpha = next(r for r in robots if r.robot_id == "alpha")
    beta = next(r for r in robots if r.robot_id == "beta")
    source_kf, target_kf, distance = _closest_pair(alpha, beta)
    assert distance < 1.5  # sanity: the fixture's overlap region was found

    truth = _true_relative_pose(alpha, source_kf, beta, target_kf)
    yaw_prior = _exact_yaw_prior(truth) + math.radians(8.0)

    edge = verify_candidate(source_kf, target_kf, yaw_prior)

    assert edge is not None
    assert edge.kind is EdgeKind.INTER_LOOP
    assert edge.is_inter_robot
    assert edge.src.robot_id == "alpha"
    assert edge.dst.robot_id == "beta"
    assert edge.inlier_ratio > 0.9

    translation_error, rotation_error = se3_distance(edge.t_src_dst, truth)
    assert translation_error < _TRANSLATION_TOLERANCE_M
    assert rotation_error < _ROTATION_TOLERANCE_RAD


def test_transform_direction_is_not_inverted() -> None:
    """A src/dst (equivalently target/source) mixup must fail this test.

    Builds the *deliberately wrong* transform a ``T_target_source`` vs
    ``T_source_target`` swap would silently produce, and asserts the real
    edge is close to ground truth while the swapped one is not. If
    verify.py's inversion step is ever dropped or doubled, this fails --
    silently-inverted transforms are otherwise undetectable from the
    edge's shape alone, per types.py's own warning about this defect.
    """
    _, robots = two_robot_fleet(seed=0)
    alpha = next(r for r in robots if r.robot_id == "alpha")
    beta = next(r for r in robots if r.robot_id == "beta")
    source_kf, target_kf, _distance = _closest_pair(alpha, beta)

    truth_src_dst = _true_relative_pose(alpha, source_kf, beta, target_kf)
    yaw_prior = _exact_yaw_prior(truth_src_dst)

    edge = verify_candidate(source_kf, target_kf, yaw_prior)
    assert edge is not None

    correct_translation_error, correct_rotation_error = se3_distance(edge.t_src_dst, truth_src_dst)
    assert correct_translation_error < _TRANSLATION_TOLERANCE_M
    assert correct_rotation_error < _ROTATION_TOLERANCE_RAD

    inverted = se3_inverse(edge.t_src_dst)
    wrong_translation_error, wrong_rotation_error = se3_distance(inverted, truth_src_dst)
    # The source/target pair here is asymmetric (different positions and
    # headings), so the inverted transform is nowhere near ground truth --
    # if it were, this whole test would be vacuous.
    assert wrong_translation_error > 10 * _TRANSLATION_TOLERANCE_M
    assert wrong_rotation_error > 10 * _ROTATION_TOLERANCE_RAD


# --------------------------------------------------------------------------- #
# False pairs: rejected. The most important property in this file.
# --------------------------------------------------------------------------- #

def test_disjoint_building_pairs_are_rejected() -> None:
    """Keyframes from two different buildings must never produce an edge.

    ``disjoint_fleet`` guarantees, by construction, that any inter-robot
    match here is a false positive -- there is no correct transform between
    them at all. Tries many keyframe pairs and many yaw priors (a real
    place-recognition stage could propose any heading) so this is not a
    single-lucky-draw result: acceptance of even one of these would be a
    map-destroying bug.
    """
    _, robots = disjoint_fleet(seed=0)
    alpha = next(r for r in robots if r.robot_id == "alpha")
    beta = next(r for r in robots if r.robot_id == "beta")
    rng = np.random.default_rng(5)

    accepted = []
    for _ in range(20):
        source_kf = alpha.keyframes[int(rng.integers(0, len(alpha.keyframes)))]
        target_kf = beta.keyframes[int(rng.integers(0, len(beta.keyframes)))]
        yaw_prior = float(rng.uniform(-math.pi, math.pi))
        edge = verify_candidate(source_kf, target_kf, yaw_prior)
        if edge is not None:
            accepted.append((source_kf.id, target_kf.id, yaw_prior, edge))

    assert accepted == []


def test_disjoint_building_pair_with_generous_thresholds_still_rejected() -> None:
    """Even a permissive config must not be fooled by a favourable false pair.

    Uses the highest-scoring false match found while calibrating this
    module's default thresholds (ratio ~0.49, the best of dozens of random
    disjoint-building draws) with every gate loosened close to that
    measurement, to check the rejection isn't only an artefact of
    conservative defaults happening to clear a wide margin.
    """
    _, robots = disjoint_fleet(seed=0)
    alpha = next(r for r in robots if r.robot_id == "alpha")
    beta = next(r for r in robots if r.robot_id == "beta")
    rng = np.random.default_rng(5)

    permissive = VerifyConfig(
        min_inliers=10,
        min_inlier_ratio=0.55,  # still above every measured false-pair ratio (<=0.49)
        max_mean_error=5.0,
        max_translation_m=100.0,
        max_yaw_deviation_from_prior_rad=math.pi,
        min_rotation_information=1.0,
        min_translation_information=1.0,
    )

    accepted = 0
    for _ in range(20):
        source_kf = alpha.keyframes[int(rng.integers(0, len(alpha.keyframes)))]
        target_kf = beta.keyframes[int(rng.integers(0, len(beta.keyframes)))]
        yaw_prior = float(rng.uniform(-math.pi, math.pi))
        if verify_candidate(source_kf, target_kf, yaw_prior, permissive) is not None:
            accepted += 1

    assert accepted == 0


# --------------------------------------------------------------------------- #
# Degenerate geometry: rejected, or honestly uncertain -- never confidently wrong
# --------------------------------------------------------------------------- #

def test_degenerate_single_wall_is_rejected_or_inflated() -> None:
    """A featureless single wall must not produce a falsely confident edge.

    Two scans of the same flat wall, offset by a slide along it -- the
    classic degenerate case (well constrained across the wall, essentially
    unconstrained sliding along it or shifting vertically). Either outcome
    in the module's contract is acceptable here: reject outright, or return
    an edge whose information matrix is honest about which directions are
    unconstrained. What is never acceptable is a confident wrong answer, so
    if an edge comes back this test inspects its information matrix
    directly rather than trusting the edge's mere existence.
    """
    rng = np.random.default_rng(11)
    source_kf = _make_wall_keyframe("wallbot", 0, rng)
    target_kf = _make_wall_keyframe("wallbot", 1, rng)

    edge = verify_candidate(source_kf, target_kf, yaw_prior=0.0)
    if edge is None:
        return  # rejecting a wholly degenerate match is a valid outcome

    translation_info = np.diag(edge.information)[3:6]  # tx, ty, tz
    strong = translation_info.max()
    weak = translation_info.min()
    # The wall's normal direction (well constrained) must dominate the
    # along-wall / vertical directions (unconstrained) by a wide margin --
    # anything close to isotropic here would mean the degeneracy was not
    # actually detected and floored.
    assert strong > 20 * weak

    # And the whole matrix must still be a legitimate information matrix.
    eigenvalues = np.linalg.eigvalsh(edge.information)
    assert eigenvalues.min() > 0.0


def test_too_few_points_is_rejected() -> None:
    """A keyframe with too little geometry to register is rejected before GICP runs."""
    rng = np.random.default_rng(0)
    tiny_points = rng.normal(size=(10, 3)).astype(np.float32)
    sparse_kf = Keyframe(
        id=KeyframeId("sparsebot", 0), stamp=0.0, t_odom_base=se3_identity(), points=tiny_points
    )
    _, robots = two_robot_fleet(seed=0)
    normal_kf = robots[0].keyframes[0]

    assert verify_candidate(sparse_kf, normal_kf, yaw_prior=0.0) is None
    assert verify_candidate(normal_kf, sparse_kf, yaw_prior=0.0) is None


# --------------------------------------------------------------------------- #
# Batch entry point
# --------------------------------------------------------------------------- #

def test_verify_candidates_batch_sequential_and_threaded_agree() -> None:
    """The batch entry point drops rejects and keeps accepts, identically either way.

    Mixes a genuine true pair with several disjoint-building false pairs so
    the batch has to do real filtering, not just pass everything through.
    Both the sequential (``max_workers=1``) and threaded (``max_workers=2``)
    paths must produce the same accepted edges -- ``VerifyConfig``'s default
    ``num_threads=1`` keeps GICP itself deterministic, so this is a
    meaningful equality check, not a coincidence of tolerant comparison.
    """
    _, true_robots = two_robot_fleet(seed=0)
    alpha_true = next(r for r in true_robots if r.robot_id == "alpha")
    beta_true = next(r for r in true_robots if r.robot_id == "beta")
    source_kf, target_kf, _ = _closest_pair(alpha_true, beta_true)
    truth = _true_relative_pose(alpha_true, source_kf, beta_true, target_kf)
    good_yaw_prior = _exact_yaw_prior(truth)

    _, false_robots = disjoint_fleet(seed=0)
    alpha_false = next(r for r in false_robots if r.robot_id == "alpha")
    beta_false = next(r for r in false_robots if r.robot_id == "beta")

    candidates = [Candidate(source_kf, target_kf, good_yaw_prior)]
    for i in range(4):
        candidates.append(
            Candidate(alpha_false.keyframes[i * 5], beta_false.keyframes[i * 3], 0.4 * i)
        )

    sequential = verify_candidates(candidates, max_workers=1)
    threaded = verify_candidates(candidates, max_workers=3)

    assert len(sequential) == 1
    assert sequential[0].src == source_kf.id
    assert sequential[0].dst == target_kf.id

    assert len(threaded) == len(sequential)
    np.testing.assert_array_equal(threaded[0].t_src_dst, sequential[0].t_src_dst)
    np.testing.assert_array_equal(threaded[0].information, sequential[0].information)


def test_verify_candidates_rejects_inside_a_running_event_loop() -> None:
    """The batch entry point must refuse to run inside an asyncio event loop.

    It performs blocking, CPU-bound GICP work with no await points; doing
    that on a running loop would silently stall every other coroutine on it
    for the whole batch. Failing loudly is the only safe behaviour.
    """

    async def _call_from_event_loop() -> None:
        verify_candidates([])

    with pytest.raises(RuntimeError, match="event loop"):
        asyncio.run(_call_from_event_loop())

    # Outside a running loop (this test function itself is sync), it must
    # work fine -- the guard should only fire when it needs to.
    assert verify_candidates([]) == []
