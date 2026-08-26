"""Geometric verification of loop-closure candidates.

Place recognition proposes that two keyframes -- possibly from different
robots -- observe the same physical place, plus a coarse yaw estimate. This
module is the gate between that proposal and the pose graph: it runs GICP
scan registration to test the proposal against actual geometry, and either
produces a validated relative-pose :class:`~swarmdeck_slam.types.Edge` or
returns ``None``.

This is the safety-critical stage of the whole system. A wrong constraint
that gets past this gate does not degrade the map -- it *drags an entire
robot's trajectory to the wrong place*, silently, because the optimizer has
no way to tell a confident wrong edge from a confident right one. Every
threshold in :class:`VerifyConfig` is therefore chosen, and documented, to
fail toward rejection: it is far cheaper to miss a real loop closure (the
graph stays a little more drifty) than to accept a wrong one (the graph
becomes actively false).

Frame contract
--------------
``verify_candidate(source, target, ...)`` returns an edge with
``edge.src = source.id``, ``edge.dst = target.id``, and
``edge.t_src_dst = T_source_target`` -- the transform that maps points
expressed in ``target``'s base frame into ``source``'s base frame, per the
package-wide ``T_a_b`` convention (see ``types.py``). ``yaw_prior`` is the
coarse estimate, from place recognition, of the yaw component of
``T_target_source`` -- i.e. small_gicp's own naming, *not* this function's
return convention. This asymmetry (function args are source-then-target,
but the GICP call and its initial guess are framed the other way around) is
exactly the kind of thing that produces a silently-inverted transform, which
is why it is spelled out here and guarded by a dedicated regression test.
"""

from __future__ import annotations

import asyncio
import math
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from typing import Iterable

import numpy as np
import small_gicp

from swarmdeck_slam.types import (
    Edge,
    EdgeKind,
    Keyframe,
    se3_distance,
    se3_identity,
    se3_inverse,
)

# small_gicp's `.H` is an *unscaled* Gauss-Newton Hessian, not a calibrated
# information matrix -- see VerifyConfig.info_scale below for what that
# means for the numbers this module hands to gtsam.
#
# Its 6x6 block ordering is not documented anywhere in small_gicp, so it was
# determined empirically (see tests/test_verify.py::test_hessian_block_ordering)
# by two independent methods that agreed: (1) finite-differencing the true
# GICP cost against known perturbation conventions, and (2) registering a
# flat disk of points, whose rotation/translation information split has a
# distinctive, easy-to-recognize shape (2 strong + 1 weak rotation entries,
# 1 strong + 2 weak translation entries) regardless of ordering. Both landed
# on: **rotation first**, i.e. ``[rx, ry, rz, tx, ty, tz]`` -- the same
# TANGENT_ORDER gtsam and this package's Edge.information already use. No
# reordering is needed between small_gicp's H and a gtsam-ready information
# matrix; only rescaling.
_ROTATION_SLICE = slice(0, 3)
_TRANSLATION_SLICE = slice(3, 6)


@dataclass(frozen=True, slots=True)
class VerifyConfig:
    """Every knob geometric verification uses, with the reasoning for its default.

    Defaults were calibrated empirically against ``tests/synthetic.py``'s
    ``two_robot_fleet`` (true matches) and ``disjoint_fleet`` (matches that
    must never be accepted, by construction). See the docstring of each
    field for the measured numbers behind it. Recalibrate against real
    sensor data before trusting these on hardware -- the synthetic fixture's
    noise model is not a substitute for that, only a floor.
    """

    # -- GICP registration parameters -----------------------------------
    downsampling_resolution: float = 0.15
    """Voxel size (m) GICP downsamples both clouds to before registering.
    Well above the synthetic fixture's sub-mm scene jitter and the ~1cm
    sensor noise it injects, so downsampling removes redundant points
    rather than real structure; well below the building's ~1m feature
    spacing, so distinct walls don't collapse into one voxel."""

    max_correspondence_distance: float = 1.0
    """Max distance (m) for a point pair to count as a correspondence.
    Sets GICP's basin of convergence around the initial guess: measured
    empirically to tolerate up to ~30 deg of yaw-prior error on a genuine
    match at this value (see verify.py's calibration notes); much smaller
    and ordinary place-recognition yaw noise stops GICP from converging at
    all, which is the "GICP will not converge without a decent initial
    guess" problem this module exists to route around."""

    max_iterations: int = 50
    """GICP iteration cap. Generous relative to the ~2-40 iterations
    observed to convergence on the synthetic fixture; a run that still
    hasn't converged by here is treated as non-convergence, not given more
    budget, because more iterations rarely rescue a bad initial guess."""

    num_threads: int = 1
    """Threads small_gicp uses *inside one registration*. Defaults to 1
    because small_gicp's own docs warn that multi-threaded downsampling has
    up to ~10% run-to-run variation in point count -- which would make the
    accept/reject boundary flaky. Raise this in a production deployment
    that doesn't need bit-identical results; keep it at 1 for tests, audits,
    and anywhere reproducibility matters more than latency. See
    :func:`verify_candidates` for verifying many candidates in parallel
    instead, which does not have this determinism cost."""

    # -- Hard gates: reject the candidate outright -----------------------
    min_points: int = 50
    """Minimum raw points required in *each* cloud before attempting
    registration. GICP's covariance/normal estimation uses 10-neighbour
    KNN internally; a cloud with only a handful of points can't support
    that regardless of what the rest of the pipeline says, and calling
    into small_gicp with too few points risks it failing outright rather
    than producing a meaningful non-convergence."""

    min_inliers: int = 100
    """Absolute floor on `result.num_inliers`, independent of the ratio
    below. A tiny cloud can hit a high inlier *ratio* by chance; this
    catches that case. Measured true matches on the synthetic fixture had
    220-704 inliers; this sits comfortably below the weakest genuine match
    while still ruling out matches supported by a statistically
    meaningless handful of points."""

    min_inlier_ratio: float = 0.65
    """`num_inliers / downsampled_source_points`, in [0, 1]. This is the
    single strongest true/false discriminator measured: genuine same-place
    matches (two_robot_fleet, both intra- and inter-robot) scored 0.94-1.0;
    matches between genuinely different buildings (disjoint_fleet) never
    exceeded 0.49 across dozens of random pairs and yaw priors. 0.65 sits
    with wide margin above the worst observed false positive and below the
    best observed true positive; the gap on the true side is deliberate --
    per the project's bias-toward-rejecting mandate, a real but
    poorly-overlapping match is exactly the case that should be dropped
    rather than risk it."""

    max_mean_error: float = 0.15
    """`result.error / num_inliers`: GICP's per-point Mahalanobis residual,
    averaged. Not directly convertible to metres (GICP's covariance
    weighting divides by the *surface-normal-direction* covariance, which
    is intentionally tiny, so this number is inflated relative to a plain
    Euclidean RMSE). Used only as a secondary, defense-in-depth gate: true
    matches that already pass the inlier-ratio gate measured 0.045-0.12;
    this threshold sits just above that band. It exists because ratio and
    residual don't perfectly co-vary -- a large, sparse, coincidentally
    aligned false match can occasionally reach a passable ratio while
    still fitting poorly."""

    max_translation_m: float = 15.0
    """Reject if the recovered `t_src_dst` translation exceeds this. A
    same-place proposal that GICP "resolves" to tens of metres apart did
    not refine the proposal -- it walked to an unrelated local minimum.
    Set generously above the few-metre offsets seen in true matches on the
    synthetic fixture, to allow for legitimate larger loop-closure search
    windows without allowing an obviously-wrong minimum through."""

    max_yaw_deviation_from_prior_rad: float = math.radians(35.0)
    """Reject if the registered yaw differs from `yaw_prior` by more than
    this. GICP is supposed to *refine* the coarse prior, not discover an
    unrelated rotation. Measured basin of convergence on a genuine match
    held exactly through +/-30 deg of prior error and broke by 45 deg
    (jumped to a different, wrong minimum); 35 deg keeps margin on the
    side that must never be crossed."""

    # -- Degeneracy handling: reject or inflate uncertainty --------------
    min_rotation_information: float = 100.0
    """Absolute floor (raw Hessian units) on the weakest eigenvalue of the
    rotation block, checked *before* any conditioning-based inflation.
    Below this there isn't a well-constrained rotation to inflate --
    reject outright rather than manufacture a covariance for a direction
    with essentially zero information. Set well under every non-degenerate
    case measured (>=650) and under the deliberately-degenerate single-wall
    fixture (~10800), so it only fires on near-total absence of
    constraint (e.g. near-collinear points), which the ratio/error gates
    above may not otherwise catch."""

    min_translation_information: float = 50.0
    """Translation-block counterpart to `min_rotation_information`, same
    role and same reasoning. Measured non-degenerate minimum: ~300; the
    single-wall degenerate fixture's weakest (along-the-wall) direction:
    ~300 as well, and its own condition-number gate (below) is what catches
    that case -- this floor is the backstop under both."""

    max_condition_number: float = 250.0
    """Per-block (rotation-only and translation-only -- never mixed, since
    they're in different units) eigenvalue ratio. Rotation and translation
    are compared separately and never against each other: rotation
    eigenvalues are in rad^-2 and translation in m^-2, and their raw
    magnitudes differ by orders of magnitude purely from lever-arm scale,
    not from any difference in how well-determined they are -- comparing
    them directly would flag well-conditioned matches as degenerate for no
    real reason. A synthetic single-wall / corridor-slide fixture (the
    textbook degenerate case: well constrained across the wall, essentially
    unconstrained sliding along it) measured condition numbers of
    ~730-800 in exactly this per-block sense; non-degenerate true matches
    measured 40-460. 250 sits between them. Exceeding it does not reject
    the candidate -- it floors the weak eigenvalue(s) in that block up to
    `max_eig / max_condition_number` before the information matrix is
    built, which is a smaller, more honest correction than discarding an
    otherwise-good match outright: the resulting edge still holds full
    confidence where GICP is actually confident and admits uncertainty
    exactly where the geometry doesn't constrain the transform, letting the
    pose-graph optimizer down-weight it instead of trusting a
    confident-looking number in an unconstrained direction. Known
    limitation: a degenerate direction that mixes rotation and translation
    (true along-corridor degeneracy is a coupled yaw+slide direction, not
    purely one block or the other) is only partially captured by this
    per-block treatment; a full non-dimensionalized 6x6 eigen-treatment
    would be needed to do this exactly, which was judged not worth the
    complexity here given how much margin the ratio/error gates already
    provide."""

    # -- Information-matrix scaling ---------------------------------------
    fitness_error_scale: float = 0.3
    """Denominator in `fitness = 1 / (1 + mean_error / fitness_error_scale)`.
    Chosen so that true matches passing the gates above (mean_error
    0.045-0.12) map to a fitness of roughly 0.75-0.9, and the
    `max_mean_error` cutoff (0.15) maps to about 0.67 -- a smooth,
    monotonically decreasing score in (0, 1] rather than a second hard
    threshold."""

    info_scale: float = 1.0
    """Final multiplier on the information matrix, applied after
    per-inlier normalization (see `_build_information`) and the fitness
    scaling above. Starts at 1.0 deliberately: per-inlier normalization
    already removes the "more points = more confidence" artefact in a raw
    Hessian, and fabricating an additional deflation factor without a
    measured reason to would just be a second unjustified magic number.
    This exists as a calibration knob for whoever owns the deployed
    system once real sensor noise statistics are known -- e.g. if GICP on
    real hardware is empirically found to be overconfident relative to
    achieved trajectory accuracy, turn this down."""

    information: str = "hessian"
    """How a surviving match's information matrix is built.

    ``hessian`` (the default, and what the live back-end runs -- see
    ``backend.PRODUCTION_VERIFY``) scales GICP's Gauss-Newton Hessian.

    ``isotropic`` replaces the Hessian, after the degeneracy gates have
    already passed, with ``I_6 * isotropic_scale * fitness``.

    The default was ``isotropic`` until 2026-08-25, on the strength of the
    synthetic fixture, where it improves ATE and ``hessian`` does not. Real
    captured data reverses that: replaying ``sessions/captures/3d-run-01``
    (``tools/replay.py --ablate isotropic hessian``) measures ``hessian``
    better on every scope. ``backend.PRODUCTION_VERIFY`` carries the table.
    Do not revert on fixture evidence alone -- the fixture is planar and
    non-repetitive, so it never exercises the degenerate corridor-slide
    geometry the conditioned Hessian exists to express, and it will keep
    preferring isotropic for that reason.

    Either way the synthetic fixture's ATE still degrades under optimization
    (``test_optimized_poses_beat_raw_odometry``, a strict xfail): GICP's
    Hessian arrives with roughly a 30:1 rotation-to-translation ratio, which
    over-constrains orientation, and no overall rescaling recovers it. That
    needs a real sensor noise model, not a tuned constant.
    """

    isotropic_scale: float = 400.0
    """Diagonal weight used when ``information='isotropic'``. Matches the
    odometry information the live back-end and the integration tests use, so
    a loop closure and an odometry hop speak in the same units."""


def _yaw_of(rotation: np.ndarray) -> float:
    """Yaw (rotation about z) of a 3x3 rotation matrix, via its 2D block.

    Not reusing a types.py helper because there isn't one for a single-axis
    component in isolation -- `se3_distance` returns the full geodesic angle
    across all three axes, which is what magnitude gating wants, but mixing
    roll/pitch into a yaw-deviation-from-prior check would reject good
    matches for having (harmless, expected) small out-of-plane wobble.
    """
    return float(np.arctan2(rotation[1, 0], rotation[0, 0]))


def _wrap_angle(angle: float) -> float:
    """Wrap to (-pi, pi]. Yaw deviation must not falsely balloon near +/-180deg."""
    return float((angle + np.pi) % (2 * np.pi) - np.pi)


def _floor_eigenvalues(block: np.ndarray, max_condition_number: float) -> np.ndarray:
    """Cap a symmetric block's condition number by raising its weak eigenvalues.

    Leaves an already well-conditioned block untouched (returns it as-is,
    not merely numerically close) so this is a no-op on the common case.
    """
    eigenvalues, eigenvectors = np.linalg.eigh(block)
    floor = eigenvalues.max() / max_condition_number
    if eigenvalues.min() >= floor:
        return block
    floored = np.maximum(eigenvalues, floor)
    return (eigenvectors * floored) @ eigenvectors.T


def _build_information(
    hessian: np.ndarray, num_inliers: int, fitness: float, config: VerifyConfig
) -> np.ndarray | None:
    """Turn small_gicp's raw Hessian into a gtsam-ready information matrix.

    Three corrections, in order, each documented on the config field that
    drives it: (1) per-block eigenvalue flooring for degenerate geometry,
    (2) per-inlier normalization so information doesn't scale with how many
    points happened to be in the cloud, (3) a fitness/info_scale multiplier
    reflecting overall match quality. Returns None if the result isn't
    positive definite -- a final safety net, since gtsam cannot use an
    information matrix that isn't valid.
    """
    symmetric = 0.5 * (hessian + hessian.T)
    rotation_block = symmetric[_ROTATION_SLICE, _ROTATION_SLICE]
    translation_block = symmetric[_TRANSLATION_SLICE, _TRANSLATION_SLICE]

    rotation_eigenvalues = np.linalg.eigvalsh(rotation_block)
    translation_eigenvalues = np.linalg.eigvalsh(translation_block)
    if rotation_eigenvalues.min() < config.min_rotation_information:
        return None
    if translation_eigenvalues.min() < config.min_translation_information:
        return None

    floored = np.array(symmetric, copy=True)
    floored[_ROTATION_SLICE, _ROTATION_SLICE] = _floor_eigenvalues(
        rotation_block, config.max_condition_number
    )
    floored[_TRANSLATION_SLICE, _TRANSLATION_SLICE] = _floor_eigenvalues(
        translation_block, config.max_condition_number
    )

    information = floored / max(num_inliers, 1) * fitness * config.info_scale
    if np.linalg.eigvalsh(information).min() <= 0.0:
        return None
    return information


def verify_candidate(
    source: Keyframe,
    target: Keyframe,
    yaw_prior: float,
    config: VerifyConfig | None = None,
) -> Edge | None:
    """Geometrically verify a proposed loop closure between two keyframes.

    Registers ``source`` against ``target`` with GICP, seeded from
    ``yaw_prior``, and gates hard on the result (see :class:`VerifyConfig`
    for every threshold and its rationale). Returns a validated
    :class:`~swarmdeck_slam.types.Edge` with ``t_src_dst`` mapping points
    from ``target``'s base frame into ``source``'s base frame, or ``None``
    if the candidate does not survive verification.

    ``yaw_prior`` is the coarse estimate, from place recognition, of the
    yaw component of ``T_target_source`` in small_gicp's own naming (the
    rotation that would map ``source``'s frame into ``target``'s frame) --
    place recognition proposes the two views are of the same physical spot,
    so the translation part of the initial guess is left at zero and only
    the heading needs seeding.
    """
    config = config or VerifyConfig()
    if len(source.points) < config.min_points or len(target.points) < config.min_points:
        return None

    source_points = np.asarray(source.points, dtype=np.float64)
    target_points = np.asarray(target.points, dtype=np.float64)

    cos_yaw, sin_yaw = math.cos(yaw_prior), math.sin(yaw_prior)
    init_t_target_source = se3_identity()
    init_t_target_source[:2, :2] = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])

    result = small_gicp.align(
        target_points,
        source_points,
        init_T_target_source=init_t_target_source,
        registration_type="GICP",
        downsampling_resolution=config.downsampling_resolution,
        max_correspondence_distance=config.max_correspondence_distance,
        num_threads=config.num_threads,
        max_iterations=config.max_iterations,
    )
    if not result.converged:
        return None
    if result.num_inliers < config.min_inliers:
        return None

    # num_threads=1 unconditionally: this is only a denominator for the
    # inlier ratio, and small_gicp's multi-threaded downsampling is
    # documented as non-deterministic in point count run to run, which
    # would make the accept/reject boundary flaky.
    downsampled_source = small_gicp.voxelgrid_sampling(
        source_points, config.downsampling_resolution, 1
    )
    inlier_ratio = result.num_inliers / max(downsampled_source.size(), 1)
    if inlier_ratio < config.min_inlier_ratio:
        return None

    mean_error = result.error / result.num_inliers
    if mean_error > config.max_mean_error:
        return None

    # T_target_source maps `source` points into `target`'s frame; invert to
    # get t_src_dst = T_source_target, matching this function's own
    # source-then-target argument order and Edge's src/dst naming. Skipping
    # this inversion is exactly the silent bug the direction-inversion
    # regression test in test_verify.py exists to catch.
    t_target_source = np.asarray(result.T_target_source, dtype=np.float64)
    t_src_dst = se3_inverse(t_target_source)

    translation_m, _ = se3_distance(se3_identity(), t_src_dst)
    if translation_m > config.max_translation_m:
        return None

    achieved_yaw = _yaw_of(t_target_source[:3, :3])
    yaw_deviation = abs(_wrap_angle(achieved_yaw - yaw_prior))
    if yaw_deviation > config.max_yaw_deviation_from_prior_rad:
        return None

    fitness = 1.0 / (1.0 + mean_error / config.fitness_error_scale)
    information = _build_information(result.H, result.num_inliers, fitness, config)
    if information is None:
        return None
    if config.information == "isotropic":
        # Degeneracy already rejected the match; this only replaces the
        # (over-rotated) Hessian with a weight the solver can actually use.
        information = np.eye(6, dtype=np.float64) * (
            config.isotropic_scale * max(fitness, 1e-6)
        )

    kind = EdgeKind.INTER_LOOP if source.id.robot_id != target.id.robot_id else EdgeKind.INTRA_LOOP
    return Edge(
        kind=kind,
        src=source.id,
        dst=target.id,
        t_src_dst=t_src_dst,
        information=information,
        fitness=fitness,
        inlier_ratio=inlier_ratio,
    )


@dataclass(frozen=True, slots=True)
class Candidate:
    """One loop-closure proposal from place recognition, ready to verify."""

    source: Keyframe
    target: Keyframe
    yaw_prior: float


def _reject_if_event_loop_running() -> None:
    """Refuse to run blocking CPU work from inside a running asyncio loop.

    GICP registration is synchronous, CPU-bound work with no `await` points;
    calling it from a coroutine would freeze that event loop for the entire
    batch. Failing loudly here is safer than "working" while silently
    starving every other task on the loop -- the caller should offload this
    call itself (`asyncio.to_thread`, a process pool, ...).
    """
    try:
        asyncio.get_running_loop()
    except RuntimeError:
        return
    raise RuntimeError(
        "verify_candidates() performs blocking, CPU-bound GICP alignment and must not "
        "be called from inside a running asyncio event loop -- it would block the loop "
        "for the whole batch. Call it from a worker thread (e.g. asyncio.to_thread) or "
        "a process pool from the async side instead."
    )


def verify_candidates(
    candidates: Iterable[Candidate],
    config: VerifyConfig | None = None,
    max_workers: int = 1,
) -> list[Edge]:
    """Verify many candidates, dropping the ones that fail (see :func:`verify_candidate`).

    Each candidate is independent and side-effect-free to verify, which is
    what makes this safe to parallelize. With ``max_workers <= 1`` this is a
    plain sequential loop -- the simplest, most debuggable path, and what
    every test in this package uses. With ``max_workers > 1`` it uses a
    thread pool: small_gicp's `align` is a pybind11 C++ extension doing
    substantial floating-point work per call, so it is a reasonable bet that
    it releases the GIL during that work the way numpy's heavy C paths do,
    but that is not verified here or documented upstream. Threads are
    correct either way (there is no shared mutable state between
    candidates), just not guaranteed to be *faster* than sequential if the
    GIL turns out to be held throughout -- if profiling later shows that,
    swap in ``ProcessPoolExecutor`` without changing this function's
    contract. Do not raise both this and ``VerifyConfig.num_threads`` above
    1 at once: GICP's own internal thread pool multiplied by an outer
    thread pool oversubscribes the machine's cores for no benefit.
    """
    _reject_if_event_loop_running()

    if max_workers <= 1:
        return [
            edge
            for candidate in candidates
            if (edge := verify_candidate(candidate.source, candidate.target, candidate.yaw_prior, config))
            is not None
        ]

    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        futures = [
            executor.submit(verify_candidate, candidate.source, candidate.target, candidate.yaw_prior, config)
            for candidate in candidates
        ]
        return [edge for future in futures if (edge := future.result()) is not None]
