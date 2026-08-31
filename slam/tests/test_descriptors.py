"""Tests for the Scan Context place-recognition descriptor and its index.

Everything here runs against the shared synthetic fleet (`tests/synthetic.py`),
not mocks: descriptor separation, cross-robot matching and yaw recovery are all
real geometry through `observe()`, because a mocked cloud cannot exercise the
one thing this module has to get right -- whether the *actual* polar-binned
height signature of a place is distinctive and its rotation recoverable.
"""

from __future__ import annotations

import time

import numpy as np
import pytest
from swarmdeck_protocol import Descriptor as WireDescriptor
from swarmdeck_protocol import decode_keyframe, encode_keyframe

from swarmdeck_slam.descriptors import (
    DEFAULT_MAX_RANGE,
    DEFAULT_RINGS,
    DEFAULT_SECTORS,
    DESCRIPTOR_KIND,
    PlaceCandidate,
    ScanContextIndex,
    alignment_hypotheses,
    best_alignment,
    ring_key,
    scan_context_descriptor,
    shift_to_yaw,
)
from swarmdeck_slam.types import KeyframeId

from synthetic import make_scene, observe, two_robot_fleet, yaw_pose

SEED = 0


def test_alignment_hypotheses_keeps_distinct_rotation_modes() -> None:
    descriptor = np.zeros((2, 12), dtype=np.uint8)
    descriptor[:, (1, 7)] = 200
    hypotheses = alignment_hypotheses(
        descriptor, descriptor, count=2, min_separation_sectors=2
    )

    assert [shift for shift, _ in hypotheses] == [0, 6]
    assert hypotheses[0][1] == pytest.approx(hypotheses[1][1])
    assert hypotheses[0][1] == pytest.approx(best_alignment(descriptor, descriptor)[1])


def test_alignment_hypotheses_keeps_antipode_when_greedy_is_clustered() -> None:
    descriptor = np.zeros((2, 12), dtype=np.uint8)
    descriptor[:, 0] = 200
    hypotheses = alignment_hypotheses(
        descriptor, descriptor, count=3, min_separation_sectors=2
    )
    assert 6 in [shift for shift, _ in hypotheses]


# --------------------------------------------------------------------------- #
# Yaw recovery
# --------------------------------------------------------------------------- #


def test_yaw_recovery_same_place_different_heading() -> None:
    """The same viewpoint at a different heading matches, and yaw is recoverable.

    Sector width at the defaults is 360/60 = 6 degrees, so "within a sector or
    two" is an error under ~12-18 degrees; we assert a tighter bound (10 deg)
    because these headings differ by far more than sensor noise, leaving no
    excuse for a larger error.
    """
    scene = make_scene(SEED)
    position = (20.0, 12.0, 0.0)
    true_deltas_deg = [-170.0, -90.0, -34.0, -5.0, 5.0, 34.0, 90.0, 170.0]

    base_yaw = 0.3
    base_points = observe(
        scene, yaw_pose(*position[:2], base_yaw), rng=np.random.default_rng(1)
    )
    base_descriptor = scan_context_descriptor(base_points)

    errors_deg = []
    for delta_deg in true_deltas_deg:
        yaw = base_yaw + np.radians(delta_deg)
        points = observe(
            scene, yaw_pose(*position[:2], yaw), rng=np.random.default_rng(2)
        )
        descriptor = scan_context_descriptor(points)

        _, distance = best_alignment(base_descriptor, descriptor)
        assert (
            distance < 0.2
        ), f"same place, different heading should match closely (dist={distance})"

        shift, _ = best_alignment(base_descriptor, descriptor)
        yaw_est = shift_to_yaw(shift, DEFAULT_SECTORS)
        true_delta = (np.radians(delta_deg) + np.pi) % (2 * np.pi) - np.pi
        error_deg = np.degrees(((yaw_est - true_delta + np.pi) % (2 * np.pi)) - np.pi)
        errors_deg.append(abs(error_deg))

    assert max(errors_deg) < 10.0, f"yaw errors (deg) too large: {errors_deg}"


# --------------------------------------------------------------------------- #
# Separation margin
# --------------------------------------------------------------------------- #


def test_different_places_separate_with_a_clear_margin() -> None:
    """Distinct places are further apart, in descriptor distance, than heading
    changes at one place -- with enough margin that a regression narrowing it
    fails this test rather than silently degrading loop-closure precision.
    """
    scene = make_scene(SEED)
    # Well separated across the 40 x 24 m building, away from walls.
    locations = [
        (5.0, 5.0),
        (5.0, 20.0),
        (35.0, 5.0),
        (35.0, 20.0),
        (20.0, 12.0),
        (30.0, 15.0),
    ]
    headings = [0.0, 1.3]

    descriptors: dict[tuple[int, int], np.ndarray] = {}
    for i, (x, y) in enumerate(locations):
        for j, yaw in enumerate(headings):
            points = observe(
                scene, yaw_pose(x, y, yaw), rng=np.random.default_rng(100 + i * 10 + j)
            )
            descriptors[(i, j)] = scan_context_descriptor(points)

    same_place, different_place = [], []
    keys = list(descriptors)
    for a in range(len(keys)):
        for b in range(a + 1, len(keys)):
            (i1, _), (i2, _) = keys[a], keys[b]
            _, distance = best_alignment(descriptors[keys[a]], descriptors[keys[b]])
            (same_place if i1 == i2 else different_place).append(distance)

    worst_same_place = max(same_place)
    best_different_place = min(different_place)
    margin = best_different_place - worst_same_place

    # Observed margin at these fixtures is ~0.19; require a clear majority of
    # that so real degradation trips this before it reaches production, without
    # being so tight that float/BLAS nondeterminism could ever flake it.
    assert margin > 0.1, (
        f"separation margin too small: worst same-place={worst_same_place:.4f}, "
        f"best different-place={best_different_place:.4f}, margin={margin:.4f}"
    )


# --------------------------------------------------------------------------- #
# Cross-robot matching
# --------------------------------------------------------------------------- #


def test_cross_robot_matching() -> None:
    """A place `alpha` visited also matches when `beta` visits it, via the index."""
    scene, robots = two_robot_fleet(SEED)
    alpha, beta = robots

    index = ScanContextIndex(
        rings=DEFAULT_RINGS, sectors=DEFAULT_SECTORS, temporal_window=2
    )
    for keyframe in alpha.keyframes:
        index.add(keyframe.id, scan_context_descriptor(keyframe.points))

    # alpha#11 and beta#19 are the fixture's closest cross-robot pair (~0.85 m
    # apart, ~91 deg heading difference) -- established empirically from the
    # ground truth, not assumed.
    query_kf = next(k for k in beta.keyframes if k.id.seq == 19)
    query_descriptor = scan_context_descriptor(query_kf.points)

    results = index.query(query_descriptor, k=5, query_id=query_kf.id)
    assert results, "expected at least one candidate"
    best = results[0]
    assert best.keyframe_id == KeyframeId(
        "alpha", 11
    ), f"expected alpha#11 top match, got {best.keyframe_id}"

    true_yaw_alpha = _yaw_of(alpha.truth[KeyframeId("alpha", 11)])
    true_yaw_beta = _yaw_of(beta.truth[query_kf.id])
    true_delta = (true_yaw_beta - true_yaw_alpha + np.pi) % (2 * np.pi) - np.pi

    error_deg = np.degrees(((best.yaw - true_delta + np.pi) % (2 * np.pi)) - np.pi)
    assert abs(error_deg) < 15.0, f"cross-robot yaw estimate off by {error_deg:.1f} deg"


def _yaw_of(t_world_base: np.ndarray) -> float:
    return float(np.arctan2(t_world_base[1, 0], t_world_base[0, 0]))


# --------------------------------------------------------------------------- #
# Temporal exclusion
# --------------------------------------------------------------------------- #


def test_temporal_window_suppresses_adjacent_matches() -> None:
    """A wide temporal window drops the query's own near-neighbours from results."""
    scene, robots = two_robot_fleet(SEED)
    alpha = robots[0]
    query_kf = next(k for k in alpha.keyframes if k.id.seq == 10)
    query_descriptor = scan_context_descriptor(query_kf.points)

    narrow = ScanContextIndex(
        rings=DEFAULT_RINGS, sectors=DEFAULT_SECTORS, temporal_window=0
    )
    wide = ScanContextIndex(
        rings=DEFAULT_RINGS, sectors=DEFAULT_SECTORS, temporal_window=3
    )
    for keyframe in alpha.keyframes:
        descriptor = scan_context_descriptor(keyframe.points)
        narrow.add(keyframe.id, descriptor)
        wide.add(keyframe.id, descriptor)

    narrow_results = narrow.query(query_descriptor, k=5, query_id=query_kf.id)
    wide_results = wide.query(query_descriptor, k=5, query_id=query_kf.id)

    narrow_seqs = {c.keyframe_id.seq for c in narrow_results}
    wide_seqs = {c.keyframe_id.seq for c in wide_results}

    assert (
        10 not in narrow_seqs and 10 not in wide_seqs
    ), "self-match must always be excluded"
    # seq=9 is the query's immediate neighbour: close enough to be the runner-up
    # match at window=0, but must be gone once the window covers it.
    assert (
        9 in narrow_seqs
    ), "expected the immediate neighbour to appear with no temporal window"
    assert all(
        abs(seq - 10) > 3 for seq in wide_seqs
    ), f"window=3 leaked an adjacent seq: {wide_seqs}"


def test_query_without_query_id_applies_no_exclusion() -> None:
    """`query_id=None` is an explicit opt-out, for exploratory queries."""
    scene, robots = two_robot_fleet(SEED)
    alpha = robots[0]
    index = ScanContextIndex(
        rings=DEFAULT_RINGS, sectors=DEFAULT_SECTORS, temporal_window=10
    )
    for keyframe in alpha.keyframes:
        index.add(keyframe.id, scan_context_descriptor(keyframe.points))

    query_kf = alpha.keyframes[10]
    descriptor = scan_context_descriptor(query_kf.points)
    results = index.query(descriptor, k=1, query_id=None)
    assert (
        results[0].keyframe_id == query_kf.id
    ), "identical descriptor should be its own best (unexcluded) match"
    assert results[0].distance == pytest.approx(0.0, abs=1e-9)


# --------------------------------------------------------------------------- #
# Wire round-trip
# --------------------------------------------------------------------------- #


def test_round_trips_through_the_wire_protocol() -> None:
    """The uint8 grid survives `Descriptor` -> `encode_keyframe` -> `decode_keyframe` exactly."""
    scene = make_scene(SEED)
    points = observe(scene, yaw_pose(5.0, 5.0, 0.3), rng=np.random.default_rng(7))
    descriptor = scan_context_descriptor(points, max_range=DEFAULT_MAX_RANGE)

    wire_descriptor = WireDescriptor(
        kind=DESCRIPTOR_KIND, data=descriptor, max_range=DEFAULT_MAX_RANGE
    )
    blob = encode_keyframe(
        robot_id="alpha",
        seq=3,
        stamp=1234.5,
        points=points,
        t_odom_base=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        descriptor=wire_descriptor,
    )
    packet = decode_keyframe(blob)

    assert packet.descriptor is not None
    assert packet.descriptor.kind == DESCRIPTOR_KIND
    assert packet.descriptor.max_range == DEFAULT_MAX_RANGE
    assert packet.descriptor.data.dtype == np.uint8
    assert packet.descriptor.data.shape == descriptor.shape
    assert np.array_equal(
        packet.descriptor.data, descriptor
    ), "descriptor grid must round-trip losslessly"


# --------------------------------------------------------------------------- #
# Empty and degenerate input
# --------------------------------------------------------------------------- #


def test_empty_cloud_does_not_raise() -> None:
    descriptor = scan_context_descriptor(np.zeros((0, 3), dtype=np.float32))
    assert descriptor.shape == (DEFAULT_RINGS, DEFAULT_SECTORS)
    assert descriptor.dtype == np.uint8
    assert np.all(descriptor == 0)


def test_all_points_beyond_max_range_does_not_raise() -> None:
    points = np.full((50, 3), 1000.0, dtype=np.float32)
    descriptor = scan_context_descriptor(points, max_range=10.0)
    assert np.all(descriptor == 0)


def test_all_points_at_origin_does_not_raise() -> None:
    """Zero radius is a degenerate but legal azimuth (atan2(0, 0) == 0); must not crash."""
    points = np.zeros((20, 3), dtype=np.float32)
    descriptor = scan_context_descriptor(points)
    assert descriptor.shape == (DEFAULT_RINGS, DEFAULT_SECTORS)
    # All points land in ring 0; every other ring stays empty (0).
    assert np.all(descriptor[1:, :] == 0)


def test_non_finite_points_are_dropped_not_raised() -> None:
    points = np.array(
        [[1.0, 1.0, 0.5], [np.nan, 2.0, 0.5], [3.0, np.inf, 0.5], [-np.inf, 1.0, 0.5]],
        dtype=np.float32,
    )
    descriptor = scan_context_descriptor(points)
    assert descriptor.shape == (DEFAULT_RINGS, DEFAULT_SECTORS)
    assert np.isfinite(descriptor).all()


def test_empty_index_query_returns_empty_list() -> None:
    index = ScanContextIndex(rings=DEFAULT_RINGS, sectors=DEFAULT_SECTORS)
    descriptor = scan_context_descriptor(np.zeros((0, 3), dtype=np.float32))
    assert index.query(descriptor, k=5) == []


def test_query_returns_fewer_than_k_when_index_is_small() -> None:
    index = ScanContextIndex(rings=DEFAULT_RINGS, sectors=DEFAULT_SECTORS)
    scene = make_scene(SEED)
    points = observe(scene, yaw_pose(5.0, 5.0, 0.0), rng=np.random.default_rng(3))
    index.add(KeyframeId("alpha", 0), scan_context_descriptor(points))
    results = index.query(scan_context_descriptor(points), k=50)
    assert len(results) == 1


# --------------------------------------------------------------------------- #
# Ring key
# --------------------------------------------------------------------------- #


def test_ring_key_is_invariant_to_sector_rotation() -> None:
    rng = np.random.default_rng(SEED)
    descriptor = rng.integers(
        0, 256, size=(DEFAULT_RINGS, DEFAULT_SECTORS), dtype=np.uint8
    )
    rolled = np.roll(descriptor, shift=17, axis=1)
    assert np.allclose(ring_key(descriptor), ring_key(rolled))
    assert np.isclose(np.linalg.norm(ring_key(descriptor)), 1.0)


# --------------------------------------------------------------------------- #
# Scaling
# --------------------------------------------------------------------------- #


def _random_index(n: int, seed: int) -> tuple[ScanContextIndex, np.ndarray]:
    rng = np.random.default_rng(seed)
    index = ScanContextIndex(
        rings=DEFAULT_RINGS, sectors=DEFAULT_SECTORS, temporal_window=5
    )
    for i in range(n):
        descriptor = rng.integers(
            0, 256, size=(DEFAULT_RINGS, DEFAULT_SECTORS), dtype=np.uint8
        )
        index.add(KeyframeId(f"robot{i % 4}", i // 4), descriptor)
    query = rng.integers(0, 256, size=(DEFAULT_RINGS, DEFAULT_SECTORS), dtype=np.uint8)
    return index, query


def _median_query_seconds(
    index: ScanContextIndex, query: np.ndarray, repeats: int = 20
) -> float:
    index.query(query, k=10)  # pay the one-time tree build outside the measurement
    samples = []
    for _ in range(repeats):
        start = time.perf_counter()
        index.query(query, k=10)
        samples.append(time.perf_counter() - start)
    return float(np.median(samples))


def test_query_latency_scales_sublinearly_with_index_size() -> None:
    """A kNN tree, not a linear scan: a 10x larger index must not take ~10x longer.

    This is the scaling behaviour requirement from the design brief -- measured,
    not assumed. Descriptors here are random (not real scans): only the index's
    search-structure performance is under test, not descriptor quality.
    """
    small_index, small_query = _random_index(200, seed=1)
    large_index, large_query = _random_index(2000, seed=2)

    small_time = _median_query_seconds(small_index, small_query)
    large_time = _median_query_seconds(large_index, large_query)

    assert (
        large_time < 0.05
    ), f"query on a 2000-entry index took {large_time * 1000:.2f} ms"
    # A linear scan would grow ~10x; allow generous headroom above 1x for
    # timing noise while still catching that failure mode.
    assert large_time < small_time * 5.0 + 0.005, (
        f"query time grew too much with index size: {small_time * 1000:.3f} ms -> "
        f"{large_time * 1000:.3f} ms"
    )
