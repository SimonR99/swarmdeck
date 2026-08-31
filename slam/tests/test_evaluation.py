"""Tests for :mod:`swarmdeck_slam.evaluation`.

Organized around the module's own claims: exact ground truth scores exactly
zero everywhere; a known injected offset produces exactly that offset back out
of the metrics that cannot absorb it (RPE, inter-robot transform error);
Umeyama alignment recovers a known rigid transform and refuses to fit away a
scale change; degenerate inputs raise clearly instead of returning a
plausible-looking NaN; component scoring keeps false merges and missed merges
apart; and ``to_dict()`` round-trips through JSON.
"""

from __future__ import annotations

import json
import math

import numpy as np
import pytest
from synthetic import two_robot_fleet, yaw_pose

from swarmdeck_slam import evaluation as ev
from swarmdeck_slam.types import (
    Component,
    KeyframeId,
    OptimizedGraph,
    se3_identity,
    se3_inverse,
    se3_medoid,
)

# --------------------------------------------------------------------------- #
# Fixtures
# --------------------------------------------------------------------------- #


def _triangle_truth() -> dict[KeyframeId, np.ndarray]:
    """Five non-collinear planar poses: enough to fully determine a 3-D rotation."""
    points = [
        (0.0, 0.0, 0.0),
        (3.0, 0.0, 0.3),
        (3.0, 4.0, 0.0),
        (0.0, 4.0, 0.6),
        (1.0, 2.0, 0.1),
    ]
    yaws = [0.0, 0.4, 1.1, -0.6, 2.0]
    truth: dict[KeyframeId, np.ndarray] = {}
    for i, ((x, y, z), yaw) in enumerate(zip(points, yaws)):
        pose = yaw_pose(x, y, yaw, z=z)
        truth[KeyframeId("r", i)] = pose
    return truth


def _collinear_truth() -> dict[KeyframeId, np.ndarray]:
    """Poses along a single straight line: rotation about that line is unobservable."""
    truth: dict[KeyframeId, np.ndarray] = {}
    for i in range(5):
        truth[KeyframeId("r", i)] = yaw_pose(float(i) * 2.0, 0.0, 0.0)
    return truth


@pytest.fixture(scope="module")
def fleet_truth():
    """Two robots sharing one scene, per the shared synthetic fixture."""
    _scene, robots = two_robot_fleet(seed=7)
    return {robot.robot_id: robot for robot in robots}


def _perfect_graph(fleet_truth: dict) -> OptimizedGraph:
    """An OptimizedGraph that reproduces ground truth exactly, both robots merged."""
    poses: dict[KeyframeId, np.ndarray] = {}
    t_world_map: dict[str, np.ndarray] = {}
    for robot in fleet_truth.values():
        poses.update(robot.truth)
        t_world_map[robot.robot_id] = robot.t_world_map_true
    component = Component(
        component_id=0, robots=frozenset(fleet_truth), anchor=next(iter(poses))
    )
    return OptimizedGraph(poses=poses, t_world_map=t_world_map, components=[component])


OFFSET_CASES = {
    "translation_only": (2.0, -1.5, 0.0),
    "rotation_only": (0.0, 0.0, 0.37),
    "combined": (2.0, -1.5, 0.37),
}


def _offset_transform(dx: float, dy: float, yaw: float) -> np.ndarray:
    return yaw_pose(dx, dy, yaw)


def _expected_translation_rotation(
    dx: float, dy: float, yaw: float
) -> tuple[float, float]:
    return math.hypot(dx, dy), abs(yaw)


def test_se3_medoid_rejects_one_bad_survey() -> None:
    expected = yaw_pose(1.0, 2.0, 0.10)
    nearby = yaw_pose(0.97, 2.0, 0.10)
    outlier = yaw_pose(14.0, 2.0, 0.10)

    selected = se3_medoid(
        [outlier, expected, nearby],
        translation_scale_m=8.0,
        rotation_scale_rad=math.radians(75.0),
    )

    assert selected is expected


def test_se3_medoid_rejects_empty_candidates() -> None:
    with pytest.raises(ValueError, match="at least one"):
        se3_medoid([])


# --------------------------------------------------------------------------- #
# ErrorStats
# --------------------------------------------------------------------------- #


def test_error_stats_basic_values() -> None:
    stats = ev.ErrorStats.from_errors([1.0, 2.0, 3.0, 4.0])
    assert stats.n == 4
    assert stats.mean == pytest.approx(2.5)
    assert stats.median == pytest.approx(2.5)
    assert stats.max == pytest.approx(4.0)
    assert stats.rmse == pytest.approx(math.sqrt((1 + 4 + 9 + 16) / 4))


def test_error_stats_rejects_empty() -> None:
    with pytest.raises(ValueError):
        ev.ErrorStats.from_errors([])


def test_error_stats_rejects_nan() -> None:
    with pytest.raises(ValueError):
        ev.ErrorStats.from_errors([1.0, float("nan")])


def test_error_stats_rejects_inf() -> None:
    with pytest.raises(ValueError):
        ev.ErrorStats.from_errors([1.0, float("inf")])


# --------------------------------------------------------------------------- #
# Anchor test: exact ground truth in, exactly zero error out, every metric.
# --------------------------------------------------------------------------- #


def test_ate_zero_for_exact_ground_truth(fleet_truth) -> None:
    # Tolerance is float64-SVD-roundoff, not slack for a real error: the point
    # cloud spans tens of metres, and the alignment SVD accumulates ~1e-8 rad
    # of noise at that scale even when source and target are identical.
    alpha = fleet_truth["alpha"]
    result = ev.compute_ate(alpha.truth, alpha.truth)
    assert result.translation_m.rmse == pytest.approx(0.0, abs=1e-9)
    assert result.translation_m.max == pytest.approx(0.0, abs=1e-9)
    assert result.rotation_rad.rmse == pytest.approx(0.0, abs=1e-6)
    assert result.rotation_rad.max == pytest.approx(0.0, abs=1e-6)
    assert result.n_poses == len(alpha.truth)


def test_rpe_zero_for_exact_ground_truth(fleet_truth) -> None:
    alpha = fleet_truth["alpha"]
    for delta in (1, 3):
        result = ev.compute_rpe(alpha.truth, alpha.truth, delta=delta)
        assert result.translation_m.rmse == pytest.approx(0.0, abs=1e-9)
        assert result.rotation_rad.rmse == pytest.approx(0.0, abs=1e-9)


def test_inter_robot_error_zero_for_exact_ground_truth(fleet_truth) -> None:
    true_map = {rid: robot.t_world_map_true for rid, robot in fleet_truth.items()}
    result = ev.inter_robot_transform_error(true_map, true_map)
    for err in result.values():
        assert err.translation_m == pytest.approx(0.0, abs=1e-9)
        assert err.rotation_deg == pytest.approx(0.0, abs=1e-9)


def test_component_score_perfect_when_grouping_matches(fleet_truth) -> None:
    component = Component(
        component_id=0, robots=frozenset(fleet_truth), anchor=KeyframeId("alpha", 0)
    )
    truth_groups = {rid: "scene-0" for rid in fleet_truth}
    score = ev.score_components([component], truth_groups)
    assert score.is_perfect
    assert score.false_merges == ()
    assert score.missed_merges == ()
    assert score.false_merge_rate == 0.0
    assert score.missed_merge_rate == 0.0


def test_evaluate_end_to_end_perfect_graph_is_all_zero(fleet_truth) -> None:
    graph = _perfect_graph(fleet_truth)
    truth_poses: dict[KeyframeId, np.ndarray] = {}
    for robot in fleet_truth.values():
        truth_poses.update(robot.truth)
    truth_t_world_map = {
        rid: robot.t_world_map_true for rid, robot in fleet_truth.items()
    }
    truth_groups = {rid: "scene-0" for rid in fleet_truth}

    report = ev.evaluate(
        "perfect",
        graph,
        truth_poses,
        truth_t_world_map,
        truth_groups,
        rpe_deltas=(1, 2),
    )

    assert set(report.ate) == {"alpha", "beta", "component:0"}
    for ate in report.ate.values():
        assert ate.translation_m.rmse == pytest.approx(0.0, abs=1e-9)
        assert ate.rotation_rad.rmse == pytest.approx(
            0.0, abs=1e-6
        )  # see roundoff note above
    for results in report.rpe.values():
        for rpe in results:
            assert rpe.translation_m.rmse == pytest.approx(0.0, abs=1e-9)
    for err in report.inter_robot.values():
        assert err.translation_m == pytest.approx(0.0, abs=1e-9)
    assert report.components.is_perfect

    # Human-readable rendering and JSON round-trip both work on the real thing.
    rendered = report.format()
    assert "perfect" in rendered
    assert str(report) == rendered
    round_tripped = json.loads(json.dumps(report.to_dict()))
    assert round_tripped["label"] == "perfect"


# --------------------------------------------------------------------------- #
# Known injected offset -> exactly that error (metrics with no alignment step)
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize("case", sorted(OFFSET_CASES))
def test_inter_robot_error_matches_known_offset_exactly(case: str) -> None:
    dx, dy, yaw = OFFSET_CASES[case]
    true_map = {"alpha": yaw_pose(1.0, 2.0, 0.2)}
    offset = _offset_transform(dx, dy, yaw)
    estimated_map = {"alpha": true_map["alpha"] @ offset}

    result = ev.inter_robot_transform_error(estimated_map, true_map)
    expected_t, expected_r = _expected_translation_rotation(dx, dy, yaw)
    assert result["alpha"].translation_m == pytest.approx(expected_t, abs=1e-9)
    assert result["alpha"].rotation_deg == pytest.approx(
        math.degrees(expected_r), abs=1e-9
    )


@pytest.mark.parametrize("case", sorted(OFFSET_CASES))
def test_rpe_matches_known_offset_exactly(case: str) -> None:
    dx, dy, yaw = OFFSET_CASES[case]
    truth = {
        KeyframeId("r", 0): yaw_pose(0.0, 0.0, 0.0),
        KeyframeId("r", 1): yaw_pose(5.0, 0.0, 0.0),
    }
    offset = _offset_transform(dx, dy, yaw)
    estimated = {
        KeyframeId("r", 0): truth[KeyframeId("r", 0)],
        KeyframeId("r", 1): truth[KeyframeId("r", 1)] @ offset,
    }

    result = ev.compute_rpe(estimated, truth, delta=1)
    expected_t, expected_r = _expected_translation_rotation(dx, dy, yaw)
    assert result.translation_m.n == 1
    assert result.translation_m.rmse == pytest.approx(expected_t, abs=1e-9)
    assert result.translation_m.max == pytest.approx(expected_t, abs=1e-9)
    assert result.rotation_rad.rmse == pytest.approx(expected_r, abs=1e-9)


def test_rpe_is_invariant_to_a_constant_global_offset(fleet_truth) -> None:
    """The defining property of RPE: a rigid offset applied to every pose cancels."""
    alpha = fleet_truth["alpha"]
    drift = yaw_pose(37.0, -12.0, 1.4)  # arbitrary, large, fixed
    shifted = {kf_id: drift @ pose for kf_id, pose in alpha.truth.items()}
    result = ev.compute_rpe(shifted, alpha.truth, delta=1)
    assert result.translation_m.rmse == pytest.approx(0.0, abs=1e-8)
    assert result.rotation_rad.rmse == pytest.approx(0.0, abs=1e-8)


# --------------------------------------------------------------------------- #
# Umeyama alignment: recovers a known rigid transform, never absorbs scale.
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize("case", sorted(OFFSET_CASES))
def test_umeyama_recovers_known_rigid_transform(case: str) -> None:
    dx, dy, yaw = OFFSET_CASES[case]
    truth = _triangle_truth()
    t_known = _offset_transform(dx, dy, yaw)
    t_known[2, 3] = 0.75  # also offset in z, since these poses are not planar
    estimated = {kf_id: se3_inverse(t_known) @ pose for kf_id, pose in truth.items()}

    alignment, common = ev.align_rigid(estimated, truth)
    assert len(common) == len(truth)
    assert np.allclose(alignment, t_known, atol=1e-8)

    ate = ev.compute_ate(estimated, truth)
    assert ate.translation_m.rmse == pytest.approx(0.0, abs=1e-7)
    assert ate.rotation_rad.rmse == pytest.approx(0.0, abs=1e-7)


def test_umeyama_identity_for_exact_match() -> None:
    truth = _triangle_truth()
    alignment, _ = ev.align_rigid(truth, truth)
    assert np.allclose(alignment, se3_identity(), atol=1e-9)


def test_umeyama_does_not_absorb_scale() -> None:
    """The test that catches the most damaging silent bug in this module.

    A trajectory scaled up by 10% is a real, metric error. If alignment ever
    grows a scale-fitting term, this residual silently drops to ~0.
    """
    truth = _triangle_truth()
    scale = 1.10
    estimated = {}
    for kf_id, pose in truth.items():
        scaled = pose.copy()
        scaled[:3, 3] = pose[:3, 3] * scale
        estimated[kf_id] = scaled

    ate = ev.compute_ate(estimated, truth)
    # Points span several metres; a 10% scale error must show up as centimetres
    # to metres of residual, not vanish into alignment noise.
    assert ate.translation_m.rmse > 0.05
    assert ate.translation_m.max > 0.05


def test_umeyama_handles_collinear_trajectory_without_nan_or_raise() -> None:
    """Guard against degenerate (rank-deficient) geometry: no NaN, no crash.

    Rotation about the trajectory's own axis is genuinely unobservable from a
    collinear point set, so this does not assert exact recovery of an injected
    rotation -- only that the result is finite and well-formed.
    """
    truth = _collinear_truth()
    offset = yaw_pose(1.0, 0.5, 0.2)
    estimated = {kf_id: pose @ offset for kf_id, pose in truth.items()}

    alignment, common = ev.align_rigid(estimated, truth)
    assert len(common) == len(truth)
    assert np.all(np.isfinite(alignment))

    ate = ev.compute_ate(estimated, truth)
    assert np.isfinite(ate.translation_m.rmse)
    assert np.isfinite(ate.rotation_rad.rmse)


# --------------------------------------------------------------------------- #
# Degenerate inputs raise clearly.
# --------------------------------------------------------------------------- #


def test_align_rigid_raises_on_single_pose() -> None:
    truth = {KeyframeId("r", 0): se3_identity()}
    with pytest.raises(ValueError, match="at least 2"):
        ev.align_rigid(truth, truth)


def test_compute_ate_raises_on_single_pose() -> None:
    truth = {KeyframeId("r", 0): se3_identity()}
    with pytest.raises(ValueError):
        ev.compute_ate(truth, truth)


def test_align_rigid_raises_on_disjoint_keys() -> None:
    a = {KeyframeId("r", 0): se3_identity(), KeyframeId("r", 1): se3_identity()}
    b = {KeyframeId("s", 0): se3_identity(), KeyframeId("s", 1): se3_identity()}
    with pytest.raises(ValueError, match="at least 2"):
        ev.align_rigid(a, b)


def test_compute_rpe_raises_when_no_robot_has_enough_span() -> None:
    truth = {
        KeyframeId("r", 0): se3_identity(),
        KeyframeId("r", 1): yaw_pose(1.0, 0.0, 0.0),
    }
    with pytest.raises(ValueError, match="no keyframe pairs"):
        ev.compute_rpe(truth, truth, delta=5)


def test_compute_rpe_raises_on_invalid_delta() -> None:
    truth = {KeyframeId("r", 0): se3_identity()}
    with pytest.raises(ValueError, match="delta"):
        ev.compute_rpe(truth, truth, delta=0)


def test_inter_robot_error_raises_on_disjoint_robots() -> None:
    with pytest.raises(ValueError, match="no robots in common"):
        ev.inter_robot_transform_error(
            {"alpha": se3_identity()}, {"beta": se3_identity()}
        )


def test_score_components_raises_on_fewer_than_two_robots() -> None:
    with pytest.raises(ValueError, match="at least 2 robots"):
        ev.score_components([], {"alpha": "scene-0"})


def test_error_stats_from_errors_raises_on_wrong_shape() -> None:
    with pytest.raises(ValueError):
        ev.ErrorStats.from_errors(np.zeros((2, 2)))


def test_ablation_raises_on_empty_reports() -> None:
    with pytest.raises(ValueError, match="at least one report"):
        ev.Ablation(reports=())


def test_ablation_raises_on_duplicate_labels(fleet_truth) -> None:
    graph = _perfect_graph(fleet_truth)
    truth_poses: dict[KeyframeId, np.ndarray] = {}
    for robot in fleet_truth.values():
        truth_poses.update(robot.truth)
    truth_t_world_map = {
        rid: robot.t_world_map_true for rid, robot in fleet_truth.items()
    }
    truth_groups = {rid: "scene-0" for rid in fleet_truth}
    report = ev.evaluate("dup", graph, truth_poses, truth_t_world_map, truth_groups)
    with pytest.raises(ValueError, match="unique"):
        ev.Ablation(reports=(report, report))


# --------------------------------------------------------------------------- #
# Component scoring: false merge vs missed merge, never averaged together.
# --------------------------------------------------------------------------- #


def test_component_score_detects_false_merge() -> None:
    # alpha and beta wrongly placed in the same component; they do not share a scene.
    component = Component(
        component_id=0,
        robots=frozenset({"alpha", "beta"}),
        anchor=KeyframeId("alpha", 0),
    )
    truth_groups = {"alpha": "scene-0", "beta": "scene-1"}
    score = ev.score_components([component], truth_groups)
    assert score.false_merges == (("alpha", "beta"),)
    assert score.missed_merges == ()
    assert score.false_merge_rate == pytest.approx(1.0)
    assert not score.is_perfect


def test_component_score_detects_missed_merge() -> None:
    # alpha and beta truly share a scene but were kept in separate components.
    components = [
        Component(
            component_id=0, robots=frozenset({"alpha"}), anchor=KeyframeId("alpha", 0)
        ),
        Component(
            component_id=1, robots=frozenset({"beta"}), anchor=KeyframeId("beta", 0)
        ),
    ]
    truth_groups = {"alpha": "scene-0", "beta": "scene-0"}
    score = ev.score_components(components, truth_groups)
    assert score.missed_merges == (("alpha", "beta"),)
    assert score.false_merges == ()
    assert score.missed_merge_rate == pytest.approx(1.0)
    assert not score.is_perfect


def test_component_score_robot_absent_from_any_component_is_singleton() -> None:
    # 'gamma' never appears in any Component -- treated as its own singleton,
    # not merged with the (also absent) 'delta'.
    truth_groups = {"gamma": "scene-0", "delta": "scene-1"}
    score = ev.score_components([], truth_groups)
    assert score.is_perfect  # correctly kept apart, by omission


def test_component_score_distinguishes_both_failure_modes_at_once() -> None:
    # alpha/beta wrongly merged (false); gamma/delta wrongly left apart (missed).
    components = [
        Component(
            component_id=0,
            robots=frozenset({"alpha", "beta"}),
            anchor=KeyframeId("alpha", 0),
        ),
        Component(
            component_id=1, robots=frozenset({"gamma"}), anchor=KeyframeId("gamma", 0)
        ),
    ]
    truth_groups = {"alpha": "s0", "beta": "s1", "gamma": "s2", "delta": "s2"}
    score = ev.score_components(components, truth_groups)
    assert score.false_merges == (("alpha", "beta"),)
    assert score.missed_merges == (("delta", "gamma"),)
    assert not score.is_perfect
    # The two rates are independent numbers, not blended into one.
    assert (
        score.false_merge_rate != score.missed_merge_rate
        or score.n_true_positive_pairs != score.n_true_negative_pairs
    )


# --------------------------------------------------------------------------- #
# to_dict() round-trips through JSON.
# --------------------------------------------------------------------------- #


def test_error_stats_to_dict_json_round_trip() -> None:
    stats = ev.ErrorStats.from_errors([0.1, 0.2, 0.3])
    payload = json.loads(json.dumps(stats.to_dict()))
    assert payload == stats.to_dict()


def test_component_score_to_dict_json_round_trip() -> None:
    component = Component(
        component_id=0,
        robots=frozenset({"alpha", "beta"}),
        anchor=KeyframeId("alpha", 0),
    )
    score = ev.score_components([component], {"alpha": "s0", "beta": "s1"})
    payload = json.loads(json.dumps(score.to_dict()))
    assert payload["false_merges"] == [["alpha", "beta"]]
    assert payload["false_merge_rate"] == pytest.approx(1.0)


def test_report_to_dict_json_round_trip(fleet_truth) -> None:
    graph = _perfect_graph(fleet_truth)
    truth_poses: dict[KeyframeId, np.ndarray] = {}
    for robot in fleet_truth.values():
        truth_poses.update(robot.truth)
    truth_t_world_map = {
        rid: robot.t_world_map_true for rid, robot in fleet_truth.items()
    }
    truth_groups = {rid: "scene-0" for rid in fleet_truth}
    report = ev.evaluate(
        "json-check", graph, truth_poses, truth_t_world_map, truth_groups
    )

    payload = json.loads(json.dumps(report.to_dict()))
    assert payload["label"] == "json-check"
    assert set(payload["ate"]) == set(report.ate)
    assert payload["ate"]["alpha"]["translation_m"]["rmse"] == pytest.approx(
        report.ate["alpha"].translation_m.rmse
    )
    assert payload["components"]["false_merges"] == []


def test_ablation_to_dict_json_round_trip(fleet_truth) -> None:
    graph = _perfect_graph(fleet_truth)
    truth_poses: dict[KeyframeId, np.ndarray] = {}
    for robot in fleet_truth.values():
        truth_poses.update(robot.truth)
    truth_t_world_map = {
        rid: robot.t_world_map_true for rid, robot in fleet_truth.items()
    }
    truth_groups = {rid: "scene-0" for rid in fleet_truth}

    report_a = ev.evaluate(
        "baseline", graph, truth_poses, truth_t_world_map, truth_groups
    )
    # A second "run" that's identical except relabelled, standing in for a
    # different configuration in the ablation ladder.
    report_b = ev.evaluate(
        "collaborative", graph, truth_poses, truth_t_world_map, truth_groups
    )
    ablation = ev.Ablation(reports=(report_a, report_b))

    rendered = ablation.format()
    assert "baseline" in rendered and "collaborative" in rendered
    payload = json.loads(json.dumps(ablation.to_dict()))
    assert [r["label"] for r in payload["reports"]] == ["baseline", "collaborative"]
