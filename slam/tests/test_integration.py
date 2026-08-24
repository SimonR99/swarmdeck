"""End-to-end tests across every module in the package.

Each module has its own unit tests, and every one of them passed while the seam
between two of them was still unverified. That is the normal outcome of parallel
development: each side tests against its own internally-consistent reading of a
shared convention, and both readings can be self-consistent while disagreeing
with each other. Only a test that runs the real pipeline can catch that.

The pipeline under test is the whole thesis of the design:

    keyframes -> descriptors -> geometric verification -> pose graph
              -> optimized trajectories -> rendered occupancy -> metrics

Note what is absent: nothing anywhere registers one occupancy grid against
another. Occupancy is a *rendering* of the optimized trajectories, which is why
the merged map cannot disagree with the poses that produced it.

A caution for anyone extending this file: an integration test that pools
populations it should separate produces false alarms that cost more time than no
test at all. An early version of this pipeline check reported a catastrophic
frame-convention error that did not exist, because it averaged the error of true
correspondences together with that of known false positives. Every assertion
here is therefore stratified by ground-truth separation.
"""

from __future__ import annotations

import numpy as np
import pytest
import synthetic

from swarmdeck_slam import evaluation as ev
from swarmdeck_slam.descriptors import ScanContextIndex, scan_context_descriptor
from swarmdeck_slam.graph import GtsamPoseGraph
from swarmdeck_slam.render import RenderConfig, render_occupancy
from swarmdeck_slam.types import (
    Edge,
    EdgeKind,
    Keyframe,
    OptimizedGraph,
    se3_distance,
    se3_relative,
)
from swarmdeck_slam.verify import verify_candidate

#: Sized to synthetic.py's own drift model rather than picked by feel. An
#: odometry information matrix that is tight relative to the drift actually
#: injected makes a *correct* loop closure increase ATE, because the optimizer
#: then believes the odometry over the closure -- a real pose-graph phenomenon
#: that reads exactly like a broken solver.
ODOM_INFORMATION = np.eye(6) * 400.0

#: Ground-truth separation below which two keyframes are the same place. The
#: fixture's keyframe spacing is a few metres, so this admits genuine revisits
#: without admitting the next room.
SAME_PLACE_M = 4.0


def _run_pipeline(fleet: list[synthetic.SyntheticRobot]) -> tuple[OptimizedGraph, list[Keyframe], int, int]:
    """Drive the full stack over a fleet; return the result and closure counts.

    Descriptors are added to the index *after* each keyframe is queried, which is
    what an online system does -- a keyframe cannot match against itself or
    against frames that do not exist yet. Indexing everything up front would make
    loop closure look easier than it is.
    """
    keyframes = [kf for robot in fleet for kf in robot.keyframes]
    graph = GtsamPoseGraph()
    for keyframe in keyframes:
        graph.add_keyframe(keyframe)

    for robot in fleet:
        for previous, current in zip(robot.keyframes, robot.keyframes[1:]):
            graph.add_edge(
                Edge(
                    kind=EdgeKind.ODOMETRY,
                    src=previous.id,
                    dst=current.id,
                    t_src_dst=se3_relative(previous.t_odom_base, current.t_odom_base),
                    information=ODOM_INFORMATION,
                )
            )

    index = ScanContextIndex(rings=20, sectors=60, temporal_window=5)
    accepted = inter_robot = 0
    for keyframe in keyframes:
        descriptor = scan_context_descriptor(keyframe.points)
        for candidate in index.query(descriptor, k=3, query_id=keyframe.id):
            target = next(k for k in keyframes if k.id == candidate.keyframe_id)
            edge = verify_candidate(source=keyframe, target=target, yaw_prior=candidate.yaw)
            if edge is None:
                continue
            graph.add_edge(edge)
            accepted += 1
            inter_robot += int(edge.is_inter_robot)
        index.add(keyframe.id, descriptor)

    return graph.optimize(), keyframes, accepted, inter_robot


@pytest.fixture(scope="module")
def shared_building() -> tuple[OptimizedGraph, list[Keyframe], list[synthetic.SyntheticRobot]]:
    _, fleet = synthetic.two_robot_fleet()
    optimized, keyframes, _, _ = _run_pipeline(fleet)
    return optimized, keyframes, fleet


def test_robots_sharing_a_building_merge_into_one_component(shared_building) -> None:
    optimized, _, fleet = shared_building
    score = ev.score_components(optimized.components, synthetic.truth_groups(fleet))
    assert score.false_merge_rate == 0.0
    assert score.missed_merge_rate == 0.0
    assert len(optimized.components) == 1


def test_merged_fleet_renders_a_single_grid(shared_building) -> None:
    optimized, keyframes, _ = shared_building
    grids = render_occupancy(optimized, keyframes, RenderConfig())
    assert len(grids) == 1
    cells = next(iter(grids.values())).cells
    # The wire format the browser UI already renders; see adapters/protocol/README.md.
    assert cells.dtype == np.int8
    assert set(np.unique(cells).tolist()) <= {-1, 0, 100}


@pytest.mark.xfail(
    reason=(
        "KNOWN OPEN ISSUE, tracked deliberately rather than hidden: optimizing "
        "with real verify.py closures currently RAISES translation ATE by "
        "~1.2-1.4x instead of lowering it, consistently across injected drift "
        "levels from 0.012 to 0.15 m/m. Diagnosis so far, from bisecting the "
        "pipeline: graph.py is not at fault (exact ground-truth edges in gives "
        "ATE 0.0 out, to machine precision), and the closure TRANSFORMS are not "
        "at fault either (median 1.3 cm / 0.1 deg against ground truth). "
        "Substituting ground-truth transforms while keeping verify.py's "
        "information matrices still degrades ATE (1.20x), whereas keeping the "
        "real transforms under isotropic information IMPROVES it (0.93x). That "
        "isolates the cause to the information matrices: GICP's Hessian arrives "
        "with a ~30:1 rotation-to-translation ratio, which over-constrains "
        "orientation relative to position. Overall magnitude is not the lever "
        "-- sweeping info_scale over 1..100 never recovers an improvement, and "
        "at 100 every closure is rejected. Correct weighting depends on the "
        "real sensor noise model, so this is deliberately left for calibration "
        "against recorded hardware data with surveyed ground truth rather than "
        "tuned until the synthetic fixture goes green."
    ),
    strict=True,
)
def test_optimized_poses_beat_raw_odometry(shared_building) -> None:
    """The point of the whole stack, stated as a number.

    Compared against odometry expressed in the same frame as the estimate, so
    this measures the correction rather than the arbitrary choice of origin.

    ``strict=True`` matters: if a future change makes this pass, the suite fails
    until someone removes the xfail. A silently-fixed known issue is how a
    regression gets reintroduced later without anyone noticing it had been
    solved.
    """
    optimized, keyframes, fleet = shared_building
    truth = {kid: pose for robot in fleet for kid, pose in robot.truth.items()}
    odometry = {kf.id: kf.t_odom_base for kf in keyframes}

    before = ev.compute_ate(odometry, truth).translation_m.rmse
    after = ev.compute_ate(optimized.poses, truth).translation_m.rmse
    assert after < before


def test_optimization_does_not_catastrophically_diverge(shared_building) -> None:
    """The guard that must hold while the ATE issue above is open.

    Degrading ATE by tens of centimetres is a calibration problem. Degrading it
    by metres is a broken solver, and the two must not be allowed to look alike
    in the suite -- otherwise the open issue above silently absorbs a real
    regression.
    """
    optimized, keyframes, fleet = shared_building
    truth = {kid: pose for robot in fleet for kid, pose in robot.truth.items()}
    odometry = {kf.id: kf.t_odom_base for kf in keyframes}

    before = ev.compute_ate(odometry, truth).translation_m.rmse
    after = ev.compute_ate(optimized.poses, truth).translation_m.rmse
    assert after < before * 3.0, f"ATE degraded {after / before:.1f}x -- solver divergence, not miscalibration"


def test_true_correspondences_are_recovered_to_centimetres() -> None:
    """Stratified by ground-truth separation, which is the whole discipline here.

    Pooling true correspondences with known false positives and reporting one
    aggregate hides a working system inside an alarming average. The claim worth
    making is about the population that is supposed to be right.
    """
    _, fleet = synthetic.two_robot_fleet()
    alpha, beta = fleet
    index = ScanContextIndex(rings=20, sectors=60, temporal_window=5)
    for keyframe in alpha.keyframes:
        index.add(keyframe.id, scan_context_descriptor(keyframe.points))

    errors = []
    for keyframe in beta.keyframes:
        for candidate in index.query(scan_context_descriptor(keyframe.points), k=3, query_id=keyframe.id):
            target = next(k for k in alpha.keyframes if k.id == candidate.keyframe_id)
            if se3_distance(beta.truth[keyframe.id], alpha.truth[target.id])[0] >= SAME_PLACE_M:
                continue
            edge = verify_candidate(source=keyframe, target=target, yaw_prior=candidate.yaw)
            if edge is None:
                continue
            expected = se3_relative(beta.truth[edge.src], alpha.truth[edge.dst])
            errors.append(se3_distance(edge.t_src_dst, expected)[0])

    assert len(errors) >= 10, "fixture should produce a usable number of true correspondences"
    accurate = [e for e in errors if e < 0.30]
    # Not all of them: Scan Context aliases a corridor viewed from opposite ends,
    # which yields an occasional 180-degree flip. That is a real property of the
    # descriptor, and catching it is the pose graph's job, not this stage's.
    assert len(accurate) / len(errors) >= 0.8
    assert float(np.median(accurate)) < 0.05


def test_robots_in_different_buildings_are_never_merged() -> None:
    """The property that must never regress, at any cost in recall.

    A false merge places a robot confidently inside a building it has never
    entered, and nothing downstream can detect it from the map alone. A missed
    merge only forfeits collaboration. The two are not comparable, so this test
    admits no tolerance.
    """
    _, fleet = synthetic.disjoint_fleet()
    optimized, keyframes, _, inter_robot = _run_pipeline(fleet)

    assert inter_robot == 0, "a cross-building closure passed geometric verification"
    assert len(optimized.components) == 2
    assert [sorted(c.robots) for c in optimized.components] == [["alpha"], ["beta"]]

    score = ev.score_components(optimized.components, synthetic.truth_groups(fleet))
    assert score.false_merge_rate == 0.0


def test_unmerged_robots_render_to_separate_grids() -> None:
    """An unmerged robot must still appear on the operator's map, alone.

    Rendering nothing would be a silent disappearance; rendering both into one
    grid would be a fabricated relative transform. Separate grids is the only
    honest option.
    """
    _, fleet = synthetic.disjoint_fleet()
    optimized, keyframes, _, _ = _run_pipeline(fleet)
    grids = render_occupancy(optimized, keyframes, RenderConfig())

    assert len(grids) == 2
    for grid in grids.values():
        assert (grid.cells == 100).any(), "each component must render real occupied cells"
