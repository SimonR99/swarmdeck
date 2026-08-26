"""Turn a collaborative-SLAM run into numbers, so "works well" stops being a
matter of opinion.

This module is the scoring harness for the rest of the package: every other
module's claim of correctness is measured against ground truth here, not by
eyeballing a rendered map. It depends on nothing but :mod:`swarmdeck_slam.types`
and numpy, so it can be exercised against any solver's output the moment that
output is expressed as an :class:`~swarmdeck_slam.types.OptimizedGraph`.

Four metrics, on purpose kept separate rather than blended into one score,
because they are blind to different failures:

* **ATE** (:func:`compute_ate`) -- global consistency of a trajectory against
  ground truth, after removing the one rigid transform SLAM cannot observe on
  its own (there is no absolute frame). Dominated by the worst loop closure,
  or its absence.
* **RPE** (:func:`compute_rpe`) -- local consistency over a fixed keyframe
  span, with *no* alignment step, so it is blind to exactly the global offset
  ATE measures and sensitive to exactly the short-range drift ATE's alignment
  step would smooth over.
* **Inter-robot transform error** (:func:`inter_robot_transform_error`) -- the
  headline number for a collaborative system: how far the recovered
  ``T_world_map`` for each robot is from the true one. This is the metric
  behind the 11-16 m failure recorded in
  ``docs/architecture/collaborative-slam.md``.
* **Component correctness** (:func:`score_components`) -- did the system group
  robots that genuinely share a frame, and keep apart robots that do not.
  Reported as two separate rates, never averaged, because a false merge
  (silently overlaying two unrelated maps) is a categorically worse failure
  than a missed one (leaving two robots that could collaborate, un-merged).

Convention note for anyone writing this up: ATE and RPE here follow the
TUM RGB-D / Zhang & Scaramuzza benchmark family -- closed-form rigid (Umeyama
1991, restricted to SE(3): rotation and translation, scale fixed at 1) trajectory
alignment for ATE, and un-aligned fixed-delta relative motion for RPE.
"""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Hashable, Mapping, Sequence
from dataclasses import dataclass

import numpy as np

from swarmdeck_slam.types import (
    Component,
    KeyframeId,
    OptimizedGraph,
    se3_distance,
    se3_identity,
    se3_kabsch,
    se3_relative,
)

# --------------------------------------------------------------------------- #
# Shared error-statistics block
# --------------------------------------------------------------------------- #


@dataclass(frozen=True, slots=True)
class ErrorStats:
    """RMSE / mean / median / max of a scalar error over ``n`` samples.

    Four numbers because they are blind to different things: mean tracks
    typical performance, RMSE punishes an outlier a mean would average away,
    median stays robust to that same outlier when "typical" is what matters,
    and max is the number a safety case actually needs -- the worst this has
    ever been. Reporting only one invites cherry-picking, intentional or not.
    """

    rmse: float
    mean: float
    median: float
    max: float
    n: int

    @staticmethod
    def from_errors(errors: Sequence[float] | np.ndarray) -> "ErrorStats":
        values = np.asarray(errors, dtype=np.float64)
        if values.ndim != 1 or values.size == 0:
            raise ValueError(
                f"ErrorStats needs a non-empty 1-D array of errors, got shape {values.shape}"
            )
        if not np.all(np.isfinite(values)):
            raise ValueError(
                "errors contain non-finite values (NaN/inf); refusing to summarize a "
                "degenerate result as if it were a plausible number"
            )
        return ErrorStats(
            rmse=float(np.sqrt(np.mean(values**2))),
            mean=float(np.mean(values)),
            median=float(np.median(values)),
            max=float(np.max(values)),
            n=int(values.size),
        )

    def to_dict(self) -> dict[str, float | int]:
        return {"rmse": self.rmse, "mean": self.mean, "median": self.median, "max": self.max, "n": self.n}

# --------------------------------------------------------------------------- #
# Rigid alignment (Umeyama / Horn, no scale) and ATE
# --------------------------------------------------------------------------- #


def align_rigid(
    estimated: Mapping[KeyframeId, np.ndarray], truth: Mapping[KeyframeId, np.ndarray]
) -> tuple[np.ndarray, tuple[KeyframeId, ...]]:
    """Closed-form rigid alignment ``T_truth_estimated`` (Umeyama 1991, no scale).

    Umeyama's algorithm generalizes Horn's absolute-orientation solution to
    also fit a scale factor. This implementation deliberately never computes
    one: lidar SLAM is metric, so a scale error is a real error in the map,
    and an alignment step that can absorb it would report a perfect
    trajectory for a map that is, say, 5% too big -- the single most
    damaging silent bug this module could ship with. What's left is exactly
    Kabsch's algorithm: the best-fit rotation and translation between two
    point sets, computed via SVD of their cross-covariance so it stays exact
    (no NaN, no divide-by-zero) even when the points are rank-deficient
    (collinear or coincident), which is a real trajectory shape (e.g. a robot
    that starts every keyframe near a straight corridor).

    Only the *translation* components of the poses are used to solve for the
    alignment -- this is what "Umeyama/Horn alignment" means in the
    trajectory-evaluation literature (TUM RGB-D, evo, KITTI): it is an
    alignment of point sets, not of full poses. The recovered rotation is
    then applied to every pose (translation and orientation alike) by
    :func:`compute_ate`.
    """
    common = tuple(sorted(set(estimated) & set(truth), key=lambda k: (k.robot_id, k.seq)))
    if len(common) < 2:
        raise ValueError(
            "rigid alignment needs at least 2 common keyframes to determine a rotation; "
            f"got {len(common)} in common (estimated has {len(estimated)}, truth has {len(truth)})"
        )
    est_pos = np.array([estimated[k][:3, 3] for k in common], dtype=np.float64)
    true_pos = np.array([truth[k][:3, 3] for k in common], dtype=np.float64)
    if not (np.all(np.isfinite(est_pos)) and np.all(np.isfinite(true_pos))):
        raise ValueError("cannot align: input poses contain non-finite translations")

    # The Kabsch fit itself lives in types.py, because graph.py needs exactly
    # the same computation to estimate each robot's T_world_map (see
    # GtsamPoseGraph._t_world_map). Two copies of a rigid fit is two places for
    # a reflection-correction bug to hide, and this module and that one are
    # scored against each other.
    return se3_kabsch(est_pos, true_pos), common


@dataclass(frozen=True, slots=True)
class AteResult:
    """Absolute trajectory error: residual after the best-fit rigid alignment.

    Blind to a global rigid offset by design -- that offset is not something a
    SLAM system observes from relative measurements alone, so charging it
    against the estimate would penalize an unobservable gauge choice rather
    than a real error. What survives alignment is real: loop closures that
    didn't fire, or fired against the wrong place.
    """

    translation_m: ErrorStats
    rotation_rad: ErrorStats
    alignment: np.ndarray  # T_truth_estimated used, kept for debugging/plotting only
    n_poses: int

    def to_dict(self) -> dict[str, object]:
        return {
            "translation_m": self.translation_m.to_dict(),
            "rotation_rad": self.rotation_rad.to_dict(),
            "n_poses": self.n_poses,
        }


def compute_ate(
    estimated: Mapping[KeyframeId, np.ndarray], truth: Mapping[KeyframeId, np.ndarray]
) -> AteResult:
    """Absolute trajectory error between two pose sets sharing keyframe ids.

    Callers choose the scope: pass one robot's own poses to ask "is this
    robot's own trajectory self-consistent", or every robot in one merged
    :class:`~swarmdeck_slam.types.Component` to ask "does the collaborative
    back end agree with itself". Mixing robots from different components would
    silently assume a shared frame that the system never established, which
    is exactly the kind of confident lie this package exists to avoid --
    :func:`evaluate` never does this.
    """
    alignment, common = align_rigid(estimated, truth)
    translation_errors = np.empty(len(common), dtype=np.float64)
    rotation_errors = np.empty(len(common), dtype=np.float64)
    for i, keyframe_id in enumerate(common):
        aligned = alignment @ estimated[keyframe_id]
        translation_errors[i], rotation_errors[i] = se3_distance(truth[keyframe_id], aligned)
    return AteResult(
        translation_m=ErrorStats.from_errors(translation_errors),
        rotation_rad=ErrorStats.from_errors(rotation_errors),
        alignment=alignment,
        n_poses=len(common),
    )


# --------------------------------------------------------------------------- #
# RPE
# --------------------------------------------------------------------------- #


@dataclass(frozen=True, slots=True)
class RpeResult:
    """Relative pose error over a fixed keyframe span, with no alignment step.

    A rigid misalignment cancels out of a *relative* motion by construction
    (``inv(T_i) @ T_j`` does not care where the world frame's origin is), so
    RPE isolates local consistency from the global drift ATE measures. It is
    blind to a slow bias that every delta-step agrees on but which is
    nonetheless wrong in the world frame -- that is what ATE is for. Scoped to
    one robot's own sequential keyframes: relative motion is only meaningfully
    comparable to ground truth when both sides reference the same physical
    span, which cross-robot pairs cannot guarantee unless the robots are
    already known to share a frame (see :func:`score_components`).
    """

    delta: int
    translation_m: ErrorStats
    rotation_rad: ErrorStats

    def to_dict(self) -> dict[str, object]:
        return {
            "delta": self.delta,
            "translation_m": self.translation_m.to_dict(),
            "rotation_rad": self.rotation_rad.to_dict(),
        }


def compute_rpe(
    estimated: Mapping[KeyframeId, np.ndarray],
    truth: Mapping[KeyframeId, np.ndarray],
    *,
    delta: int = 1,
) -> RpeResult:
    """RPE for one robot's trajectory (or several, aggregated) at a fixed span.

    ``delta`` is a count of keyframes, not seconds or metres: with keyframes
    emitted at roughly regular spacing this is the usual "drift per step"
    reading, and staying index-based rather than time-based means the metric
    keeps working even if a robot's clock or keyframe cadence is irregular.
    Ids are grouped by ``robot_id`` and matched within that robot only (see
    :class:`RpeResult`); pairs are taken between keyframes that are ``delta``
    apart in each robot's own sorted-by-``seq`` sequence, so a keyframe
    missing from one side (e.g. rejected by the optimizer) shifts later pairs
    rather than silently comparing the wrong two keyframes.
    """
    if delta < 1:
        raise ValueError(f"delta must be >= 1 keyframe, got {delta}")

    common = set(estimated) & set(truth)
    by_robot: dict[str, list[KeyframeId]] = defaultdict(list)
    for keyframe_id in common:
        by_robot[keyframe_id.robot_id].append(keyframe_id)

    translation_errors: list[float] = []
    rotation_errors: list[float] = []
    for ids in by_robot.values():
        ids.sort(key=lambda k: k.seq)
        for i in range(len(ids) - delta):
            a, b = ids[i], ids[i + delta]
            rel_true = se3_relative(truth[a], truth[b])
            rel_est = se3_relative(estimated[a], estimated[b])
            translation_error, rotation_error = se3_distance(rel_true, rel_est)
            translation_errors.append(translation_error)
            rotation_errors.append(rotation_error)

    if not translation_errors:
        raise ValueError(
            f"no keyframe pairs at delta={delta}: every robot has fewer than {delta + 1} "
            "keyframes in common between estimate and ground truth"
        )
    return RpeResult(
        delta=delta,
        translation_m=ErrorStats.from_errors(translation_errors),
        rotation_rad=ErrorStats.from_errors(rotation_errors),
    )


# --------------------------------------------------------------------------- #
# Inter-robot transform error
# --------------------------------------------------------------------------- #


@dataclass(frozen=True, slots=True)
class InterRobotTransformError:
    """Error in one robot's recovered ``T_world_map`` -- the headline number.

    Everything else in this module can be locally excellent while this is 15 m
    off: a plausible per-robot trajectory fused into the wrong place in the
    fleet frame. That failure already happened once in this repository (see
    ``docs/architecture/collaborative-slam.md``, 11-16 m against a grid
    registration baseline of 0.03-0.20 m), which is why it gets its own metric
    rather than being folded into ATE.
    """

    translation_m: float
    rotation_deg: float

    def to_dict(self) -> dict[str, float]:
        return {"translation_m": self.translation_m, "rotation_deg": self.rotation_deg}


def inter_robot_transform_error(
    estimated: Mapping[str, np.ndarray], truth: Mapping[str, np.ndarray]
) -> dict[str, InterRobotTransformError]:
    """Per-robot error between recovered and true ``T_world_map``.

    Degrees rather than radians on purpose: this number is meant for a table
    or a lab notebook, not further arithmetic, and degrees are what a reader
    can sanity-check against "is the map upside down" at a glance.

    A robot present in ``truth`` but absent from ``estimated`` (never merged
    into any component, so it has no recovered ``T_world_map``) is simply
    absent from the result rather than scored as infinite error -- that
    failure is a *component* failure, and :func:`score_components` is where it
    is counted, so it is not double-charged here.

    CALLER CONTRACT: both frames must already be in the SAME gauge. This
    compares them directly, with no alignment step, because the synthetic
    fixture builds truth in the gauge the solver anchors in. Real ground truth
    is not: it arrives in world coordinates while the graph anchors on one
    keyframe, so every robot picks up the same rigid offset and this reports
    it as per-robot error. ``tools/replay.py`` pre-multiplies truth by the
    inverse of a joint ``align_rigid`` for exactly this reason -- uncorrected,
    it read ~8.8 m for two robots that were within 0.2 m of each other.

    KNOWN BLIND SPOT, and the reason not to tune against this number alone.
    ``T_world_map`` is a single rigid frame fitted to a whole trajectory
    (:meth:`~swarmdeck_slam.graph.GtsamPoseGraph._t_world_map`). When a change
    improves the trajectory by altering its SHAPE rather than sliding it, the
    best-fit frame moves as well, and its distance from truth's own best-fit
    frame can grow while every individual pose gets closer to ground truth.
    That is not hypothetical: enabling ``CollaborativeBackend``'s registration
    prior worsens this metric (0.2342 -> 0.3146 m) while cross-robot relative
    POSE error improves 1.9313 -> 1.2465 m on the same run. When the two
    disagree, believe the pose-level measurement -- :func:`compute_ate`
    aligns internally and is immune, and a direct cross-robot relative-pose
    comparison routes through no summary at all.
    """
    common = sorted(set(estimated) & set(truth))
    if not common:
        raise ValueError(
            "no robots in common between estimated and true t_world_map "
            f"(estimated: {sorted(estimated)}, truth: {sorted(truth)})"
        )
    result: dict[str, InterRobotTransformError] = {}
    for robot_id in common:
        translation_m, rotation_rad = se3_distance(truth[robot_id], estimated[robot_id])
        result[robot_id] = InterRobotTransformError(
            translation_m=translation_m, rotation_deg=float(np.degrees(rotation_rad))
        )
    return result


# --------------------------------------------------------------------------- #
# Component correctness
# --------------------------------------------------------------------------- #


@dataclass(frozen=True, slots=True)
class ComponentScore:
    """Confusion matrix over robot *pairs*, kept as two separate rates.

    A false merge overlays two maps that do not actually share a frame:
    reported obstacles and free space become simply wrong, silently, with
    nothing downstream able to tell -- see
    :meth:`~swarmdeck_slam.types.OptimizedGraph.share_frame`, whose entire
    purpose is to prevent exactly this. A missed merge only costs the
    (real, but recoverable) benefit of collaboration. Averaging the two rates
    into one score would let a system trade a much worse failure for a
    cheaper one and still look unchanged, which is why they are reported, and
    must be read, separately.
    """

    false_merges: tuple[tuple[str, str], ...]
    missed_merges: tuple[tuple[str, str], ...]
    n_true_positive_pairs: int
    n_true_negative_pairs: int

    @property
    def false_merge_rate(self) -> float:
        """Fraction of genuinely-separate pairs wrongly overlaid; 0 if none existed to get wrong."""
        if self.n_true_negative_pairs == 0:
            return 0.0
        return len(self.false_merges) / self.n_true_negative_pairs

    @property
    def missed_merge_rate(self) -> float:
        """Fraction of genuinely-shared pairs left unmerged; 0 if none existed to find."""
        if self.n_true_positive_pairs == 0:
            return 0.0
        return len(self.missed_merges) / self.n_true_positive_pairs

    @property
    def is_perfect(self) -> bool:
        return not self.false_merges and not self.missed_merges

    def to_dict(self) -> dict[str, object]:
        return {
            "false_merges": [list(pair) for pair in self.false_merges],
            "missed_merges": [list(pair) for pair in self.missed_merges],
            "false_merge_rate": self.false_merge_rate,
            "missed_merge_rate": self.missed_merge_rate,
            "n_true_positive_pairs": self.n_true_positive_pairs,
            "n_true_negative_pairs": self.n_true_negative_pairs,
        }


def score_components(
    estimated: Sequence[Component], truth_groups: Mapping[str, Hashable]
) -> ComponentScore:
    """Score a set of recovered components against which robots truly share a frame.

    ``truth_groups`` maps each robot id to any hashable group key; two robots
    are "truly shared" iff their keys compare equal (a scene id, a building
    id, whatever the ground-truth source uses -- this function does not care).
    A robot present in ``truth_groups`` but absent from every estimated
    :class:`~swarmdeck_slam.types.Component` is treated as its own singleton
    component, which is the correct reading of
    :meth:`~swarmdeck_slam.types.OptimizedGraph.component_of` returning
    ``None``: no verified relative transform to anyone.
    """
    robots = sorted(truth_groups)
    if len(robots) < 2:
        raise ValueError(f"component scoring needs at least 2 robots to form a pair, got {len(robots)}")

    estimated_group: dict[str, int] = {}
    for component in estimated:
        for robot_id in component.robots:
            estimated_group[robot_id] = component.component_id

    false_merges: list[tuple[str, str]] = []
    missed_merges: list[tuple[str, str]] = []
    n_true_positive_pairs = 0
    n_true_negative_pairs = 0
    for i, a in enumerate(robots):
        for b in robots[i + 1 :]:
            truly_shared = truth_groups[a] == truth_groups[b]
            estimated_shared = (
                a in estimated_group and b in estimated_group and estimated_group[a] == estimated_group[b]
            )
            if truly_shared:
                n_true_positive_pairs += 1
                if not estimated_shared:
                    missed_merges.append((a, b))
            else:
                n_true_negative_pairs += 1
                if estimated_shared:
                    false_merges.append((a, b))

    return ComponentScore(
        false_merges=tuple(false_merges),
        missed_merges=tuple(missed_merges),
        n_true_positive_pairs=n_true_positive_pairs,
        n_true_negative_pairs=n_true_negative_pairs,
    )


# --------------------------------------------------------------------------- #
# Report
# --------------------------------------------------------------------------- #


@dataclass(frozen=True, slots=True)
class Report:
    """Everything this harness knows about one run, in one pasteable object.

    ``ate`` and ``rpe`` are keyed by *scope* -- a robot id for that robot's own
    trajectory, or ``"component:<id>"`` for a merged group's joint
    consistency -- because "is this robot's own front end any good" and "does
    the collaborative back end agree with itself" are different questions
    that the ablation ladder (independent maps -> shared allocation ->
    collaborative correction) needs answered separately; collapsing them would
    hide which stage of the ladder is responsible for a regression.
    """

    label: str
    ate: dict[str, AteResult]
    rpe: dict[str, tuple[RpeResult, ...]]
    inter_robot: dict[str, InterRobotTransformError]
    components: ComponentScore

    def format(self) -> str:
        lines = [f"=== {self.label} ==="]

        lines.append("-- ATE (rigid-aligned, no scale) --")
        if not self.ate:
            lines.append("  (no scope had enough common keyframes to align)")
        for scope in sorted(self.ate):
            ate = self.ate[scope]
            lines.append(
                f"  {scope:<16} trans[m]  rmse={ate.translation_m.rmse:.4f} "
                f"mean={ate.translation_m.mean:.4f} median={ate.translation_m.median:.4f} "
                f"max={ate.translation_m.max:.4f}  (n={ate.n_poses})"
            )
            lines.append(
                f"  {'':<16} rot[deg]  rmse={np.degrees(ate.rotation_rad.rmse):.4f} "
                f"mean={np.degrees(ate.rotation_rad.mean):.4f} "
                f"median={np.degrees(ate.rotation_rad.median):.4f} "
                f"max={np.degrees(ate.rotation_rad.max):.4f}"
            )

        lines.append("-- RPE (no alignment; per robot) --")
        if not self.rpe:
            lines.append("  (no robot had enough common keyframes at any requested delta)")
        for scope in sorted(self.rpe):
            for rpe in self.rpe[scope]:
                lines.append(
                    f"  {scope:<16} delta={rpe.delta:<3} "
                    f"trans[m] rmse={rpe.translation_m.rmse:.4f} max={rpe.translation_m.max:.4f}  "
                    f"rot[deg] rmse={np.degrees(rpe.rotation_rad.rmse):.4f} "
                    f"max={np.degrees(rpe.rotation_rad.max):.4f}"
                )

        lines.append("-- Inter-robot T_world_map error --")
        if not self.inter_robot:
            lines.append("  (no robot had a recovered T_world_map matched to ground truth)")
        for robot_id in sorted(self.inter_robot):
            err = self.inter_robot[robot_id]
            lines.append(f"  {robot_id:<16} {err.translation_m:.4f} m, {err.rotation_deg:.4f} deg")

        lines.append("-- Component correctness --")
        lines.append(
            f"  false merges:  {len(self.components.false_merges)}/{self.components.n_true_negative_pairs} "
            f"(rate={self.components.false_merge_rate:.3f})  <- worse failure mode"
        )
        lines.append(
            f"  missed merges: {len(self.components.missed_merges)}/{self.components.n_true_positive_pairs} "
            f"(rate={self.components.missed_merge_rate:.3f})"
        )
        if self.components.false_merges:
            lines.append(f"  false-merge pairs:  {list(self.components.false_merges)}")
        if self.components.missed_merges:
            lines.append(f"  missed-merge pairs: {list(self.components.missed_merges)}")

        return "\n".join(lines)

    def __str__(self) -> str:
        return self.format()

    def to_dict(self) -> dict[str, object]:
        return {
            "label": self.label,
            "ate": {scope: ate.to_dict() for scope, ate in self.ate.items()},
            "rpe": {scope: [r.to_dict() for r in results] for scope, results in self.rpe.items()},
            "inter_robot": {robot_id: err.to_dict() for robot_id, err in self.inter_robot.items()},
            "components": self.components.to_dict(),
        }


def evaluate(
    label: str,
    graph: OptimizedGraph,
    truth_poses: Mapping[KeyframeId, np.ndarray],
    truth_t_world_map: Mapping[str, np.ndarray],
    truth_groups: Mapping[str, Hashable],
    *,
    rpe_deltas: Sequence[int] = (1,),
) -> Report:
    """Score one :class:`~swarmdeck_slam.types.OptimizedGraph` in a single call.

    The intended entry point for the other modules in this package: build an
    ``OptimizedGraph`` however you like (that is exactly ``graph.py``'s solver
    output type) and hand it here alongside ground truth from
    ``tests.synthetic.SyntheticRobot`` -- ``.truth`` merged across robots for
    ``truth_poses``, ``.t_world_map_true`` per robot for ``truth_t_world_map``
    -- plus a robot -> group-id mapping recording which robots the *scene*
    actually puts in one frame, for ``truth_groups`` (e.g. every robot from one
    call to ``two_robot_fleet`` shares a scene, so shares a group id).

    ATE and RPE are computed per robot (that robot's own keyframes against its
    own ground truth) and, for every component with 2+ robots, once more
    jointly across the whole component -- see :class:`Report`. A scope is
    simply omitted, never zeroed or NaN'd, when it has fewer than 2 common
    keyframes to work with; call :func:`compute_ate` / :func:`compute_rpe`
    directly if you want a hard failure on that condition instead.
    """
    robot_ids = sorted({keyframe_id.robot_id for keyframe_id in graph.poses} | set(truth_t_world_map))

    def _poses_for(
        robot_ids_subset: Sequence[str],
    ) -> tuple[dict[KeyframeId, np.ndarray], dict[KeyframeId, np.ndarray]]:
        wanted = set(robot_ids_subset)
        est = {k: v for k, v in graph.poses.items() if k.robot_id in wanted}
        true = {k: v for k, v in truth_poses.items() if k.robot_id in wanted}
        return est, true

    ate: dict[str, AteResult] = {}
    rpe: dict[str, tuple[RpeResult, ...]] = {}
    for robot_id in robot_ids:
        est, true = _poses_for([robot_id])
        if len(set(est) & set(true)) >= 2:
            ate[robot_id] = compute_ate(est, true)

        results: list[RpeResult] = []
        for delta in rpe_deltas:
            try:
                results.append(compute_rpe(est, true, delta=delta))
            except ValueError:
                continue
        if results:
            rpe[robot_id] = tuple(results)

    for component in graph.components:
        if len(component.robots) < 2:
            continue
        est, true = _poses_for(sorted(component.robots))
        if len(set(est) & set(true)) >= 2:
            ate[f"component:{component.component_id}"] = compute_ate(est, true)

    inter_robot = inter_robot_transform_error(graph.t_world_map, truth_t_world_map)
    components = score_components(graph.components, truth_groups)

    return Report(label=label, ate=ate, rpe=rpe, inter_robot=inter_robot, components=components)


# --------------------------------------------------------------------------- #
# Ablation
# --------------------------------------------------------------------------- #


@dataclass(frozen=True, slots=True)
class Ablation:
    """Labelled :class:`Report` runs, tabulated side by side.

    The research plan this harness serves is an ablation ladder -- independent
    maps, then shared allocation, then collaborative correction -- so
    comparing labelled runs is the normal use of this module, not an
    afterthought bolted on for convenience.
    """

    reports: tuple[Report, ...]

    def __post_init__(self) -> None:
        if not self.reports:
            raise ValueError("an ablation needs at least one report")
        labels = [r.label for r in self.reports]
        if len(labels) != len(set(labels)):
            raise ValueError(f"ablation labels must be unique, got {labels}")

    def format(self) -> str:
        labels = [r.label for r in self.reports]
        lines = ["Ablation: " + " | ".join(labels)]

        all_scopes = sorted({scope for r in self.reports for scope in r.ate})
        if all_scopes:
            lines.append("-- ATE translation RMSE [m] --")
            lines.append(f"  {'scope':<16}" + "".join(f"{lbl:>16}" for lbl in labels))
            for scope in all_scopes:
                row = f"  {scope:<16}"
                for r in self.reports:
                    row += (
                        f"{r.ate[scope].translation_m.rmse:>16.4f}" if scope in r.ate else f"{'--':>16}"
                    )
                lines.append(row)

        all_robots = sorted({robot for r in self.reports for robot in r.inter_robot})
        if all_robots:
            lines.append("-- Inter-robot T_world_map translation error [m] --")
            lines.append(f"  {'robot':<16}" + "".join(f"{lbl:>16}" for lbl in labels))
            for robot_id in all_robots:
                row = f"  {robot_id:<16}"
                for r in self.reports:
                    row += (
                        f"{r.inter_robot[robot_id].translation_m:>16.4f}"
                        if robot_id in r.inter_robot
                        else f"{'--':>16}"
                    )
                lines.append(row)

        lines.append("-- Component correctness (count false / missed merges) --")
        lines.append(f"  {'':<16}" + "".join(f"{lbl:>16}" for lbl in labels))
        lines.append(
            f"  {'false merges':<16}"
            + "".join(f"{len(r.components.false_merges):>16}" for r in self.reports)
        )
        lines.append(
            f"  {'missed merges':<16}"
            + "".join(f"{len(r.components.missed_merges):>16}" for r in self.reports)
        )
        return "\n".join(lines)

    def __str__(self) -> str:
        return self.format()

    def to_dict(self) -> dict[str, object]:
        return {"reports": [r.to_dict() for r in self.reports]}
