"""``GtsamPoseGraph``: the collaborative pose-graph optimizer.

This is the module that replaces occupancy-grid stitching
(``server/swarmdeck_server/mapsvc/registration.py``) with trajectory
optimization. See ``docs/architecture/collaborative-slam.md`` for why the grid
approach failed: it re-registered whole grids pairwise against one reference,
with no loop-consistency check and no feedback into either robot's own
trajectory. This module optimizes one joint graph of keyframe poses instead,
and every occupancy grid downstream is rendered from its output -- there is no
grid registration anywhere in this design.

Two-layer defense against bad inter-robot loop closures
---------------------------------------------------------
A wrong inter-robot closure is the one input that can destroy the whole
fleet map: it is usually *locally self-consistent* (a real registration
result, just against the wrong place), so nothing about the edge itself looks
wrong. Two independent checks run in sequence:

1. **PCM** (:func:`GtsamPoseGraph._pcm_filter`) checks inter-robot closures
   for pairwise agreement *before* they ever reach the solver: two closures
   between the same pair of robots are consistent only if composing them
   around the loop (through each robot's own odometry) returns near-identity.
   This is the primary defense, because it is the only one of the two that
   can catch an outlier that fits its own local neighbourhood perfectly --
   GNC below cannot, since GNC judges factors by residual against the rest of
   the graph, and a self-consistent outlier has a small residual by
   definition. PCM's blind spot is the mirror case: two *independently wrong*
   closures that happen to agree with each other. No consistency check can
   tell "agreement" from "correctness" apart.

2. **GNC** (``gtsam.GncLMOptimizer``, Graduated Non-Convexity with a
   truncated-least-squares loss) runs over everything that survives PCM, plus
   all intra-robot loop closures (which PCM never sees -- there is no second
   robot to cross-check against). It catches the closures that are
   individually plausible but a poor fit against the *rest* of the optimized
   graph, which is exactly what PCM's pairwise, odometry-only check cannot
   see. ``ODOMETRY`` and the anchor priors this class adds are registered as
   GNC "known inliers", which forces their weight to 1 regardless of residual
   -- rejecting either would disconnect or gauge-free the graph, and neither
   kind can be a mismatch in the way a loop closure can.

Components and gauge freedom
-----------------------------
Connectivity (which robots share a frame at all) is decided once, by PCM,
before the solver runs -- see :func:`GtsamPoseGraph._components`. GNC's later
down-weighting of an individual edge changes how much that edge influences
the *solution*; it never retroactively un-merges a component. This keeps
anchor placement (which must happen before optimization, since it fixes
gauge) well-defined: exactly one prior per PCM-derived component, on a
deterministically chosen anchor keyframe. A component that PCM never
connects to another gets its own anchor and is reported as its own
:class:`~swarmdeck_slam.types.Component`, never overlaid onto another robot's
frame on assumption.

Scalability
-----------
This class rebuilds the full factor graph and reruns GNC + a Levenberg-
Marquardt refit on every :meth:`optimize` call ("batch"). That is the
documented, deliberate limit for now: :meth:`optimize` wall time grows with
graph size (measured in ``tests/test_graph.py::test_optimize_scaling`` --
see that test for the current numbers), because GNC reruns LM from scratch at
every one of its outer mu-steps rather than reusing the previous
factorization. Nothing about the public interface assumes batch operation:
``add_keyframe``/``add_edge`` only ever append to internal state, and
:meth:`optimize` is the sole place that touches ``gtsam``. An incremental
backend (``gtsam.ISAM2``) would replace the body of :meth:`optimize` --
specifically ``_build_factors`` and the GNC/refit calls -- with an
``update()`` call per new factor batch, without changing any caller-visible
type. The natural point to stop using batch GNC+LM is somewhere past the
scale where a re-optimization needs to complete faster than new keyframes
arrive; the timing test gives the current measured numbers for this backend
rather than a guess.
"""

from __future__ import annotations

from collections import defaultdict, deque
from typing import Final

import gtsam
import numpy as np
from scipy.stats import chi2

from swarmdeck_slam.types import (
    Component,
    Edge,
    EdgeKind,
    Keyframe,
    KeyframeId,
    KeyRegistry,
    OptimizedGraph,
    se3_identity,
    se3_inverse,
    se3_relative,
)

# Gauge-fixing prior: pins one keyframe per component so the solver has
# something to measure every other pose relative to. The sigmas are small
# enough to be effectively "fixed" against any real sensor uncertainty this
# package handles (odometry and ICP-derived closures are never sub-millimetre
# confident), but nonzero -- an exactly-zero-sigma (Constrained) noise model
# is unnecessary here and would make the anchor factor a special case the GNC
# and LM stages both have to handle differently from every other factor.
_ANCHOR_SIGMAS: Final = np.array([1e-3] * 6)

# gtsam's GNC reports weights that land close to 0 or close to 1 in practice
# (a TLS loss is explicitly designed to be nearly binary once mu has
# graduated) -- this is a robust midpoint, not a tuned threshold.
_DEFAULT_GNC_WEIGHT_THRESHOLD: Final = 0.5

# SE(3) tangent space is 6-dimensional; the PCM consistency test below treats
# a composed loop's residual as a single chi-squared statistic over all 6.
_PCM_DOF: Final = 6


def _canonical_pair(edge: Edge) -> frozenset[str]:
    """Group key for two candidate closures spanning the same pair of robots."""
    return frozenset((edge.src.robot_id, edge.dst.robot_id))


def _orient(edge: Edge, first_robot: str) -> tuple[KeyframeId, KeyframeId, np.ndarray]:
    """Re-express ``edge`` as ``(first_kf, second_kf, T_first_second)`` with
    ``first_kf`` on ``first_robot``, whichever side of the edge that was.

    PCM has to compare two closures that may have been discovered with either
    robot as ``src``; without this, every composition below would need a case
    split on which endpoint belongs to which robot.
    """
    if edge.src.robot_id == first_robot:
        return edge.src, edge.dst, edge.t_src_dst
    return edge.dst, edge.src, se3_inverse(edge.t_src_dst)


def _loop_residual(
    edge_i: Edge, edge_j: Edge, keyframes: dict[KeyframeId, Keyframe]
) -> tuple[np.ndarray, np.ndarray]:
    """Compose the four-hop loop ``src_i -> src_j -> dst_j -> dst_i -> src_i``
    formed by two candidate closures over the same robot pair, using each
    robot's own odometry to bridge between its two keyframes.

    Returns ``(residual_tangent, covariance)`` where ``residual_tangent`` is
    the composed loop's ``gtsam.Pose3.Logmap`` (rotation-first, matching
    :data:`~swarmdeck_slam.types.TANGENT_ORDER`) and should be near zero if
    the two closures agree, and ``covariance`` is the sum of the two edges'
    inverted information matrices.

    Two approximations, both standard for a lightweight PCM pre-filter and
    both conservative in the direction of over-rejecting rather than
    under-rejecting: the odometry hops are treated as exact (no covariance
    contribution), since within one robot's short-baseline continuous odom
    frame they are far more precise than the inter-robot closures being
    checked; and the two closures' covariances are simply summed rather than
    propagated through the loop via each transform's SE(3) adjoint. A
    from-scratch PCM implementation that needs tighter bounds should replace
    this with adjoint-propagated covariance (Mangelson et al., 2018).
    """
    robot_a = edge_i.src.robot_id
    a_i, b_i, t_a_b_i = _orient(edge_i, robot_a)
    a_j, b_j, t_a_b_j = _orient(edge_j, robot_a)

    t_a_i_a_j = se3_relative(keyframes[a_i].t_odom_base, keyframes[a_j].t_odom_base)
    t_b_j_b_i = se3_relative(keyframes[b_j].t_odom_base, keyframes[b_i].t_odom_base)

    loop = se3_inverse(t_a_b_i) @ t_a_i_a_j @ t_a_b_j @ t_b_j_b_i
    residual = gtsam.Pose3.Logmap(gtsam.Pose3(loop))
    covariance = np.linalg.inv(edge_i.information) + np.linalg.inv(edge_j.information)
    return residual, covariance


def _consistent(residual: np.ndarray, covariance: np.ndarray, confidence: float) -> bool:
    mahalanobis_sq = float(residual @ np.linalg.solve(covariance, residual))
    return mahalanobis_sq <= chi2.ppf(confidence, df=_PCM_DOF)


def _greedy_max_clique(nodes: list[int], adjacency: dict[int, set[int]]) -> list[int]:
    """Grow one clique from the highest-degree vertex, restricting the
    candidate pool to common neighbours at each step.

    Finding a true maximum clique is NP-hard; this is the standard O(n^2)
    greedy substitute (the candidate counts here -- closures between one pair
    of robots -- are small enough that the difference from exact almost never
    matters in practice). Its failure mode is one-directional and safe:
    starting from a different vertex can sometimes grow into a strictly
    larger clique that this single pass never finds, so it can under-accept
    -- reject a geometrically consistent closure that exact max-clique would
    have kept. It cannot over-accept relative to the consistency graph it was
    given: every vertex kept is pairwise consistent with every other vertex
    kept, by construction of the adjacency. What it (and exact max-clique)
    cannot do is protect against two independently wrong closures that
    happen to be pairwise consistent -- see the module docstring.
    """
    remaining = set(nodes)
    clique: list[int] = []
    while remaining:
        best = max(remaining, key=lambda n: (len(adjacency[n] & remaining), -n))
        clique.append(best)
        remaining = adjacency[best] & remaining
    return clique


class GtsamPoseGraph:
    """Batch GNC+LM implementation of ``PoseGraphOptimizer`` (see ``types.py``).

    Deterministic and side-effect free beyond its own accumulated state: no
    RNG is used anywhere in this class, so the same sequence of
    ``add_keyframe``/``add_edge`` calls always produces the same
    :class:`~swarmdeck_slam.types.OptimizedGraph`.
    """

    def __init__(
        self,
        *,
        pcm_confidence: float = 0.99,
        min_pcm_clique_size: int = 2,
        gnc_weight_threshold: float = _DEFAULT_GNC_WEIGHT_THRESHOLD,
    ) -> None:
        """
        Args:
            pcm_confidence: chi-squared confidence level (0, 1) for the PCM
                pairwise-consistency test. Higher accepts more (looser).
            min_pcm_clique_size: an inter-robot closure with fewer than this
                many mutually consistent partners is rejected outright, even
                if it is internally consistent with itself. The default of 2
                means a closure needs at least one independent corroborating
                closure between the same two robots before it can merge
                their frames -- a single closure, however confident, has no
                cross-check available and is exactly the failure mode PCM
                exists to catch (see module docstring). Set to 1 to accept
                uncorroborated closures when overlap is known to be sparse.
            gnc_weight_threshold: minimum final GNC weight for a loop-closure
                factor to be kept in the Levenberg-Marquardt refit.
        """
        self._keys = KeyRegistry()
        self._keyframes: dict[KeyframeId, Keyframe] = {}
        self._edges: list[Edge] = []
        self._pcm_confidence = pcm_confidence
        self._min_pcm_clique_size = min_pcm_clique_size
        self._gnc_weight_threshold = gnc_weight_threshold
        self._anchor_noise = gtsam.noiseModel.Diagonal.Sigmas(_ANCHOR_SIGMAS)
        self._lm_params = gtsam.LevenbergMarquardtParams()

    # ------------------------------------------------------------------ #
    # PoseGraphOptimizer protocol
    # ------------------------------------------------------------------ #

    def add_keyframe(self, keyframe: Keyframe) -> None:
        self._keyframes[keyframe.id] = keyframe
        self._keys.key(keyframe.id)  # assign its integer key eagerly and deterministically

    def add_edge(self, edge: Edge) -> None:
        self._edges.append(edge)

    def optimize(self) -> OptimizedGraph:
        if not self._keyframes:
            return OptimizedGraph()

        odometry = [e for e in self._edges if e.kind is EdgeKind.ODOMETRY]
        intra_loop = [e for e in self._edges if e.kind is EdgeKind.INTRA_LOOP]
        inter_loop = [e for e in self._edges if e.kind is EdgeKind.INTER_LOOP]
        self._validate_endpoints(odometry + intra_loop + inter_loop)

        pcm_accepted, pcm_rejected = self._pcm_filter(inter_loop)
        components = self._components(pcm_accepted)
        init_t_world_odom = self._initial_t_world_odom(components, pcm_accepted)

        graph, initial, known_inliers, loop_factors = self._build_factors(
            odometry, intra_loop, pcm_accepted, components, init_t_world_odom
        )

        gnc_params = gtsam.GncLMParams()
        gnc_params.setKnownInliers(known_inliers)
        gnc_params.setLossType(gtsam.GncLossType.TLS)
        gnc_optimizer = gtsam.GncLMOptimizer(graph, initial, gnc_params)
        gnc_result = gnc_optimizer.optimize()
        weights = gnc_optimizer.getWeights()

        # Stage 2: refit with plain LM over exactly the factors GNC trusted.
        # gtsam.GncLMOptimizer exposes neither iterations() nor a clean
        # error() (graph.error() over the GNC result still counts the full,
        # non-robustified residual of factors GNC only down-weighted, which
        # is enormous and not a meaningful "final error" -- verified against
        # a synthetic outlier while building this module). Excluding rejected
        # factors and re-solving gives both a real iteration/error account
        # and a solution no longer influenced, even slightly, by a
        # distrusted factor's residual pull.
        refit_graph = gtsam.NonlinearFactorGraph()
        for idx in known_inliers:
            refit_graph.add(graph.at(idx))
        gnc_rejected: list[Edge] = []
        for idx, edge in loop_factors:
            if weights[idx] > self._gnc_weight_threshold:
                refit_graph.add(graph.at(idx))
            else:
                gnc_rejected.append(edge)

        lm_optimizer = gtsam.LevenbergMarquardtOptimizer(refit_graph, gnc_result, self._lm_params)
        final_values = lm_optimizer.optimize()

        poses = {
            kf_id: final_values.atPose3(self._keys.key(kf_id)).matrix()
            for kf_id in self._keyframes
        }
        return OptimizedGraph(
            poses=poses,
            t_world_map=self._t_world_map(poses),
            components=components,
            rejected_edges=[*pcm_rejected, *gnc_rejected],
            iterations=lm_optimizer.iterations(),
            final_error=lm_optimizer.error(),
        )

    # ------------------------------------------------------------------ #
    # PCM: pairwise consistency maximization over INTER_LOOP candidates
    # ------------------------------------------------------------------ #

    def _pcm_filter(self, candidates: list[Edge]) -> tuple[list[Edge], list[Edge]]:
        """Split candidate inter-robot closures into accepted/rejected.

        Candidates are grouped by the (unordered) pair of robots they span,
        since consistency is only checkable between closures that could
        plausibly contradict each other -- a closure between robots A/B has
        nothing to compose against a closure between A/C. Within each group,
        a consistency graph is built from the pairwise chi-squared test
        (:func:`_consistent`) and the kept set is a greedy clique
        (:func:`_greedy_max_clique`) of size at least
        ``min_pcm_clique_size``.

        Groups are processed in a sorted (not hash-set) order so that
        ``optimize()`` output does not depend on ``PYTHONHASHSEED`` --
        required for this class's determinism guarantee.
        """
        groups: dict[frozenset[str], list[Edge]] = defaultdict(list)
        for edge in candidates:
            groups[_canonical_pair(edge)].append(edge)

        accepted: list[Edge] = []
        rejected: list[Edge] = []
        for pair in sorted(groups, key=lambda fs: tuple(sorted(fs))):
            group_edges = groups[pair]
            n = len(group_edges)
            adjacency: dict[int, set[int]] = {i: set() for i in range(n)}
            for i in range(n):
                for j in range(i + 1, n):
                    residual, covariance = _loop_residual(
                        group_edges[i], group_edges[j], self._keyframes
                    )
                    if _consistent(residual, covariance, self._pcm_confidence):
                        adjacency[i].add(j)
                        adjacency[j].add(i)
            kept = set(_greedy_max_clique(list(range(n)), adjacency))
            if len(kept) < self._min_pcm_clique_size:
                kept = set()
            for i, edge in enumerate(group_edges):
                (accepted if i in kept else rejected).append(edge)
        return accepted, rejected

    # ------------------------------------------------------------------ #
    # Components and gauge freedom
    # ------------------------------------------------------------------ #

    def _components(self, accepted_inter: list[Edge]) -> list[Component]:
        """Union-find over robots joined by PCM-accepted inter-robot closures.

        This is the sole authority on which robots share a frame -- decided
        once, before the solver runs, and never revised by GNC's later
        per-edge weighting (see module docstring). A robot with no accepted
        inter-robot closure at all is still a valid, singleton component: an
        unmerged map is a correct statement of ignorance.
        """
        robots = sorted({kf_id.robot_id for kf_id in self._keyframes})
        parent = {r: r for r in robots}

        def find(r: str) -> str:
            while parent[r] != r:
                parent[r] = parent[parent[r]]
                r = parent[r]
            return r

        for edge in accepted_inter:
            root_a, root_b = find(edge.src.robot_id), find(edge.dst.robot_id)
            if root_a != root_b:
                parent[max(root_a, root_b)] = min(root_a, root_b)

        groups: dict[str, set[str]] = defaultdict(set)
        for robot in robots:
            groups[find(robot)].add(robot)

        components: list[Component] = []
        for component_id, root in enumerate(sorted(groups)):
            members = groups[root]
            anchor_robot = min(members)
            anchor_seq = min(
                kf_id.seq for kf_id in self._keyframes if kf_id.robot_id == anchor_robot
            )
            components.append(
                Component(
                    component_id=component_id,
                    robots=frozenset(members),
                    anchor=KeyframeId(anchor_robot, anchor_seq),
                )
            )
        return components

    def _initial_t_world_odom(
        self, components: list[Component], accepted_inter: list[Edge]
    ) -> dict[str, np.ndarray]:
        """Seed every robot's ``T_world_odom`` for the LM initial guess.

        Each component's anchor robot starts at identity (its odom frame
        *is* the component's arbitrary gauge choice for "world"). Every other
        robot in the component gets an estimate propagated by BFS over the
        PCM-accepted closures that connect it back to the anchor, composing
        one closure's measured transform with both endpoints' own odometry.
        A good initial guess matters here specifically because Pose3's SO(3)
        component is non-convex: a merged robot starting from an arbitrary
        offset risks LM converging to a local optimum with the wrong
        rotation instead of the true relative transform.
        """
        adjacency: dict[str, list[Edge]] = defaultdict(list)
        for edge in accepted_inter:
            adjacency[edge.src.robot_id].append(edge)
            adjacency[edge.dst.robot_id].append(edge)

        init: dict[str, np.ndarray] = {}
        for component in components:
            anchor_robot = component.anchor.robot_id
            if anchor_robot in init:
                continue
            init[anchor_robot] = se3_identity()
            frontier = deque([anchor_robot])
            while frontier:
                robot = frontier.popleft()
                for edge in adjacency[robot]:
                    first_kf, second_kf, t_first_second = _orient(edge, robot)
                    other = second_kf.robot_id
                    if other in init:
                        continue
                    t_world_first = init[robot] @ self._keyframes[first_kf].t_odom_base
                    t_world_second = t_world_first @ t_first_second
                    init[other] = t_world_second @ se3_inverse(self._keyframes[second_kf].t_odom_base)
                    frontier.append(other)
        return init

    # ------------------------------------------------------------------ #
    # Factor graph construction
    # ------------------------------------------------------------------ #

    def _between_factor(self, edge: Edge) -> gtsam.BetweenFactorPose3:
        return gtsam.BetweenFactorPose3(
            self._keys.key(edge.src),
            self._keys.key(edge.dst),
            gtsam.Pose3(edge.t_src_dst),
            gtsam.noiseModel.Gaussian.Information(edge.information),
        )

    def _build_factors(
        self,
        odometry: list[Edge],
        intra_loop: list[Edge],
        pcm_accepted_inter: list[Edge],
        components: list[Component],
        init_t_world_odom: dict[str, np.ndarray],
    ) -> tuple[gtsam.NonlinearFactorGraph, gtsam.Values, list[int], list[tuple[int, Edge]]]:
        """Assemble the graph GNC will see: structural factors (odometry +
        one anchor prior per component, both registered as GNC known-inliers
        so they can never be rejected) plus every loop-closure candidate
        (intra-robot, and inter-robot closures that survived PCM) as
        ordinary, non-robust factors for GNC to weight.
        """
        initial = gtsam.Values()
        for kf_id, keyframe in self._keyframes.items():
            guess = init_t_world_odom[kf_id.robot_id] @ keyframe.t_odom_base
            initial.insert(self._keys.key(kf_id), gtsam.Pose3(guess))

        graph = gtsam.NonlinearFactorGraph()
        known_inliers: list[int] = []

        for edge in odometry:
            known_inliers.append(graph.size())
            graph.add(self._between_factor(edge))

        for component in components:
            known_inliers.append(graph.size())
            anchor_kf = self._keyframes[component.anchor]
            anchor_pose = init_t_world_odom[component.anchor.robot_id] @ anchor_kf.t_odom_base
            graph.add(
                gtsam.PriorFactorPose3(
                    self._keys.key(component.anchor), gtsam.Pose3(anchor_pose), self._anchor_noise
                )
            )

        loop_factors: list[tuple[int, Edge]] = []
        for edge in [*intra_loop, *pcm_accepted_inter]:
            idx = graph.size()
            graph.add(self._between_factor(edge))
            loop_factors.append((idx, edge))

        return graph, initial, known_inliers, loop_factors

    # ------------------------------------------------------------------ #
    # Output assembly
    # ------------------------------------------------------------------ #

    def _t_world_map(self, poses: dict[KeyframeId, np.ndarray]) -> dict[str, np.ndarray]:
        """Per-robot ``T_world_odom`` correction, from each robot's most
        recently optimized keyframe.

        This is deliberately a snapshot, not a continuously maintained
        quantity: it is exact at the stamp of that keyframe and only
        approximate for a fresher odometry reading applied against it
        afterwards (exactly the classic ROS ``map -> odom`` correction
        published by a SLAM node, which is why a controller consuming raw
        ``odom`` never has to block on the next optimization to keep
        running). Callers that need the current best estimate for a robot
        that has moved since its last keyframe should compose this with that
        robot's live odometry, not wait for another ``optimize()`` call.
        """
        latest: dict[str, KeyframeId] = {}
        for kf_id in self._keyframes:
            current = latest.get(kf_id.robot_id)
            if current is None or kf_id.seq > current.seq:
                latest[kf_id.robot_id] = kf_id
        return {
            robot_id: poses[kf_id] @ se3_inverse(self._keyframes[kf_id].t_odom_base)
            for robot_id, kf_id in latest.items()
        }

    def _validate_endpoints(self, edges: list[Edge]) -> None:
        missing = {
            kf_id for edge in edges for kf_id in (edge.src, edge.dst) if kf_id not in self._keyframes
        }
        if missing:
            raise ValueError(
                f"edge(s) reference keyframes never added via add_keyframe: {sorted(missing)}"
            )
