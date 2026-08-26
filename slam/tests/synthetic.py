"""A deterministic synthetic fleet, shared by every test in this package.

Pose-graph code is unusually easy to test wrongly. Hand-built two-pose fixtures
pass against implementations that are mirrored, transposed, or rotation-first
when they should be translation-first, because with one edge there is nothing to
be inconsistent with. Everything here therefore generates a *closed loop with
known ground truth and injected drift*, which is the smallest fixture that can
actually catch those defects.

Ground truth is exact and reproducible from a seed, so any metric computed
against it is a real number rather than a plausible-looking one.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from swarmdeck_slam.types import (
    Keyframe,
    KeyframeId,
    TrajectoryId,
    se3_from_quat_xyz,
    se3_identity,
    se3_inverse,
    transform_points,
)

WALL_HEIGHT = 2.4
MAX_RANGE = 30.0


def yaw_pose(x: float, y: float, yaw: float, z: float = 0.0) -> np.ndarray:
    """A planar pose as a 4x4 ``T_parent_child``. Ground robots live on this manifold."""
    return se3_from_quat_xyz([x, y, z, 0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0)])


def make_scene(seed: int = 0, spacing: float = 0.08) -> np.ndarray:
    """Sample the surfaces of a closed, asymmetric building.

    Asymmetry is the point. A rectangular room registers against itself at 90 and
    180 degrees, so a symmetric scene silently rewards a place-recognition stage
    that cannot tell those apart -- and that failure is exactly what a
    multi-robot system must not have.
    """
    rng = np.random.default_rng(seed)
    segments = [
        # Outer shell: 40 x 24 m.
        ((0.0, 0.0), (40.0, 0.0)),
        ((40.0, 0.0), (40.0, 24.0)),
        ((40.0, 24.0), (0.0, 24.0)),
        ((0.0, 24.0), (0.0, 0.0)),
        # Interior partitions, deliberately at irregular positions and angles.
        ((12.0, 0.0), (12.0, 9.0)),
        ((12.0, 15.0), (12.0, 24.0)),
        ((26.0, 4.0), (26.0, 24.0)),
        ((12.0, 9.0), (19.0, 6.0)),
        ((26.0, 4.0), (33.0, 11.0)),
        ((33.0, 11.0), (40.0, 11.0)),
    ]
    clouds = []
    for (x0, y0), (x1, y1) in segments:
        length = float(np.hypot(x1 - x0, y1 - y0))
        count = max(2, int(length / spacing))
        t = np.linspace(0.0, 1.0, count)
        xs = x0 + t * (x1 - x0)
        ys = y0 + t * (y1 - y0)
        zs = rng.uniform(0.05, WALL_HEIGHT, size=count)
        clouds.append(np.stack([xs, ys, zs], axis=1))
    scene = np.concatenate(clouds, axis=0)
    # Sub-millimetre jitter breaks the exact colinearity of sampled walls, which
    # would otherwise make GICP's covariance estimation degenerate.
    scene += rng.normal(scale=0.002, size=scene.shape)
    return scene.astype(np.float64)


def observe(scene: np.ndarray, t_world_base: np.ndarray, *, azimuth_bins: int = 720,
            max_range: float = MAX_RANGE, noise: float = 0.01,
            rng: np.random.Generator | None = None) -> np.ndarray:
    """Render a lidar view of ``scene`` from a pose, in the **base frame**.

    Keeping only the nearest return per azimuth bin approximates occlusion. Without
    it every robot sees through every wall, all clouds are near-identical, and loop
    closure looks far more reliable than it is on hardware.
    """
    rng = rng or np.random.default_rng(0)
    local = transform_points(se3_inverse(t_world_base), scene)
    distance = np.linalg.norm(local[:, :2], axis=1)
    visible = (distance > 0.4) & (distance < max_range)
    local, distance = local[visible], distance[visible]
    if local.size == 0:
        return np.zeros((0, 3), dtype=np.float32)

    azimuth = np.arctan2(local[:, 1], local[:, 0])
    bins = np.clip(((azimuth + np.pi) / (2 * np.pi) * azimuth_bins).astype(int), 0, azimuth_bins - 1)
    # Nearest-per-bin via a sort on (bin, distance) and taking each bin's first row.
    order = np.lexsort((distance, bins))
    keep = order[np.concatenate([[True], np.diff(bins[order]) != 0])]
    hit = local[keep]
    return (hit + rng.normal(scale=noise, size=hit.shape)).astype(np.float32)


@dataclass(slots=True)
class SyntheticRobot:
    """One robot's ground truth alongside the drifted odometry it reports."""

    robot_id: str
    keyframes: list[Keyframe]
    truth: dict[KeyframeId, np.ndarray]  # T_world_base, exact
    t_world_map_true: np.ndarray
    #: Which physical building this robot drove through. Robots sharing a scene
    #: *can* legitimately be merged into one component; robots in different
    #: scenes must NEVER be, and scoring that distinction needs the truth stated
    #: rather than inferred from whether the optimizer happened to merge them.
    scene_id: str = "default"
    #: Which run of that robot this is. Two entries with the same ``robot_id``
    #: and different sessions are one machine before and after a reboot -- one
    #: physical robot, two trajectories, two unrelated map frames.
    session: str = ""

    @property
    def trajectory_id(self) -> TrajectoryId:
        return TrajectoryId(self.robot_id, self.session)


def simulate_robot(
    scene: np.ndarray,
    robot_id: str,
    waypoints: list[tuple[float, float]],
    *,
    seed: int = 0,
    n_keyframes: int = 24,
    drift_per_metre: float = 0.012,
    yaw_drift_per_metre: float = 0.004,
    start_in_world: np.ndarray | None = None,
    scene_id: str = "default",
    session: str = "",
    first_seq: int = 0,
    stamp_offset: float = 0.0,
) -> SyntheticRobot:
    """Drive a robot along ``waypoints``, emitting keyframes with drifting odometry.

    Drift accumulates as a random walk scaled by distance travelled, which is what
    real odometry does: the error is unbounded in the open and only a loop closure
    removes it. A fixture with white noise instead of a random walk cannot
    distinguish an optimizer that closes loops from one that merely smooths.
    """
    rng = np.random.default_rng(seed)
    path = np.asarray(waypoints, dtype=np.float64)
    legs = np.linalg.norm(np.diff(path, axis=0), axis=1)
    arclength = np.concatenate([[0.0], np.cumsum(legs)])
    samples = np.linspace(0.0, float(arclength[-1]), n_keyframes)
    xs = np.interp(samples, arclength, path[:, 0])
    ys = np.interp(samples, arclength, path[:, 1])
    yaws = np.arctan2(np.gradient(ys), np.gradient(xs))

    t_world_map = se3_identity() if start_in_world is None else start_in_world
    keyframes: list[Keyframe] = []
    truth: dict[KeyframeId, np.ndarray] = {}
    drift = np.zeros(3)  # dx, dy, dyaw accumulated in the odom frame

    for i in range(n_keyframes):
        t_map_base = yaw_pose(float(xs[i]), float(ys[i]), float(yaws[i]))
        t_world_base = t_world_map @ t_map_base
        points = observe(scene, t_world_base, rng=rng)

        travelled = 0.0 if i == 0 else float(samples[i] - samples[i - 1])
        drift += rng.normal(scale=[drift_per_metre * travelled] * 2 + [yaw_drift_per_metre * travelled])
        t_odom_base = yaw_pose(
            float(xs[i] + drift[0]), float(ys[i] + drift[1]), float(yaws[i] + drift[2])
        )

        keyframe_id = KeyframeId(robot_id, first_seq + i, session)
        keyframes.append(
            Keyframe(
                id=keyframe_id,
                stamp=stamp_offset + float(i),
                t_odom_base=t_odom_base,
                points=points,
            )
        )
        truth[keyframe_id] = t_world_base

    return SyntheticRobot(robot_id, keyframes, truth, t_world_map, scene_id, session)


def two_robot_fleet(seed: int = 0) -> tuple[np.ndarray, list[SyntheticRobot]]:
    """Two robots with genuinely overlapping coverage and different start frames.

    The offset start frame is what makes this a *collaborative* fixture: the
    optimizer has to recover ``T_world_map`` for the second robot rather than
    getting it for free from a shared origin.
    """
    scene = make_scene(seed)
    alpha = simulate_robot(
        scene, "alpha", [(3.0, 3.0), (9.0, 3.0), (9.0, 20.0), (3.0, 20.0), (3.0, 3.0)], seed=seed + 1
    )
    beta = simulate_robot(
        scene,
        "beta",
        [(6.0, 12.0), (20.0, 12.0), (22.0, 20.0), (8.0, 18.0), (6.0, 12.0)],
        seed=seed + 2,
        start_in_world=yaw_pose(0.0, 0.0, 0.0),
    )
    return scene, [alpha, beta]


def reframe(robot: SyntheticRobot, t_newmap_oldmap: np.ndarray) -> SyntheticRobot:
    """Re-express a robot's REPORTED poses in a different map frame.

    Ground truth (where the robot physically is, and what it sees) is
    untouched; only ``t_odom_base`` moves. That is precisely what a SLAM node
    restart does -- the building did not move, the frame the robot describes it
    in did -- and it is what makes ``t_world_map_true`` the answer the
    optimizer has to recover rather than something the fixture handed it.
    """
    keyframes = [
        Keyframe(
            id=kf.id,
            stamp=kf.stamp,
            t_odom_base=t_newmap_oldmap @ kf.t_odom_base,
            points=kf.points,
            descriptor=kf.descriptor,
            descriptor_kind=kf.descriptor_kind,
        )
        for kf in robot.keyframes
    ]
    return SyntheticRobot(
        robot.robot_id,
        keyframes,
        robot.truth,
        robot.t_world_map_true @ se3_inverse(t_newmap_oldmap),
        robot.scene_id,
        robot.session,
    )


def restarted_robot(
    seed: int = 0, session: str = "boot-2", overlap: bool = True
) -> tuple[np.ndarray, list[SyntheticRobot]]:
    """ONE robot, twice: a tour, a power cycle, and a second tour.

    Both entries carry ``robot_id == "alpha"``, so anything keyed on the robot
    sees a single continuous stream -- which is exactly the illusion that made
    the pre-session back-end fabricate an odometry edge across the reboot and
    drop the second tour's first keyframes as duplicate ``seq`` values. The
    ``seq`` counter restarts at 0 in the second entry for the same reason: that
    is what the producer does.

    The second tour reports its poses in a DIFFERENT map frame, because a
    restarted SLAM node starts a fresh frame wherever the robot happens to be
    standing. Recovering the transform between the two frames is the whole job,
    and it has to come from place recognition rather than be assumed from the
    matching robot id.

    ``overlap=False`` sends the second tour through a different part of the
    building, so the two segments genuinely cannot be related -- the fixture
    for "declining to merge is correct".
    """
    scene = make_scene(seed)
    before = simulate_robot(
        scene, "alpha", [(3.0, 3.0), (9.0, 3.0), (9.0, 20.0), (3.0, 20.0), (3.0, 3.0)],
        seed=seed + 1,
    )
    route = (
        [(3.0, 4.0), (9.0, 4.0), (9.0, 19.0), (3.0, 19.0), (3.0, 4.0)]
        if overlap
        else [(30.0, 6.0), (36.0, 6.0), (36.0, 20.0), (30.0, 20.0), (30.0, 6.0)]
    )
    after = simulate_robot(
        scene, "alpha", route,
        seed=seed + 7,
        session=session,
        # Stamps continue past the first tour: the reboot took a minute, and
        # "which segment is the robot in NOW" is answered by the clock.
        stamp_offset=100.0,
    )
    return scene, [before, reframe(after, yaw_pose(-6.0, 2.5, 0.7))]


def disjoint_fleet(seed: int = 0) -> tuple[list[np.ndarray], list[SyntheticRobot]]:
    """Two robots in two genuinely different buildings, which must never merge.

    The counterpart to :func:`two_robot_fleet`, and the harder fixture: it is easy
    to build a system that merges robots that belong together, and much harder to
    build one that refuses to merge robots that do not. A false merge places a
    robot confidently in a room it has never entered, and no downstream consumer
    can detect that from the map alone -- so the ability to *decline* is the
    property worth testing hardest.

    The two scenes are generated from different seeds, so any inter-robot loop
    closure found between them is by construction a false positive.
    """
    scene_a = make_scene(seed)
    scene_b = make_scene(seed + 500)
    alpha = simulate_robot(
        scene_a, "alpha", [(3.0, 3.0), (9.0, 3.0), (9.0, 20.0), (3.0, 20.0), (3.0, 3.0)],
        seed=seed + 1, scene_id="building_a",
    )
    beta = simulate_robot(
        scene_b, "beta", [(30.0, 6.0), (36.0, 6.0), (36.0, 20.0), (30.0, 20.0), (30.0, 6.0)],
        seed=seed + 2, scene_id="building_b",
    )
    return [scene_a, scene_b], [alpha, beta]


def truth_groups(fleet: list[SyntheticRobot]) -> dict[str, str]:
    """``{robot_id: scene_id}`` for scoring component decisions.

    Derived from the fixture rather than hand-written at each call site, so a test
    cannot quietly assert against a grouping that disagrees with the data it was
    generated from.
    """
    return {robot.robot_id: robot.scene_id for robot in fleet}
