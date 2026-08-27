"""Odometry-free, multi-hypothesis registration for saved keyframe clouds.

This module deliberately does not accept a pose. A keyframe contributes only
its cloud and the registration result is inferred in three increasingly local
stages:

1. Scan Context proposes several yaw modes instead of discarding symmetric
   alternatives (notably yaw and yaw + pi in corridors).
2. FFT correlation of bird's-eye occupancy images proposes translations for
   each yaw, giving GICP a useful basin even when the scans are metres apart.
3. GICP refines each full SE(3) transform and a symmetric nearest-neighbour
   overlap test ranks the surviving modes.

The caller must still resolve competing modes using neighbouring registrations
and graph consistency. A low residual is not proof of a unique orientation;
that is the central lesson from ``sessions/captures/hw-run-01``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import small_gicp
from scipy.ndimage import maximum_filter
from scipy.signal import fftconvolve
from scipy.spatial import cKDTree

from swarmdeck_slam.descriptors import (
    alignment_hypotheses,
    scan_context_descriptor,
    shift_to_yaw,
)
from swarmdeck_slam.types import se3_distance, se3_identity


@dataclass(frozen=True, slots=True)
class OdomFreeConfig:
    """Configuration for coarse-to-fine cloud registration.

    Height defaults match the confirmed Bunker hardware capture: the Ouster
    frame is 0.52 m above the floor and the useful physical band is
    0.15--1.80 m above it. They are parameters rather than hidden constants so
    current packets carrying calibration can override them.
    """

    min_z: float = -0.37
    max_z: float = 1.28
    min_radius: float = 0.80
    max_radius: float = 18.0
    voxel_size: float = 0.20
    grid_resolution: float = 0.25
    grid_half_extent: float = 20.0
    yaw_hypotheses: int = 4
    yaw_separation_sectors: int = 3
    translation_hypotheses_per_yaw: int = 3
    translation_peak_separation_m: float = 1.0
    max_initial_translation_m: float = 15.0
    max_correspondence_distance: float = 1.0
    max_iterations: int = 50
    overlap_distance: float = 0.35
    min_symmetric_overlap: float = 0.42
    max_symmetric_rmse: float = 0.30
    min_inliers: int = 150
    deduplicate_translation_m: float = 0.35
    deduplicate_yaw_rad: float = math.radians(8.0)


@dataclass(frozen=True, slots=True)
class PreparedCloud:
    """Filtered representation cached once for repeated pair registration."""

    points: np.ndarray
    descriptor: np.ndarray
    occupancy: np.ndarray


@dataclass(frozen=True, slots=True)
class RegistrationHypothesis:
    """One geometrically valid transform mapping ``source`` into ``target``."""

    t_target_source: np.ndarray
    yaw_prior: float
    descriptor_distance: float
    coarse_score: float
    symmetric_overlap: float
    symmetric_rmse: float
    gicp_mean_error: float
    num_inliers: int
    score: float


def _validate_config(config: OdomFreeConfig) -> None:
    if config.max_z <= config.min_z:
        raise ValueError("max_z must exceed min_z")
    if not 0.0 <= config.min_radius < config.max_radius:
        raise ValueError("radii must satisfy 0 <= min_radius < max_radius")
    if config.voxel_size <= 0.0 or config.grid_resolution <= 0.0:
        raise ValueError("voxel and grid resolutions must be positive")
    if config.grid_half_extent <= 0.0:
        raise ValueError("grid_half_extent must be positive")


def _occupancy(points: np.ndarray, config: OdomFreeConfig) -> np.ndarray:
    cells = int(math.ceil(2.0 * config.grid_half_extent / config.grid_resolution))
    grid = np.zeros((cells, cells), dtype=np.float32)
    indices = np.floor(
        (points[:, :2] + config.grid_half_extent) / config.grid_resolution
    ).astype(np.int64)
    valid = np.all((indices >= 0) & (indices < cells), axis=1)
    indices = indices[valid]
    grid[indices[:, 1], indices[:, 0]] = 1.0
    return grid


def prepare_cloud(
    points: np.ndarray, config: OdomFreeConfig | None = None
) -> PreparedCloud:
    """Filter one raw keyframe and cache its descriptor and bird's-eye grid."""
    config = config or OdomFreeConfig()
    _validate_config(config)
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"points must be [n, 3], got {points.shape}")
    finite = np.all(np.isfinite(points), axis=1)
    radius = np.linalg.norm(points[:, :2], axis=1)
    keep = (
        finite
        & (points[:, 2] >= config.min_z)
        & (points[:, 2] <= config.max_z)
        & (radius >= config.min_radius)
        & (radius <= config.max_radius)
    )
    filtered = points[keep]
    if len(filtered):
        sampled = small_gicp.voxelgrid_sampling(filtered, config.voxel_size, 1)
        filtered = np.asarray(sampled.points(), dtype=np.float64)[:, :3]
    descriptor = scan_context_descriptor(
        filtered,
        max_range=config.max_radius,
        height_min=config.min_z,
        height_max=config.max_z,
    )
    return PreparedCloud(filtered, descriptor, _occupancy(filtered, config))


def _rotation_z(yaw: float) -> np.ndarray:
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def _translation_peaks(
    target_occupancy: np.ndarray,
    rotated_source_points: np.ndarray,
    config: OdomFreeConfig,
) -> list[tuple[np.ndarray, float]]:
    source_occupancy = _occupancy(rotated_source_points, config)
    correlation = fftconvolve(
        target_occupancy, source_occupancy[::-1, ::-1], mode="full"
    )
    peak_radius = max(
        1, int(round(config.translation_peak_separation_m / config.grid_resolution))
    )
    local_max = correlation == maximum_filter(
        correlation, size=2 * peak_radius + 1, mode="constant"
    )
    height, width = source_occupancy.shape
    y_index, x_index = np.indices(correlation.shape)
    shift_x = x_index - (width - 1)
    shift_y = y_index - (height - 1)
    max_cells = config.max_initial_translation_m / config.grid_resolution
    valid = local_max & (np.hypot(shift_x, shift_y) <= max_cells)
    locations = np.argwhere(valid)
    if not len(locations):
        return [(np.zeros(2, dtype=np.float64), 0.0)]
    values = correlation[valid]
    order = np.argsort(values, kind="stable")[::-1]
    normalizer = math.sqrt(
        max(float(target_occupancy.sum() * source_occupancy.sum()), 1.0)
    )
    peaks: list[tuple[np.ndarray, float]] = []
    for index in order[: config.translation_hypotheses_per_yaw]:
        y, x = locations[index]
        translation = np.array(
            [shift_x[y, x], shift_y[y, x]], dtype=np.float64
        ) * config.grid_resolution
        peaks.append((translation, float(correlation[y, x] / normalizer)))
    return peaks


def _symmetric_fit(
    target_points: np.ndarray,
    source_points: np.ndarray,
    t_target_source: np.ndarray,
    distance: float,
) -> tuple[float, float]:
    transformed = source_points @ t_target_source[:3, :3].T + t_target_source[:3, 3]
    source_distances = cKDTree(target_points).query(transformed, workers=1)[0]
    target_distances = cKDTree(transformed).query(target_points, workers=1)[0]
    source_overlap = float(np.mean(source_distances <= distance))
    target_overlap = float(np.mean(target_distances <= distance))
    overlap = 2.0 * source_overlap * target_overlap / max(
        source_overlap + target_overlap, 1e-12
    )
    inlier_distances = np.concatenate(
        [
            source_distances[source_distances <= distance],
            target_distances[target_distances <= distance],
        ]
    )
    if not len(inlier_distances):
        return overlap, math.inf
    rmse = float(np.sqrt(np.mean(np.square(inlier_distances))))
    return overlap, rmse


def _is_duplicate(
    candidate: RegistrationHypothesis,
    selected: list[RegistrationHypothesis],
    config: OdomFreeConfig,
) -> bool:
    for existing in selected:
        translation, rotation = se3_distance(
            existing.t_target_source, candidate.t_target_source
        )
        if (
            translation < config.deduplicate_translation_m
            and rotation < config.deduplicate_yaw_rad
        ):
            return True
    return False


def register_clouds(
    target: PreparedCloud,
    source: PreparedCloud,
    config: OdomFreeConfig | None = None,
) -> list[RegistrationHypothesis]:
    """Return distinct valid ``T_target_source`` modes, highest score first.

    No relative pose, map pose, or odometry value is accepted by this API.
    An empty list means that geometry could not justify a connection.
    """
    config = config or OdomFreeConfig()
    _validate_config(config)
    if len(target.points) < config.min_inliers or len(source.points) < config.min_inliers:
        return []

    yaw_modes = alignment_hypotheses(
        target.descriptor,
        source.descriptor,
        count=config.yaw_hypotheses,
        min_separation_sectors=config.yaw_separation_sectors,
    )
    hypotheses: list[RegistrationHypothesis] = []
    for shift, descriptor_distance in yaw_modes:
        yaw = float(shift_to_yaw(shift, target.descriptor.shape[1]))
        rotation = _rotation_z(yaw)
        rotated_source = source.points @ rotation.T
        for translation_xy, coarse_score in _translation_peaks(
            target.occupancy, rotated_source, config
        ):
            initial = se3_identity()
            initial[:3, :3] = rotation
            initial[:2, 3] = translation_xy
            result = small_gicp.align(
                target.points,
                source.points,
                init_T_target_source=initial,
                registration_type="GICP",
                downsampling_resolution=config.voxel_size,
                max_correspondence_distance=config.max_correspondence_distance,
                num_threads=1,
                max_iterations=config.max_iterations,
            )
            if not result.converged or result.num_inliers < config.min_inliers:
                continue
            transform = np.asarray(result.T_target_source, dtype=np.float64)
            overlap, rmse = _symmetric_fit(
                target.points,
                source.points,
                transform,
                config.overlap_distance,
            )
            if overlap < config.min_symmetric_overlap or rmse > config.max_symmetric_rmse:
                continue
            mean_error = float(result.error / max(result.num_inliers, 1))
            # Ranking is deliberately dominated by a metric independent of
            # GICP's own correspondences. Coarse correlation and descriptor
            # scores only settle close calls; neither can certify a match.
            score = (
                0.70 * overlap
                + 0.20 * coarse_score
                + 0.10 * (1.0 - descriptor_distance)
                - 0.10 * min(rmse / config.overlap_distance, 1.0)
            )
            hypothesis = RegistrationHypothesis(
                t_target_source=transform,
                yaw_prior=yaw,
                descriptor_distance=descriptor_distance,
                coarse_score=coarse_score,
                symmetric_overlap=overlap,
                symmetric_rmse=rmse,
                gicp_mean_error=mean_error,
                num_inliers=int(result.num_inliers),
                score=score,
            )
            if not _is_duplicate(hypothesis, hypotheses, config):
                hypotheses.append(hypothesis)

    hypotheses.sort(key=lambda item: item.score, reverse=True)
    # Re-run de-duplication in score order: an earlier coarse seed may converge
    # to the same mode as a later, better seed.
    unique: list[RegistrationHypothesis] = []
    for hypothesis in hypotheses:
        if not _is_duplicate(hypothesis, unique, config):
            unique.append(hypothesis)
    return unique
