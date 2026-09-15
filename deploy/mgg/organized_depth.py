"""Conservative surface samples from an organized simulator depth image.

Used only by the simulation mapping relay. Samples interpolate qualified
adjacent depth pixels; subpixel holes remain below the sensor's resolution.
Missing and discontinuous measurements are never extrapolated into surfaces.
"""

from __future__ import annotations

import math

import numpy as np


def rasterize_flat_depth_quads(
    depth,
    *,
    fx,
    fy,
    cx,
    cy,
    max_range_m=20.0,
    max_vertical_span_m=0.08,
    max_surface_tilt_rad=math.radians(5.0),
    max_edge_m=3.0,
    spacing_m=0.15,
    max_pixels=100_000,
    max_candidate_quads=20_000,
    max_output_points=1_000_000,
    max_subdivisions=32,
):
    """Rasterize near-horizontal adjacent-pixel triangles in camera coordinates.

    Camera coordinates are forward/left/up, matching ``inputs.py`` simulation
    clouds. The near-horizontal test assumes the level camera mount used by the
    simulator; pitched cameras and general slopes are intentionally unsupported.
    The 8 cm span bound stays below the simulation fleet's smallest 15 cm step setting.
    Together with the normal and edge bounds it admits small road undulations;
    it does not establish the absence of a hidden pit between sensor pixels.
    A budget violation raises instead of publishing partial evidence.
    """
    values = np.asarray(depth)
    if values.ndim != 2 or values.size == 0 or values.size > max_pixels:
        raise ValueError("organized depth image exceeds its pixel budget")
    scalars = (
        fx,
        fy,
        cx,
        cy,
        max_range_m,
        max_vertical_span_m,
        max_surface_tilt_rad,
        max_edge_m,
        spacing_m,
    )
    if not all(math.isfinite(float(value)) for value in scalars):
        raise ValueError("organized depth parameters must be finite")
    if (
        fx <= 0
        or fy <= 0
        or max_range_m <= 0
        or max_vertical_span_m < 0
        or not 0 <= max_surface_tilt_rad < math.pi / 2
        or max_edge_m <= 0
        or spacing_m <= 0
        or max_candidate_quads <= 0
        or max_output_points <= 0
        or max_subdivisions < 1
    ):
        raise ValueError("organized depth parameters are invalid")

    height, width = values.shape
    rows, columns = np.mgrid[0:height, 0:width]
    finite = np.isfinite(values) & (values > 0.05) & (values < max_range_m)
    # Invalid pixels cannot qualify a quad. Keep them out of intermediate
    # geometry too, avoiding invalid arithmetic and warnings on every frame.
    values = np.where(finite, values, 0.0)
    points = np.empty((height, width, 3), dtype=np.float64)
    points[..., 0] = values
    points[..., 1] = -(columns - cx) * values / fx
    points[..., 2] = -(rows - cy) * values / fy
    p00 = points[:-1, :-1]
    p01 = points[:-1, 1:]
    p10 = points[1:, :-1]
    p11 = points[1:, 1:]
    candidates = finite[:-1, :-1] & finite[:-1, 1:] & finite[1:, :-1] & finite[1:, 1:]
    vertical = np.stack((p00[..., 2], p01[..., 2], p10[..., 2], p11[..., 2]))
    candidates &= np.ptp(vertical, axis=0) <= max_vertical_span_m
    # Include both diagonals: a thin image-space quad must not connect surfaces
    # separated by a depth discontinuity. Small quads need no added samples;
    # their original endpoints already occupy the native cloud.
    edge_lengths = np.stack(
        tuple(
            np.linalg.norm(a - b, axis=2)
            for a, b in (
                (p00, p01),
                (p01, p11),
                (p11, p10),
                (p10, p00),
                (p00, p11),
                (p01, p10),
            )
        )
    )
    longest = np.max(edge_lengths, axis=0)
    candidates &= (longest <= max_edge_m) & (longest > spacing_m)

    normals_a = np.cross(p01 - p00, p11 - p00)
    normals_b = np.cross(p11 - p00, p10 - p00)
    norm_a = np.linalg.norm(normals_a, axis=2)
    norm_b = np.linalg.norm(normals_b, axis=2)
    minimum_up = math.cos(max_surface_tilt_rad)
    candidates &= (
        (norm_a > 1e-9)
        & (norm_b > 1e-9)
        & (np.abs(normals_a[..., 2]) >= minimum_up * norm_a)
        & (np.abs(normals_b[..., 2]) >= minimum_up * norm_b)
    )
    # The relay already retains every second simulator pixel. Rasterizing the
    # intervening column once supplies the missing surface without duplicating
    # the same radial strips from both neighbouring quads.
    candidates[:, 1::2] = False

    indices = np.argwhere(candidates)
    if len(indices) > max_candidate_quads:
        raise ValueError("organized depth raster exceeds its quad budget")
    if not len(indices):
        return np.empty((0, 3), dtype=np.float32)
    row, column = indices[:, 0], indices[:, 1]
    corners = (
        points[row, column],
        points[row, column + 1],
        points[row + 1, column + 1],
        points[row + 1, column],
    )
    starts = np.concatenate(
        (corners[0], corners[1], corners[2], corners[3], corners[0], corners[1])
    )
    ends = np.concatenate(
        (corners[1], corners[2], corners[3], corners[0], corners[2], corners[3])
    )
    segment_lengths = np.linalg.norm(ends - starts, axis=1)
    subdivisions = np.maximum(1, np.ceil(segment_lengths / spacing_m).astype(int))
    if subdivisions.max(initial=0) > max_subdivisions:
        raise ValueError("organized depth edge exceeds subdivision budget")
    estimated = int(np.sum(subdivisions + 1))
    if estimated > max_output_points:
        raise ValueError("organized depth raster exceeds its output budget")
    output = []
    for count in np.unique(subdivisions):
        selected = subdivisions == count
        weights = np.linspace(0.0, 1.0, count + 1)[None, :, None]
        output.append(
            (
                starts[selected, None, :]
                + (ends[selected] - starts[selected])[:, None, :] * weights
            ).reshape(-1, 3)
        )
    # Edge sampling is sufficient for long, sub-spacing camera strips. For a
    # genuinely wide triangle, add a world-aligned XY lattice in its interior.
    interiors = []
    interior_estimate = 0
    triangle_a = np.concatenate((corners[0], corners[0]))
    triangle_b = np.concatenate((corners[1], corners[2]))
    triangle_c = np.concatenate((corners[2], corners[3]))
    triangle_area = np.linalg.norm(
        np.cross(triangle_b - triangle_a, triangle_c - triangle_a), axis=1
    )
    triangle_longest = np.max(
        np.stack(
            tuple(
                np.linalg.norm(a - b, axis=1)
                for a, b in (
                    (triangle_a, triangle_b),
                    (triangle_b, triangle_c),
                    (triangle_c, triangle_a),
                )
            )
        ),
        axis=0,
    )
    wide = triangle_area / triangle_longest > spacing_m
    for a, b, c in zip(triangle_a[wide], triangle_b[wide], triangle_c[wide]):
        x_values = np.arange(
            math.ceil(min(a[0], b[0], c[0]) / spacing_m) * spacing_m,
            max(a[0], b[0], c[0]),
            spacing_m,
        )
        y_values = np.arange(
            math.ceil(min(a[1], b[1], c[1]) / spacing_m) * spacing_m,
            max(a[1], b[1], c[1]),
            spacing_m,
        )
        interior_estimate += len(x_values) * len(y_values)
        if estimated + interior_estimate > max_output_points:
            raise ValueError("organized depth raster exceeds its output budget")
        if not len(x_values) or not len(y_values):
            continue
        xx, yy = np.meshgrid(x_values, y_values)
        q = np.column_stack((xx.ravel(), yy.ravel()))
        basis = np.column_stack(((b - a)[:2], (c - a)[:2]))
        determinant = np.linalg.det(basis)
        if abs(determinant) <= 1e-12:
            continue
        weights = np.linalg.solve(basis, (q - a[:2]).T).T
        inside = (
            (weights[:, 0] >= 0) & (weights[:, 1] >= 0) & (weights.sum(axis=1) <= 1)
        )
        weights = weights[inside]
        if len(weights):
            interiors.append(a + weights[:, :1] * (b - a) + weights[:, 1:] * (c - a))
    output.extend(interiors)

    if not output:
        return np.empty((0, 3), dtype=np.float32)
    # Adjacent triangles share edges and vertices. Exact quantization at a
    # micron removes duplicates without expanding the observed surface.
    result = np.concatenate(output, axis=0).reshape(-1, 3)
    keys = np.rint(result * 1_000_000.0).astype(np.int64)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return result[np.sort(indices)].astype(np.float32)
