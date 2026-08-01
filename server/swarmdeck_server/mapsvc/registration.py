"""Dynamic map registration: estimate T_world_robot from occupancy grids alone.

Each robot's SLAM map is expressed in its own frame, with the origin at wherever
that robot happened to start. To show one merged map we need the rigid transform
(dx, dy, dyaw) that aligns each robot's grid onto a reference robot's grid.

Method — brute-force rotation + FFT translation search:

  for each candidate yaw:
      rotate robot B's occupied cells by yaw
      cross-correlate with robot A's occupied cells via FFT   (all shifts at once)
      keep the best peak

FFT cross-correlation evaluates *every* translation in one O(n log n) pass, so
the whole search costs (number of yaw candidates) transforms rather than a full
3D sweep. Coarse yaw pass, then a fine pass around the winner.

numpy only — no OpenCV, no PCL, no extra packages. It is far simpler than a
pose-graph SLAM backend and needs no inter-robot communication, which matters
for a mixed ROS 1 / ROS 2 fleet where robots cannot share a SLAM system.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

OCCUPIED_MIN = 50


@dataclass
class Registration:
    dx: float
    dy: float
    dyaw: float
    score: float
    overlap: int
    ratio: float = 0.0  # runner-up peak / best peak; low means unambiguous

    @property
    def confident(self) -> bool:
        """Enough shared structure, a strong peak, and no credible rival.

        The ratio test is what stops a repetitive building from producing a
        confident-but-wrong merge: in a corridor of near-identical rooms,
        shifting by one room width can score nearly as well as the truth, so a
        near-tie means "do not trust this", not "good enough".
        """
        return self.overlap >= 80 and self.score >= 0.20 and self.ratio <= 0.80


def occupied_points(cells: np.ndarray, res: float, ox: float, oy: float) -> np.ndarray:
    """Occupied cells -> (N, 2) points in that grid's own metric frame."""
    ys, xs = np.nonzero(cells >= OCCUPIED_MIN)
    if xs.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    return np.column_stack([ox + (xs + 0.5) * res, oy + (ys + 0.5) * res])


def _rasterize(pts: np.ndarray, res: float, shape: tuple[int, int],
               ox: float, oy: float) -> np.ndarray:
    """Points -> binary image on a fixed grid."""
    img = np.zeros(shape, dtype=np.float32)
    if pts.size == 0:
        return img
    gx = np.floor((pts[:, 0] - ox) / res).astype(np.int64)
    gy = np.floor((pts[:, 1] - oy) / res).astype(np.int64)
    m = (gx >= 0) & (gx < shape[1]) & (gy >= 0) & (gy < shape[0])
    img[gy[m], gx[m]] = 1.0
    return img


def _best_shift(ref: np.ndarray, mov: np.ndarray) -> tuple[int, int, float, int, float]:
    """Cross-correlate two binary images.

    Returns (shift_x, shift_y, score, overlap, ratio). Score is the correlation
    peak normalised by the smaller point count, so it reads as "fraction of the
    sparser map that lined up". Ratio is the best rival peak (outside a
    neighbourhood of the winner) over the winner — a Lowe-style ambiguity test.
    """
    h, w = ref.shape
    F = np.fft.rfft2(ref) * np.conj(np.fft.rfft2(mov))
    corr = np.fft.irfft2(F, s=(h, w))

    idx = int(np.argmax(corr))
    py, px = divmod(idx, w)
    peak = float(corr[py, px])

    # Runner-up outside a +/-8 cell neighbourhood of the winner (wrapping).
    masked = corr.copy()
    r = 8
    ys = (np.arange(py - r, py + r + 1)) % h
    xs = (np.arange(px - r, px + r + 1)) % w
    masked[np.ix_(ys, xs)] = -np.inf
    rival = float(masked.max()) if np.isfinite(masked).any() else 0.0
    ratio = (rival / peak) if peak > 0 else 1.0

    sy, sx = py, px
    if sy > h // 2:
        sy -= h
    if sx > w // 2:
        sx -= w

    denom = max(1.0, min(ref.sum(), mov.sum()))
    return sx, sy, peak / denom, int(peak), ratio


def register(
    ref_cells: np.ndarray,
    ref_meta: tuple[float, float, float],
    mov_cells: np.ndarray,
    mov_meta: tuple[float, float, float],
    *,
    grid_res: float = 0.10,
    extent: float = 32.0,
    coarse_step_deg: float = 4.0,
    fine_step_deg: float = 0.5,
) -> Registration:
    """Estimate the rigid transform taking `mov` into `ref`'s frame.

    `*_meta` is (resolution, origin_x, origin_y) for that grid.

    Registration runs on a coarser grid than the maps themselves (default 10 cm);
    occupancy maps are noisy enough that finer costs time without buying accuracy.
    """
    ref_pts = occupied_points(ref_cells, *ref_meta)
    mov_pts = occupied_points(mov_cells, *mov_meta)
    if len(ref_pts) < 40 or len(mov_pts) < 40:
        return Registration(0.0, 0.0, 0.0, 0.0, 0, 1.0)

    n = int(extent / grid_res)
    shape = (n, n)
    ox = oy = -extent / 2

    # Centre both clouds; the FFT search recovers the residual translation.
    ref_c = ref_pts.mean(axis=0)
    mov_c = mov_pts.mean(axis=0)
    ref_img = _rasterize(ref_pts - ref_c, grid_res, shape, ox, oy)

    def sweep(cands: np.ndarray) -> tuple[float, int, int, float, int, float]:
        best = (0.0, 0, 0, -1.0, 0, 1.0)  # yaw, sx, sy, score, overlap, ratio
        centred = mov_pts - mov_c
        for yaw in cands:
            c, s = math.cos(yaw), math.sin(yaw)
            rot = np.column_stack(
                [centred[:, 0] * c - centred[:, 1] * s,
                 centred[:, 0] * s + centred[:, 1] * c]
            )
            img = _rasterize(rot, grid_res, shape, ox, oy)
            sx, sy, score, overlap, ratio = _best_shift(ref_img, img)
            if score > best[3]:
                best = (float(yaw), sx, sy, score, overlap, ratio)
        return best

    coarse = np.deg2rad(np.arange(-180.0, 180.0, coarse_step_deg))
    yaw, sx, sy, score, overlap, ratio = sweep(coarse)

    fine = np.deg2rad(
        np.arange(math.degrees(yaw) - coarse_step_deg,
                  math.degrees(yaw) + coarse_step_deg + 1e-9, fine_step_deg)
    )
    yaw, sx, sy, score, overlap, ratio = max(
        sweep(fine), (yaw, sx, sy, score, overlap, ratio), key=lambda b: b[3]
    )

    # Compose: rotate about mov's centroid, then translate onto ref's centroid
    # plus the residual shift the correlation found.
    c, s = math.cos(yaw), math.sin(yaw)
    rot_c = np.array([mov_c[0] * c - mov_c[1] * s, mov_c[0] * s + mov_c[1] * c])
    shift = np.array([sx * grid_res, sy * grid_res])
    trans = ref_c + shift - rot_c

    return Registration(
        float(trans[0]), float(trans[1]), float(yaw), float(score), overlap, float(ratio)
    )
