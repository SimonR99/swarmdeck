"""Dynamic map registration: estimate T_world_robot from occupancy grids alone.

Each robot's SLAM map is expressed in its own frame, with the origin at wherever
that robot happened to start. To show one merged map we need the rigid transform
(dx, dy, dyaw) that aligns each robot's grid onto a reference robot's grid.

Method — coarse-to-fine rotation sweep with an FFT translation search:

  for each candidate yaw:
      rotate robot B's occupied cells by yaw
      cross-correlate with robot A's map via FFT   (all shifts at once)
      keep the best peak

FFT cross-correlation evaluates *every* translation in one O(n log n) pass, so
the whole search costs (number of yaw candidates) transforms rather than a full
3D sweep.

Two properties of this problem drive the design and are easy to get wrong:

1. The correlation peak in yaw is *narrow*. A yaw error at radius r displaces a
   point by r*dyaw, so the peak is only about (cell_size / r) radians wide — at
   r = 15 m and 10 cm cells, well under a degree. Sampling yaw on a 4 deg grid
   against a peak that sharp misses it outright, lands on whatever rotational
   symmetry the building has, and reports a confident 90 deg error. Every stage
   here therefore searches a resolution whose peak is wide enough to be *found*
   at that stage's yaw step, and only the last stage sees full detail.

2. Matching occupied cells alone cannot distinguish a floor plan from the same
   floor plan rotated onto its own symmetry: both put walls on walls. Observed
   *free* space breaks the tie, because a wrong rotation drives walls straight
   through rooms another robot has already seen to be empty. So the reference is
   rasterized signed — occupied positive, known-free negative, unknown zero —
   and contradiction is scored against the match.

numpy only — no OpenCV, no PCL, no extra packages. It is far simpler than a
pose-graph SLAM backend and needs no inter-robot communication, which matters
for a mixed ROS 1 / ROS 2 fleet where robots cannot share a SLAM system. It is
also only a *map stitcher*: it never feeds a correction back to any robot, so it
cannot fix anyone's drift. See docs/collaborative-slam.md.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

OCCUPIED_MIN = 50

# Weight on "this match puts a wall where the reference saw empty floor". Held
# equal to the reward for a wall landing on a wall: contradicting an observation
# is exactly as informative as confirming one, and anything less lets a building's
# outer shell — which matches itself under rotation — outvote the interior walls
# that are the only evidence of which way round the map goes.
FREE_PENALTY = 1.0

# Cells within this radius of a reference wall are exempt from the free-space
# penalty. Two robots never agree on a wall to the cell, so without the guard a
# correct match is punished for landing one cell beside the wall instead of on it.
GUARD_CELLS = 1

# Rasters for each stage, metres per cell. The coarse stage is deliberately
# blunt: it only has to pick the right few degrees, and a blunt raster is what
# makes a 4 deg yaw step legitimate (see the module docstring).
COARSE_RES = 0.50
MEDIUM_RES = 0.25
FINE_RES = 0.10

# Yaw steps per stage, degrees. Each stage refines within +/- the previous step.
COARSE_STEP = 4.0
MEDIUM_STEP = 1.0
FINE_STEP = 0.25

# Distinct rotation hypotheses carried out of the coarse stage. More than one
# because the blunt raster ranks near-ties loosely, and because the runners-up
# are what the yaw ambiguity ratio is measured against.
TOP_K = 3

# Two yaw hypotheses closer than this are the same hypothesis, not rivals.
YAW_RIVAL_SEP_DEG = 12.0

# Least shared known area, as a fraction of the smaller map's known area, that
# can support a transform. Below this the robots have not really seen the same
# place and many transforms explain the sliver they do share equally well.
MIN_SUPPORT = 0.35


@dataclass
class Registration:
    dx: float
    dy: float
    dyaw: float
    score: float
    overlap: int
    ratio: float = 0.0  # runner-up peak / best peak; low means unambiguous
    yaw_ratio: float = 0.0  # best rival at a *different* yaw / best; see below
    support: float = 1.0  # fraction of the smaller map's known area that is shared

    @property
    def confident(self) -> bool:
        """Enough shared structure, a strong peak, and no credible rival.

        The ratio test is what stops a repetitive building from producing a
        confident-but-wrong merge: in a corridor of near-identical rooms,
        shifting by one room width can score nearly as well as the truth, so a
        near-tie means "do not trust this", not "good enough".

        `ratio` covers rival *translations* at the winning yaw. That alone is
        blind to the failure that actually bites — a rival *rotation*, which a
        square or cross-shaped building always has — so `yaw_ratio` covers rival
        yaw hypotheses too. `support` then refuses the case where the maps hardly
        overlap at all, where a good score over a sliver means little.
        """
        return (
            self.overlap >= 80
            and self.score >= 0.20
            and self.ratio <= 0.80
            and self.yaw_ratio <= 0.80
            and self.support >= MIN_SUPPORT
        )


def occupied_points(cells: np.ndarray, res: float, ox: float, oy: float) -> np.ndarray:
    """Occupied cells -> (N, 2) points in that grid's own metric frame."""
    ys, xs = np.nonzero(cells >= OCCUPIED_MIN)
    if xs.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    return np.column_stack([ox + (xs + 0.5) * res, oy + (ys + 0.5) * res])


def free_points(cells: np.ndarray, res: float, ox: float, oy: float) -> np.ndarray:
    """Known-free cells -> (N, 2) points. Unknown (-1) is not free."""
    ys, xs = np.nonzero((cells >= 0) & (cells < OCCUPIED_MIN))
    if xs.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    return np.column_stack([ox + (xs + 0.5) * res, oy + (ys + 0.5) * res])


def _rotate(pts: np.ndarray, yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack([pts[:, 0] * c - pts[:, 1] * s, pts[:, 0] * s + pts[:, 1] * c])


def _rasterize(pts: np.ndarray, res: float, n: int, org: float) -> np.ndarray:
    """Points -> binary image on a square grid of n cells starting at org."""
    img = np.zeros((n, n), dtype=np.float64)
    if pts.size == 0:
        return img
    gx = np.floor((pts[:, 0] - org) / res).astype(np.int64)
    gy = np.floor((pts[:, 1] - org) / res).astype(np.int64)
    m = (gx >= 0) & (gx < n) & (gy >= 0) & (gy < n)
    img[gy[m], gx[m]] = 1.0
    return img


def _dilate(img: np.ndarray, k: int) -> np.ndarray:
    """Square max-filter, radius k cells. Widens the correlation peak so a
    coarse yaw step can find it at all."""
    if k <= 0:
        return img
    out = img
    for axis in (0, 1):
        stack = [np.roll(out, d, axis=axis) for d in range(-k, k + 1)]
        out = np.maximum.reduce(stack)
    return out


def _parabolic(left: float, mid: float, right: float) -> float:
    """Sub-sample offset of a peak sampled at (-1, 0, +1). Zero if not a peak."""
    denom = left - 2.0 * mid + right
    if denom >= 0.0:
        return 0.0
    return float(np.clip(0.5 * (left - right) / denom, -0.5, 0.5))


class _Stage:
    """One resolution of the search: a reference image and its cached FFT.

    The reference does not change as yaw sweeps, so its transform is computed
    once per stage rather than once per candidate — the original code paid for it
    on every single yaw.
    """

    def __init__(
        self,
        ref_occ: np.ndarray,
        ref_free: np.ndarray,
        res: float,
        radius: float,
        dilate: int = 0,
    ) -> None:
        self.res = res
        self.n = max(8, int(math.ceil(2.0 * radius / res)) + 2)
        self.org = -0.5 * self.n * res
        # Zero-pad to twice the grid so the correlation is linear, not circular.
        # Without this, structure wraps around the edge and manufactures rival
        # peaks that corrupt the ambiguity test.
        self.fft_shape = (2 * self.n, 2 * self.n)

        occ = _dilate(_rasterize(ref_occ, res, self.n, self.org), dilate)
        free = _rasterize(ref_free, res, self.n, self.org)
        # Occupied wins its cell, and the guard band around it is neutral, so only
        # a wall well clear of any reference wall counts as a contradiction.
        signed = occ - FREE_PENALTY * free * (_dilate(occ, GUARD_CELLS) == 0.0)
        self._ref_fft = np.fft.rfft2(signed, s=self.fft_shape)
        self.ref_occ_count = float(occ.sum())

    def correlate(self, mov_pts: np.ndarray, yaw: float) -> np.ndarray:
        img = _rasterize(_rotate(mov_pts, yaw), self.res, self.n, self.org)
        spec = self._ref_fft * np.conj(np.fft.rfft2(img, s=self.fft_shape))
        return np.fft.irfft2(spec, s=self.fft_shape)

    def peak(self, corr: np.ndarray) -> tuple[float, float, float, float]:
        """Best shift in metres, its score, and the rival-translation ratio."""
        h, w = corr.shape
        idx = int(np.argmax(corr))
        py, px = divmod(idx, w)
        best = float(corr[py, px])

        # Runner-up outside a neighbourhood of the winner, in cells. Anything
        # closer is the same peak, not a competing hypothesis.
        r = max(4, int(round(1.0 / self.res)))
        masked = corr.copy()
        ys = np.arange(py - r, py + r + 1) % h
        xs = np.arange(px - r, px + r + 1) % w
        masked[np.ix_(ys, xs)] = -np.inf
        rival = float(masked.max()) if np.isfinite(masked).any() else 0.0
        ratio = 1.0 if best <= 0.0 else max(0.0, rival) / best

        # Sub-cell peak position. Quantising to the raster is the accuracy floor
        # of the whole estimate, and interpolation is nearly free.
        oy = _parabolic(
            float(corr[(py - 1) % h, px]), best, float(corr[(py + 1) % h, px])
        )
        ox = _parabolic(
            float(corr[py, (px - 1) % w]), best, float(corr[py, (px + 1) % w])
        )

        sy, sx = py, px
        if sy > h // 2:
            sy -= h
        if sx > w // 2:
            sx -= w
        return (sx + ox) * self.res, (sy + oy) * self.res, best, ratio


def _sweep(
    stage: _Stage, mov_pts: np.ndarray, yaws: np.ndarray
) -> list[tuple[float, float, float, float, float]]:
    """(yaw, dx, dy, score, ratio) for every candidate, normalised by point count."""
    denom = max(1.0, min(stage.ref_occ_count, float(len(mov_pts))))
    out = []
    for yaw in yaws:
        dx, dy, peak, ratio = stage.peak(stage.correlate(mov_pts, float(yaw)))
        out.append((float(yaw), dx, dy, peak / denom, ratio))
    return out


def _wrap(yaw: float) -> float:
    return (yaw + math.pi) % (2 * math.pi) - math.pi


def _cell_keys(pts: np.ndarray, lo: np.ndarray, res: float, span: int) -> np.ndarray:
    """Points -> sorted unique flat cell indices, for set arithmetic on grids."""
    if len(pts) == 0:
        return np.empty(0, dtype=np.int64)
    k = np.floor((pts - lo) / res).astype(np.int64)
    return np.unique(k[:, 1] * span + k[:, 0])


def _evaluate(
    ref_occ: np.ndarray,
    ref_free: np.ndarray,
    mov_occ: np.ndarray,
    mov_free: np.ndarray,
    yaw: float,
    dx: float,
    dy: float,
    res: float,
) -> tuple[float, int, float]:
    """Agreement and support at the winning transform.

    The search runs on signed images so free space can veto a symmetric alias,
    but a signed peak is not a quantity anyone can reason about. Report the thing
    the thresholds were calibrated against — the fraction of the sparser map
    whose walls actually landed on walls — plus how much *known* area the two
    maps genuinely share, which is the precondition the whole estimate rests on.
    """
    if len(ref_occ) == 0 or len(mov_occ) == 0:
        return 0.0, 0, 0.0
    rot = _rotate(mov_occ, yaw) + np.array([dx, dy])
    rot_free = (
        _rotate(mov_free, yaw) + np.array([dx, dy])
        if len(mov_free)
        else np.empty((0, 2))
    )
    clouds = [c for c in (ref_occ, ref_free, rot, rot_free) if len(c)]
    lo = np.minimum.reduce([c.min(axis=0) for c in clouds]) - res
    hi = np.maximum.reduce([c.max(axis=0) for c in clouds]) + res
    span = int((hi[0] - lo[0]) / res) + 2

    ref_o = _cell_keys(ref_occ, lo, res, span)
    mov_o = _cell_keys(rot, lo, res, span)
    overlap = int(np.intersect1d(ref_o, mov_o, assume_unique=True).size)
    score = overlap / max(1, min(ref_o.size, mov_o.size))

    # Shared known area. Two maps that barely overlap admit many transforms that
    # explain the sliver equally well, so a strong match over a tiny shared
    # region is not evidence — it is the ill-posed case (docs/KNOWN_ISSUES.md).
    ref_k = np.union1d(ref_o, _cell_keys(ref_free, lo, res, span))
    mov_k = np.union1d(mov_o, _cell_keys(rot_free, lo, res, span))
    shared = int(np.intersect1d(ref_k, mov_k, assume_unique=True).size)
    support = shared / max(1, min(ref_k.size, mov_k.size))
    return score, overlap, support


def register(
    ref_cells: np.ndarray,
    ref_meta: tuple[float, float, float],
    mov_cells: np.ndarray,
    mov_meta: tuple[float, float, float],
    *,
    yaw_prior: float | None = None,
    yaw_window_deg: float = 40.0,
) -> Registration:
    """Estimate the rigid transform taking `mov` into `ref`'s frame.

    `*_meta` is (resolution, origin_x, origin_y) for that grid.

    `yaw_prior` restricts the rotation sweep to a window around a known
    approximate heading. Deployments that configure start poses should pass it:
    it is ~9x less work and it removes the symmetric-alias failure mode outright,
    because an alias 90 deg away is never a candidate. Omit it when starts are
    genuinely unknown.
    """
    ref_occ = occupied_points(ref_cells, *ref_meta)
    mov_occ = occupied_points(mov_cells, *mov_meta)
    if len(ref_occ) < 40 or len(mov_occ) < 40:
        return Registration(0.0, 0.0, 0.0, 0.0, 0, 1.0, 1.0, 0.0)

    # Centre both clouds; the FFT search recovers the residual translation.
    ref_c = ref_occ.mean(axis=0)
    mov_c = mov_occ.mean(axis=0)
    ref_occ_c = ref_occ - ref_c
    mov_occ_c = mov_occ - mov_c
    ref_free_c = free_points(ref_cells, *ref_meta)
    if len(ref_free_c):
        ref_free_c = ref_free_c - ref_c

    # Rotation is about each centroid, so the enclosing radius is yaw-invariant
    # and sizing the grid from it means nothing is ever silently clipped.
    radius = float(
        max(
            np.hypot(ref_occ_c[:, 0], ref_occ_c[:, 1]).max(),
            np.hypot(mov_occ_c[:, 0], mov_occ_c[:, 1]).max(),
        )
    )

    # Dilate enough that a half-step yaw error still overlaps: the worst
    # displacement is radius * step/2, and a k-cell dilation tolerates
    # (k + 0.5) cells of it.
    tolerance = radius * math.radians(COARSE_STEP) / 2.0
    dilate = max(1, int(math.ceil(tolerance / COARSE_RES - 0.5)))

    coarse = _Stage(ref_occ_c, ref_free_c, COARSE_RES, radius, dilate=dilate)
    medium = _Stage(ref_occ_c, ref_free_c, MEDIUM_RES, radius)
    fine = _Stage(ref_occ_c, ref_free_c, FINE_RES, radius)

    if yaw_prior is None:
        candidates = np.deg2rad(np.arange(-180.0, 180.0, COARSE_STEP))
    else:
        candidates = np.deg2rad(
            np.arange(-yaw_window_deg, yaw_window_deg + 1e-9, COARSE_STEP)
        ) + _wrap(yaw_prior)

    coarse_results = _sweep(coarse, mov_occ_c, candidates)
    coarse_results.sort(key=lambda r: -r[3])

    # Keep the best candidate from each *distinct* rotation hypothesis. Adjacent
    # yaws are the same hypothesis sampled twice, so taking the top few outright
    # would spend the whole budget re-examining one peak and leave no rival to
    # compare against.
    sep = math.radians(YAW_RIVAL_SEP_DEG)
    hypotheses: list[tuple[float, float, float, float, float]] = []
    for cand in coarse_results:
        if all(abs(_wrap(cand[0] - kept[0])) > sep for kept in hypotheses):
            hypotheses.append(cand)
        if len(hypotheses) == TOP_K:
            break

    # Refine every hypothesis to the same resolution before ranking them. The
    # coarse raster is deliberately too blunt to tell a true match from a
    # symmetric alias — that is what makes its yaw step legitimate — so judging
    # ambiguity on coarse scores rejects good matches in any rectangular
    # building. Medium resolution can see the interior walls that break the tie.
    refined = [
        max(
            _sweep(
                medium,
                mov_occ_c,
                np.deg2rad(
                    np.arange(
                        math.degrees(cand[0]) - COARSE_STEP,
                        math.degrees(cand[0]) + COARSE_STEP + 1e-9,
                        MEDIUM_STEP,
                    )
                ),
            ),
            key=lambda r: r[3],
        )
        for cand in hypotheses
    ]
    refined.sort(key=lambda r: -r[3])
    best = refined[0]

    # Best surviving hypothesis at a genuinely different rotation. This is the
    # test the original ratio check could not express, and the one that catches a
    # square building matching itself at 90 deg.
    yaw_ratio = (
        1.0
        if best[3] <= 0.0
        else max(0.0, refined[1][3]) / best[3]
        if len(refined) > 1
        else 0.0
    )

    fine_results = _sweep(
        fine,
        mov_occ_c,
        np.deg2rad(
            np.arange(
                math.degrees(best[0]) - MEDIUM_STEP,
                math.degrees(best[0]) + MEDIUM_STEP + 1e-9,
                FINE_STEP,
            )
        ),
    )
    k = max(range(len(fine_results)), key=lambda i: fine_results[i][3])
    yaw, dx, dy, _, ratio = fine_results[k]

    # Interpolate the yaw peak too: 0.25 deg of residual is 6 cm of error at a
    # 15 m lever arm, which is the same order as everything else here.
    if 0 < k < len(fine_results) - 1:
        yaw += _parabolic(
            fine_results[k - 1][3], fine_results[k][3], fine_results[k + 1][3]
        ) * math.radians(FINE_STEP)

    # Compose: rotate about mov's centroid, then translate onto ref's centroid
    # plus the residual shift the correlation found.
    rot_c = _rotate(mov_c.reshape(1, 2), yaw)[0]
    trans = ref_c + np.array([dx, dy]) - rot_c

    mov_free_c = free_points(mov_cells, *mov_meta)
    if len(mov_free_c):
        mov_free_c = mov_free_c - mov_c
    score, overlap, support = _evaluate(
        ref_occ_c, ref_free_c, mov_occ_c, mov_free_c, yaw, dx, dy, FINE_RES
    )
    return Registration(
        float(trans[0]),
        float(trans[1]),
        _wrap(yaw),
        float(score),
        overlap,
        float(ratio),
        float(yaw_ratio),
        float(support),
    )
