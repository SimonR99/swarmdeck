"""Scan Context place-recognition descriptors and their candidate index.

Robots stream voxel-downsampled keyframe clouds instead of occupancy grids, and
the back-end has to propose loop-closure candidates -- including *between*
robots -- without ever comparing two full clouds, because a full-cloud
comparison is exactly the O(n^2 x cloud_size) cost this stage exists to avoid.
Scan Context (Kim & Kim, 2018) solves this by summarizing a cloud's ground
footprint as a small polar-binned height image: ``uint8 [rings, sectors]``.
That is also compact enough to exchange over a constrained radio link, which is
why the wire format (``swarmdeck_protocol.keyframe.Descriptor``) carries exactly
this shape plus a ``max_range`` float -- see :data:`DESCRIPTOR_KIND`.

Two properties make this useful for *multi-robot* loop closure specifically:

* The descriptor is built from geometry alone (bin = polar cell, value = max
  height in it), so it does not care which robot, or which sensor, produced the
  cloud -- only where the cloud's points are relative to its own base frame.
* Its *ring key* (the per-ring mean, see :func:`ring_key`) is invariant to the
  robot's heading at capture time, because a heading change is exactly a
  circular shift of the descriptor's sector axis and a circular shift does not
  change a row's mean. That turns "has any robot been near here, at any
  heading" into an ordinary Euclidean k-NN problem, which is what makes it
  tractable to index thousands of keyframes from four or more robots.

The full descriptor is *not* rotation invariant -- that is deliberate. The ring
key is a coarse, fast pre-filter; the full ``[rings, sectors]`` grid is then
compared column-by-column, at every possible sector shift, to find the best
rotational alignment. That best shift is both the fine-grained similarity score
and a yaw estimate, returned to the caller so the geometric verification stage
(GICP) has an initial rotation guess -- without one, GICP does not converge on
real loop closures with any reliability.

Frame convention, per ``types.py``: everything here treats a keyframe's cloud as
living in that keyframe's own base frame at capture. See
:class:`ScanContextIndex` for the exact yaw sign convention returned by
:meth:`ScanContextIndex.query`.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final, Sequence

import numpy as np
from scipy.spatial import cKDTree

from swarmdeck_slam.types import KeyframeId

#: Wire/graph identifier for this descriptor kind. Carried in
#: ``swarmdeck_protocol.keyframe.Descriptor.kind`` and ``Keyframe.descriptor_kind``
#: so a future descriptor can be introduced without breaking readers of this one.
DESCRIPTOR_KIND: Final[str] = "scan_context_v1"

DEFAULT_RINGS: Final[int] = 20
DEFAULT_SECTORS: Final[int] = 60

#: Generous default for a 3-D lidar keyframe; override to match a sensor's
#: actual usable range. Points beyond this radius are dropped, not clipped --
#: clipping would fabricate a phantom surface at the range limit, which is
#: exactly the kind of artifact that produces confident false loop closures
#: (the same reasoning the wire protocol uses for its own point quantization).
DEFAULT_MAX_RANGE: Final[float] = 40.0

#: Quantization range for the per-bin "max height" value, in metres, in the
#: keyframe's base frame. Spans a typical ground robot's sensor mounting offset
#: plus indoor ceiling height with margin; override for taller structures,
#: below-floor terrain, or aerial platforms. Value 0 is reserved for "bin has no
#: points" (see :func:`scan_context_descriptor`), so the 254 remaining uint8
#: levels span this range -- about 1.6 cm of resolution at the defaults.
DEFAULT_HEIGHT_MIN: Final[float] = -1.0
DEFAULT_HEIGHT_MAX: Final[float] = 3.0

_TAU: Final[float] = 2.0 * np.pi


def scan_context_descriptor(
    points: np.ndarray,
    *,
    rings: int = DEFAULT_RINGS,
    sectors: int = DEFAULT_SECTORS,
    max_range: float = DEFAULT_MAX_RANGE,
    height_min: float = DEFAULT_HEIGHT_MIN,
    height_max: float = DEFAULT_HEIGHT_MAX,
) -> np.ndarray:
    """Build a Scan Context descriptor from a cloud in the base frame.

    Bins the XY plane into ``rings`` concentric annuli of equal radial width up
    to ``max_range`` and ``sectors`` equal-angle wedges, and encodes the max
    height of the points falling in each bin, quantized to ``uint8``. An empty
    bin -- no points fell in it -- encodes as 0; a non-empty bin encodes as
    ``1..255`` over ``[height_min, height_max]``, clipped. Reserving 0 for
    "empty" rather than folding it into the height range keeps "nothing was
    seen there" distinguishable from "the floor was seen there", which matters
    for matching partially-occluded views of the same place.

    An empty input cloud, or one with no points inside ``max_range``, returns
    an all-zero descriptor rather than raising -- a keyframe captured in a
    featureless void is a valid (if useless) observation, not an error.

    Vectorized throughout: binning is array arithmetic and the per-bin max is a
    single scatter-max (:func:`numpy.ufunc.at`), not a Python loop over bins.
    """
    if rings <= 0 or sectors <= 0:
        raise ValueError(f"rings and sectors must be positive, got {rings}x{sectors}")
    if max_range <= 0.0:
        raise ValueError(f"max_range must be positive, got {max_range}")
    if height_max <= height_min:
        raise ValueError(
            f"height_max ({height_max}) must exceed height_min ({height_min})"
        )
    pts = np.asarray(points)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError(f"points must be [n, 3], got {pts.shape}")

    flat = np.zeros(rings * sectors, dtype=np.uint8)
    if pts.shape[0] == 0:
        return flat.reshape(rings, sectors)

    xy = pts[:, :2].astype(np.float64, copy=False)
    z = pts[:, 2].astype(np.float64, copy=False)
    radius = np.hypot(xy[:, 0], xy[:, 1])

    in_range = np.isfinite(radius) & np.isfinite(z) & (radius < max_range)
    if not np.any(in_range):
        return flat.reshape(rings, sectors)
    xy, z, radius = xy[in_range], z[in_range], radius[in_range]

    azimuth = np.mod(np.arctan2(xy[:, 1], xy[:, 0]), _TAU)  # [0, tau)
    ring_idx = np.minimum((radius / max_range * rings).astype(np.int64), rings - 1)
    sector_idx = np.minimum((azimuth / _TAU * sectors).astype(np.int64), sectors - 1)
    bin_idx = ring_idx * sectors + sector_idx

    heights = np.full(rings * sectors, -np.inf, dtype=np.float64)
    np.maximum.at(heights, bin_idx, z)

    occupied = np.isfinite(heights)
    scale = 254.0 / (height_max - height_min)
    quantized = 1.0 + (np.clip(heights, height_min, height_max) - height_min) * scale
    quantized = np.clip(np.rint(quantized), 1, 255)
    flat = np.where(occupied, quantized, 0.0).astype(np.uint8)
    return flat.reshape(rings, sectors)


def ring_key(descriptor: np.ndarray) -> np.ndarray:
    """Rotation-invariant per-ring summary: L2-normalized mean across sectors.

    A heading change is a circular shift of the sector axis, which does not
    change a row mean. L2-normalizing the key makes Euclidean k-NN compare
    *shape* rather than absolute max-height scale, so two lidars at different
    mounting heights (or a one-ring scan whose every bin shares one z) still
    retrieve the same place.
    """
    key = descriptor.astype(np.float64).mean(axis=1)
    norm = float(np.linalg.norm(key))
    if norm > 1e-12:
        return key / norm
    return key


def _shift_grid(sectors: int) -> np.ndarray:
    """``grid[shift, sector] = (sector + shift) % sectors``, for every shift at once."""
    sector_idx = np.arange(sectors)
    return (sector_idx[None, :] + sector_idx[:, None]) % sectors


def _column_shift_search(
    candidates: np.ndarray, query: np.ndarray, *, sectors: int
) -> tuple[np.ndarray, np.ndarray]:
    """Best column-shift alignment of ``query`` against each of ``candidates``.

    ``candidates`` is ``[k, rings, sectors]`` float64, ``query`` is
    ``[rings, sectors]`` float64. For every candidate and every possible sector
    shift, the per-sector cosine similarity between the shifted candidate
    column and the query's column is averaged across sectors; the shift with
    the highest mean similarity is that candidate's best alignment.

    Fully vectorized over shifts *and* over candidates at once via
    :func:`numpy.einsum` -- there is no Python loop here, over sectors, rings,
    or candidates. This is only ever called with ``candidates`` already cut
    down to a k-NN shortlist (tens of entries, not the whole index), because
    its cost is ``O(k * rings * sectors^2)``.

    Returns ``(best_shift[k] int64, best_distance[k] float64)`` where distance
    is ``1 - mean_cosine_similarity``, in ``[0, 1]`` (bin values are
    non-negative, so cosine similarity itself never goes negative).
    """
    grid = _shift_grid(sectors)  # [n_shifts, sectors]
    shifted = candidates[..., grid]  # [k, rings, n_shifts, sectors]

    candidate_norms = np.linalg.norm(candidates, axis=1)  # [k, sectors]
    shifted_norms = candidate_norms[:, grid]  # [k, n_shifts, sectors]
    query_norms = np.linalg.norm(query, axis=0)  # [sectors]

    dot = np.einsum("krns,rs->kns", shifted, query)  # [k, n_shifts, sectors]
    denom = shifted_norms * query_norms[None, None, :]
    cosine = np.divide(dot, denom, out=np.zeros_like(dot), where=denom > 1e-12)

    distance = 1.0 - cosine.mean(axis=2)  # [k, n_shifts]
    best_shift = np.argmin(distance, axis=1)
    best_distance = np.take_along_axis(distance, best_shift[:, None], axis=1)[:, 0]
    return best_shift, best_distance


def best_alignment(candidate: np.ndarray, query: np.ndarray) -> tuple[int, float]:
    """Single-pair column-shift distance and alignment shift.

    Convenience wrapper around the batched search in :func:`_column_shift_search`
    for callers comparing exactly one pair (tests; ad-hoc debugging). See
    :meth:`ScanContextIndex.query` for the yaw sign convention derived from the
    returned shift.
    """
    if candidate.shape != query.shape:
        raise ValueError(
            f"shape mismatch: candidate {candidate.shape} vs query {query.shape}"
        )
    sectors = candidate.shape[1]
    shifts, distances = _column_shift_search(
        candidate[None, ...].astype(np.float64),
        query.astype(np.float64),
        sectors=sectors,
    )
    return int(shifts[0]), float(distances[0])


def alignment_hypotheses(
    candidate: np.ndarray,
    query: np.ndarray,
    *,
    count: int = 4,
    min_separation_sectors: int = 3,
    extra_shifts: Sequence[int] | None = None,
    include_antipode: bool = True,
) -> list[tuple[int, float]]:
    """Return several distinct column-shift alignments, best first.

    :func:`best_alignment` is sufficient when GICP only needs one yaw seed,
    but it is unsafe for odometry-free reconstruction. Real indoor scans can
    have two equally convincing orientations -- most importantly the common
    180-degree corridor ambiguity. Throwing the runner-up away before
    geometric and graph-level checks makes that ambiguity impossible to
    recover from later.

    This function scores every circular shift with the same metric as
    :func:`best_alignment`, then greedily keeps the best shifts separated by
    at least ``min_separation_sectors`` on the circular sector axis. The
    separation prevents neighbouring bins around one broad minimum from
    crowding out a genuinely different orientation hypothesis.
    """
    if candidate.shape != query.shape:
        raise ValueError(
            f"shape mismatch: candidate {candidate.shape} vs query {query.shape}"
        )
    if candidate.ndim != 2:
        raise ValueError(f"descriptors must be [rings, sectors], got {candidate.shape}")
    if count <= 0:
        raise ValueError(f"count must be positive, got {count}")
    sectors = candidate.shape[1]
    if not 0 <= min_separation_sectors <= sectors // 2:
        raise ValueError(
            "min_separation_sectors must be between 0 and half the sector count, "
            f"got {min_separation_sectors} for {sectors} sectors"
        )

    candidates = candidate[None, ...].astype(np.float64)
    query_float = query.astype(np.float64)
    grid = _shift_grid(sectors)
    shifted = candidates[..., grid]
    candidate_norms = np.linalg.norm(candidates, axis=1)
    shifted_norms = candidate_norms[:, grid]
    query_norms = np.linalg.norm(query_float, axis=0)
    dot = np.einsum("krns,rs->kns", shifted, query_float)
    denom = shifted_norms * query_norms[None, None, :]
    cosine = np.divide(dot, denom, out=np.zeros_like(dot), where=denom > 1e-12)
    distances = 1.0 - cosine.mean(axis=2)[0]

    selected: list[tuple[int, float]] = []
    # Indoor corridors admit a 180-degree alias that is often not in the
    # greedy top-k (they all sit in one 18-degree basin). Keep it on the
    # ballot so Viterbi and the weak odom vote can still choose it. Extra
    # shifts (planar cardinals) are appended the same way.
    for shift in np.argsort(distances, kind="stable"):
        shift_int = int(shift)
        if any(
            min(abs(shift_int - chosen), sectors - abs(shift_int - chosen))
            < min_separation_sectors
            for chosen, _ in selected
        ):
            continue
        selected.append((shift_int, float(distances[shift_int])))
        if len(selected) >= min(count, sectors):
            break

    def _separated(shift: int) -> bool:
        return not any(
            min(abs(shift - chosen), sectors - abs(shift - chosen))
            < min_separation_sectors
            for chosen, _ in selected
        )

    mandatory: list[int] = []
    if include_antipode and selected:
        mandatory.append((selected[0][0] + sectors // 2) % sectors)
    if extra_shifts:
        mandatory.extend(int(shift) % sectors for shift in extra_shifts)
    for shift in mandatory:
        if _separated(shift):
            selected.append((shift, float(distances[shift])))
    return selected


def shift_to_yaw(shift: int | np.ndarray, sectors: int) -> float | np.ndarray:
    """Sector shift -> radians, wrapped to ``(-pi, pi]``. See :meth:`ScanContextIndex.query`."""
    delta = np.asarray(shift, dtype=np.float64) * (_TAU / sectors)
    wrapped = np.mod(delta + np.pi, _TAU) - np.pi
    return wrapped if isinstance(shift, np.ndarray) else float(wrapped)


@dataclass(frozen=True, slots=True)
class PlaceCandidate:
    """One place-recognition proposal returned by :meth:`ScanContextIndex.query`."""

    keyframe_id: KeyframeId
    #: Column-shift Scan Context distance in ``[0, 1]``; 0 is identical, larger
    #: is less similar. Threshold this, do not compare it across differently
    #: shaped indexes.
    distance: float
    #: Estimated yaw in radians, ``(-pi, pi]``. Rotates points from the
    #: *query* keyframe's base frame into the *candidate* (this result's)
    #: keyframe's base frame -- i.e. the z-rotation of ``T_candidate_query``.
    #: Equivalently: candidate_yaw + this angle ~= query_yaw, in whatever
    #: common frame both were captured in. This is the ``target=candidate,
    #: source=query`` convention small_gicp uses for ``T_target_source``: hand
    #: it to GICP unchanged when registering the query cloud onto the
    #: candidate cloud. Negate it if your edge points the other way.
    yaw: float


class ScanContextIndex:
    """Ring-key k-NN candidate lookup over Scan Context descriptors.

    Two-stage search, because comparing the full ``[rings, sectors]`` grid at
    every rotation against every indexed keyframe is the linear-scan cost this
    module exists to avoid:

    1. **Coarse**: k-NN over each descriptor's rotation-invariant :func:`ring_key`
       via ``scipy.spatial.cKDTree`` -- ``O(log n)`` per query.
    2. **Fine**: for just that k-NN shortlist, the full column-shift search
       (:func:`_column_shift_search`) scores every rotational alignment and
       returns the best one, both as the ranking distance and as a yaw prior.

    **Rebuild/append strategy.** ``cKDTree`` is immutable -- it has no insert.
    :meth:`add` appends to plain Python lists (O(1) amortized) and marks the
    tree stale; the tree and its backing arrays are rebuilt lazily, once, on
    the next :meth:`query`. This means:

    * A batch of adds followed by queries costs one ``O(n log n)`` rebuild for
      the whole batch, not one per add -- the expected access pattern (a
      session's keyframes stream in, queries happen periodically).
    * An add/query/add/query interleaving pays the full ``O(n log n)`` rebuild
      on *every* query, same as rebuilding on every add would. There is no
      partial-index fallback here; if that interleaving is the norm for a
      caller, batch adds before querying.

    Self-matches and temporally adjacent keyframes from the same robot are
    excluded via ``temporal_window`` (in sequence-number units, from
    :class:`KeyframeId.seq`) -- without it, every keyframe trivially "closes a
    loop" with its own immediate neighbours and the graph fills with edges that
    carry no information the odometry chain didn't already have.
    """

    def __init__(self, *, rings: int, sectors: int, temporal_window: int = 5) -> None:
        if rings <= 0 or sectors <= 0:
            raise ValueError(
                f"rings and sectors must be positive, got {rings}x{sectors}"
            )
        if temporal_window < 0:
            raise ValueError(
                f"temporal_window must be non-negative, got {temporal_window}"
            )
        self._rings = rings
        self._sectors = sectors
        self._temporal_window = temporal_window

        self._ids: list[KeyframeId] = []
        self._descriptors: list[np.ndarray] = []
        self._ring_keys: list[np.ndarray] = []

        self._tree: cKDTree | None = None
        self._ring_key_matrix = np.empty((0, rings), dtype=np.float64)
        self._robot_ids = np.empty(0, dtype=object)
        self._seqs = np.empty(0, dtype=np.int64)

    def __len__(self) -> int:
        return len(self._ids)

    def add(self, keyframe_id: KeyframeId, descriptor: np.ndarray) -> None:
        """Add one keyframe's descriptor. O(1) amortized; invalidates the k-NN tree."""
        self._validate(descriptor)
        self._ids.append(keyframe_id)
        self._descriptors.append(descriptor)
        self._ring_keys.append(ring_key(descriptor))
        self._tree = None  # stale -- rebuilt lazily in _ensure_index

    def query(
        self,
        descriptor: np.ndarray,
        k: int,
        *,
        query_id: KeyframeId | None = None,
    ) -> list[PlaceCandidate]:
        """Return up to ``k`` candidates, ranked by ascending column-shift distance.

        ``query_id``, when given, excludes candidates from the same robot
        within ``temporal_window`` sequence numbers (self-matches included,
        since a self-match has a sequence delta of zero). Pass ``None`` to
        search without exclusion, e.g. an exploratory query not tied to any
        particular indexed keyframe.

        Returns fewer than ``k`` results if the index (after exclusion) does
        not hold that many candidates; never raises for a small or empty index.
        """
        self._validate(descriptor)
        if k <= 0:
            raise ValueError(f"k must be positive, got {k}")
        n = len(self._ids)
        if n == 0:
            return []
        self._ensure_index()

        key = ring_key(descriptor)
        margin = 2 * self._temporal_window + 1 if query_id is not None else 0
        fetch = min(n, k + margin)
        kept = np.empty(0, dtype=np.int64)
        while True:
            _, idx = self._tree.query(key, k=fetch)
            idx = np.atleast_1d(idx).astype(np.int64)
            kept = idx[self._exclusion_mask(idx, query_id)]
            if kept.shape[0] >= k or fetch >= n:
                break
            fetch = min(n, fetch * 2)
        kept = kept[:k]
        if kept.shape[0] == 0:
            return []

        candidates = np.stack([self._descriptors[i] for i in kept], axis=0).astype(
            np.float64
        )
        shifts, distances = _column_shift_search(
            candidates, descriptor.astype(np.float64), sectors=self._sectors
        )
        yaws = shift_to_yaw(shifts, self._sectors)

        results = [
            PlaceCandidate(keyframe_id=self._ids[i], distance=float(d), yaw=float(y))
            for i, d, y in zip(kept, distances, yaws)
        ]
        results.sort(key=lambda c: c.distance)
        return results

    def _validate(self, descriptor: np.ndarray) -> None:
        if descriptor.dtype != np.uint8:
            raise ValueError(f"descriptor must be uint8, got {descriptor.dtype}")
        if descriptor.shape != (self._rings, self._sectors):
            raise ValueError(
                f"descriptor shape {descriptor.shape} does not match index shape "
                f"({self._rings}, {self._sectors})"
            )

    def _ensure_index(self) -> None:
        if self._tree is not None:
            return
        self._ring_key_matrix = np.stack(self._ring_keys, axis=0)
        self._robot_ids = np.array([kid.robot_id for kid in self._ids], dtype=object)
        self._seqs = np.array([kid.seq for kid in self._ids], dtype=np.int64)
        self._tree = cKDTree(self._ring_key_matrix)

    def _exclusion_mask(
        self, idx: np.ndarray, query_id: KeyframeId | None
    ) -> np.ndarray:
        if query_id is None:
            return np.ones(idx.shape[0], dtype=bool)
        same_robot = self._robot_ids[idx] == query_id.robot_id
        nearby = np.abs(self._seqs[idx] - query_id.seq) <= self._temporal_window
        return ~(same_robot & nearby)
