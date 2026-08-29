"""Ground-truth occupancy of the Gazebo indoor world.

The reconstructed map is scored against this raster, not against onboard
SLAM. Pair that with a rigid SE(2) alignment so an arbitrary reconstruction
gauge is not counted as map error.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.signal import fftconvolve

from swarmdeck_slam.render import OCCUPIED, RenderedGrid

_REPO_SDF = (
    Path(__file__).resolve().parents[2]
    / "swarmdeck_ros"
    / "src"
    / "swarmdeck_sim"
    / "worlds"
    / "indoor.sdf"
)


@dataclass(frozen=True, slots=True)
class Box2:
    cx: float
    cy: float
    sx: float
    sy: float
    yaw: float = 0.0


@dataclass(frozen=True, slots=True)
class Occupancy:
    cells: np.ndarray  # bool [height, width]
    origin_x: float
    origin_y: float
    resolution: float

    @property
    def height(self) -> int:
        return int(self.cells.shape[0])

    @property
    def width(self) -> int:
        return int(self.cells.shape[1])


@dataclass(frozen=True, slots=True)
class OccupancyScore:
    iou: float
    precision: float
    recall: float
    yaw_rad: float
    shift_x_m: float
    shift_y_m: float
    estimated_occupied: int
    truth_occupied: int


def _pose6(text: str | None) -> tuple[float, float, float]:
    parts = [float(item) for item in (text or "0 0 0 0 0 0").split()]
    parts.extend([0.0] * (6 - len(parts)))
    return parts[0], parts[1], parts[5]


def _compose(parent: tuple[float, float, float], child: tuple[float, float, float]) -> tuple[float, float, float]:
    yaw = parent[2] + child[2]
    cosine, sine = math.cos(parent[2]), math.sin(parent[2])
    return (
        parent[0] + cosine * child[0] - sine * child[1],
        parent[1] + sine * child[0] + cosine * child[1],
        yaw,
    )


def boxes_from_sdf(path: Path | None = None) -> list[Box2]:
    """Collision boxes of every static model except the ground plane."""
    sdf = path or _REPO_SDF
    world = ET.parse(sdf).getroot().find("world")
    if world is None:
        raise ValueError(f"{sdf} has no <world>")
    boxes: list[Box2] = []
    for model in world.findall("model"):
        name = model.get("name") or ""
        if name == "ground_plane":
            continue
        model_pose = _pose6(model.findtext("pose"))
        for collision in model.iter("collision"):
            size_el = collision.find(".//box/size")
            if size_el is None or not size_el.text:
                continue
            sx, sy, _sz = (float(item) for item in size_el.text.split()[:3])
            local = _pose6(collision.findtext("pose"))
            cx, cy, yaw = _compose(model_pose, local)
            boxes.append(Box2(cx, cy, sx, sy, yaw))
    if not boxes:
        raise ValueError(f"{sdf} contained no collision boxes")
    return boxes


def rasterize(
    boxes: list[Box2],
    *,
    resolution: float = 0.05,
    pad_m: float = 1.0,
) -> Occupancy:
    xs = [box.cx - 0.5 * box.sx - pad_m for box in boxes] + [
        box.cx + 0.5 * box.sx + pad_m for box in boxes
    ]
    ys = [box.cy - 0.5 * box.sy - pad_m for box in boxes] + [
        box.cy + 0.5 * box.sy + pad_m for box in boxes
    ]
    origin_x = math.floor(min(xs) / resolution) * resolution
    origin_y = math.floor(min(ys) / resolution) * resolution
    width = int(math.ceil((max(xs) - origin_x) / resolution)) + 1
    height = int(math.ceil((max(ys) - origin_y) / resolution)) + 1
    px = origin_x + (np.arange(width) + 0.5) * resolution
    py = origin_y + (np.arange(height) + 0.5) * resolution
    xx, yy = np.meshgrid(px, py)
    occupied = np.zeros((height, width), dtype=bool)
    for box in boxes:
        cosine, sine = math.cos(-box.yaw), math.sin(-box.yaw)
        dx, dy = xx - box.cx, yy - box.cy
        lx = cosine * dx - sine * dy
        ly = sine * dx + cosine * dy
        occupied |= (np.abs(lx) <= 0.5 * box.sx) & (np.abs(ly) <= 0.5 * box.sy)
    return Occupancy(occupied, origin_x, origin_y, resolution)


def occupancy_from_grid(grid: RenderedGrid) -> Occupancy:
    return Occupancy(grid.cells == OCCUPIED, grid.origin_x, grid.origin_y, grid.resolution)


def _occupied_xy(occupancy: Occupancy) -> np.ndarray:
    rows, cols = np.nonzero(occupancy.cells)
    if rows.size == 0:
        return np.zeros((0, 2), dtype=np.float64)
    return np.column_stack(
        [
            occupancy.origin_x + (cols + 0.5) * occupancy.resolution,
            occupancy.origin_y + (rows + 0.5) * occupancy.resolution,
        ]
    )


def _paint_on(
    points: np.ndarray, origin_x: float, origin_y: float, resolution: float, height: int, width: int
) -> np.ndarray:
    cells = np.zeros((height, width), dtype=bool)
    if points.size == 0:
        return cells
    cols = np.floor((points[:, 0] - origin_x) / resolution).astype(int)
    rows = np.floor((points[:, 1] - origin_y) / resolution).astype(int)
    keep = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    cells[rows[keep], cols[keep]] = True
    return cells


def _best_shift(estimated: np.ndarray, truth: np.ndarray) -> tuple[int, int]:
    correlation = fftconvolve(
        truth.astype(np.float64), estimated[::-1, ::-1], mode="same"
    )
    peak = np.unravel_index(int(np.argmax(correlation)), correlation.shape)
    shift_row = int(peak[0] - truth.shape[0] // 2)
    shift_col = int(peak[1] - truth.shape[1] // 2)
    return shift_row, shift_col


def _shift_zero(cells: np.ndarray, drow: int, dcol: int) -> np.ndarray:
    if drow == 0 and dcol == 0:
        return cells
    out = np.zeros_like(cells)
    src_r0 = max(0, -drow)
    src_r1 = cells.shape[0] - max(0, drow)
    dst_r0 = max(0, drow)
    dst_r1 = cells.shape[0] - max(0, -drow)
    src_c0 = max(0, -dcol)
    src_c1 = cells.shape[1] - max(0, dcol)
    dst_c0 = max(0, dcol)
    dst_c1 = cells.shape[1] - max(0, -dcol)
    if src_r1 > src_r0 and src_c1 > src_c0:
        out[dst_r0:dst_r1, dst_c0:dst_c1] = cells[src_r0:src_r1, src_c0:src_c1]
    return out


def _iou(estimated: np.ndarray, truth: np.ndarray) -> tuple[float, float, float, int, int]:
    intersection = int(np.count_nonzero(estimated & truth))
    union = int(np.count_nonzero(estimated | truth))
    predicted = int(np.count_nonzero(estimated))
    actual = int(np.count_nonzero(truth))
    iou = 0.0 if union == 0 else intersection / union
    precision = 0.0 if predicted == 0 else intersection / predicted
    recall = 0.0 if actual == 0 else intersection / actual
    return iou, precision, recall, predicted, actual


def score_occupancy(
    estimated: Occupancy,
    truth: Occupancy,
    *,
    yaw_step_deg: float = 2.0,
) -> OccupancyScore:
    """IoU after the one rigid SE(2) a reconstruction cannot observe."""
    est_xy = _occupied_xy(estimated)
    truth_xy = _occupied_xy(truth)
    if est_xy.size == 0 or truth_xy.size == 0:
        return OccupancyScore(
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            int(est_xy.shape[0]),
            int(truth_xy.shape[0]),
        )
    centroid = est_xy.mean(axis=0)
    resolution = truth.resolution
    best: OccupancyScore | None = None
    for degrees in np.arange(-180.0, 180.0, yaw_step_deg):
        yaw = math.radians(float(degrees))
        cosine, sine = math.cos(yaw), math.sin(yaw)
        rotation = np.array([[cosine, -sine], [sine, cosine]])
        rotated = (est_xy - centroid) @ rotation.T + centroid
        mins = np.minimum(rotated.min(axis=0), truth_xy.min(axis=0)) - resolution
        maxs = np.maximum(rotated.max(axis=0), truth_xy.max(axis=0)) + resolution
        origin_x = math.floor(float(mins[0]) / resolution) * resolution
        origin_y = math.floor(float(mins[1]) / resolution) * resolution
        width = int(math.ceil((maxs[0] - origin_x) / resolution)) + 1
        height = int(math.ceil((maxs[1] - origin_y) / resolution)) + 1
        painted = _paint_on(rotated, origin_x, origin_y, resolution, height, width)
        truth_cells = _paint_on(truth_xy, origin_x, origin_y, resolution, height, width)
        aligned = painted
        shift_row, shift_col = 0, 0
        iou, precision, recall, predicted, actual = _iou(aligned, truth_cells)
        fft_row, fft_col = _best_shift(painted, truth_cells)
        shifted = _shift_zero(painted, fft_row, fft_col)
        shifted_iou, p2, r2, pred2, act2 = _iou(shifted, truth_cells)
        if shifted_iou > iou:
            aligned, iou, precision, recall = shifted, shifted_iou, p2, r2
            predicted, actual = pred2, act2
            shift_row, shift_col = fft_row, fft_col
        candidate = OccupancyScore(
            iou,
            precision,
            recall,
            yaw,
            shift_col * resolution,
            shift_row * resolution,
            predicted,
            actual,
        )
        if best is None or score_beats(candidate, best):
            best = candidate
    assert best is not None
    return best


def score_beats(candidate: OccupancyScore, incumbent: OccupancyScore) -> bool:
    if candidate.iou != incumbent.iou:
        return candidate.iou > incumbent.iou
    return abs(candidate.yaw_rad) < abs(incumbent.yaw_rad)


def overlay_png(estimated: Occupancy, truth: Occupancy, score: OccupancyScore) -> np.ndarray:
    """RGB: truth walls grey, reconstructed hits blue, agreement black."""
    est_xy = _occupied_xy(estimated)
    truth_xy = _occupied_xy(truth)
    centroid = est_xy.mean(axis=0) if est_xy.size else np.zeros(2)
    cosine, sine = math.cos(score.yaw_rad), math.sin(score.yaw_rad)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    rotated = (est_xy - centroid) @ rotation.T + centroid if est_xy.size else est_xy
    rotated = rotated.copy()
    rotated[:, 0] += score.shift_x_m
    rotated[:, 1] += score.shift_y_m
    mins = np.minimum(
        rotated.min(axis=0) if rotated.size else truth_xy.min(axis=0),
        truth_xy.min(axis=0),
    ) - truth.resolution
    maxs = np.maximum(
        rotated.max(axis=0) if rotated.size else truth_xy.max(axis=0),
        truth_xy.max(axis=0),
    ) + truth.resolution
    occupancy = Occupancy(
        np.zeros(
            (
                int(math.ceil((maxs[1] - mins[1]) / truth.resolution)) + 1,
                int(math.ceil((maxs[0] - mins[0]) / truth.resolution)) + 1,
            ),
            dtype=bool,
        ),
        float(mins[0]),
        float(mins[1]),
        truth.resolution,
    )
    aligned = _paint_on(
        rotated, occupancy.origin_x, occupancy.origin_y, occupancy.resolution, occupancy.height, occupancy.width
    )
    truth_cells = _paint_on(
        truth_xy, occupancy.origin_x, occupancy.origin_y, occupancy.resolution, occupancy.height, occupancy.width
    )
    rgb = np.full((occupancy.height, occupancy.width, 3), 255, dtype=np.uint8)
    rgb[truth_cells] = (180, 184, 190)
    rgb[aligned & ~truth_cells] = (40, 90, 200)
    rgb[aligned & truth_cells] = (40, 44, 52)
    return rgb[::-1]
