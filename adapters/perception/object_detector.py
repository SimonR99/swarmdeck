"""Portable client for SwarmDeck's prompted YOLOE detector.

ROS 1, ROS 2 and Gazebo deliberately keep their camera and depth handling in
their existing adapters.  Neural inference lives in a local sidecar so those
three very different runtime images do not each need their own PyTorch/CUDA
installation.  This module is the small, fail-closed boundary shared by all of
them: BGR pixels in, normalized protocol detections out.

The sidecar segments rather than merely boxes, so a detection carries an
outline as well as a rectangle.  That outline is what makes a depth reading
trustworthy: a pool noodle lying diagonally filled 27% of its own bounding box
in the reference frame, and the rest of that box is floor.
"""

from __future__ import annotations

import base64
import json
import math
import os
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any, Sequence, Tuple, List, Optional, Union

import cv2
import numpy as np

from adapters.perception.catalog import CALIBRATED_CONFIDENCE, CATALOG, CLASS_NAMES, resolve


def crop_detection_jpeg_base64(
    image: np.ndarray,
    bbox: tuple[float, float, float, float] | list[float],
    margin: float = 0.20,
    max_dim: int = 240,
    jpeg_quality: int = 85,
) -> str | None:
    """Crop detection bounding box with +20% margin and encode to base64 JPEG."""
    if image is None or not isinstance(image, np.ndarray) or image.size == 0 or image.ndim != 3:
        return None
    try:
        h, w = image.shape[:2]
        if w <= 0 or h <= 0:
            return None
        bx, by, bw, bh = bbox
        if bw <= 0 or bh <= 0:
            return None

        cx = bx + bw / 2.0
        cy = by + bh / 2.0
        exp_w = bw * (1.0 + margin)
        exp_h = bh * (1.0 + margin)

        nx0 = max(0.0, cx - exp_w / 2.0)
        ny0 = max(0.0, cy - exp_h / 2.0)
        nx1 = min(1.0, cx + exp_w / 2.0)
        ny1 = min(1.0, cy + exp_h / 2.0)

        px0 = int(round(nx0 * w))
        py0 = int(round(ny0 * h))
        px1 = int(round(nx1 * w))
        py1 = int(round(ny1 * h))

        if px1 <= px0 or py1 <= py0:
            return None

        crop = image[py0:py1, px0:px1]
        ch, cw = crop.shape[:2]
        if ch == 0 or cw == 0:
            return None

        if max(ch, cw) > max_dim:
            scale = max_dim / float(max(ch, cw))
            new_w = max(1, int(round(cw * scale)))
            new_h = max(1, int(round(ch * scale)))
            crop = cv2.resize(crop, (new_w, new_h), interpolation=cv2.INTER_AREA)

        ok, encoded = cv2.imencode(".jpg", crop, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
        if not ok:
            return None

        b64 = base64.b64encode(encoded.tobytes()).decode("ascii")
        return f"data:image/jpeg;base64,{b64}"
    except Exception:
        return None


DEFAULT_DETECTOR_URL = "http://127.0.0.1:8091"

#: Enough outline to be worth projecting through, few enough to stay cheap on
#: the websocket at several frames a second.
MAX_POLYGON_POINTS = 24

#: Lowest / highest an operator floor is allowed to sit.  Below this the
#: sidecar would accept noise; above it a class can never fire.
FLOOR_MIN = 0.05
FLOOR_MAX = 0.95


def default_class_floors() -> dict[str, float]:
    return {target.name: float(target.min_score) for target in CATALOG}


def format_class_floors(floors: dict[str, float], classes: tuple[str, ...]) -> str:
    return ",".join(
        f"{name}:{floors[name]:.2f}" for name in classes if name in floors
    )


def parse_class_floors(header: str | None) -> dict[str, float]:
    """Read ``name:0.40,...`` from the detector request header."""
    if not header:
        return {}
    floors: dict[str, float] = {}
    for part in header.split(","):
        name, sep, raw = part.partition(":")
        if not sep:
            continue
        key = name.strip()
        if not key:
            continue
        try:
            floors[key] = max(FLOOR_MIN, min(FLOOR_MAX, float(raw)))
        except (TypeError, ValueError):
            continue
    return floors


@dataclass(frozen=True)
class Detection:
    bbox: tuple[float, float, float, float]
    score: float
    #: Catalog class name.  Required: a detection that defaulted to some class
    #: would put a confident label on the map with nothing behind it.
    label: str
    #: Normalized object outline, or empty when the model returned no mask.
    polygon: tuple[tuple[float, float], ...] = ()

    def as_protocol(self, track_id: str) -> dict:
        return {
            "id": track_id,
            "class": self.label,
            "score": round(self.score, 3),
            "bbox": [round(value, 5) for value in self.bbox],
            "polygon": (
                [[round(x, 4), round(y, 4)] for x, y in self.polygon]
                if self.polygon
                else None
            ),
            "map_position": None,
        }


def intersection_over_union(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> float:
    """Return IoU for normalized ``x, y, width, height`` boxes."""
    ax, ay, aw, ah = first
    bx, by, bw, bh = second
    ix = max(0.0, min(ax + aw, bx + bw) - max(ax, bx))
    iy = max(0.0, min(ay + ah, by + bh) - max(ay, by))
    intersection = ix * iy
    union = aw * ah + bw * bh - intersection
    return intersection / max(1e-9, union)


def suppress_overlaps(
    detections: list[Detection],
    threshold: float = 0.65,
    cross_class_threshold: float = 0.80,
) -> list[Detection]:
    """Drop duplicate proposals while preserving distinct objects.

    Two thresholds, because the prompts compete.  Within a class, near-copies
    of one object are what we are removing.  Across classes the overlap has to
    be near-total before we act: a duck sitting on a wooden block genuinely
    produces two boxes that share most of their area, and only the higher score
    of a nearly identical pair is a prompt claiming another prompt's object.
    """
    kept: list[Detection] = []
    for candidate in sorted(detections, key=lambda item: item.score, reverse=True):
        if any(
            intersection_over_union(candidate.bbox, item.bbox)
            >= (threshold if candidate.label == item.label else cross_class_threshold)
            for item in kept
        ):
            continue
        kept.append(candidate)
    return kept[:8]


def track_ids(detections: list[Detection]) -> list[tuple[Detection, str]]:
    """Pair each detection with the id the protocol will carry.

    Slots are numbered within a class, not across the batch, because the
    backend keys its detection table on this id.  A global index would move a
    duck from ``rubber_duck_1`` to ``rubber_duck_0`` the moment a block ahead of
    it in the batch dropped below threshold, and the operator would watch one
    marker become two.
    """
    counts: dict[str, int] = {}
    paired: list[tuple[Detection, str]] = []
    for detection in detections:
        index = counts.get(detection.label, 0)
        counts[detection.label] = index + 1
        paired.append((detection, f"{detection.label}_{index}"))
    return paired


class ObjectDetector:
    """Send throttled camera frames to the local YOLOE inference sidecar.

    An unavailable model must never interrupt camera upload or robot control.
    ``detect_bgr`` therefore returns an empty batch and records ``last_error``
    instead of leaking transport/model failures into a ROS callback.
    """

    def __init__(
        self,
        sensitivity: float = 0.55,
        endpoint: str | None = None,
        timeout_s: float = 1.5,
        classes: object = None,
    ) -> None:
        self.sensitivity = max(0.1, min(1.0, float(sensitivity)))
        self.endpoint = (
            endpoint
            or os.environ.get("SWARMDECK_DETECTOR_URL")
            # The pre-catalog name, still set by deployed robot compose files.
            or os.environ.get("SWARMDECK_DUCK_DETECTOR_URL")
            or DEFAULT_DETECTOR_URL
        ).rstrip("/")
        self.timeout_s = max(0.1, float(timeout_s))
        self.last_error: str | None = None
        self.classes = classes
        self.class_floors = None

    @property
    def classes(self) -> tuple[str, ...]:
        """Which catalog classes this detector asks for, in catalog order."""
        return self._classes

    @classes.setter
    def classes(self, names: object) -> None:
        """Narrow the detector, or -- given nothing usable -- widen it.

        An empty or unrecognisable selection means every class, matching
        ``catalog.resolve`` and the backend's validation.  Detection is turned
        off with the dashboard's own switch; a class list that could empty
        itself would be a second off switch that nothing displays.
        """
        self._classes = tuple(target.name for target in resolve(names))

    @property
    def class_floors(self) -> dict[str, float]:
        """Per-class score floors the sidecar will actually apply."""
        return dict(self._class_floors)

    @class_floors.setter
    def class_floors(self, raw: object) -> None:
        """Overlay operator floors on the catalog defaults.

        Unknown names are ignored.  Missing keys keep the catalog floor so a
        partial settings payload cannot silently drop a class.
        """
        floors = default_class_floors()
        if isinstance(raw, dict):
            for name, value in raw.items():
                key = str(name).strip()
                if key not in floors:
                    continue
                try:
                    floors[key] = max(FLOOR_MIN, min(FLOOR_MAX, float(value)))
                except (TypeError, ValueError):
                    pass
        self._class_floors = floors

    @property
    def confidence_threshold(self) -> float:
        """Map the dashboard's sensitivity control onto a per-class scale.

        Each class carries its own calibrated floor, so this returns a
        multiplier for those floors rather than an absolute confidence: 0.55 --
        the default -- maps to ``CALIBRATED_CONFIDENCE`` and therefore a scale
        of exactly 1.0, preserving the behaviour the control had when there was
        only one class.  Raising sensitivity lowers every floor together.
        """
        return max(0.05, min(0.50, 0.55 - 0.55 * self.sensitivity))

    @property
    def score_scale(self) -> float:
        return self.confidence_threshold / CALIBRATED_CONFIDENCE

    def detect_bgr(self, image: np.ndarray) -> list[Detection]:
        if image.ndim != 3 or image.shape[2] != 3:
            return []
        height, width = image.shape[:2]
        if width < 16 or height < 16:
            return []
        if not self._classes:
            return []

        ok, encoded = cv2.imencode(
            ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 88]
        )
        if not ok:
            return []

        request = urllib.request.Request(
            f"{self.endpoint}/detect",
            data=encoded.tobytes(),
            headers={
                "Content-Type": "image/jpeg",
                "X-SwarmDeck-Confidence": f"{self.confidence_threshold:.4f}",
                "X-SwarmDeck-Classes": ",".join(self._classes),
                "X-SwarmDeck-Class-Floors": format_class_floors(
                    self._class_floors, self._classes
                ),
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                payload = json.loads(response.read())
            detections = self._parse_response(payload)
            self.last_error = None
            return suppress_overlaps(detections)
        except (
            OSError,
            TimeoutError,
            urllib.error.URLError,
            ValueError,
            TypeError,
            KeyError,
            json.JSONDecodeError,
        ) as exc:
            self.last_error = str(exc)
            return []

    @staticmethod
    def _parse_polygon(raw: object) -> tuple[tuple[float, float], ...]:
        """Read an outline, or nothing at all -- never a partial one.

        A truncated polygon is worse than no polygon: depth sampling would
        happily take the median inside whatever shape it was handed.
        """
        if not isinstance(raw, list) or len(raw) < 3:
            return ()
        points: list[tuple[float, float]] = []
        for item in raw[:MAX_POLYGON_POINTS]:
            if not isinstance(item, (list, tuple)) or len(item) != 2:
                return ()
            try:
                x, y = (float(value) for value in item)
            except (TypeError, ValueError):
                return ()
            if not math.isfinite(x) or not math.isfinite(y):
                return ()
            points.append((max(0.0, min(1.0, x)), max(0.0, min(1.0, y))))
        return tuple(points) if len(points) >= 3 else ()

    @staticmethod
    def _parse_response(payload: object) -> list[Detection]:
        if not isinstance(payload, dict) or not isinstance(payload.get("detections"), list):
            raise ValueError("detector response has no detections list")

        parsed: list[Detection] = []
        for item in payload["detections"]:
            if not isinstance(item, dict):
                continue
            bbox = item.get("bbox")
            if not isinstance(bbox, list) or len(bbox) != 4:
                continue
            try:
                x, y, width, height = (float(value) for value in bbox)
                score = float(item["score"])
            except (KeyError, TypeError, ValueError):
                continue
            if not all(math.isfinite(value) for value in (x, y, width, height, score)):
                continue
            label = str(item.get("class", "")).strip()
            if label not in CLASS_NAMES:
                continue
            x = max(0.0, min(1.0, x))
            y = max(0.0, min(1.0, y))
            width = max(0.0, min(1.0 - x, width))
            height = max(0.0, min(1.0 - y, height))
            if width <= 0.0 or height <= 0.0 or not 0.0 <= score <= 1.0:
                continue
            parsed.append(
                Detection(
                    (x, y, width, height),
                    score,
                    label,
                    ObjectDetector._parse_polygon(item.get("polygon")),
                )
            )
        return parsed
