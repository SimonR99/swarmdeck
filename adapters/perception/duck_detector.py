"""Portable client for SwarmDeck's YOLOE rubber-duck detector.

ROS 1, ROS 2 and Gazebo deliberately keep their camera and depth handling in
their existing adapters.  Neural inference lives in a local sidecar so those
three very different runtime images do not each need their own PyTorch/CUDA
installation.  This module is the small, fail-closed boundary shared by all of
them: BGR pixels in, normalized protocol boxes out.
"""

from __future__ import annotations

import json
import math
import os
import urllib.error
import urllib.request
from dataclasses import dataclass

import cv2
import numpy as np


DEFAULT_DETECTOR_URL = "http://127.0.0.1:8091"


@dataclass(frozen=True)
class DuckDetection:
    bbox: tuple[float, float, float, float]
    score: float

    def as_protocol(self, track_id: str) -> dict:
        return {
            "id": track_id,
            "class": "rubber_duck",
            "score": round(self.score, 3),
            "bbox": [round(value, 5) for value in self.bbox],
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
    detections: list[DuckDetection], threshold: float = 0.65
) -> list[DuckDetection]:
    """Suppress duplicate YOLOE proposals while preserving distinct ducks."""
    kept: list[DuckDetection] = []
    for candidate in sorted(detections, key=lambda item: item.score, reverse=True):
        if any(intersection_over_union(candidate.bbox, item.bbox) >= threshold for item in kept):
            continue
        kept.append(candidate)
    return kept[:8]


class RubberDuckDetector:
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
    ) -> None:
        self.sensitivity = max(0.1, min(1.0, float(sensitivity)))
        self.endpoint = (
            endpoint
            or os.environ.get("SWARMDECK_DUCK_DETECTOR_URL")
            or DEFAULT_DETECTOR_URL
        ).rstrip("/")
        self.timeout_s = max(0.1, float(timeout_s))
        self.last_error: str | None = None

    @property
    def confidence_threshold(self) -> float:
        """Map the existing sensitivity control onto a YOLO confidence floor.

        Higher sensitivity accepts weaker proposals, matching the dashboard's
        previous semantics.  The default 0.55 maps to 0.25, which retained both
        physical Botman ducks during live validation.
        """
        return max(0.05, min(0.50, 0.55 - 0.55 * self.sensitivity))

    def detect_bgr(self, image: np.ndarray) -> list[DuckDetection]:
        if image.ndim != 3 or image.shape[2] != 3:
            return []
        height, width = image.shape[:2]
        if width < 16 or height < 16:
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
    def _parse_response(payload: object) -> list[DuckDetection]:
        if not isinstance(payload, dict) or not isinstance(payload.get("detections"), list):
            raise ValueError("detector response has no detections list")

        parsed: list[DuckDetection] = []
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
            x = max(0.0, min(1.0, x))
            y = max(0.0, min(1.0, y))
            width = max(0.0, min(1.0 - x, width))
            height = max(0.0, min(1.0 - y, height))
            if width <= 0.0 or height <= 0.0 or not 0.0 <= score <= 1.0:
                continue
            parsed.append(DuckDetection((x, y, width, height), score))
        return parsed
