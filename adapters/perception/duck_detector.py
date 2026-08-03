"""Portable rubber-duck detector operating only on BGR camera pixels.

There are deliberately no ROS or Gazebo imports here.  A physical-robot adapter
can feed the same detector OpenCV frames.  The baseline combines yellow body and
orange beak evidence; the public ``detect_bgr`` contract is also the seam where a
custom ONNX detector can be installed later without touching transport or UI.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


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


class RubberDuckDetector:
    """Detect yellow bodies that have an adjacent orange beak.

    This conservative two-colour shape test avoids reporting yellow robot
    wheels, batteries and furniture as ducks.  Thresholds are intentionally
    broad enough for ordinary indoor camera exposure, not tuned to Gazebo IDs.
    """

    def __init__(self, sensitivity: float = 0.55) -> None:
        self.sensitivity = max(0.1, min(1.0, float(sensitivity)))
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    def detect_bgr(self, image: np.ndarray) -> list[DuckDetection]:
        if image.ndim != 3 or image.shape[2] != 3:
            return []
        height, width = image.shape[:2]
        if width < 16 or height < 16:
            return []

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        yellow = cv2.inRange(hsv, (18, 85, 75), (42, 255, 255))
        orange = cv2.inRange(hsv, (3, 115, 70), (18, 255, 255))
        yellow = cv2.morphologyEx(yellow, cv2.MORPH_CLOSE, self.kernel)
        yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, self.kernel)

        min_area = max(28.0, width * height * (0.00055 - 0.00035 * self.sensitivity))
        contours, _ = cv2.findContours(yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        found: list[DuckDetection] = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if w < 7 or h < 7 or w / max(h, 1) > 2.4 or h / max(w, 1) > 2.8:
                continue
            fill = area / max(1.0, w * h)
            if fill < 0.22:
                continue

            pad_x, pad_y = max(4, w // 2), max(3, h // 3)
            x0, y0 = max(0, x - pad_x), max(0, y - pad_y)
            x1, y1 = min(width, x + w + pad_x), min(height, y + h + pad_y)
            orange_pixels = int(cv2.countNonZero(orange[y0:y1, x0:x1]))
            required_orange = max(3, int(area * (0.018 - 0.010 * self.sensitivity)))
            if orange_pixels < required_orange:
                continue

            # Include the beak search area, but do not let a remote orange
            # object create an enormous box.
            orange_roi = orange[y0:y1, x0:x1]
            points = cv2.findNonZero(orange_roi)
            if points is not None:
                ox, oy, ow, oh = cv2.boundingRect(points)
                bx0, by0 = min(x, x0 + ox), min(y, y0 + oy)
                bx1, by1 = max(x + w, x0 + ox + ow), max(y + h, y0 + oy + oh)
            else:
                bx0, by0, bx1, by1 = x, y, x + w, y + h

            margin = 3
            bx0, by0 = max(0, bx0 - margin), max(0, by0 - margin)
            bx1, by1 = min(width, bx1 + margin), min(height, by1 + margin)
            score = min(0.98, 0.58 + 0.18 * fill + min(0.20, orange_pixels / max(area, 1) * 2.0))
            found.append(
                DuckDetection(
                    (bx0 / width, by0 / height, (bx1 - bx0) / width, (by1 - by0) / height),
                    score,
                )
            )

        # Colour segmentation can split a real duck into a large body proposal
        # plus a smaller highlight/face proposal.  Suppress proposals contained
        # by a larger one.  Area is considered before score deliberately: the
        # small saturated patch often has a slightly higher colour score than
        # the useful full-object box.
        by_area = sorted(found, key=lambda item: item.bbox[2] * item.bbox[3], reverse=True)
        kept: list[DuckDetection] = []
        for candidate in by_area:
            if any(self._contained_overlap(candidate.bbox, item.bbox) > 0.72 for item in kept):
                continue
            kept.append(candidate)
        return sorted(kept, key=lambda item: item.score, reverse=True)[:8]

    @staticmethod
    def _contained_overlap(
        first: tuple[float, float, float, float],
        second: tuple[float, float, float, float],
    ) -> float:
        """Intersection divided by the smaller box area."""
        ax, ay, aw, ah = first
        bx, by, bw, bh = second
        ix = max(0.0, min(ax + aw, bx + bw) - max(ax, bx))
        iy = max(0.0, min(ay + ah, by + bh) - max(ay, by))
        return ix * iy / max(1e-9, min(aw * ah, bw * bh))
