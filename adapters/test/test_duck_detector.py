import json
import importlib.util
import sys
import urllib.error
from types import SimpleNamespace

import numpy as np
import pytest

# Adapter tests run in the deliberately ROS/CV-free backend virtualenv. The
# client only needs OpenCV to encode its HTTP payload, so provide that narrow
# seam here when the real package is absent instead of adding OpenCV to the
# backend's production dependency set.
if importlib.util.find_spec("cv2") is None:
    sys.modules["cv2"] = SimpleNamespace(
        IMWRITE_JPEG_QUALITY=1,
        imencode=lambda *_args, **_kwargs: (
            True,
            np.array([0xFF, 0xD8, 0xFF, 0xD9], dtype=np.uint8),
        ),
    )

from adapters.perception.duck_detector import (
    DuckDetection,
    RubberDuckDetector,
    intersection_over_union,
    suppress_overlaps,
)


class Response:
    def __init__(self, payload):
        self.payload = payload

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False

    def read(self):
        return json.dumps(self.payload).encode()


def test_yoloe_client_posts_jpeg_and_suppresses_duplicate_boxes(monkeypatch):
    captured = {}

    def urlopen(request, timeout):
        captured["request"] = request
        captured["timeout"] = timeout
        return Response(
            {
                "detections": [
                    {"bbox": [0.10, 0.20, 0.20, 0.30], "score": 0.91},
                    {"bbox": [0.105, 0.205, 0.195, 0.295], "score": 0.72},
                    {"bbox": [0.60, 0.50, 0.15, 0.20], "score": 0.84},
                ]
            }
        )

    monkeypatch.setattr("urllib.request.urlopen", urlopen)
    detector = RubberDuckDetector(endpoint="http://detector:8091")
    detections = detector.detect_bgr(np.zeros((120, 160, 3), dtype=np.uint8))

    assert [item.score for item in detections] == [0.91, 0.84]
    assert captured["request"].full_url == "http://detector:8091/detect"
    assert captured["request"].headers["Content-type"] == "image/jpeg"
    assert float(captured["request"].headers["X-swarmdeck-confidence"]) == pytest.approx(
        0.2475
    )
    assert captured["timeout"] == 1.5
    assert detector.last_error is None


def test_yoloe_client_fails_closed_when_sidecar_is_unavailable(monkeypatch):
    def unavailable(_request, timeout):
        raise urllib.error.URLError(f"offline after {timeout}s")

    monkeypatch.setattr("urllib.request.urlopen", unavailable)
    detector = RubberDuckDetector(endpoint="http://detector:8091")

    assert detector.detect_bgr(np.zeros((120, 160, 3), dtype=np.uint8)) == []
    assert "offline" in detector.last_error


def test_sensitivity_maps_to_confidence_with_original_direction():
    conservative = RubberDuckDetector(sensitivity=0.1)
    default = RubberDuckDetector(sensitivity=0.55)
    sensitive = RubberDuckDetector(sensitivity=1.0)

    assert conservative.confidence_threshold == pytest.approx(0.495)
    assert default.confidence_threshold == pytest.approx(0.2475)
    assert sensitive.confidence_threshold == pytest.approx(0.05)


def test_iou_suppression_keeps_distinct_ducks():
    first = DuckDetection((0.1, 0.2, 0.2, 0.3), 0.9)
    duplicate = DuckDetection((0.105, 0.205, 0.195, 0.295), 0.8)
    second = DuckDetection((0.6, 0.5, 0.15, 0.2), 0.7)

    assert intersection_over_union(first.bbox, duplicate.bbox) > 0.9
    assert suppress_overlaps([duplicate, second, first]) == [first, second]


@pytest.mark.parametrize(
    "shape",
    [(10, 10, 3), (100, 100), (100, 100, 4)],
)
def test_invalid_camera_frames_are_ignored_without_network(shape, monkeypatch):
    urlopen = monkeypatch.setattr(
        "urllib.request.urlopen",
        lambda *_args, **_kwargs: pytest.fail("network should not be used"),
    )
    detector = RubberDuckDetector()

    assert detector.detect_bgr(np.zeros(shape, dtype=np.uint8)) == []
    assert urlopen is None
