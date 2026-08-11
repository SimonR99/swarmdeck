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

from adapters.perception.catalog import CATALOG, CLASS_NAMES, prompt_bindings, resolve
from adapters.perception.object_detector import (
    Detection,
    ObjectDetector,
    intersection_over_union,
    suppress_overlaps,
    track_ids,
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
                    {"class": "rubber_duck", "bbox": [0.10, 0.20, 0.20, 0.30], "score": 0.91},
                    {"class": "rubber_duck", "bbox": [0.105, 0.205, 0.195, 0.295], "score": 0.72},
                    {"class": "wooden_block", "bbox": [0.60, 0.50, 0.15, 0.20], "score": 0.84},
                ]
            }
        )

    monkeypatch.setattr("urllib.request.urlopen", urlopen)
    detector = ObjectDetector(endpoint="http://detector:8091")
    detections = detector.detect_bgr(np.zeros((120, 160, 3), dtype=np.uint8))

    assert [item.score for item in detections] == [0.91, 0.84]
    assert [item.label for item in detections] == ["rubber_duck", "wooden_block"]
    assert captured["request"].full_url == "http://detector:8091/detect"
    assert captured["request"].headers["Content-type"] == "image/jpeg"
    assert float(captured["request"].headers["X-swarmdeck-confidence"]) == pytest.approx(
        0.2475
    )
    assert captured["request"].headers["X-swarmdeck-classes"] == ",".join(CLASS_NAMES)
    assert captured["timeout"] == 1.5
    assert detector.last_error is None


def test_yoloe_client_fails_closed_when_sidecar_is_unavailable(monkeypatch):
    def unavailable(_request, timeout):
        raise urllib.error.URLError(f"offline after {timeout}s")

    monkeypatch.setattr("urllib.request.urlopen", unavailable)
    detector = ObjectDetector(endpoint="http://detector:8091")

    assert detector.detect_bgr(np.zeros((120, 160, 3), dtype=np.uint8)) == []
    assert "offline" in detector.last_error


def test_sensitivity_maps_to_confidence_with_original_direction():
    conservative = ObjectDetector(sensitivity=0.1)
    default = ObjectDetector(sensitivity=0.55)
    sensitive = ObjectDetector(sensitivity=1.0)

    assert conservative.confidence_threshold == pytest.approx(0.495)
    assert default.confidence_threshold == pytest.approx(0.2475)
    assert sensitive.confidence_threshold == pytest.approx(0.05)
    # The default sensitivity must leave every class on its calibrated floor,
    # which is what the measured min_score values were chosen against.
    assert default.score_scale == pytest.approx(0.99, abs=0.01)
    assert conservative.score_scale > 1.0 > sensitive.score_scale


def test_iou_suppression_keeps_distinct_objects():
    first = Detection((0.1, 0.2, 0.2, 0.3), 0.9, "rubber_duck")
    duplicate = Detection((0.105, 0.205, 0.195, 0.295), 0.8, "rubber_duck")
    second = Detection((0.6, 0.5, 0.15, 0.2), 0.7, "wooden_block")

    assert intersection_over_union(first.bbox, duplicate.bbox) > 0.9
    assert suppress_overlaps([duplicate, second, first]) == [first, second]


def test_a_duck_on_a_block_survives_but_a_double_labelled_box_does_not():
    """Cross-class overlap is only suppressed when it is near-total."""
    block = Detection((0.30, 0.30, 0.40, 0.40), 0.90, "wooden_block")
    duck_on_top = Detection((0.32, 0.28, 0.34, 0.36), 0.80, "rubber_duck")
    same_object = Detection((0.301, 0.301, 0.398, 0.398), 0.50, "disc_cone")

    kept = suppress_overlaps([block, duck_on_top, same_object])

    assert [item.label for item in kept] == ["wooden_block", "rubber_duck"]


def test_detections_are_slotted_within_their_own_class():
    batch = [
        Detection((0.1, 0.1, 0.1, 0.1), 0.9, "wooden_block"),
        Detection((0.3, 0.1, 0.1, 0.1), 0.8, "rubber_duck"),
        Detection((0.5, 0.1, 0.1, 0.1), 0.7, "wooden_block"),
    ]

    assert [track_id for _, track_id in track_ids(batch)] == [
        "wooden_block_0",
        "rubber_duck_0",
        "wooden_block_1",
    ]


def test_unknown_classes_from_the_sidecar_are_dropped(monkeypatch):
    """A class this build does not know is not a class we can draw or filter."""
    monkeypatch.setattr(
        "urllib.request.urlopen",
        lambda *_a, **_k: Response(
            {
                "detections": [
                    {"class": "loch_ness_monster", "bbox": [0.1, 0.1, 0.2, 0.2], "score": 0.99},
                    {"class": "pool_noodle", "bbox": [0.5, 0.5, 0.2, 0.2], "score": 0.60},
                ]
            }
        ),
    )
    detector = ObjectDetector()
    detections = detector.detect_bgr(np.zeros((120, 160, 3), dtype=np.uint8))

    assert [item.label for item in detections] == ["pool_noodle"]


def test_class_selection_is_normalized_to_catalog_order():
    detector = ObjectDetector(classes=["pool_noodle", "nonsense", "rubber_duck"])
    assert detector.classes == ("rubber_duck", "pool_noodle")

    detector.classes = None
    assert detector.classes == CLASS_NAMES


@pytest.mark.parametrize("empty", [[], (), ["nothing_real"], None, "duck"])
def test_an_empty_selection_widens_rather_than_silently_detecting_nothing(empty):
    """The one failure an operator cannot see is a detector that sees nothing."""
    detector = ObjectDetector(classes=["disc_cone"])
    detector.classes = empty

    assert detector.classes == CLASS_NAMES


def test_polygons_are_taken_whole_or_not_at_all(monkeypatch):
    monkeypatch.setattr(
        "urllib.request.urlopen",
        lambda *_a, **_k: Response(
            {
                "detections": [
                    {
                        "class": "pool_noodle",
                        "bbox": [0.1, 0.1, 0.2, 0.2],
                        "score": 0.6,
                        "polygon": [[0.1, 0.1], [0.3, 0.1], [0.3, 0.3]],
                    },
                    {
                        "class": "disc_cone",
                        "bbox": [0.5, 0.5, 0.2, 0.2],
                        "score": 0.6,
                        "polygon": [[0.5, 0.5], "broken", [0.7, 0.7]],
                    },
                ]
            }
        ),
    )
    detections = ObjectDetector().detect_bgr(np.zeros((120, 160, 3), dtype=np.uint8))

    assert len(detections[0].polygon) == 3
    assert detections[1].polygon == ()


def test_protocol_item_carries_class_and_outline():
    detection = Detection(
        (0.1, 0.2, 0.3, 0.4), 0.876, "disc_cone", ((0.1, 0.2), (0.4, 0.2), (0.4, 0.6))
    )
    item = detection.as_protocol("disc_cone_0")

    assert item["class"] == "disc_cone"
    assert item["score"] == 0.876
    assert item["polygon"] == [[0.1, 0.2], [0.4, 0.2], [0.4, 0.6]]
    assert item["map_position"] is None


@pytest.mark.parametrize(
    "shape",
    [(10, 10, 3), (100, 100), (100, 100, 4)],
)
def test_invalid_camera_frames_are_ignored_without_network(shape, monkeypatch):
    urlopen = monkeypatch.setattr(
        "urllib.request.urlopen",
        lambda *_args, **_kwargs: pytest.fail("network should not be used"),
    )
    detector = ObjectDetector()

    assert detector.detect_bgr(np.zeros(shape, dtype=np.uint8)) == []
    assert urlopen is None


def test_catalog_prompts_are_unique_and_owned():
    """Two classes sharing a prompt would make the model's answer ambiguous."""
    prompts, owners = prompt_bindings()

    assert len(prompts) == len(set(prompts))
    assert len(prompts) == len(owners)
    assert set(owners) == set(CLASS_NAMES)
    assert all(0.0 < target.min_score < 1.0 for target in CATALOG)


def test_resolving_an_empty_selection_means_everything():
    assert resolve([]) == CATALOG
    assert resolve(None) == CATALOG
    assert resolve(["nothing_real"]) == CATALOG
    assert [target.name for target in resolve(["disc_cone"])] == ["disc_cone"]
