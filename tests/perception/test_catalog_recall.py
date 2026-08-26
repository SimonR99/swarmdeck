"""Does the prompted detector actually see the objects we claim it sees?

Every other perception test in this repo mocks the model away, which is right:
adapters must be testable without PyTorch.  But the catalog's prompts and score
floors are empirical claims about a neural network, and nothing above this
level can tell a good prompt from a bad one.  So this file runs the real
weights against the reference photographs in ``tests/perception/fixtures/``.

It is therefore not part of ``make test``.  Run it inside the detector image,
which is the only place the model and its weights exist:

    docker run --rm \\
      -v swarmdeck_duck_detector_models:/models \\
      -v "$PWD:/app" -e YOLO_CONFIG_DIR=/tmp/ul \\
      --entrypoint python3 swarmdeck-duck-detector:cpu \\
      -m pytest /app/tests/perception -q

The photographs are close-ups taken by hand -- the objects are far larger in
frame than they will be from a robot's camera across a room -- so treat a pass
here as "the prompt binds to the right concept", not as a field accuracy
figure.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

cv2 = pytest.importorskip(
    "cv2", reason="OpenCV is required only inside the duck-detector image"
)

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from adapters.perception import catalog  # noqa: E402
from adapters.perception.yoloe_server import YoloeModel  # noqa: E402

FIXTURES = REPO / "tests" / "perception" / "fixtures"

#: image stem -> the class that photograph is of.
EXPECTED = {
    "cubes": "wooden_block",
    "disc_cone": "disc_cone",
    "filament": "filament_spool",
    "noodle": "pool_noodle",
}


@pytest.fixture(scope="module")
def model() -> YoloeModel:
    weights = Path("/models/yoloe-26n-seg.pt")
    if not weights.exists():  # pragma: no cover - depends on the image's volume
        pytest.skip(f"{weights} not present; run inside the detector image")
    return YoloeModel(str(weights), device="cpu")


@pytest.fixture(scope="module")
def frames() -> dict:
    loaded = {}
    for stem in EXPECTED:
        path = FIXTURES / f"{stem}.jpg"
        if not path.exists():  # pragma: no cover - reference set is committed
            pytest.skip(f"missing reference photograph {path}")
        loaded[stem] = cv2.imread(str(path))
    return loaded


@pytest.mark.parametrize("stem,expected", sorted(EXPECTED.items()))
def test_reference_photograph_yields_its_own_class(model, frames, stem, expected):
    """At the default sensitivity, each photograph's object is found."""
    detections = model.detect(frames[stem], scale=1.0, classes=catalog.CLASS_NAMES)
    found = {item.label for item in detections}

    assert expected in found, f"{stem}.jpg: expected {expected}, got " + (
        ", ".join(f"{d.label}@{d.score:.2f}" for d in detections) or "nothing"
    )


@pytest.mark.parametrize("stem,expected", sorted(EXPECTED.items()))
def test_detections_carry_a_usable_outline(model, frames, stem, expected):
    """The segmentation half: a box without a mask is a depth reading of the floor."""
    detections = [
        item
        for item in model.detect(frames[stem], scale=1.0, classes=catalog.CLASS_NAMES)
        if item.label == expected
    ]
    assert detections, f"{stem}.jpg produced no {expected}"

    best = max(detections, key=lambda item: item.score)
    assert (
        len(best.polygon) >= 3
    ), f"{stem}.jpg: {expected} came back without an outline"
    assert all(0.0 <= x <= 1.0 and 0.0 <= y <= 1.0 for x, y in best.polygon)


def test_selecting_one_class_excludes_the_others(model, frames):
    """The per-request class filter is what the dashboard toggles drive."""
    detections = model.detect(frames["cubes"], scale=1.0, classes=("rubber_duck",))

    assert {item.label for item in detections} <= {"rubber_duck"}


def test_lowering_sensitivity_cannot_add_detections(model, frames):
    """Sensitivity must stay monotonic: the floors scale, they do not invert."""
    strict = model.detect(frames["noodle"], scale=2.0, classes=catalog.CLASS_NAMES)
    default = model.detect(frames["noodle"], scale=1.0, classes=catalog.CLASS_NAMES)

    assert len(strict) <= len(default)
