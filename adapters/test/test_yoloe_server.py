import io
import threading
from email.message import Message

import cv2
import numpy as np
import pytest

from adapters.perception import catalog
from adapters.perception import yoloe_server
from adapters.perception.yoloe_server import (
    DetectorHandler,
    InferenceBusy,
    YoloeModel,
)


class _EmptyResult:
    boxes = ()
    masks = None


def _model_with_blocking_predict(started: threading.Event, release: threading.Event):
    class FakeModel:
        def predict(self, *args, **kwargs):
            started.set()
            assert release.wait(2), "fake inference was not released"
            return [_EmptyResult()]

    model = YoloeModel.__new__(YoloeModel)
    model.model_name = "test"
    model.device = "cpu"
    model._lock = threading.Lock()
    model._model = FakeModel()
    model.prompts, model.owners = catalog.prompt_bindings()
    return model


def test_model_rejects_overlapping_inference_without_queuing():
    started = threading.Event()
    release = threading.Event()
    model = _model_with_blocking_predict(started, release)
    image = np.zeros((32, 32, 3), dtype=np.uint8)
    result: list[object] = []

    first = threading.Thread(
        target=lambda: result.append(
            model.detect(image, 1.0, (catalog.CATALOG[0].name,))
        )
    )
    first.start()
    assert started.wait(2)
    with pytest.raises(InferenceBusy, match="inference busy"):
        model.detect(image, 1.0, (catalog.CATALOG[0].name,))
    release.set()
    first.join(timeout=2)
    assert not first.is_alive()
    assert result == [[]]


def test_handler_returns_busy_without_logging_an_inference_failure(monkeypatch):
    class BusyModel:
        def detect(self, *args, **kwargs):
            raise InferenceBusy("inference busy")

    handler = DetectorHandler.__new__(DetectorHandler)
    handler.model = BusyModel()
    handler.path = "/detect"
    monkeypatch.setattr(
        yoloe_server.cv2,
        "imdecode",
        lambda encoded, mode: np.zeros((32, 32, 3), dtype=np.uint8),
        raising=False,
    )
    monkeypatch.setattr(yoloe_server.cv2, "IMREAD_COLOR", 1, raising=False)
    ok, encoded = cv2.imencode(".jpg", np.zeros((32, 32, 3), dtype=np.uint8))
    assert ok
    handler.rfile = io.BytesIO(encoded.tobytes())
    handler.headers = Message()
    handler.headers["Content-Length"] = str(len(encoded))
    response: list[tuple[int, dict]] = []
    handler._json = lambda status, payload: response.append((status, payload))
    handler.do_POST()
    assert response == [(503, {"error": "inference busy"})]
