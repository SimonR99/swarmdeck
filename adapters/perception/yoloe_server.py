#!/usr/bin/env python3
"""Small HTTP sidecar serving prompted YOLOE rubber-duck inference."""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

# The file runs directly in the detector image; make ``adapters`` importable.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from adapters.perception.duck_detector import DuckDetection, suppress_overlaps


MAX_IMAGE_BYTES = 12 * 1024 * 1024


class YoloeDuckModel:
    """One serialized YOLOE model shared by concurrent HTTP handlers."""

    def __init__(self, model_name: str, prompt: str, device: str | None = None) -> None:
        # Import Ultralytics first.  On JetPack 5, importing torch before YOLOE
        # leaves the TensorRT/PyTorch stack initialized in a state where YOLOE
        # silently produces no proposals.
        from ultralytics import YOLOE
        import torch

        self.model_name = model_name
        self.prompt = prompt
        self.device = device or ("0" if torch.cuda.is_available() else "cpu")
        self._lock = threading.Lock()
        self._model = YOLOE(model_name)
        self._model.set_classes([prompt])

    def detect(self, image: np.ndarray, confidence: float) -> list[DuckDetection]:
        height, width = image.shape[:2]
        if width < 16 or height < 16:
            return []
        with self._lock:
            result = self._model.predict(
                image,
                imgsz=640,
                conf=max(0.05, min(0.90, confidence)),
                device=self.device,
                verbose=False,
            )[0]

        found: list[DuckDetection] = []
        for box in result.boxes:
            x0, y0, x1, y1 = (float(value) for value in box.xyxy[0])
            found.append(
                DuckDetection(
                    (
                        max(0.0, x0 / width),
                        max(0.0, y0 / height),
                        max(0.0, (x1 - x0) / width),
                        max(0.0, (y1 - y0) / height),
                    ),
                    float(box.conf[0]),
                )
            )
        return suppress_overlaps(found)


class DetectorHandler(BaseHTTPRequestHandler):
    model: YoloeDuckModel

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        if self.path != "/health":
            self._json(404, {"error": "not found"})
            return
        self._json(
            200,
            {
                "status": "ready",
                "model": self.model.model_name,
                "prompt": self.model.prompt,
                "device": self.model.device,
            },
        )

    def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        if self.path != "/detect":
            self._json(404, {"error": "not found"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > MAX_IMAGE_BYTES:
                raise ValueError("invalid image size")
            confidence = float(self.headers.get("X-SwarmDeck-Confidence", "0.25"))
            encoded = np.frombuffer(self.rfile.read(length), dtype=np.uint8)
            image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError("request is not a decodable image")
            detections = self.model.detect(image, confidence)
            self._json(
                200,
                {
                    "detections": [
                        {
                            "bbox": [round(value, 6) for value in detection.bbox],
                            "score": round(detection.score, 6),
                        }
                        for detection in detections
                    ]
                },
            )
        except (TypeError, ValueError) as exc:
            self._json(400, {"error": str(exc)})
        except Exception as exc:  # model/runtime errors must be visible to health logs
            self.log_error("inference failed: %s", exc)
            self._json(503, {"error": "inference unavailable"})

    def log_message(self, message: str, *args: object) -> None:
        print(f"[duck-detector] {self.address_string()} {message % args}", flush=True)

    def _json(self, status: int, payload: dict) -> None:
        body = json.dumps(payload, separators=(",", ":")).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8091)
    parser.add_argument(
        "--model", default=os.environ.get("SWARMDECK_YOLOE_MODEL", "yoloe-26n-seg.pt")
    )
    parser.add_argument(
        "--prompt", default=os.environ.get("SWARMDECK_YOLOE_PROMPT", "yellow duck toy")
    )
    parser.add_argument("--device", default=os.environ.get("SWARMDECK_YOLO_DEVICE"))
    args = parser.parse_args()

    print(
        f"[duck-detector] loading {args.model!r} prompt={args.prompt!r}",
        flush=True,
    )
    DetectorHandler.model = YoloeDuckModel(args.model, args.prompt, args.device)
    server = ThreadingHTTPServer((args.host, args.port), DetectorHandler)
    print(
        f"[duck-detector] ready on {args.host}:{args.port}; "
        f"device={DetectorHandler.model.device}",
        flush=True,
    )
    server.serve_forever()


if __name__ == "__main__":
    main()
