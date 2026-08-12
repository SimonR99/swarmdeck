#!/usr/bin/env python3
"""Small HTTP sidecar serving prompted YOLOE open-vocabulary segmentation."""

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
from adapters.perception import catalog
from adapters.perception.object_detector import (
    MAX_POLYGON_POINTS,
    Detection,
    parse_class_floors,
    suppress_overlaps,
)


MAX_IMAGE_BYTES = 12 * 1024 * 1024


def outline(mask: np.ndarray) -> tuple[tuple[float, float], ...]:
    """Reduce a YOLOE instance mask to a normalized polygon.

    Only the largest contour survives.  A mask that broke into pieces -- the
    far end of a pool noodle behind a chair leg -- is better represented by the
    piece we are sure about than by a hull spanning the gap, because the whole
    point of carrying the outline is to sample depth on the object only.
    """
    if mask is None or mask.size == 0:
        return ()
    binary = (mask > 0.5).astype(np.uint8)
    if int(binary.sum()) < 9:
        return ()
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return ()
    largest = max(contours, key=cv2.contourArea)
    if len(largest) < 3:
        return ()

    # Simplify until the point budget is met; the tolerance is a fraction of
    # the contour's own perimeter, so it adapts to object size.
    perimeter = cv2.arcLength(largest, True)
    simplified = largest
    for epsilon in (0.004, 0.008, 0.016, 0.032):
        simplified = cv2.approxPolyDP(largest, epsilon * perimeter, True)
        if len(simplified) <= MAX_POLYGON_POINTS:
            break
    points = simplified.reshape(-1, 2)[:MAX_POLYGON_POINTS]
    if len(points) < 3:
        return ()

    # Masks come back on the model's own grid, not the source image's.
    mask_h, mask_w = binary.shape[:2]
    return tuple(
        (
            float(max(0.0, min(1.0, x / mask_w))),
            float(max(0.0, min(1.0, y / mask_h))),
        )
        for x, y in points
    )


class YoloeModel:
    """One serialized YOLOE model shared by concurrent HTTP handlers.

    The whole catalog is bound once at startup.  Binding text prompts runs the
    MobileCLIP encoder, which is far too slow to redo per request, so a request
    that wants a subset filters the answers instead of rebinding the model.
    """

    def __init__(self, model_name: str, device: str | None = None) -> None:
        # Import Ultralytics first.  On JetPack 5, importing torch before YOLOE
        # leaves the TensorRT/PyTorch stack initialized in a state where YOLOE
        # silently produces no proposals.
        from ultralytics import YOLOE
        import torch

        self.model_name = model_name
        self.prompts, self.owners = catalog.prompt_bindings()
        self.device = device or ("0" if torch.cuda.is_available() else "cpu")
        self._lock = threading.Lock()
        self._model = YOLOE(model_name)
        self._model.set_classes(list(self.prompts))

    def detect(
        self,
        image: np.ndarray,
        scale: float,
        classes: tuple[str, ...],
        floors: dict[str, float] | None = None,
    ) -> list[Detection]:
        height, width = image.shape[:2]
        if width < 16 or height < 16 or not classes:
            return []

        active = {}
        for target in catalog.CATALOG:
            if target.name not in classes:
                continue
            if floors and target.name in floors:
                floor = floors[target.name]
            else:
                floor = target.min_score * scale
            active[target.name] = max(0.02, min(0.95, float(floor)))
        if not active:
            return []

        with self._lock:
            result = self._model.predict(
                image,
                imgsz=640,
                # Let the model return anything the most permissive class would
                # accept; the per-class floors below are what actually decide.
                conf=min(active.values()),
                device=self.device,
                verbose=False,
            )[0]

        masks = None
        if result.masks is not None:
            masks = result.masks.data.cpu().numpy()

        found: list[Detection] = []
        for index, box in enumerate(result.boxes):
            prompt_index = int(box.cls[0])
            if not 0 <= prompt_index < len(self.owners):
                continue
            label = self.owners[prompt_index]
            score = float(box.conf[0])
            if label not in active or score < active[label]:
                continue
            x0, y0, x1, y1 = (float(value) for value in box.xyxy[0])
            polygon = ()
            if masks is not None and index < len(masks):
                polygon = outline(masks[index])
            found.append(
                Detection(
                    (
                        max(0.0, x0 / width),
                        max(0.0, y0 / height),
                        max(0.0, (x1 - x0) / width),
                        max(0.0, (y1 - y0) / height),
                    ),
                    score,
                    label,
                    polygon,
                )
            )
        return suppress_overlaps(found)


class DetectorHandler(BaseHTTPRequestHandler):
    model: YoloeModel

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        if self.path != "/health":
            self._json(404, {"error": "not found"})
            return
        self._json(
            200,
            {
                "status": "ready",
                "model": self.model.model_name,
                "device": self.model.device,
                "classes": [
                    {
                        "name": target.name,
                        "label": target.label,
                        "prompts": list(target.prompts),
                        "min_score": target.min_score,
                    }
                    for target in catalog.CATALOG
                ],
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
            scale = max(0.05, confidence / catalog.CALIBRATED_CONFIDENCE)
            requested = self.headers.get("X-SwarmDeck-Classes")
            classes = tuple(
                target.name
                for target in catalog.resolve(
                    None if requested is None else requested.split(",")
                )
            )
            floors = parse_class_floors(self.headers.get("X-SwarmDeck-Class-Floors"))
            encoded = np.frombuffer(self.rfile.read(length), dtype=np.uint8)
            image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError("request is not a decodable image")
            detections = self.model.detect(image, scale, classes, floors or None)
            self._json(
                200,
                {
                    "detections": [
                        {
                            "class": detection.label,
                            "bbox": [round(value, 6) for value in detection.bbox],
                            "score": round(detection.score, 6),
                            "polygon": (
                                [[round(x, 5), round(y, 5)] for x, y in detection.polygon]
                                if detection.polygon
                                else None
                            ),
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
        print(f"[detector] {self.address_string()} {message % args}", flush=True)

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
    parser.add_argument("--device", default=os.environ.get("SWARMDECK_YOLO_DEVICE"))
    args = parser.parse_args()

    print(
        f"[detector] loading {args.model!r} for "
        f"{len(catalog.CATALOG)} classes / {len(catalog.prompt_bindings()[0])} prompts",
        flush=True,
    )
    DetectorHandler.model = YoloeModel(args.model, args.device)
    server = ThreadingHTTPServer((args.host, args.port), DetectorHandler)
    print(
        f"[detector] ready on {args.host}:{args.port}; "
        f"device={DetectorHandler.model.device}",
        flush=True,
    )
    server.serve_forever()


if __name__ == "__main__":
    main()
