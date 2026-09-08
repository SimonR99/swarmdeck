"""Shared snapshot and content-addressed spatial chunk cache for 3D viewers."""

from collections import OrderedDict
from dataclasses import dataclass
import hashlib
import threading
import time
import zlib

import numpy as np


@dataclass
class Product:
    body: bytes
    headers: dict
    manifest: dict | None = None


class CloudDelivery:
    def __init__(self, max_bytes=128 * 1024 * 1024):
        self.lock = threading.Lock()
        self.products = OrderedDict()
        self.chunks = OrderedDict()
        self.chunk_bytes = 0
        self.product_bytes = 0
        self.max_bytes = max_bytes

    def prepare(self, scope, revision, build, *, upstream=False):
        # Single flight: concurrent browsers share fusion, upstream I/O and encoding.
        with self.lock:
            now = time.monotonic()
            cached = self.products.get(scope)
            if (
                cached
                and cached[0] == revision
                and (not upstream or now - cached[1] < 2)
            ):
                self.products.move_to_end(scope)
                return cached[2]
            body, headers = build()
            etag = (
                '"'
                + hashlib.sha256(
                    body + repr(sorted(headers.items())).encode()
                ).hexdigest()
                + '"'
            )
            product = Product(
                body, {**headers, "ETag": etag, "Cache-Control": "no-cache"}
            )
            if cached:
                self.product_bytes -= len(cached[2].body)
            self.product_bytes += len(body)
            self.products[scope] = (revision, time.monotonic(), product)
            self.products.move_to_end(scope)
            while len(self.products) > 16 or (
                self.product_bytes > 64 * 1024 * 1024 and len(self.products) > 1
            ):
                _, removed = self.products.popitem(last=False)
                self.product_bytes -= len(removed[2].body)
            return product

    def manifest(self, product):
        with self.lock:
            if product.manifest is not None and all(
                c["id"] in self.chunks for c in product.manifest["chunks"]
            ):
                return product.manifest
            h = product.headers
            count = int(h["X-Cloud-Points"])
            width = 12 if h["X-Cloud-Format"] == "xyz32" else 6
            has_rgb = h["X-Cloud-RGB"] == "1"
            if not 0 <= count <= 2_000_000:
                raise ValueError("invalid cloud point count")
            decoder = zlib.decompressobj()
            expected = count * (width + 1 + 3 * has_rgb)
            raw = decoder.decompress(product.body, expected + 1)
            if not decoder.eof or decoder.unused_data or len(raw) != expected:
                raise ValueError("invalid cloud snapshot")
            xyz = np.frombuffer(
                raw, dtype="<f4" if width == 12 else "<i2", count=count * 3
            ).reshape(-1, 3)
            owners = np.frombuffer(raw, np.uint8, count=count, offset=count * width)
            rgb = (
                np.frombuffer(raw, np.uint8, offset=count * (width + 1)).reshape(-1, 3)
                if has_rgb
                else None
            )
            # Stable 4m tiles, separately owned by each robot. A new room does not
            # shift every subsequent network chunk as fixed-size array blocks do.
            tiles = np.floor(
                xyz.astype(np.float64) * float(h["X-Cloud-Scale"]) / 4
            ).astype(np.int64)
            order = np.lexsort(
                (
                    xyz[:, 2],
                    xyz[:, 1],
                    xyz[:, 0],
                    tiles[:, 2],
                    tiles[:, 1],
                    tiles[:, 0],
                    owners,
                )
            )
            tile_keys = np.column_stack((owners[order], tiles[order]))
            starts = (
                np.r_[
                    0,
                    np.flatnonzero(np.any(tile_keys[1:] != tile_keys[:-1], axis=1)) + 1,
                    count,
                ]
                if count
                else [0]
            )
            chunks = []
            for start, end in zip(starts[:-1], starts[1:]):
                selected = order[start:end]
                payload = xyz[selected].tobytes() + owners[selected].tobytes()
                if rgb is not None:
                    payload += rgb[selected].tobytes()
                identity = hashlib.sha256(payload).hexdigest()
                if identity not in self.chunks:
                    compressed = zlib.compress(payload, 1)
                    self.chunks[identity] = compressed
                    self.chunk_bytes += len(compressed)
                self.chunks.move_to_end(identity)
                chunks.append({"id": identity, "points": int(end - start)})
            while self.chunk_bytes > self.max_bytes and self.chunks:
                _, removed = self.chunks.popitem(last=False)
                self.chunk_bytes -= len(removed)
            product.manifest = {"version": 1, "headers": h, "chunks": chunks}
            return product.manifest

    def chunk(self, identity):
        with self.lock:
            payload = self.chunks.get(identity)
            if payload is not None:
                self.chunks.move_to_end(identity)
            return payload
