#!/usr/bin/env python3
"""Exercise one persistent native importer through replace and pose-only calls."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import select
import selectors
import subprocess
import time
from pathlib import Path

MAX_LINE_BYTES = 64 * 1024


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _request(
    request_id: str, mode: str, snapshot: Path, root: Path, output: Path
) -> dict[str, object]:
    return {
        "protocol": 1,
        "type": "request",
        "request_id": request_id,
        "op": "apply",
        "mode": mode,
        "map_id": "onboard",
        "snapshot_path": str(snapshot),
        "snapshot_sha256": _digest(snapshot),
        "chunks_dir": str(root / "chunks"),
        "output_path": str(output),
    }


class JsonLines:
    def __init__(self, process: subprocess.Popen[bytes], deadline: float):
        assert process.stdin is not None and process.stdout is not None
        self.process = process
        self.stdin = process.stdin
        self.stdout = process.stdout
        self.deadline = deadline
        self.buffer = bytearray()
        self.selector = selectors.DefaultSelector()
        self.selector.register(self.stdout, selectors.EVENT_READ)
        os.set_blocking(self.stdin.fileno(), False)

    def close(self) -> None:
        self.selector.close()

    def receive(self) -> dict[str, object]:
        while True:
            newline = self.buffer.find(b"\n")
            if newline >= 0:
                raw = bytes(self.buffer[:newline])
                del self.buffer[: newline + 1]
                if not raw or len(raw) > MAX_LINE_BYTES:
                    raise RuntimeError("native response exceeded 64 KiB")
                value = json.loads(raw)
                if not isinstance(value, dict):
                    raise RuntimeError("native response is not an object")
                return value
            if len(self.buffer) > MAX_LINE_BYTES:
                raise RuntimeError("native response exceeded 64 KiB")
            remaining = self.deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("native JSONL response exceeded smoke deadline")
            events = self.selector.select(remaining)
            if not events:
                raise TimeoutError("native JSONL response exceeded smoke deadline")
            block = os.read(self.stdout.fileno(), 8192)
            if not block:
                raise RuntimeError("native importer exited before its response")
            self.buffer.extend(block)

    def send(self, value: dict[str, object]) -> dict[str, object]:
        payload = (
            json.dumps(value, sort_keys=True, separators=(",", ":")).encode() + b"\n"
        )
        if len(payload) > MAX_LINE_BYTES:
            raise RuntimeError("native request exceeded 64 KiB")
        view = memoryview(payload)
        fd = self.stdin.fileno()
        while view:
            remaining = self.deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("native JSONL request exceeded smoke deadline")
            _, writable, _ = select.select((), (fd,), (), remaining)
            if not writable:
                raise TimeoutError("native JSONL request exceeded smoke deadline")
            try:
                written = os.write(fd, view)
            except BrokenPipeError as error:
                raise RuntimeError(
                    "native importer exited while receiving request"
                ) from error
            view = view[written:]
        return self.receive()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument(
        "--binary",
        type=Path,
        default=Path("/mapping_ws/install/swarmdeck_mapping/bin/swarmdeck-mola-import"),
    )
    parser.add_argument("--timeout", type=float, default=45.0)
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")

    root = args.root
    first_path = root / "snapshot.json"
    first = json.loads(first_path.read_text())
    second = copy.deepcopy(first)
    manifest = second["manifests"][0]
    manifest["graph_revision"]["revision"] += 1
    for submap in manifest["submaps"]:
        submap["pose_revision"] = copy.deepcopy(manifest["graph_revision"])
    manifest["submaps"][0]["T_component_submap"][0][3] = 1.0
    canonical = json.dumps(second["manifests"], sort_keys=True, separators=(",", ":"))
    second["snapshot_id"] = hashlib.sha256(canonical.encode()).hexdigest()
    second_path = root / "snapshot-pose.json"
    second_path.write_text(json.dumps(second, sort_keys=True, separators=(",", ":")))

    process = subprocess.Popen(
        (str(args.binary), "--serve"),
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=None,
    )
    client = JsonLines(process, time.monotonic() + args.timeout)
    try:
        # Read the initial ready event through the same bounded selector path.
        ready = client.receive()
        if ready.get("protocol") != 1 or ready.get("type") != "ready":
            raise RuntimeError("native importer returned an invalid ready event")

        first_response = client.send(
            _request(
                "replace-1", "replace", first_path, root, root / "mola-v1.metricmap"
            )
        )
        if not first_response.get("ok") or first_response.get("result") != "replaced":
            raise RuntimeError(f"replace request failed: {first_response}")

        second_response = client.send(
            _request(
                "pose-1", "pose_only", second_path, root, root / "mola-v2.metricmap"
            )
        )
        if (
            not second_response.get("ok")
            or second_response.get("result") != "corrected"
        ):
            raise RuntimeError(f"pose-only request failed: {second_response}")
        if second_response.get("mode") != "pose_only":
            raise RuntimeError("native response did not preserve pose-only mode")
        if first_response.get("geometry_revision") != second_response.get(
            "geometry_revision"
        ):
            raise RuntimeError("pose-only request changed geometry revision")
        for output in (root / "mola-v1.metricmap", root / "mola-v2.metricmap"):
            if output.stat().st_size <= 0:
                raise RuntimeError(f"native output is empty: {output}")
    finally:
        client.close()
        if process.stdin is not None:
            process.stdin.close()
        if process.poll() is None:
            process.terminate()
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


if __name__ == "__main__":
    main()
