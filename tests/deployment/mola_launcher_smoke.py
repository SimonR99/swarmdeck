#!/usr/bin/env python3
"""Bounded smoke test for the installed MOLA launcher and map module."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--artifact", type=Path)
    args = parser.parse_args()
    args.fixture = args.fixture.resolve()
    args.config = args.config.resolve()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    launcher = shutil.which("mola-cli")
    if launcher is None:
        raise RuntimeError("mola-cli is not installed in the mapping image")

    snapshot = args.fixture / "snapshot.json"
    chunks = args.fixture / "chunks"
    value = json.loads(snapshot.read_text())
    manifests = value.get("manifests")
    if not isinstance(manifests, list) or len(manifests) != 1:
        raise RuntimeError("launcher fixture must contain one manifest")
    revision = manifests[0].get("graph_revision")
    if not isinstance(revision, dict) or not isinstance(revision.get("component_id"), str):
        raise RuntimeError("launcher fixture has no component identity")

    artifact = (args.artifact or args.fixture / "launcher.metricmap").resolve()
    if artifact.exists():
        raise RuntimeError("launcher smoke needs a fresh artifact path")
    config = args.config.read_text()
    marker = "      # Optional: omit this key to publish maps entirely in memory."
    if "artifact_file:" not in config:
        if marker not in config:
            raise RuntimeError("launcher config has no artifact insertion point")
        config = config.replace(marker, "      artifact_file: ${SWARMDECK_MOLA_ARTIFACT}\n" + marker)
    generated_config = args.fixture / "launcher-smoke.yaml"
    generated_config.write_text(config)

    environment = os.environ.copy()
    environment.update(
        {
            "SWARMDECK_MOLA_SNAPSHOT": str(snapshot),
            "SWARMDECK_MOLA_CHUNKS": str(chunks),
            "SWARMDECK_MOLA_COMPONENT": revision["component_id"],
            "SWARMDECK_MOLA_ARTIFACT": str(artifact),
        }
    )
    process = subprocess.Popen(
        [launcher, str(generated_config)],
        cwd=str(args.fixture),
        env=environment,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=None,
    )
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            if artifact.is_file() and artifact.stat().st_size > 0:
                if process.poll() is not None:
                    raise RuntimeError(f"mola-cli exited before smoke completed: {process.returncode}")
                break
            if process.poll() is not None:
                raise RuntimeError(f"mola-cli exited before publishing: {process.returncode}")
            time.sleep(0.05)
        else:
            raise TimeoutError("mola-cli did not publish a nonempty artifact within the smoke deadline")
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3.0)
    if process.returncode not in (0, -signal.SIGINT):
        raise RuntimeError(f"mola-cli shutdown failed: {process.returncode}")
    if not artifact.is_file() or artifact.stat().st_size == 0:
        raise RuntimeError("mola-cli artifact disappeared or is empty")


if __name__ == "__main__":
    main()
