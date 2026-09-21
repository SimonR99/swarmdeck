"""Every remaining ROS launch file must build a LaunchDescription."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

pytest.importorskip("launch", reason="ROS 2 launch not installed outside the image")
pytest.importorskip("launch_ros", reason="ROS 2 launch_ros not installed outside the image")

SRC = Path(__file__).resolve().parents[2]
LAUNCH_FILES = sorted(SRC.rglob("launch/*.launch.py"))


def _load(path: Path):
    spec = importlib.util.spec_from_file_location(f"launch_file_{path.stem}", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("path", LAUNCH_FILES, ids=lambda path: path.name)
def test_generate_launch_description_builds(path: Path):
    module = _load(path)
    generate = getattr(module, "generate_launch_description", None)
    assert generate is not None, f"{path.name} has no generate_launch_description"
    assert generate() is not None


def test_remaining_launch_files_are_discovered():
    assert {path.name for path in LAUNCH_FILES} == {
        "session.launch.py",
        "nav.launch.py",
        "botman.launch.py",
        "aslan.launch.py",
        "asimov.launch.py",
    }
