"""Every launch file must be able to build its LaunchDescription.

This exists because of a specific, repeated failure. `generate_launch_description()`
is plain Python that nothing executes until Gazebo starts, so a name that is used
but never bound — `grid_3d = LaunchConfiguration("grid_3d")` added to one launch
file and forgotten in another — imports fine, compiles fine, passes every other
test, and then aborts the whole session at runtime with

    [ERROR] [launch]: Caught exception in launch: name 'grid_3d' is not defined

which costs a full stack startup to discover. Calling the function catches it in
milliseconds.

Skipped where `launch` is unavailable (the plain pytest environment has no ROS),
so this is a real gate only inside the ROS image — `make docker-test` and CI.
Kept cheap and dependency-free on purpose: it builds descriptions, it does not
launch anything.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

pytest.importorskip("launch", reason="ROS 2 launch not installed outside the image")
pytest.importorskip("launch_ros")

SRC = Path(__file__).resolve().parents[3]
LAUNCH_FILES = sorted(SRC.rglob("launch/*.launch.py"))


def _load(path: Path):
    spec = importlib.util.spec_from_file_location(f"lf_{path.stem}", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("path", LAUNCH_FILES, ids=lambda p: p.name)
def test_generate_launch_description_runs(path: Path):
    module = _load(path)
    generate = getattr(module, "generate_launch_description", None)
    assert generate is not None, f"{path.name} has no generate_launch_description"
    # Session bringup resolves the study config and spawns per-robot stacks from
    # an OpaqueFunction, so its body only runs with a real launch context. The
    # top-level call still catches undefined names outside that function.
    description = generate()
    assert description is not None


def test_at_least_the_known_launch_files_are_covered():
    """Guards against the glob silently matching nothing and the suite passing."""
    names = {p.name for p in LAUNCH_FILES}
    for expected in ("session.launch.py", "slam.launch.py", "slam_rtabmap.launch.py"):
        assert expected in names, f"{expected} not discovered; glob is wrong"
