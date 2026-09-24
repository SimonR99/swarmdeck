"""Exercise the native parked-lidar schedule without a renderer or GPU."""

from pathlib import Path
import shutil
import subprocess

import pytest

REPO = Path(__file__).resolve().parents[4]
PATCHES = REPO / "deploy/patches/argos"


def test_parked_lidar_keeps_full_schedule_until_still_and_resumes_on_motion(tmp_path):
    compiler = shutil.which("c++")
    if compiler is None:
        pytest.skip("a C++ compiler is required for the native schedule regression")
    source = tmp_path / "schedule.cpp"
    source.write_text(r"""
#include "swarmdeck_lidar_schedule.h"
#include <cassert>

int main() {
    SwarmDeckLidarSchedule schedule;
    // 100 Hz physics, 10 Hz moving scans, 2 Hz parked scans after 1 s still.
    for(unsigned tick = 0; tick < 200; ++tick) {
        const bool due = schedule.IsDue(tick, 0, 10, 50, 100, tick == 0);
        assert(due == (tick % (tick < 100 ? 10 : 50) == 0));
    }
    // Motion on a full-rate boundary must not wait for the parked boundary.
    assert(schedule.IsDue(210, 0, 10, 50, 100, true));
    assert(schedule.IsDue(220, 0, 10, 50, 100, true));
    assert(!schedule.IsDue(221, 0, 10, 50, 100, true));
    // Brief stops never throttle a moving robot between control updates.
    assert(schedule.IsDue(230, 0, 10, 50, 100, false));
    // Simulation reset must restart the stillness grace period.
    assert(schedule.IsDue(0, 0, 10, 50, 100, false));
    assert(schedule.IsDue(10, 0, 10, 50, 100, false));

    SwarmDeckLidarSchedule disabled;
    for(unsigned tick = 0; tick < 200; ++tick) {
        assert(disabled.IsDue(tick, 3, 10, 0, 100, false)
               == ((tick + 3) % 10 == 0));
    }
    // Separate faces observing the same anchor remain phase-coincident.
    SwarmDeckLidarSchedule faces[4];
    for(unsigned tick = 0; tick < 400; ++tick) {
        const bool moved = tick == 201 || tick == 219;
        const bool due = faces[0].IsDue(tick, 0, 10, 50, 100, moved);
        for(unsigned face = 1; face < 4; ++face) {
            assert(faces[face].IsDue(tick, 0, 10, 50, 100, moved) == due);
        }
    }
}
""")
    binary = tmp_path / "schedule"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(PATCHES),
            str(source),
            "-o",
            str(binary),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run([str(binary)], check=True, capture_output=True, text=True)


def test_parked_lidar_patch_is_idempotent_and_rejects_upstream_drift(tmp_path):
    import sys

    upstream = REPO.parent / "argos3/src/plugins/simulator/photorealism"
    if not upstream.exists():
        pytest.skip("the pinned sibling ARGoS sources are required")
    files = (
        "CMakeLists.txt",
        "render_core/pr_camera_pool.h",
        "render_core/pr_camera_pool.cpp",
        "sensors/photorealistic_lidar_default_sensor.cpp",
    )
    root = tmp_path / "argos3"
    destination = root / "src/plugins/simulator/photorealism"
    for name in files:
        target = destination / name
        target.parent.mkdir(parents=True, exist_ok=True)
        # Read the pinned files, not a sibling checkout's uncommitted changes.
        source = subprocess.check_output(
            [
                "git",
                "-C",
                str(REPO.parent / "argos3"),
                "show",
                f"83ca4602:src/plugins/simulator/photorealism/{name}",
            ],
            text=True,
        )
        target.write_text(source)
    command = [sys.executable, str(PATCHES / "apply_parked_lidar.py"), str(root)]
    subprocess.run(command, check=True, capture_output=True, text=True)
    patched = {name: (destination / name).read_text() for name in files}
    subprocess.run(command, check=True, capture_output=True, text=True)
    assert patched == {name: (destination / name).read_text() for name in files}
    assert "  render_core/swarmdeck_lidar_schedule.h\n" in patched["CMakeLists.txt"]
    assert (destination / "render_core/swarmdeck_lidar_schedule.h").read_bytes() == (
        PATCHES / "swarmdeck_lidar_schedule.h"
    ).read_bytes()
    # An unrecognized pool schedule must fail rather than silently ignore the option.
    pool = destination / "render_core/pr_camera_pool.cpp"
    pool.write_text(
        patched["render_core/pr_camera_pool.cpp"].replace(
            "if(bSwarmDeckLidarDue)",
            "if(false)",
        )
    )
    before = {name: (destination / name).read_bytes() for name in files}
    rejected = subprocess.run(command, capture_output=True, text=True)
    assert rejected.returncode != 0
    assert "insertion anchor" in rejected.stderr
    assert before == {name: (destination / name).read_bytes() for name in files}
