"""Add opt-in parked-lidar scheduling to the pinned ARGoS camera pool."""

import argparse
from pathlib import Path
import shutil

PATCHES = {
    "CMakeLists.txt": [
        (
            "  render_core/pr_camera_pool.h\n",
            "  render_core/pr_camera_pool.h\n  render_core/swarmdeck_lidar_schedule.h\n",
        ),
    ],
    "render_core/pr_camera_pool.h": [
        (
            "#include <chrono>",
            '#include "swarmdeck_lidar_schedule.h"\n#include <chrono>',
        ),
        (
            "      UInt32 FramerateDivider = 1;",
            "      UInt32 FramerateDivider = 1;\n"
            "      UInt32 ParkedFramerateDivider = 0;\n"
            "      UInt32 ParkedAfterTicks = 100;",
        ),
        (
            "         SPRCameraConfig Config;",
            "         SPRCameraConfig Config;\n"
            "         SwarmDeckLidarSchedule ParkedSchedule;\n"
            "         CVector3 PreviousAnchorPosition;\n"
            "         CQuaternion PreviousAnchorOrientation;",
        ),
    ],
    "sensors/photorealistic_lidar_default_sensor.cpp": [
        (
            "         /* Faces */",
            """         GetNodeAttributeOrDefault(t_tree, "parked_framerate_divider",
                                   sBase.ParkedFramerateDivider,
                                   sBase.ParkedFramerateDivider);
         GetNodeAttributeOrDefault(t_tree, "parked_after_ticks",
                                   sBase.ParkedAfterTicks, sBase.ParkedAfterTicks);
         if(sBase.ParkedFramerateDivider != 0 &&
            (sBase.ParkedFramerateDivider < sBase.FramerateDivider ||
             sBase.ParkedAfterTicks == 0)) {
            THROW_ARGOSEXCEPTION("parked lidar requires a slower divider and a positive stillness interval");
         }
         /* Faces */""",
        ),
    ],
    "render_core/pr_camera_pool.cpp": [
        (
            "         if((un_tick + unPhase) % sCamera.Config.FramerateDivider == 0) {",
            """         bool bSwarmDeckLidarDue =
            (un_tick + unPhase) % sCamera.Config.FramerateDivider == 0;
         if(sCamera.Config.ParkedFramerateDivider > sCamera.Config.FramerateDivider) {
            // Use the physical anchor, not noisy odometry or a stale cmd_vel.
            // Exact equality is deliberately conservative: any motion restores
            // the normal schedule, including turning, sliding or being pushed.
            const auto& cAnchor = *sCamera.Config.Anchor;
            const bool bMoved = sCamera.PreviousAnchorPosition != cAnchor.Position ||
               !(sCamera.PreviousAnchorOrientation == cAnchor.Orientation);
            sCamera.PreviousAnchorPosition = cAnchor.Position;
            sCamera.PreviousAnchorOrientation = cAnchor.Orientation;
            bSwarmDeckLidarDue = sCamera.ParkedSchedule.IsDue(
               un_tick, unPhase, sCamera.Config.FramerateDivider,
               sCamera.Config.ParkedFramerateDivider,
               sCamera.Config.ParkedAfterTicks, bMoved);
         }
         if(bSwarmDeckLidarDue) {""",
        ),
    ],
}


def patch_source(source: str, replacements: list[tuple[str, str]]) -> str:
    for old, new in replacements:
        if source.count(new) == 1:
            continue
        if source.count(old) != 1:
            raise ValueError(f"Expected exactly one insertion anchor: {old.strip()}")
        source = source.replace(old, new)
    return source


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("argos_source", type=Path)
    args = parser.parse_args()
    root = args.argos_source / "src/plugins/simulator/photorealism"
    # Validate every upstream anchor before modifying any source file.
    patched = {
        root / name: patch_source((root / name).read_text(), replacements)
        for name, replacements in PATCHES.items()
    }
    for path, source in patched.items():
        path.write_text(source)
    helper = Path(__file__).with_name("swarmdeck_lidar_schedule.h")
    shutil.copyfile(helper, root / "render_core" / helper.name)


if __name__ == "__main__":
    main()
