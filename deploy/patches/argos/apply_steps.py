"""Apply bounded step traversal after the native contact-traction patch."""

import argparse
from pathlib import Path
import re
import shutil

STEP_LIMITS = {"bunker": 0.15, "bunker-mini": 0.15, "scout-mini": 0.15, "spot": 0.30}

_STEP_HELPER_INCLUDE = '#include "swarmdeck_step.h"'
_STEP_CALL = re.compile(
    r"""
    SwarmDeckStep\(
        \s*GetJoltEngine\(\)\.GetSystem\(\),\s*
        cId,\s*cPosition,\s*cRotation,\s*
        cForward\s*\*\s*\(fLinear\s*<\s*0\s*\?\s*-distance\s*:\s*distance\),\s*
        (?P<limit>\d+\.\d{2})f\s*
    \);
    """,
    re.VERBOSE,
)


def require_anchor(source: str, anchor: str) -> None:
    """Fail on upstream source drift, including when Python runs with -O."""
    if source.count(anchor) != 1:
        raise ValueError(f"Expected exactly one insertion anchor: {anchor.strip()}")


def patch_model(s: str, limit: float) -> str:
    """Add one control-tick step probe to a contact-driven robot model."""
    drive = "      SetDriveVelocity(fLinear, fAngular);"
    require_anchor(s, drive)
    if "SwarmDeckStep" in s:
        matches = list(_STEP_CALL.finditer(s))
        if len(matches) != 1 or s.count("SwarmDeckStep") != 1:
            raise ValueError(
                "Expected exactly one existing generated SwarmDeckStep call"
            )
        if s.count(_STEP_HELPER_INCLUDE) != 1:
            raise ValueError(
                "Existing SwarmDeckStep call requires exactly one helper include"
            )
        start, end = matches[0].span("limit")
        return s[:start] + f"{limit:.2f}" + s[end:]
    if _STEP_HELPER_INCLUDE in s:
        raise ValueError(
            "Found SwarmDeck step helper include without its generated call"
        )
    step_body = f"""
      JPH::BodyInterface& cInterface = GetJoltEngine().GetBodyInterface();
      const JPH::BodyID& cId = m_vecBodies[0].Id;
      JPH::RVec3 cPosition;
      JPH::Quat cRotation;
      cInterface.GetPositionAndRotation(cId, cPosition, cRotation);
      JPH::Vec3 cForward = cRotation * JPH::Vec3::sAxisX();
      // Probe once per control tick; contact friction supplies motor traction.
      float distance = std::abs(fLinear) < 0.001 ? 0.0f :
         std::clamp(std::abs(float(fLinear)) * float(GetJoltEngine().GetPhysicsClockTick()), 0.10f, 0.12f);
      SwarmDeckStep(GetJoltEngine().GetSystem(), cId, cPosition, cRotation,
                    cForward * (fLinear < 0 ? -distance : distance), {limit:.2f}f);"""
    return _STEP_HELPER_INCLUDE + "\n" + s.replace(drive, drive + step_body)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("argos_source", type=Path)
    args = parser.parse_args()
    helper = Path(__file__).with_name("swarmdeck_step.h")
    for robot, limit in STEP_LIMITS.items():
        folder = args.argos_source / "src/plugins/robots" / robot / "simulator"
        model = folder / f"jolt_{robot.replace('-', '_')}_model.cpp"
        try:
            patched = patch_model(model.read_text(), limit)
        except ValueError as exc:
            raise ValueError(f"{model}: {exc}") from exc
        model.write_text(patched)
        shutil.copyfile(helper, folder / helper.name)


if __name__ == "__main__":
    main()
