"""Apply bounded step traversal and internal-edge correction to pinned ARGoS models."""

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
    """Return the patched model; repeated application leaves it unchanged."""
    anchor = "      /* Maintain vertical velocity from gravity */"
    require_anchor(s, anchor)
    contact_anchor = "      cSettings.mMotionQuality = JPH::EMotionQuality::LinearCast;"
    require_anchor(s, contact_anchor)
    if "cSettings.mEnhancedInternalEdgeRemoval = true;" not in s:
        s = s.replace(
            contact_anchor,
            """      // Reject ghost edge contacts where adjacent/decorative road triangles meet.
      // Apply the extra local contact work only to moving robot bodies.
      cSettings.mEnhancedInternalEdgeRemoval = true;
"""
            + contact_anchor,
        )
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
        match = matches[0]
        start, end = match.span("limit")
        return s[:start] + f"{limit:.2f}" + s[end:]
    if _STEP_HELPER_INCLUDE in s:
        raise ValueError(
            "Found SwarmDeck step helper include without its generated call"
        )
    s = _STEP_HELPER_INCLUDE + "\n" + s
    s = s.replace(
        anchor,
        f"""      // Probe only one small commanded advance, never jump across a gap.
      // Clear the rounded body corner even with a 1 ms physics substep.
      float distance = std::abs(fLinear) < 0.001 ? 0.0f :
         std::clamp(std::abs(float(fLinear)) * float(GetJoltEngine().GetPhysicsClockTick()), 0.10f, 0.12f);
      SwarmDeckStep(GetJoltEngine().GetSystem(), cId, cPosition, cRotation,
                    cForward * (fLinear < 0 ? -distance : distance), {limit:.2f}f);

""" + anchor,
    )
    return s


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
