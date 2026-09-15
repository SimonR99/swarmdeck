from pathlib import Path
import runpy

import pytest

REPO = Path(__file__).resolve().parents[2]
PATCHER = runpy.run_path(str(REPO / "deploy/patches/argos/apply_steps.py"))
patch_model = PATCHER["patch_model"]


def upstream_model() -> str:
    return """#include <algorithm>
void update() {
      cSettings.mMotionQuality = JPH::EMotionQuality::LinearCast;
      /* Maintain vertical velocity from gravity */
}
"""


def test_existing_step_call_updates_limit_and_remains_idempotent():
    old = patch_model(upstream_model(), 0.10)
    updated = patch_model(old, 0.15)

    assert updated != old
    assert updated == old.replace(
        "cForward * (fLinear < 0 ? -distance : distance), 0.10f);",
        "cForward * (fLinear < 0 ? -distance : distance), 0.15f);",
    )
    assert updated.count('#include "swarmdeck_step.h"') == 1
    assert updated.count("SwarmDeckStep") == 1
    assert patch_model(updated, 0.15) == updated


@pytest.mark.parametrize(
    "malformed",
    [
        lambda source: source.replace(
            "cForward * (fLinear < 0 ? -distance : distance)",
            "JPH::Vec3::sZero()",
        ),
        lambda source: source.replace(
            "      /* Maintain vertical velocity from gravity */",
            "      SwarmDeckStep();\n\n"
            "      /* Maintain vertical velocity from gravity */",
        ),
    ],
)
def test_malformed_existing_step_call_fails_closed(malformed):
    existing = patch_model(upstream_model(), 0.10)

    with pytest.raises(
        ValueError, match="exactly one existing generated SwarmDeckStep call"
    ):
        patch_model(malformed(existing), 0.15)
