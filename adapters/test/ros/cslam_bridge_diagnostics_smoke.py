#!/usr/bin/env python3
"""Focused native regression for bridge optimizer-result diagnostics."""

from pathlib import Path
from types import SimpleNamespace as NS
import tempfile
import uuid

from autonomy.contracts import IDENTITY_SE3
from autonomy.cslam import CslamMapper
from autonomy.mapping import CorrectionAwareMapper, SubmapStore
from deploy.autonomy.cslam_bridge import Bridge


def value(robot: int, seq: int, x: float):
    return NS(
        key=NS(robot_id=robot, keyframe_id=seq),
        pose=NS(
            position=NS(x=x, y=0.0, z=0.0),
            orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def result(mission: str, clock: int, x: float):
    return NS(
        success=True,
        mission_id=mission,
        solution_clock=clock,
        optimizer_robot_id=0,
        origin_robot_id=0,
        estimates=[value(0, 0, x)],
        anchor_estimates=[value(0, 0, x)],
    )


def main() -> None:
    mission = str(uuid.uuid4())
    with tempfile.TemporaryDirectory(prefix="cslam-diagnostics-") as directory:
        core = CslamMapper(
            CorrectionAwareMapper(SubmapStore(Path(directory))),
            "robot_0",
            0,
            mission,
            {0: "robot_0"},
        )
        core.capture(0, 1, IDENTITY_SE3, [[1.0, 0.0, 0.0]])
        bridge = NS(
            core=core,
            solution_results_received=0,
            solution_results_accepted=0,
            solution_results_unchanged=0,
            solution_count=0,
        )

        wrong_mission = result(str(uuid.uuid4()), 10, 0.0)
        Bridge.optimized(bridge, wrong_mission)
        assert bridge.solution_results_received == 1
        assert bridge.solution_results_accepted == 0
        assert bridge.solution_results_unchanged == 0
        assert bridge.solution_count == 0
        assert core.solution_order == (0, -1)

        unchanged = result(mission, 1, 0.0)
        Bridge.optimized(bridge, unchanged)
        assert bridge.solution_results_received == 2
        assert bridge.solution_results_accepted == 1
        assert bridge.solution_results_unchanged == 1
        assert bridge.solution_count == 0
        assert core.solution_order == (1, 0)

        Bridge.optimized(bridge, unchanged)
        assert bridge.solution_results_received == 3
        assert bridge.solution_results_accepted == 1
        assert bridge.solution_results_unchanged == 1
        assert bridge.solution_count == 0
        assert core.solution_order == (1, 0)

        changed = result(mission, 2, 1.0)
        Bridge.optimized(bridge, changed)
        assert bridge.solution_results_received == 4
        assert bridge.solution_results_accepted == 2
        assert bridge.solution_results_unchanged == 1
        assert bridge.solution_count == 1
        assert core.solution_order == (2, 0)
        assert core.correction_revision == 1

    print(
        "PASS: wrong-mission/stale results rejected; accepted unchanged and "
        "pose-changing results counted separately"
    )


if __name__ == "__main__":
    main()
