import pytest
from pydantic import ValidationError

from agent_cortex.missions import MissionExecutive, MissionPlan, MissionTask


def test_mission_wave_parallelizes_robots_but_serializes_each_robot():
    plan = MissionPlan(
        objective="diagnose the fleet",
        tasks=[
            MissionTask(
                task_id="spot-doctor",
                robot_id="spot_0",
                action="doctor",
                requires_approval=False,
            ),
            MissionTask(
                task_id="spot-nav",
                robot_id="spot_0",
                action="navigate",
                approved=True,
            ),
            MissionTask(
                task_id="aslan-doctor",
                robot_id="aslan_0",
                action="doctor",
                requires_approval=False,
            ),
        ],
    )

    wave = MissionExecutive.ready_wave(plan)

    assert [(task.robot_id, task.action) for task in wave] == [
        ("spot_0", "doctor"),
        ("aslan_0", "doctor"),
    ]


def test_mission_dependencies_and_approval_gate_dispatch():
    diagnose = MissionTask(
        task_id="diagnose",
        robot_id="tars_0",
        action="doctor",
        requires_approval=False,
    )
    repair = MissionTask(
        task_id="repair",
        robot_id="tars_0",
        action="deploy",
        depends_on=["diagnose"],
    )
    plan = MissionPlan(objective="repair TARS", tasks=[diagnose, repair])

    assert MissionExecutive.ready_wave(plan) == [diagnose]
    MissionExecutive.mark_result(diagnose, succeeded=True)
    assert MissionExecutive.ready_wave(plan) == []
    repair.approved = True
    assert MissionExecutive.ready_wave(plan) == [repair]


def test_mission_rejects_cycles():
    with pytest.raises(ValidationError, match="cycle"):
        MissionPlan(
            objective="invalid",
            tasks=[
                MissionTask(
                    task_id="a",
                    robot_id="spot_0",
                    action="doctor",
                    depends_on=["b"],
                ),
                MissionTask(
                    task_id="b",
                    robot_id="aslan_0",
                    action="doctor",
                    depends_on=["a"],
                ),
            ],
        )
