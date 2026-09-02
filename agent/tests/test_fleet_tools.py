import asyncio

import pytest

from agent_cortex.contracts import FleetAction
from agent_cortex.fleet_tools import RobotToolFleetTools


def test_doctor_builds_read_only_consolidated_check_without_approval():
    tools = RobotToolFleetTools(
        script="/app/scripts/robot_tool.py", server_url="http://server:8080"
    )

    commands = tools.build_commands(
        FleetAction(action="doctor", robot_ids=["tars_0"])
    )

    _, argv, timeout = commands[0]
    assert argv[-3:] == ["doctor", "tars_0", "--services"]
    assert "--server" in argv
    assert timeout == 90.0


@pytest.mark.parametrize(
    "action",
    [
        FleetAction(action="deploy", robot_ids=["tars_0"]),
        FleetAction(action="stop", robot_ids=["all"]),
        FleetAction(
            action="navigate", robot_ids=["tars_0"], parameters={"x": 1, "y": 2}
        ),
    ],
)
def test_mutating_actions_require_approval(action):
    with pytest.raises(PermissionError, match="requires operator approval"):
        RobotToolFleetTools().build_commands(action)


def test_deploy_maps_robot_identity_to_existing_profile():
    command = RobotToolFleetTools().build_commands(
        FleetAction(action="deploy", robot_ids=["tars_0"]), operator_approved=True
    )[0][1]
    assert command[-2:] == ["deploy", "scout"]


def test_motion_parameters_are_bounded_and_passed_without_a_shell():
    tools = RobotToolFleetTools()
    with pytest.raises(ValueError, match="linear must be between"):
        tools.build_commands(
            FleetAction(
                action="drive",
                robot_ids=["tars_0"],
                parameters={"linear": 100, "duration": 1},
            ),
            operator_approved=True,
        )

    _, argv, _ = tools.build_commands(
        FleetAction(
            action="navigate",
            robot_ids=["tars_0"],
            parameters={"x": 1.5, "y": -2, "yaw": 0.2},
        ),
        operator_approved=True,
    )[0]
    assert argv[-8:] == [
        "navigate",
        "tars_0",
        "--x",
        "1.5",
        "--y",
        "-2.0",
        "--yaw",
        "0.2",
    ]


def test_invoke_can_run_multiple_robots_concurrently_through_injected_runner():
    seen = []

    async def runner(argv, timeout):
        seen.append((argv[-2:], timeout))
        return {"returncode": 0, "stdout": "{}", "stderr": "", "payload": {}}

    tools = RobotToolFleetTools(runner=runner)
    result = asyncio.run(
        tools.invoke(
            FleetAction(
                action="doctor",
                robot_ids=["spot_0", "aslan_0"],
                parameters={"services": False},
            )
        )
    )

    assert result["ok"] is True
    assert [item["robot_id"] for item in result["results"]] == ["spot_0", "aslan_0"]
    assert [entry[0] for entry in seen] == [
        ["doctor", "spot_0"],
        ["doctor", "aslan_0"],
    ]
