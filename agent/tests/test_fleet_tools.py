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


def test_battery_uses_one_read_only_fleet_snapshot_without_approval():
    tools = RobotToolFleetTools(
        script="/app/scripts/robot_tool.py", server_url="http://server:8080"
    )

    commands = tools.build_commands(
        FleetAction(action="battery", robot_ids=["tars_0", "spot_0"])
    )

    assert len(commands) == 1
    _, argv, timeout = commands[0]
    assert argv[-1] == "list"
    assert "--services" not in argv
    assert timeout == 30.0


def test_battery_normalizes_multiple_robots_from_one_list_call():
    seen = []

    async def runner(argv, timeout):
        seen.append((argv, timeout))
        return {
            "returncode": 0,
            "stdout": "",
            "stderr": "",
            "payload": [
                {"robot_id": "tars_0", "battery": 0.4193549, "online": True},
                {"robot_id": "spot_0", "battery": 0.8, "online": False},
            ],
        }

    result = asyncio.run(
        RobotToolFleetTools(runner=runner).invoke(
            FleetAction(action="battery", robot_ids=["tars", "spot_0"])
        )
    )

    assert result == {
        "ok": True,
        "action": "battery",
        "results": [
            {
                "robot_id": "tars_0",
                "returncode": 0,
                "battery_fraction": 0.4193549,
                "battery_percent": 41.9,
                "online": True,
            },
            {
                "robot_id": "spot_0",
                "returncode": 0,
                "battery_fraction": 0.8,
                "battery_percent": 80.0,
                "online": False,
            },
        ],
    }
    assert len(seen) == 1
    assert seen[0][0][-1] == "list"


def test_battery_reports_a_missing_robot_without_hiding_valid_results():
    async def runner(argv, timeout):
        return {
            "returncode": 0,
            "stdout": "",
            "stderr": "",
            "payload": [{"robot_id": "tars_0", "battery": None, "online": True}],
        }

    result = asyncio.run(
        RobotToolFleetTools(runner=runner).invoke(
            FleetAction(action="battery", robot_ids=["tars_0", "missing_0"])
        )
    )

    assert result["ok"] is False
    assert result["results"][0]["returncode"] == 0
    assert result["results"][0]["battery_percent"] is None
    assert result["results"][1]["returncode"] == 1
    assert "not found" in result["results"][1]["error"]
