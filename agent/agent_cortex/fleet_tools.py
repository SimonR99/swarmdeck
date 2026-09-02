"""Policy-gated adapter from typed Cortex actions to the stable robot CLI.

This is the fleet authority boundary, not an LLM tool loop.  Planner models can
produce :class:`FleetAction` values, but a caller must approve mutating actions
before this adapter will construct or execute an argv.  No command uses a shell.
"""

from __future__ import annotations

import asyncio
import json
import math
import os
import re
import sys
from pathlib import Path
from typing import Any, Awaitable, Callable, Dict

from .contracts import FleetAction


CommandResult = Dict[str, Any]
CommandRunner = Callable[[list[str], float], Awaitable[CommandResult]]

_SAFE_TARGET = re.compile(r"^[A-Za-z0-9_-]{1,64}$")
_DEPLOY_PROFILE = {
    "all": "all",
    "aslan": "aslan",
    "aslan_0": "aslan",
    "botman": "botman",
    "botman_0": "botman",
    "scout": "scout",
    "scout_0": "scout",
    "tars": "scout",
    "tars_0": "scout",
    "spot": "spot",
    "spot_0": "spot",
    "asimov": "asimov",
    "asimov_0": "asimov",
}
_BODY_ACTIONS = {
    "claim",
    "release",
    "stand",
    "sit",
    "damping",
    "lie_to_stand",
    "lock_stand",
    "walk_mode",
    "run_mode",
    "wave",
    "set_height",
}


def _target(value: str) -> str:
    if not _SAFE_TARGET.fullmatch(value):
        raise ValueError(f"invalid robot target: {value!r}")
    return value


def _number(
    parameters: Dict[str, Any],
    name: str,
    *,
    required: bool = False,
    default: float = 0.0,
    minimum: float,
    maximum: float,
) -> float:
    if required and name not in parameters:
        raise ValueError(f"{name} is required")
    try:
        value = float(parameters.get(name, default))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a number") from exc
    if not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(f"{name} must be between {minimum} and {maximum}")
    return value


class RobotToolFleetTools:
    """Execute validated FleetActions through SwarmDeck's consolidated CLI."""

    def __init__(
        self,
        *,
        script: str | Path | None = None,
        server_url: str | None = None,
        runner: CommandRunner | None = None,
    ) -> None:
        self.script = str(
            script or os.environ.get("CORTEX_ROBOT_TOOL", "/app/scripts/robot_tool.py")
        )
        self.server_url = server_url or os.environ.get(
            "SWARMDECK_SERVER_URL", "http://server:8080"
        )
        self.runner = runner or self._run_command

    def status(self) -> Dict[str, Any]:
        return {
            "name": "robot_tool",
            "available": Path(self.script).is_file(),
            "transport": "validated argv; no shell",
            "read_only_without_approval": ["doctor"],
            "mutations_require_approval": True,
            "active": False,
        }

    def build_commands(
        self, action: FleetAction, *, operator_approved: bool = False
    ) -> list[tuple[str, list[str], float]]:
        if not action.robot_ids:
            raise ValueError("at least one robot target is required")
        # Approval is deliberately out-of-band from FleetAction.  A model may
        # propose that object, so no value inside it is accepted as authority.
        if action.action != "doctor" and not operator_approved:
            raise PermissionError(f"{action.action} requires operator approval")

        commands: list[tuple[str, list[str], float]] = []
        for robot_id in action.robot_ids:
            robot = _target(robot_id)
            base = [
                sys.executable,
                self.script,
                "--json",
                "--server",
                self.server_url,
            ]
            params = action.parameters

            if action.action == "doctor":
                argv = [*base, "doctor", robot]
                if params.get("services", True):
                    argv.append("--services")
                timeout = 90.0
            elif action.action == "deploy":
                profile = _DEPLOY_PROFILE.get(robot)
                if profile is None:
                    raise ValueError(f"no deployment profile for {robot!r}")
                argv = [*base, "deploy", profile]
                timeout = 360.0
            elif action.action == "drive":
                if robot == "all":
                    raise ValueError("drive requires one explicit robot ID")
                linear = _number(params, "linear", minimum=-1.5, maximum=1.5)
                angular = _number(params, "angular", minimum=-3.0, maximum=3.0)
                duration = _number(params, "duration", minimum=0.0, maximum=30.0)
                argv = [
                    *base,
                    "drive",
                    robot,
                    "--linear",
                    str(linear),
                    "--angular",
                    str(angular),
                    "--duration",
                    str(duration),
                ]
                timeout = duration + 30.0
            elif action.action == "navigate":
                if robot == "all":
                    raise ValueError("navigate requires one explicit robot ID")
                x = _number(
                    params, "x", required=True, minimum=-10_000.0, maximum=10_000.0
                )
                y = _number(
                    params, "y", required=True, minimum=-10_000.0, maximum=10_000.0
                )
                yaw = _number(params, "yaw", minimum=-math.tau, maximum=math.tau)
                argv = [
                    *base,
                    "navigate",
                    robot,
                    "--x",
                    str(x),
                    "--y",
                    str(y),
                    "--yaw",
                    str(yaw),
                ]
                timeout = 30.0
            elif action.action in {"cancel", "stop"}:
                if action.action == "cancel" and robot == "all":
                    raise ValueError("cancel requires one explicit robot ID")
                argv = [*base, action.action, robot]
                timeout = 30.0
            elif action.action == "body":
                if robot == "all":
                    raise ValueError("body requires one explicit robot ID")
                body_action = params.get("action")
                if body_action not in _BODY_ACTIONS:
                    raise ValueError("invalid or missing body action")
                argv = [*base, "body", robot, "--action", str(body_action)]
                if body_action == "set_height":
                    height = _number(
                        params,
                        "height",
                        required=True,
                        minimum=-1.0,
                        maximum=1.0,
                    )
                    argv.extend(["--height", str(height)])
                timeout = 30.0
            else:  # pragma: no cover - FleetAction validates this first.
                raise ValueError(f"unsupported fleet action: {action.action}")

            commands.append((robot, argv, timeout))
        return commands

    async def invoke(
        self, action: FleetAction, *, operator_approved: bool = False
    ) -> Dict[str, Any]:
        commands = self.build_commands(action, operator_approved=operator_approved)

        async def run_one(
            robot_id: str, argv: list[str], timeout: float
        ) -> Dict[str, Any]:
            result = await self.runner(argv, timeout)
            return {"robot_id": robot_id, **result}

        results = await asyncio.gather(
            *(run_one(robot, argv, timeout) for robot, argv, timeout in commands)
        )
        return {
            "ok": all(result.get("returncode") == 0 for result in results),
            "action": action.action,
            "results": results,
        }

    @staticmethod
    async def _run_command(argv: list[str], timeout: float) -> CommandResult:
        process = await asyncio.create_subprocess_exec(
            *argv,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        try:
            stdout_bytes, stderr_bytes = await asyncio.wait_for(
                process.communicate(), timeout=timeout
            )
        except asyncio.TimeoutError:
            process.kill()
            await process.wait()
            return {
                "returncode": 124,
                "stdout": "",
                "stderr": f"robot tool timed out after {timeout:.0f}s",
            }

        stdout = stdout_bytes.decode(errors="replace")
        stderr = stderr_bytes.decode(errors="replace")
        payload: Any = None
        if stdout.strip():
            try:
                payload = json.loads(stdout)
            except json.JSONDecodeError:
                payload = None
        return {
            "returncode": process.returncode,
            "stdout": stdout,
            "stderr": stderr,
            "payload": payload,
        }
