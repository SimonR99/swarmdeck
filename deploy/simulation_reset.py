#!/usr/bin/env python3
"""Host-side Compose supervisor for epoch-safe simulation resets."""

from __future__ import annotations

import argparse
import fcntl
import json
import os
import re
import subprocess
import time
import urllib.request
from contextlib import contextmanager
from pathlib import Path
from threading import Event, Lock, Thread
from uuid import uuid4

SUPERVISOR_STALE_NS = 15_000_000_000
MIN_DOMAIN_ID = 1
MAX_DOMAIN_ID = 232


def atomic_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.{uuid4().hex}.tmp")
    temporary.write_text(value)
    if path.exists():
        temporary.chmod(path.stat().st_mode & 0o777)
    os.replace(temporary, path)


def atomic_json(path: Path, value: dict) -> None:
    atomic_text(path, json.dumps(value, sort_keys=True))


@contextmanager
def protocol_lock(root: Path):
    """Serialize the two-file request/status protocol across host processes."""
    root.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(root / "protocol.lock", os.O_RDONLY | os.O_CREAT, 0o666)
    with os.fdopen(descriptor) as handle:
        fcntl.flock(handle, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(handle, fcntl.LOCK_UN)


def validate_domain(value: int | str) -> int:
    if isinstance(value, bool):
        raise ValueError(
            f"SWARMDECK_TEST_DOMAIN must be {MIN_DOMAIN_ID}..{MAX_DOMAIN_ID}"
        )
    try:
        domain = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"SWARMDECK_TEST_DOMAIN must be {MIN_DOMAIN_ID}..{MAX_DOMAIN_ID}"
        ) from exc
    if not MIN_DOMAIN_ID <= domain <= MAX_DOMAIN_ID:
        raise ValueError(
            f"SWARMDECK_TEST_DOMAIN must be {MIN_DOMAIN_ID}..{MAX_DOMAIN_ID}"
        )
    return domain


def next_domain(previous: int) -> int:
    # DDS domains 1..232 are valid here. Advancing deterministically avoids a
    # collision with delayed packets from the just-ended graph.
    return validate_domain(previous) % MAX_DOMAIN_ID + MIN_DOMAIN_ID


def write_deployment_env(path: Path, mission_id: str, domain: int) -> None:
    """Update epoch keys without discarding the deployment's other settings."""
    lines = path.read_text().splitlines() if path.exists() else []
    replacements = {
        "SWARMDECK_MISSION_ID": mission_id,
        "SWARMDECK_TEST_DOMAIN": str(domain),
    }
    found: set[str] = set()
    output: list[str] = []
    assignment = re.compile(r"^(\s*)(?:export\s+)?([A-Za-z_][A-Za-z0-9_]*)\s*=.*$")
    for line in lines:
        match = assignment.match(line)
        key = match.group(2) if match else ""
        if key in replacements:
            if key not in found:
                output.append(f"{key}={replacements[key]}")
                found.add(key)
            # Drop duplicate definitions. Leaving an old duplicate below the
            # generated value would make Compose use the wrong epoch.
            continue
        output.append(line)
    for key, value in replacements.items():
        if key not in found:
            output.append(f"{key}={value}")
    atomic_text(path, "\n".join(output) + "\n")


def read_deployment_env(path: Path) -> dict[str, str]:
    current: dict[str, str] = {}
    if not path.exists():
        return current
    for line in path.read_text().splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or "=" not in stripped:
            continue
        if stripped.startswith("export "):
            stripped = stripped[7:].lstrip()
        key, value = stripped.split("=", 1)
        current[key.strip()] = value.strip()
    return current


def ensure_deployment_env(path: Path) -> dict[str, str]:
    """Make --env-file usable before either the stack or supervisor is started."""
    current = read_deployment_env(path)
    complete = all(
        key in current for key in ("SWARMDECK_MISSION_ID", "SWARMDECK_TEST_DOMAIN")
    )
    if complete:
        validate_domain(current["SWARMDECK_TEST_DOMAIN"])
        return current
    mission = current.get(
        "SWARMDECK_MISSION_ID", os.environ.get("SWARMDECK_MISSION_ID", "")
    ) or str(uuid4())
    domain = validate_domain(
        current.get(
            "SWARMDECK_TEST_DOMAIN", os.environ.get("SWARMDECK_TEST_DOMAIN", "173")
        )
    )
    write_deployment_env(path, mission, domain)
    return read_deployment_env(path)


class Supervisor:
    def __init__(
        self,
        root: Path,
        env_file: Path,
        command: list[str],
        services: list[str],
        server_url: str = "",
        expected_robots: int = 0,
        prune_service: str = "",
        prune_path: str = "/maps",
    ):
        self.prune_service, self.prune_path = prune_service, prune_path
        self.root, self.env_file = root, env_file
        self.compose_command, self.services = command, services
        self.server_url, self.expected_robots = server_url.rstrip("/"), expected_robots
        self._status_lock = Lock()

    def status(self, request: dict, phase: str, **fields) -> None:
        with self._status_lock:
            atomic_json(
                self.root / "status.json",
                {
                    "version": 1,
                    "request_id": request["request_id"],
                    "phase": phase,
                    "updated_at_ns": time.time_ns(),
                    **fields,
                },
            )

    def supervisor_heartbeat(self) -> None:
        atomic_json(
            self.root / "supervisor.json",
            {"version": 1, "pid": os.getpid(), "updated_at_ns": time.time_ns()},
        )

    def run_command(
        self,
        request: dict,
        phase: str,
        arguments: list[str],
        environment: dict[str, str],
    ) -> None:
        stopped = Event()

        def heartbeat() -> None:
            while not stopped.wait(2.0):
                self.status(request, phase, ok=None)

        thread = Thread(target=heartbeat, daemon=True)
        thread.start()
        try:
            subprocess.run(arguments, check=True, timeout=180, env=environment)
        finally:
            stopped.set()
            thread.join(timeout=3.0)

    def require_services_running(self, environment: dict[str, str]) -> None:
        result = subprocess.run(
            [
                *self.compose_command,
                "--env-file",
                str(self.env_file),
                "ps",
                "--status",
                "running",
                "--services",
                *self.services,
            ],
            check=True,
            capture_output=True,
            text=True,
            timeout=10,
            env=environment,
        )
        running = set(result.stdout.splitlines())
        missing = sorted(set(self.services) - running)
        if missing:
            raise RuntimeError(f"services exited during reset: {', '.join(missing)}")

    def run(self, request: dict) -> None:
        try:
            current = ensure_deployment_env(self.env_file)
            previous_domain = current.get(
                "SWARMDECK_TEST_DOMAIN", os.environ.get("SWARMDECK_TEST_DOMAIN", "173")
            )
            previous_domain_id = validate_domain(previous_domain)
            domain = next_domain(previous_domain_id)
            mission = str(uuid4())
            stop_environment = dict(os.environ)
            for key in ("SWARMDECK_MISSION_ID", "SWARMDECK_TEST_DOMAIN"):
                if key in current:
                    stop_environment[key] = current[key]
            self.status(request, "stopping", ok=None)
            self.run_command(
                request,
                "stopping",
                [
                    *self.compose_command,
                    "--env-file",
                    str(self.env_file),
                    "stop",
                    *self.services,
                ],
                stop_environment,
            )
            write_deployment_env(self.env_file, mission, domain)
            # Shell variables outrank --env-file during Compose interpolation.
            # Override inherited epoch values explicitly or a supervisor started
            # from the original deployment shell recreates the original mission.
            start_environment = dict(os.environ)
            start_environment.update(
                SWARMDECK_MISSION_ID=mission,
                SWARMDECK_TEST_DOMAIN=str(domain),
            )
            if self.prune_service:
                # Peers write one directory per mission into a volume only the
                # simulation uses, and nothing reads a mission after its reset.
                # The services are stopped, so nothing holds the old files.
                # Reported as part of stopping: the reset protocol names no
                # separate phase for it and its readers need not learn one.
                self.run_command(
                    request,
                    "stopping",
                    [
                        *self.compose_command,
                        "--env-file",
                        str(self.env_file),
                        "run",
                        "--rm",
                        "--no-deps",
                        "-T",
                        "--entrypoint",
                        "sh",
                        self.prune_service,
                        "-c",
                        'find "$1" -mindepth 1 -maxdepth 1 -exec rm -rf {} +',
                        "sh",
                        self.prune_path,
                    ],
                    start_environment,
                )
            self.status(
                request, "starting", ok=None, mission_id=mission, domain_id=domain
            )
            self.run_command(
                request,
                "starting",
                [
                    *self.compose_command,
                    "--env-file",
                    str(self.env_file),
                    "up",
                    "-d",
                    "--force-recreate",
                    "--wait",
                    "--wait-timeout",
                    "120",
                    *self.services,
                ],
                start_environment,
            )
            # `--wait` returns when services without a healthcheck are merely
            # running. Confirm every requested long-lived process survived the
            # startup transition even when fleet readiness checking is disabled.
            self.require_services_running(start_environment)
            if self.expected_robots:
                self.status(
                    request, "verifying", ok=None, mission_id=mission, domain_id=domain
                )
                deadline = time.monotonic() + 120
                while time.monotonic() < deadline:
                    self.status(
                        request,
                        "verifying",
                        ok=None,
                        mission_id=mission,
                        domain_id=domain,
                    )
                    try:
                        with urllib.request.urlopen(
                            f"{self.server_url}/api/fleet", timeout=2
                        ) as response:
                            body = json.loads(response.read())
                        online = [
                            robot
                            for robot in body.get("robots", [])
                            if robot.get("online") is True
                        ]
                        ready = [
                            robot
                            for robot in online
                            if robot.get("navigation_ready") is True
                            and "reset" in (robot.get("capabilities") or [])
                        ]
                        if len(ready) >= self.expected_robots:
                            break
                    except (OSError, ValueError, json.JSONDecodeError):
                        pass
                    time.sleep(1)
                else:
                    raise TimeoutError(
                        f"only {len(ready) if 'ready' in locals() else 0}/"
                        f"{self.expected_robots} robots navigation-ready "
                        f"({len(online) if 'online' in locals() else 0} online)"
                    )
                # ARGoS or another unmonitored process may exit while adapters
                # are becoming ready, so check the complete service set again.
                self.require_services_running(start_environment)
            self.status(request, "done", ok=True, mission_id=mission, domain_id=domain)
        except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as exc:
            self.status(
                request, "failed", ok=False, error=f"{type(exc).__name__}: {exc}"
            )

    def poll_once(self, *, recover_active: bool = False) -> bool:
        request_path = self.root / "request.json"
        with protocol_lock(self.root):
            try:
                request = json.loads(request_path.read_text())
            except (FileNotFoundError, OSError, json.JSONDecodeError):
                return False
            if (
                not isinstance(request, dict)
                or request.get("version") != 1
                or not isinstance(request.get("request_id"), str)
            ):
                return False
            status = {}
            try:
                status = json.loads((self.root / "status.json").read_text())
            except (FileNotFoundError, OSError, json.JSONDecodeError):
                pass
            if status.get("request_id") == request["request_id"]:
                active = status.get("phase") in {
                    "accepted",
                    "stopping",
                    "starting",
                    "verifying",
                }
                updated = status.get("updated_at_ns", status.get("requested_at_ns"))
                stale = not isinstance(updated, int) or (
                    time.time_ns() - updated > SUPERVISOR_STALE_NS
                )
                if status.get("phase") != "accepted" and not (
                    active and (recover_active or stale)
                ):
                    return False
            self.status(request, "stopping", ok=None)
        self.run(request)
        return True


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--env-file", type=Path, required=True)
    parser.add_argument("--project", required=True)
    parser.add_argument("--compose-file", action="append", required=True)
    parser.add_argument("--service", action="append", required=True)
    parser.add_argument("--poll", type=float, default=0.5)
    parser.add_argument("--server-url", default="http://127.0.0.1:8080")
    parser.add_argument("--expected-robots", type=int, default=0)
    parser.add_argument(
        "--prune-maps-service",
        default="",
        help="service that mounts the simulation's peer maps volume read-write; "
        "its earlier missions are deleted at every reset",
    )
    parser.add_argument("--prune-maps-path", default="/maps")
    args = parser.parse_args()
    service_pattern = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*")
    if args.prune_maps_service and (
        service_pattern.fullmatch(args.prune_maps_service) is None
        or re.fullmatch(r"/[A-Za-z0-9_./-]+", args.prune_maps_path) is None
        or ".." in args.prune_maps_path
        or args.prune_maps_path.rstrip("/") == ""
    ):
        parser.error("prune service or path is not valid")
    if any(service_pattern.fullmatch(service) is None for service in args.service):
        parser.error(
            "service names may contain only letters, digits, dot, underscore and dash"
        )
    compose_files = []
    for value in args.compose_file:
        path = Path(value)
        if path.suffix not in {".yml", ".yaml"} or not path.is_file():
            parser.error(f"compose file is not an existing YAML file: {value}")
        compose_files.append(str(path.resolve()))
    command = ["docker", "compose", "-p", args.project]
    for path in compose_files:
        command.extend(("-f", path))
    if args.expected_robots < 0:
        parser.error("expected robots must be nonnegative")
    try:
        ensure_deployment_env(args.env_file)
    except (OSError, ValueError) as exc:
        parser.error(f"cannot initialize deployment env: {exc}")
    supervisor = Supervisor(
        args.root,
        args.env_file,
        command,
        args.service,
        args.server_url,
        args.expected_robots,
        args.prune_maps_service,
        args.prune_maps_path,
    )
    args.root.mkdir(parents=True, exist_ok=True)
    # A second supervisor could otherwise claim a request after a stale timeout
    # while the first one is still changing the same Compose project.
    descriptor = os.open(args.root / "supervisor.lock", os.O_RDONLY | os.O_CREAT, 0o666)
    with os.fdopen(descriptor) as supervisor_lock:
        try:
            fcntl.flock(supervisor_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            parser.error(f"another supervisor already owns {args.root}")
            raise AssertionError("unreachable") from exc
        recover_active = True
        while True:
            supervisor.supervisor_heartbeat()
            supervisor.poll_once(recover_active=recover_active)
            recover_active = False
            supervisor.supervisor_heartbeat()
            time.sleep(max(0.1, args.poll))


if __name__ == "__main__":
    main()
