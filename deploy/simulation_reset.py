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
import sys
import urllib.request
from contextlib import contextmanager
from pathlib import Path
from threading import Event, Lock, Thread
from uuid import UUID, uuid4

# Also support direct host execution by deploy/simulation_launch.py.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from autonomy.map_epochs import robot_run_id
from deploy.mgg.reset_protocol import (
    open_lock,
    prepare_directory,
    prepare_reset_directories,
)

SUPERVISOR_STALE_NS = 15_000_000_000
MIN_DOMAIN_ID = 1
MAX_DOMAIN_ID = 232


def atomic_text(path: Path, value: str, *, mode: int | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.{uuid4().hex}.tmp")
    with temporary.open("w") as stream:
        stream.write(value)
        stream.flush()
        os.fsync(stream.fileno())
    if mode is not None:
        temporary.chmod(mode)
    elif path.exists():
        temporary.chmod(path.stat().st_mode & 0o777)
    os.replace(temporary, path)
    descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def atomic_json(path: Path, value: dict) -> None:
    atomic_text(path, json.dumps(value, sort_keys=True), mode=0o644)


@contextmanager
def protocol_lock(root: Path):
    """Serialize the two-file request/status protocol across host processes."""
    prepare_directory(root)
    descriptor = open_lock(root / "protocol.lock")
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
        prune_volume: str = "",
        prune_image: str = "",
        robot_ids: list[str] | None = None,
    ):
        self.prune_volume, self.prune_image = prune_volume, prune_image
        self.root, self.env_file = root, env_file
        self.compose_command, self.services = command, services
        self.server_url, self.expected_robots = server_url.rstrip("/"), expected_robots
        self._status_lock = Lock()
        self.robot_services = {
            robot: f"peer{index}" for index, robot in enumerate(robot_ids or [])
        }
        required = {"mgg", "mapping", "mapping-query", *self.robot_services.values()}
        if not required.issubset(services):
            self.robot_services = {}
        # Publish the complete writable protocol tree before advertising a
        # heartbeat: a root server must never win creation of requests/ first.
        prepare_reset_directories(root, list(self.robot_services))

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
            {
                "version": 1,
                "pid": os.getpid(),
                "updated_at_ns": time.time_ns(),
                "supported_robot_ids": list(self.robot_services),
                "mission_id": read_deployment_env(self.env_file).get(
                    "SWARMDECK_MISSION_ID"
                ),
            },
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
            if self.prune_volume and self.prune_image:
                # Peers write one directory per mission into a volume only the
                # simulation uses, and nothing reads a mission after its reset.
                # The services are stopped, so nothing holds the old files. A
                # plain container does this: the stack's own services share
                # the stopped simulator's network and cannot start without it.
                # Reported as part of stopping: the reset protocol names no
                # separate phase for it and its readers need not learn one.
                self.run_command(
                    request,
                    "stopping",
                    [
                        "docker",
                        "run",
                        "--rm",
                        "--network",
                        "none",
                        "--volume",
                        f"{self.prune_volume}:/maps",
                        "--entrypoint",
                        "sh",
                        self.prune_image,
                        "-c",
                        "find /maps -mindepth 1 -maxdepth 1 -exec rm -rf {} +",
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

    def robot_status(self, request: dict, previous: dict, phase: str, **fields) -> dict:
        result = {
            **request,
            **previous,
            "version": 1,
            "phase": phase,
            "updated_at_ns": time.time_ns(),
            "ok": None,
            "error": None,
            **fields,
        }
        directory = self.root / "robots" / request["robot_id"]
        with protocol_lock(directory):
            # Journal first: loss of the convenience status file cannot replay
            # an already completed reset, even after a later request.
            atomic_json(
                directory / "requests" / f"{request['request_id']}.json", result
            )
            atomic_json(directory / "status.json", result)
        return result

    @staticmethod
    def remaining(deadline: float) -> float:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("robot map reset exceeded its 60 second deadline")
        return remaining

    def robot_command(self, arguments: list[str], environment: dict, deadline: float):
        return subprocess.run(
            [*self.compose_command, "--env-file", str(self.env_file), *arguments],
            check=True,
            capture_output=True,
            text=True,
            env=environment,
            timeout=self.remaining(deadline),
        )

    def planner_state(self, request: dict, state: str, deadline: float) -> None:
        directory = self.root / "robots" / request["robot_id"]
        atomic_json(
            directory / "mgg-request.json",
            {
                "version": 1,
                "request_id": request["request_id"],
                "robot_id": request["robot_id"],
                "mission_id": request["mission_id"],
                "state": state,
                "map_epoch": request["map_epoch"],
            },
        )
        while self.remaining(deadline):
            try:
                result = json.loads((directory / "mgg-status.json").read_text())
                if (
                    result.get("request_id") == request["request_id"]
                    and result.get("mission_id") == request["mission_id"]
                    and result.get("state") == "failed"
                ):
                    raise RuntimeError(
                        result.get("error") or "robot planner reset failed"
                    )
                if (
                    result.get("request_id") == request["request_id"]
                    and result.get("mission_id") == request["mission_id"]
                    and result.get("state") == state
                    and 0 <= time.time_ns() - result["updated_at_ns"] < 3_000_000_000
                    and (state == "stopped" or type(result.get("pid")) is int)
                    and (
                        state == "stopped"
                        or (
                            type(result.get("map_epoch")) is int
                            and result["map_epoch"] >= request["map_epoch"]
                            and result.get("run_id")
                            == robot_run_id(
                                request["mission_id"],
                                request["robot_id"],
                                result["map_epoch"],
                            )
                            and result.get("source_reset_run_id") == result["run_id"]
                        )
                    )
                ):
                    return
            except (OSError, ValueError, KeyError, TypeError):
                pass
            time.sleep(min(0.1, self.remaining(deadline)))

    def robot_ros(
        self, request: dict, operation: str, environment: dict, deadline: float
    ):
        # argv is passed through bash's positional parameters, not interpolated
        # into shell code. The same installed interfaces serve every robot.
        result = self.robot_command(
            [
                "exec",
                "-T",
                "mgg",
                "bash",
                "-lc",
                "source /opt/ros/jazzy/setup.bash && "
                "source /opt/mgg/ros2/install/setup.bash && "
                'exec python3 /app/deploy/mgg/robot_reset.py "$@"',
                "robot-reset",
                operation,
                "--robot",
                request["robot_id"],
                "--mission",
                request["mission_id"],
                "--minimum",
                str(request["map_epoch"]),
                "--timeout",
                str(max(0.1, self.remaining(deadline) - 0.5)),
            ],
            environment,
            deadline,
        )
        return json.loads(result.stdout.splitlines()[-1])

    def run_robot(self, request: dict, status: dict) -> None:
        try:
            if request["robot_id"] not in self.robot_services:
                raise RuntimeError(
                    f"robot-local reset unavailable for {request['robot_id']} "
                    "because the CSLAM peer is not active"
                )
            current = read_deployment_env(self.env_file)
            if request["mission_id"] != current.get("SWARMDECK_MISSION_ID"):
                raise ValueError(
                    "robot reset mission no longer matches the running fleet"
                )
            environment = dict(os.environ)
            environment.update(current)
            deadline = (
                time.monotonic()
                + (status["deadline_at_ns"] - time.time_ns()) / 1_000_000_000
            )
            self.remaining(deadline)
            service = self.robot_services[request["robot_id"]]
            if status["phase"] == "accepted":
                status = self.robot_status(request, status, "stopping")
            if status["phase"] == "stopping":
                # End only this robot's planner publishers before cancelling
                # its old Nav2 actions; no missing PCI service can block recovery.
                self.planner_state(request, "stopped", deadline)
                if not status.get("quiesced"):
                    self.robot_ros(request, "quiesce", environment, deadline)
                    status = self.robot_status(
                        request, status, "stopping", quiesced=True
                    )
                self.robot_command(
                    ["stop", "--timeout", "5", service], environment, deadline
                )
                status = self.robot_status(request, status, "starting")
            if status["phase"] == "starting":
                # Idempotent across supervisor crashes. Unlike restart or
                # force-recreate, up will not kill a peer already started by
                # this request and therefore cannot claim a duplicate epoch.
                self.robot_command(
                    ["up", "-d", "--no-deps", service], environment, deadline
                )
                self.planner_state(request, "running", deadline)
                status = self.robot_status(request, status, "verifying")
            if status["phase"] == "verifying":
                fresh = self.robot_ros(request, "verify", environment, deadline)
                epoch = fresh["map_epoch"]
                if epoch < request["map_epoch"] or fresh["run_id"] != robot_run_id(
                    request["mission_id"], request["robot_id"], epoch
                ):
                    raise ValueError("readiness probe returned a stale robot run")
                status = self.robot_status(request, status, "verifying", **fresh)
                while self.remaining(deadline):
                    try:
                        with urllib.request.urlopen(
                            f"{self.server_url}/api/fleet",
                            timeout=min(2, self.remaining(deadline)),
                        ) as response:
                            body = json.loads(response.read())
                        robot = next(
                            (
                                r
                                for r in body.get("robots", [])
                                if r.get("robot_id") == request["robot_id"]
                            ),
                            {},
                        )
                        frame = robot.get("live_mapping") or {}
                        if (
                            robot.get("online") is True
                            and robot.get("navigation_ready") is True
                            and frame.get("mission_id") == request["mission_id"]
                            and frame.get("robot_map_epoch") == epoch
                            and frame.get("run_id") == fresh["run_id"]
                            and frame.get("mapping_graph_revision", 0) > 0
                        ):
                            self.robot_status(request, status, "done", ok=True)
                            return
                    except (OSError, ValueError):
                        pass
                    time.sleep(min(0.2, self.remaining(deadline)))
        except (
            OSError,
            RuntimeError,
            ValueError,
            KeyError,
            TypeError,
            subprocess.SubprocessError,
        ) as exc:
            error = f"{type(exc).__name__}: {exc}"
            if isinstance(exc, subprocess.CalledProcessError):
                error += f": {exc.stderr or exc.stdout}"
            self.robot_status(request, status, "failed", ok=False, error=error[-4000:])

    def poll_robots(self) -> bool:
        robots = self.root / "robots"
        if not robots.exists():
            return False
        handled = False
        for directory in sorted(robots.iterdir()):
            if not directory.is_dir() or not re.fullmatch(
                r"[A-Za-z_][A-Za-z0-9_]*", directory.name
            ):
                continue
            with protocol_lock(directory):
                try:
                    request = json.loads((directory / "request.json").read_text())
                    if (
                        request["version"] != 1
                        or request["robot_id"] != directory.name
                        or str(UUID(request["request_id"])) != request["request_id"]
                        or type(request["requested_at_ns"]) is not int
                    ):
                        continue
                    robot_run_id(
                        request["mission_id"], request["robot_id"], request["map_epoch"]
                    )
                    journal = directory / "requests" / f"{request['request_id']}.json"
                    try:
                        status = json.loads(journal.read_text())
                    except FileNotFoundError:
                        status = {
                            **request,
                            "phase": "accepted",
                            "ok": None,
                            "deadline_at_ns": min(
                                request["requested_at_ns"], time.time_ns()
                            )
                            + 60_000_000_000,
                        }
                    status.setdefault(
                        "deadline_at_ns",
                        min(request["requested_at_ns"], time.time_ns())
                        + 60_000_000_000,
                    )
                    if status["phase"] in {"done", "failed"}:
                        atomic_json(directory / "status.json", status)
                        continue
                    # Bind retries to the original payload as well as its ID.
                    request = {
                        key: status[key]
                        for key in (
                            "version",
                            "request_id",
                            "robot_id",
                            "mission_id",
                            "map_epoch",
                            "requested_at_ns",
                        )
                    }
                    atomic_json(journal, status)
                except (OSError, ValueError, KeyError, TypeError):
                    continue
            self.run_robot(request, status)
            handled = True
        return handled


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
    parser.add_argument("--robot-id", action="append", default=[])
    parser.add_argument(
        "--prune-maps-volume",
        default="",
        help="Docker volume holding the simulation's peer maps; every earlier "
        "mission in it is deleted at each reset",
    )
    parser.add_argument(
        "--prune-maps-image",
        default="",
        help="locally available image with a POSIX shell, used to empty the volume",
    )
    args = parser.parse_args()
    service_pattern = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*")
    if bool(args.prune_maps_volume) != bool(args.prune_maps_image) or (
        args.prune_maps_volume
        and (
            re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", args.prune_maps_volume) is None
            or re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_./:@-]*", args.prune_maps_image)
            is None
        )
    ):
        parser.error("--prune-maps-volume and --prune-maps-image go together")
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
    if any(
        re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", robot) is None
        for robot in args.robot_id
    ):
        parser.error("robot IDs must be simple ROS namespaces")
    if len(set(args.robot_id)) != len(args.robot_id):
        parser.error("robot IDs must be unique")
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
        args.prune_maps_volume,
        args.prune_maps_image,
        args.robot_id,
    )
    # A second supervisor could otherwise claim a request after a stale timeout
    # while the first one is still changing the same Compose project.
    descriptor = open_lock(args.root / "supervisor.lock")
    with os.fdopen(descriptor) as supervisor_lock:
        try:
            fcntl.flock(supervisor_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            parser.error(f"another supervisor already owns {args.root}")
            raise AssertionError("unreachable") from exc
        stopped = Event()

        def heartbeat():
            while not stopped.wait(2.0):
                supervisor.supervisor_heartbeat()

        supervisor.supervisor_heartbeat()
        thread = Thread(target=heartbeat, daemon=True)
        thread.start()
        try:
            recover_active = True
            while True:
                supervisor.poll_once(recover_active=recover_active)
                supervisor.poll_robots()
                recover_active = False
                time.sleep(max(0.1, args.poll))
        finally:
            stopped.set()
            thread.join(timeout=3)


if __name__ == "__main__":
    main()
