#!/usr/bin/env python3
"""Launch and manage the complete ARGoS simulation stack."""

from __future__ import annotations

import argparse
import ast
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from uuid import uuid4

try:
    from .simulation_reset import (
        atomic_json,
        ensure_deployment_env,
        next_domain,
        protocol_lock,
        prepare_reset_directories,
        read_deployment_env,
        validate_domain,
        write_deployment_env,
    )
except ImportError:  # Direct execution from deploy/.
    from simulation_reset import (  # type: ignore[no-redef]
        atomic_json,
        ensure_deployment_env,
        next_domain,
        protocol_lock,
        prepare_reset_directories,
        read_deployment_env,
        validate_domain,
        write_deployment_env,
    )


REPO = Path(__file__).resolve().parents[1]
COMPOSE = REPO / "deploy" / "compose"
RESET_ROOT = REPO / "sessions" / "simulation-reset"
ENV_FILE = RESET_ROOT / "deployment.env"
STATE_ROOT = REPO / "sessions" / "simulation-launch"
PROJECT_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*")
MAX_SIMULATION_PEERS = 4
OPTIONAL_SIMULATION_SERVICES = frozenset(
    {
        "duck_detector",
        "fast_livo2",
        "mapping",
        *(f"peer{index}" for index in range(MAX_SIMULATION_PEERS)),
    }
)

ALIASES = {
    "default": "configs/4robot.yaml",
    "4robot": "configs/4robot.yaml",
    "bistro": "configs/4robot_bistro.yaml",
    "4robot_bistro": "configs/4robot_bistro.yaml",
    "subt_finals": "configs/4robot_subt_finals.yaml",
    "4robot_subt_finals": "configs/4robot_subt_finals.yaml",
    "3robot": "configs/3robot.yaml",
    "dev": "configs/3robot.yaml",
}


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        description=__doc__,
        epilog=(
            "Custom YAML must contain literal scalar fleet.robot_count and "
            "fleet.robot_prefix fields; the launcher intentionally does not require PyYAML."
        ),
    )
    actions = result.add_mutually_exclusive_group()
    actions.add_argument("--down", action="store_const", const="down", dest="action")
    actions.add_argument(
        "--status", "--ps", action="store_const", const="status", dest="action"
    )
    actions.add_argument("--logs", action="store_const", const="logs", dest="action")
    actions.add_argument(
        "--dry-run", action="store_const", const="dry-run", dest="action"
    )
    result.set_defaults(action="up")
    result.add_argument("-s", "--scenario", default=os.getenv("SCENARIO", "default"))
    result.add_argument(
        "-r",
        "--render",
        choices=("software", "gpu", "nvidia", "dri", "intel", "amd"),
        default=os.getenv("RENDER", "software").lower(),
    )
    render = result.add_mutually_exclusive_group()
    render.add_argument(
        "--gpu", "--nvidia", action="store_const", const="gpu", dest="render"
    )
    render.add_argument(
        "--dri",
        "--intel",
        "--amd",
        action="store_const",
        const="dri",
        dest="render",
    )
    render.add_argument(
        "--software", action="store_const", const="software", dest="render"
    )
    result.add_argument(
        "-o",
        "--odometry",
        choices=("fast_livo2", "drift"),
        default=os.getenv("ODOMETRY", "fast_livo2").lower(),
    )
    result.add_argument("--drift", action="store_const", const="drift", dest="odometry")
    result.add_argument(
        "-t", "--targets", type=int, default=int(os.getenv("TARGETS", "10"))
    )
    result.add_argument(
        "--fast-livo2", action="store_const", const="fast_livo2", dest="odometry"
    )
    result.add_argument(
        "-e", "--explore", type=int, default=int(os.getenv("EXPLORE_SECONDS", "0"))
    )
    result.add_argument(
        "--robot-poses",
        choices=("cslam", "ground_truth"),
        default=os.getenv("ROBOT_POSES", "cslam").lower(),
        help=(
            "inter-robot transforms the MGG planners share roadmaps with: "
            "cslam (default, as on hardware) shares nothing until C-SLAM links "
            "two robots into one map component, which inter-robot closures "
            "(off by default in simulation) do; ground_truth shares from the "
            "start with the simulator's poses"
        ),
    )
    result.add_argument(
        "--detector",
        action=argparse.BooleanOptionalAction,
        default=os.getenv("DETECTOR", "0").lower() in {"1", "true", "yes", "on"},
        help="run the YOLOE object detector sidecar (off by default)",
    )
    result.add_argument("--no-build", action="store_false", dest="build", default=True)
    result.add_argument("--build", action="store_true", dest="build")
    result.add_argument(
        "--no-detach", action="store_false", dest="detach", default=True
    )
    result.add_argument(
        "--dev",
        action="store_true",
        help="3-robot scenario with DRI rendering and drift odometry",
    )
    return result


def scalar(value: str) -> str:
    value = value.split("#", 1)[0].strip()
    if value[:1] in {'"', "'"}:
        parsed = ast.literal_eval(value)
        if not isinstance(parsed, str):
            raise ValueError("expected a string")
        return parsed
    return value


def platforms_from_config(path: Path, names: list[str]) -> dict[str, str]:
    """Resolve each robot's platform, the way the ARGoS session generator does.

    `fleet.robot_type` is the fleet default and `fleet.robot_types` overrides
    it per robot. The peer bridge needs this because it cannot import the
    simulation package to look a chassis up itself, and a peer-body mask sized
    from the wrong platform masks the wrong volume. Parsed with the same
    indentation scan as the fleet scalars so the host keeps needing no YAML
    dependency; unknown keys are simply not matched.
    """
    in_fleet = False
    child_indent: int | None = None
    in_types = False
    default: str | None = None
    overrides: dict[str, str] = {}
    for line in path.read_text().splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        if not line.startswith((" ", "\t")):
            in_fleet = re.fullmatch(r"fleet\s*:\s*(?:#.*)?", line) is not None
            in_types = False
            continue
        if not in_fleet:
            continue
        indentation = len(line) - len(line.lstrip(" "))
        if child_indent is None:
            child_indent = indentation
        if indentation == child_indent:
            in_types = re.match(r"^\s+robot_types\s*:\s*(?:#.*)?$", line) is not None
            match = re.match(r"^\s+robot_type\s*:\s*(.*?)\s*$", line)
            if match and match.group(1).split("#", 1)[0].strip():
                default = scalar(match.group(1))
            continue
        if in_types and indentation > child_indent:
            match = re.match(r"^\s+([A-Za-z_][A-Za-z0-9_]*)\s*:\s*(.*?)\s*$", line)
            if match:
                overrides[match.group(1)] = scalar(match.group(2))
    unknown = sorted(set(overrides) - set(names))
    if unknown:
        raise ValueError(
            f"{path} fleet.robot_types names robots outside this fleet: {unknown}"
        )
    # Mirrors DEFAULT_ROBOT_PROFILE in the simulation package: a config that
    # names no platform spawns Scout Minis, and the mask must describe the
    # bodies that were actually spawned.
    return {name: overrides.get(name, default or "scout_mini") for name in names}


def fleet_from_config(path: Path) -> tuple[int, str]:
    """Read the two required fleet scalars without adding a host YAML dependency."""
    in_fleet = False
    child_indent: int | None = None
    count: int | None = None
    prefix: str | None = None
    for line in path.read_text().splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        if not line.startswith((" ", "\t")):
            in_fleet = re.fullmatch(r"fleet\s*:\s*(?:#.*)?", line) is not None
            continue
        if not in_fleet:
            continue
        if "\t" in line[: len(line) - len(line.lstrip())]:
            raise ValueError(f"{path} uses tabs in fleet; use YAML space indentation")
        indentation = len(line) - len(line.lstrip(" "))
        if child_indent is None:
            child_indent = indentation
        if indentation != child_indent:
            continue
        match = re.match(r"^\s+(robot_count|robot_prefix)\s*:\s*(.*?)\s*$", line)
        if not match:
            continue
        value = scalar(match.group(2))
        if match.group(1) == "robot_count":
            count = int(value)
        else:
            prefix = value
    if count is None or prefix is None:
        raise ValueError(
            f"{path} must define literal scalar fleet.robot_count and "
            "fleet.robot_prefix fields"
        )
    if count < 1:
        raise ValueError(f"{path} must configure at least one simulation robot")
    if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", prefix) is None:
        raise ValueError(
            f"{path} fleet.robot_prefix must contain only ROS namespace "
            "letters, digits and underscores, and may not start with a digit"
        )
    return count, prefix


def scenario_path(value: str) -> Path:
    candidate = REPO / ALIASES[value] if value in ALIASES else Path(value)
    if not candidate.is_absolute():
        candidate = REPO / candidate
    candidate = candidate.resolve()
    if not candidate.is_file():
        raise ValueError(f"scenario config not found: {value}")
    return candidate


def custom_config_overlay(project: str, source: Path) -> Path:
    path = STATE_ROOT / f"{project}.custom-config.yml"
    quoted_source = json.dumps(str(source))
    lines = ["services:"]
    for service in ("server", "sim", "mgg"):
        lines.extend(
            (
                f"  {service}:",
                "    volumes:",
                "      - type: bind",
                f"        source: {quoted_source}",
                "        target: /app/configs/swarmdeck-launch.yaml",
                "        read_only: true",
            )
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n")
    return path


def compose_files(render: str) -> list[Path]:
    files = [COMPOSE / "docker-compose.yml"]
    if render in {"gpu", "nvidia"}:
        files.append(COMPOSE / "docker-compose.gpu.yml")
    elif render in {"dri", "intel", "amd"}:
        files.append(COMPOSE / "docker-compose.dri.yml")
    return files


def new_epoch() -> dict[str, str]:
    with protocol_lock(RESET_ROOT):
        existed = ENV_FILE.exists()
        current = ensure_deployment_env(ENV_FILE)
        if existed:
            write_deployment_env(
                ENV_FILE,
                str(uuid4()),
                next_domain(validate_domain(current["SWARMDECK_TEST_DOMAIN"])),
            )
        return read_deployment_env(ENV_FILE)


def process_environment(
    spec: dict, epoch: dict[str, str] | None = None
) -> dict[str, str]:
    environment = dict(os.environ)
    platforms = spec.get("robot_platforms") or {}
    environment.update(
        COMPOSE_PROFILES="argos",
        SWARMDECK_CONFIG=spec["container_config"],
        SWARMDECK_ODOMETRY=spec["odometry"],
        SWARMDECK_ROBOT_POSES=spec.get("robot_poses", "cslam"),
        SWARMDECK_TARGETS=str(spec["targets"]),
        EXPLORE_SECONDS=str(spec["explore"]),
        SWARMDECK_ROBOT_COUNT=str(spec["robot_count"]),
        SWARMDECK_PEER_NAMES=json.dumps(spec["robot_names"], separators=(",", ":")),
        SWARMDECK_PEER_BODY_MASK="true" if platforms else "false",
        SWARMDECK_PEER_PLATFORMS=json.dumps(
            platforms, separators=(",", ":"), sort_keys=True
        ),
        SWARMDECK_CAPTURE_PROVIDER="simulation",
        SWARMDECK_SLAM_BACKEND="cslam",
        SWARMDECK_MOLA_PLANNER_MAPS="true",
        # The detector is optional; an empty URL runs the adapters without it.
        SWARMDECK_DETECTOR_URL=(
            "http://duck_detector:8091" if "duck_detector" in spec["services"] else ""
        ),
    )
    if epoch:
        environment.update(
            SWARMDECK_MISSION_ID=epoch["SWARMDECK_MISSION_ID"],
            SWARMDECK_TEST_DOMAIN=epoch["SWARMDECK_TEST_DOMAIN"],
        )
    return environment


def command(spec: dict) -> list[str]:
    result = ["docker", "compose", "-p", spec["project"], "--env-file", str(ENV_FILE)]
    for path in spec["compose_files"]:
        result.extend(("-f", path))
    return result


def state_path(project: str) -> Path:
    return STATE_ROOT / f"{project}.json"


def save_state(spec: dict) -> None:
    STATE_ROOT.mkdir(parents=True, exist_ok=True)
    atomic_json(state_path(spec["project"]), spec)


def load_state(project: str) -> dict:
    path = state_path(project)
    try:
        state = json.loads(path.read_text())
    except (FileNotFoundError, json.JSONDecodeError) as exc:
        raise ValueError(f"no saved simulation stack for project {project!r}") from exc
    required = {"version", "project", "compose_files", "services", "reset_services"}
    if (
        not isinstance(state, dict)
        or state.get("version") != 2
        or not required.issubset(state)
    ):
        raise ValueError(f"saved simulation state is invalid or obsolete: {path}")
    return state


def stop_services(spec: dict, services: list[str]) -> int:
    if not services:
        return 0
    epoch = ensure_deployment_env(ENV_FILE)
    environment = process_environment(spec, epoch)
    compose = command(spec)
    return subprocess.run(
        [*compose, "--profile", "argos", "stop", *services], env=environment
    ).returncode


def remove_services(spec: dict, services: list[str]) -> int:
    if not services:
        return 0
    epoch = ensure_deployment_env(ENV_FILE)
    environment = process_environment(spec, epoch)
    return subprocess.run(
        [*command(spec), "--profile", "argos", "rm", "-f", *services],
        env=environment,
    ).returncode


def retire_services(spec: dict, services: list[str]) -> int:
    stopped = stop_services(spec, services)
    if stopped:
        return stopped
    return remove_services(spec, services)


def retire_discovered_optional_services(project: str, desired: list[str]) -> int:
    """Remove known old-launcher sidecars absent from the selected stack.

    The previous shell launcher did not persist its Compose file/service set.
    On the first run after an upgrade, inspect only containers carrying this
    exact Compose project label, then act only on the bounded optional-service
    allowlist. This leaves unrelated services and every volume untouched.
    """

    listing = subprocess.run(
        [
            "docker",
            "ps",
            "-a",
            "--filter",
            f"label=com.docker.compose.project={project}",
            "--format",
            '{{.ID}}\t{{.Label "com.docker.compose.service"}}',
        ],
        capture_output=True,
        text=True,
    )
    if listing.returncode:
        return listing.returncode
    desired_set = set(desired)
    container_ids = []
    for line in getattr(listing, "stdout", "").splitlines():
        try:
            container_id, service = line.split("\t", 1)
        except ValueError:
            continue
        if service in OPTIONAL_SIMULATION_SERVICES - desired_set and re.fullmatch(
            r"[0-9a-f]{12,64}", container_id
        ):
            container_ids.append(container_id)
    if not container_ids:
        return 0
    stopped = subprocess.run(["docker", "stop", *container_ids]).returncode
    if stopped:
        return stopped
    return subprocess.run(["docker", "rm", "-f", *container_ids]).returncode


def supervisor_owner() -> str | None:
    try:
        metadata = json.loads((RESET_ROOT / "launcher-supervisor.json").read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return None
    if metadata.get("pid") != supervisor_pid():
        return None
    project = metadata.get("project")
    return project if isinstance(project, str) else None


def supervisor_pid() -> int | None:
    try:
        pid = int((RESET_ROOT / "launcher-supervisor.pid").read_text())
        cmdline = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ")
    except (FileNotFoundError, PermissionError, ValueError, ProcessLookupError):
        return None
    expected = str(REPO / "deploy" / "simulation_reset.py").encode()
    return pid if expected in cmdline else None


def process_alive(pid: int) -> bool:
    try:
        state = Path(f"/proc/{pid}/stat").read_text().split()[2]
        os.kill(pid, 0)
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return False
    return state != "Z"


def stop_supervisor(project: str | None = None) -> None:
    owner = supervisor_owner()
    if project is not None and owner is not None and owner != project:
        raise RuntimeError(
            f"simulation reset supervisor belongs to Compose project {owner!r}; "
            f"stop that project before starting {project!r}"
        )
    pid = supervisor_pid()
    if pid is not None:
        try:
            os.kill(pid, signal.SIGTERM)
            for _ in range(50):
                if not process_alive(pid):
                    break
                time.sleep(0.1)
        except ProcessLookupError:
            pass
        if process_alive(pid):
            raise RuntimeError("simulation reset supervisor did not stop")
    (RESET_ROOT / "launcher-supervisor.pid").unlink(missing_ok=True)
    (RESET_ROOT / "launcher-supervisor.json").unlink(missing_ok=True)


def peer_maps_prune_arguments(spec: dict, environment: dict[str, str]) -> list[str]:
    """Name the simulation's peer maps volume and an image able to empty it.

    Both come from the resolved Compose model, so overrides that retag the
    mapping image or rename the volume are followed. Without a mapping service
    there is no such volume and nothing to prune.
    """
    if "mapping" not in spec["reset_services"]:
        return []
    command = ["docker", "compose", "-p", spec["project"]]
    for compose_file in spec["compose_files"]:
        command.extend(("-f", compose_file))
    try:
        model = json.loads(
            subprocess.run(
                [*command, "config", "--format", "json"],
                env=environment,
                capture_output=True,
                text=True,
                check=True,
                timeout=60,
            ).stdout
        )
        volume = model["volumes"]["peer_maps"]["name"]
        image = model["services"]["mapping"]["image"]
    except (OSError, KeyError, ValueError, subprocess.SubprocessError):
        return []
    return ["--prune-maps-volume", volume, "--prune-maps-image", image]


def start_supervisor(spec: dict, environment: dict[str, str]) -> None:
    stop_supervisor(spec["project"])
    arguments = [
        sys.executable,
        str(REPO / "deploy" / "simulation_reset.py"),
        "--root",
        str(RESET_ROOT),
        "--env-file",
        str(ENV_FILE),
        "--project",
        spec["project"],
        "--expected-robots",
        str(spec["robot_count"]),
        "--server-url",
        environment.get("SWARMDECK_RESET_SERVER_URL", "http://127.0.0.1:8080"),
    ]
    for compose_file in spec["compose_files"]:
        arguments.extend(("--compose-file", compose_file))
    for service in spec["reset_services"]:
        arguments.extend(("--service", service))
    for robot in spec["robot_names"]:
        arguments.extend(("--robot-id", robot))
    arguments.extend(peer_maps_prune_arguments(spec, environment))
    prepare_reset_directories(RESET_ROOT, spec["robot_names"])
    log = (RESET_ROOT / "supervisor.log").open("ab")
    try:
        process = subprocess.Popen(
            arguments,
            env=environment,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    finally:
        log.close()
    for _ in range(40):
        if process.poll() is not None:
            raise RuntimeError("simulation reset supervisor exited during startup")
        try:
            heartbeat = json.loads((RESET_ROOT / "supervisor.json").read_text())
        except (FileNotFoundError, json.JSONDecodeError):
            heartbeat = {}
        if heartbeat.get("pid") == process.pid:
            break
        time.sleep(0.05)
    else:
        process.terminate()
        raise RuntimeError("simulation reset supervisor did not become ready")
    (RESET_ROOT / "launcher-supervisor.pid").write_text(f"{process.pid}\n")
    atomic_json(
        RESET_ROOT / "launcher-supervisor.json",
        {"version": 1, "pid": process.pid, "project": spec["project"]},
    )


def build_spec(args: argparse.Namespace, project: str) -> dict:
    if args.dev:
        args.scenario, args.render, args.odometry = "3robot", "dri", "drift"
    if os.getenv("SWARMDECK_SLAM_BACKEND", "cslam") != "cslam":
        raise ValueError("SWARMDECK_SLAM_BACKEND must be cslam")
    if args.targets < 0 or args.explore < 0:
        raise ValueError("targets and exploration seconds must be nonnegative")
    source = scenario_path(args.scenario)
    count, prefix = fleet_from_config(source)
    names = [f"{prefix}{index}" for index in range(count)]
    if count > MAX_SIMULATION_PEERS:
        raise ValueError(
            f"CSLAM simulation supports at most {MAX_SIMULATION_PEERS} peers; "
            f"{source} configures {count}"
        )
    try:
        relative = source.relative_to(REPO / "configs")
        container_config = f"/app/configs/{relative.as_posix()}"
        custom_overlay = None
    except ValueError:
        container_config = "/app/configs/swarmdeck-launch.yaml"
        custom_overlay = custom_config_overlay(project, source)
    files = compose_files(args.render)
    if custom_overlay:
        files.append(custom_overlay)
    peers = [f"peer{index}" for index in range(count)]
    services = [
        "server",
        "ui",
        *(["duck_detector"] if args.detector else []),
        "mediamtx",
        "sim",
        "argos",
        "mgg",
        *peers,
        "mapping",
    ]
    if args.odometry == "fast_livo2":
        services.append("fast_livo2")
    reset_services = [
        service
        for service in services
        if service not in {"ui", "duck_detector", "mediamtx"}
    ]
    return {
        "version": 2,
        "project": project,
        "scenario": str(source),
        "container_config": container_config,
        "robot_count": count,
        "robot_names": names,
        "robot_platforms": platforms_from_config(source, names),
        "render": args.render,
        "odometry": args.odometry,
        "robot_poses": args.robot_poses,
        "targets": args.targets,
        "explore": args.explore,
        "compose_files": [str(path) for path in files],
        "services": services,
        "reset_services": reset_services,
    }


def print_dry_run(spec: dict, build: bool, detach: bool) -> None:
    flags = (["--build"] if build else []) + ["--force-recreate"]
    flags += ["-d"] if detach else []
    print("Simulation Bring-Up Configuration (Dry Run):")
    print(f"  Project:   {spec['project']}")
    print(f"  Scenario:  {spec['scenario']} ({spec['robot_count']} robots)")
    print(f"  Render:    {spec['render']}")
    print(f"  Odometry:  {spec['odometry']}")
    print(f"  Robot poses: {spec['robot_poses']}")
    detector = "on" if "duck_detector" in spec["services"] else "off"
    print(f"  Detector:  {detector}")
    invocation = command(spec) + [
        "--profile",
        "argos",
        "up",
        *flags,
        *spec["services"],
    ]
    print(f"  Compose:   {' '.join(invocation)}")


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    project = os.getenv("COMPOSE_PROJECT", "swarmdeck")
    if PROJECT_PATTERN.fullmatch(project) is None:
        raise ValueError("COMPOSE_PROJECT contains invalid characters")
    previous = load_state(project) if state_path(project).is_file() else None
    if args.action in {"down", "status", "logs"} and previous is not None:
        spec = previous
    else:
        spec = build_spec(args, project)
    if args.action == "dry-run":
        print_dry_run(spec, args.build, args.detach)
        return 0

    epoch = ensure_deployment_env(ENV_FILE)
    environment = process_environment(spec, epoch)
    compose = command(spec)
    if args.action == "status":
        return subprocess.run(
            [*compose, "--profile", "argos", "ps"], env=environment
        ).returncode
    if args.action == "logs":
        return subprocess.run(
            [*compose, "--profile", "argos", "logs", "-f", *spec["services"]],
            env=environment,
        ).returncode
    if args.action == "down":
        stop_supervisor(project)
        print(f"Stopping SwarmDeck simulation stack ({project})...")
        stopped = subprocess.run(
            [*compose, "--profile", "argos", "stop", *spec["services"]],
            env=environment,
        )
        removed = subprocess.run(
            [*compose, "--profile", "argos", "rm", "-f", *spec["services"]],
            env=environment,
        )
        return stopped.returncode or removed.returncode

    stop_supervisor(project)
    active = previous or spec
    stopped = stop_services(active, active["reset_services"])
    if stopped:
        return stopped
    if previous is None:
        retired = retire_discovered_optional_services(project, spec["services"])
        if retired:
            return retired
    else:
        superseded = [
            service
            for service in previous["services"]
            if service not in spec["services"]
        ]
        removed = remove_services(previous, superseded)
        if removed:
            return removed
    epoch = new_epoch()
    environment = process_environment(spec, epoch)
    flags = (["--build"] if args.build else []) + ["--force-recreate"]
    flags += ["-d"] if args.detach else []
    print(
        f"Starting {spec['robot_count']}-robot ARGoS simulation with "
        f"{spec['odometry']} odometry..."
    )
    start_supervisor(spec, environment)
    try:
        result = subprocess.run(
            [*compose, "--profile", "argos", "up", *flags, *spec["services"]],
            env=environment,
        )
    except BaseException:
        stop_supervisor(project)
        raise
    if result.returncode:
        stop_supervisor(project)
        retire_services(spec, spec["services"])
        if previous is None:
            save_state(spec)
        return result.returncode
    save_state(spec)
    if args.detach:
        print("SwarmDeck is running at http://localhost:5173")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"sim-up: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
