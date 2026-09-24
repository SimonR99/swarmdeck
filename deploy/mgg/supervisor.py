#!/usr/bin/env python3
"""Supervise one independent ROS launch process group per simulated robot."""

from __future__ import annotations

import fcntl
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time
from uuid import uuid4

try:
    from .reset_protocol import open_lock, prepare_reset_directories
except ImportError:  # Direct execution inside the MGG container.
    from reset_protocol import open_lock, prepare_reset_directories

STATUS_HEARTBEAT_NS = 1_000_000_000


def read_json(path: Path) -> dict:
    try:
        value = json.loads(path.read_text())
    except FileNotFoundError:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f"expected an object in {path}")
    return value


def atomic_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid4().hex}.tmp")
    with temporary.open("w") as stream:
        json.dump(value, stream, sort_keys=True)
        stream.flush()
        os.fsync(stream.fileno())
        # Readers include the unprivileged host even when MGG uses umask 077.
        os.fchmod(stream.fileno(), 0o644)
    os.replace(temporary, path)
    descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def robot_names() -> list[str]:
    configured = os.environ.get("SWARMDECK_PEER_NAMES")
    if configured:
        names = json.loads(configured)
    else:
        import yaml

        with open(
            os.environ.get("SWARMDECK_CONFIG", "/app/configs/4robot.yaml")
        ) as stream:
            fleet = yaml.safe_load(stream)["fleet"]
        count = int(
            os.environ.get("SWARMDECK_ROBOT_COUNT") or fleet.get("robot_count", 4)
        )
        names = [f"{fleet.get('robot_prefix', 'robot_')}{i}" for i in range(count)]
    if (
        not isinstance(names, list)
        or not names
        or any(
            not isinstance(name, str)
            or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name)
            for name in names
        )
        or len(set(names)) != len(names)
    ):
        raise ValueError("invalid simulation robot names")
    return names


class PlannerSupervisor:
    def __init__(
        self, root: Path, mission_id: str, names: list[str], maps: Path, reset_source
    ):
        self.root, self.mission_id, self.names, self.maps = (
            root,
            mission_id,
            names,
            maps,
        )
        prepare_reset_directories(root, names)
        self.processes: dict[str, subprocess.Popen] = {}
        self.runs: dict[str, str | None] = {}
        self.statuses: dict[str, tuple[dict, int]] = {}
        self.reset_source = reset_source

    def stop(self, robot: str) -> None:
        process = self.processes.pop(robot, None)
        if process is None:
            return
        # ROS launch owns every planner, PCI and input relay in this group.
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            # ROS launch escalates unresponsive children to SIGTERM after
            # five seconds. Allow that cleanup before forcing the whole group;
            # MGG shares sim's /dev/shm, which outlives this planner.
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            pass
        # A crashed launch leader can already be reaped while its children
        # remain alive. Always retire the owned group, not only the leader.
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=2)

    def prepare_source(self, robot: str, epoch: dict) -> dict:
        if not epoch or epoch["map_epoch"] == 0:
            return {}
        path = self.root / "robots" / robot / "source-reset.json"
        previous = read_json(path)
        run_id = epoch["run_id"]
        if previous.get("run_id") == run_id:
            if previous.get("phase") == "done":
                return previous
            # A ROS service has no idempotency token. If its ACK was lost when
            # this supervisor crashed, never clear newly accumulated data a
            # second time under the same run ID.
            raise RuntimeError(
                previous.get("error")
                or "local source reset interrupted; a fresh map reset is required"
            )
        record = {
            "mission_id": self.mission_id,
            "robot_id": robot,
            "run_id": run_id,
            "phase": "starting",
        }
        atomic_json(path, record)
        desired = read_json(path.parent / "mgg-request.json")
        status = read_json(path.parent / "status.json")
        # A manual reset already stopped this planner and cancelled its Nav2
        # actions. Reuse only that exact, still-live request's durable proof;
        # an automatic frontend restart must quiesce its own new lifetime.
        quiesced = (
            desired.get("state") == "running"
            and desired.get("mission_id") == self.mission_id
            and desired.get("map_epoch") == epoch["map_epoch"]
            and isinstance(desired.get("request_id"), str)
            and status.get("request_id") == desired["request_id"]
            and status.get("mission_id") == self.mission_id
            and status.get("robot_id") == robot
            and status.get("map_epoch") == epoch["map_epoch"]
            and status.get("run_id") == run_id
            and status.get("phase") == "starting"
            and status.get("quiesced") is True
            and type(status.get("deadline_at_ns")) is int
            and time.time_ns() < status["deadline_at_ns"]
        )
        try:
            reset = self.reset_source(robot, time.monotonic() + 15, quiesced=quiesced)
            stamp = reset["source_reset_stamp"]
            if (
                not isinstance(stamp, dict)
                or type(stamp.get("sec")) is not int
                or type(stamp.get("nanosec")) is not int
                or stamp["sec"] < 0
                or not 0 <= stamp["nanosec"] < 1_000_000_000
                or stamp["sec"] + stamp["nanosec"] == 0
            ):
                raise ValueError("local source reset returned no ROS timestamp fence")
            record.update(
                phase="done", source_reset_run_id=run_id, source_reset_stamp=stamp
            )
            atomic_json(path, record)
            return record
        except (OSError, ValueError, KeyError, RuntimeError) as exc:
            error = str(exc)
            atomic_json(path, {**record, "phase": "failed", "error": error[-4000:]})
            raise RuntimeError(error) from exc

    def step(self) -> None:
        for robot in self.names:
            directory = self.root / "robots" / robot
            desired = read_json(directory / "mgg-request.json")
            if desired.get("mission_id") != self.mission_id:
                desired = {}
            state = desired.get("state", "running")
            if state not in {"running", "stopped"}:
                raise ValueError(f"invalid MGG desired state for {robot}")
            minimum_epoch = desired["map_epoch"] if desired else 0
            if type(minimum_epoch) is not int or minimum_epoch < 0:
                raise ValueError(f"invalid requested map epoch for {robot}")
            epoch = read_json(self.maps / self.mission_id / robot / "map-epoch.json")
            run_id = epoch.get("run_id")
            waiting_for_epoch = (
                state == "running" and epoch.get("map_epoch", -1) < minimum_epoch
            )
            process = self.processes.get(robot)
            if process is not None and (
                state == "stopped"
                or waiting_for_epoch
                or process.poll() is not None
                or self.runs.get(robot) != run_id
            ):
                self.stop(robot)
                process = None
            source_reset = {}
            error = None
            if waiting_for_epoch:
                state = "starting"
            elif state == "running":
                try:
                    source_reset = self.prepare_source(robot, epoch)
                    if process is None:
                        environment = dict(os.environ, SWARMDECK_MGG_ROBOT=robot)
                        process = subprocess.Popen(
                            [
                                "ros2",
                                "launch",
                                str(Path(__file__).with_name("fleet.launch.py")),
                            ],
                            env=environment,
                            start_new_session=True,
                        )
                        self.processes[robot] = process
                        self.runs[robot] = run_id
                except (OSError, ValueError, RuntimeError) as exc:
                    state, error = "failed", str(exc)
            status = {
                "version": 1,
                "robot_id": robot,
                "mission_id": self.mission_id,
                "request_id": desired.get("request_id"),
                "state": state,
                "run_id": run_id,
                "pid": process.pid if process else None,
                "map_epoch": epoch.get("map_epoch"),
                "source_reset_run_id": source_reset.get("source_reset_run_id"),
                "source_reset_stamp": source_reset.get("source_reset_stamp"),
                "error": error,
            }
            # deploy/simulation_reset.py treats a status older than 3 s as
            # stale, so an unchanged status is rewritten (two fsyncs) once a
            # second rather than on every 0.2 s step.
            previous = self.statuses.get(robot)
            now_ns = time.time_ns()
            if (
                previous is not None
                and previous[0] == status
                and now_ns - previous[1] < STATUS_HEARTBEAT_NS
            ):
                continue
            atomic_json(
                directory / "mgg-status.json", {**status, "updated_at_ns": now_ns}
            )
            self.statuses[robot] = (status, now_ns)

    def close(self) -> None:
        for robot in tuple(self.processes):
            self.stop(robot)


def main() -> None:
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions

    try:
        from .robot_reset import SourceResetter
    except ImportError:  # Direct execution inside the MGG container.
        from robot_reset import SourceResetter

    root = Path(os.environ.get("SWARMDECK_SIM_RESET_DIR", "/run/swarmdeck-reset"))
    names = robot_names()
    prepare_reset_directories(root, names)
    with os.fdopen(open_lock(root / "mgg-supervisor.lock")) as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        stopping = False

        def stop(_signal, _frame):
            nonlocal stopping
            stopping = True

        signal.signal(signal.SIGTERM, stop)
        signal.signal(signal.SIGINT, stop)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = supervisor = None
        try:
            node = rclpy.create_node(
                "swarmdeck_planner_supervisor",
                parameter_overrides=[Parameter("use_sim_time", value=True)],
            )
            resetters = {robot: SourceResetter(node, robot) for robot in names}

            def reset_source(robot, deadline, *, quiesced):
                return resetters[robot].reset(deadline, quiesced=quiesced)

            supervisor = PlannerSupervisor(
                root,
                os.environ.get("SWARMDECK_MISSION_ID", ""),
                names,
                Path(os.environ.get("SWARMDECK_MAPS_ROOT", "/maps")),
                reset_source,
            )
            while not stopping and rclpy.ok():
                supervisor.step()
                rclpy.spin_once(node, timeout_sec=0.2)
        finally:
            if supervisor is not None:
                supervisor.close()
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
