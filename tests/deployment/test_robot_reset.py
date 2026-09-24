"""Robot reset orchestration at the persistent supervisor boundary (no ROS)."""

import io
import json
import os
import subprocess
import time
import sys
from concurrent.futures import Future
from types import ModuleType, SimpleNamespace
from uuid import uuid4

import pytest

from autonomy.map_epochs import claim_map_epoch, robot_run_id
from deploy.mgg.supervisor import PlannerSupervisor, atomic_json
from deploy.simulation_reset import Supervisor, write_deployment_env
from deploy.mgg.reset_protocol import prepare_reset_directories
from deploy.simulation_reset import atomic_json as host_atomic_json, protocol_lock

MISSION = "6f6afc5c-9a34-4eb4-8243-731629872d25"


def acknowledge_source(*args, **kwargs):
    return {"source_reset_stamp": {"sec": 1, "nanosec": 0}}


@pytest.mark.parametrize("first", ["host", "mgg"])
def test_protocol_remains_cross_uid_accessible_under_restrictive_umask(tmp_path, first):
    root = tmp_path / "reset"
    previous_umask = os.umask(0o077)
    try:
        if first == "host":
            Supervisor(
                root,
                tmp_path / "deployment.env",
                [],
                ["mgg", "mapping", "peer0"],
                robot_ids=["robot_0"],
            )
        else:
            PlannerSupervisor(
                root, MISSION, ["robot_0"], tmp_path / "maps", acknowledge_source
            )
        directory = root / "robots" / "robot_0"
        lock_inode = (directory / "protocol.lock").stat().st_ino
        # A replacement supervisor must use the same lock inode, not split
        # mutual exclusion by replacing a lock owned by the other process.
        prepare_reset_directories(root, ["robot_0"])
        with protocol_lock(directory):
            assert (directory / "protocol.lock").stat().st_ino == lock_inode
            for writer in (atomic_json, host_atomic_json, atomic_json):
                writer(directory / "status.json", {"phase": "done"})
                writer(directory / "requests" / "result.json", {"phase": "done"})
                for path in (
                    directory / "status.json",
                    directory / "requests" / "result.json",
                ):
                    assert path.stat().st_mode & 0o777 == 0o644
        for path in (root, root / "robots", directory, directory / "requests"):
            # No sticky bit: the host must replace root-owned atomic files.
            assert path.stat().st_mode & 0o7777 == 0o777
        assert (directory / "protocol.lock").stat().st_mode & 0o777 == 0o644
    finally:
        os.umask(previous_umask)


def test_legacy_reset_directory_repair_does_not_broaden_session_data(tmp_path):
    root = tmp_path / "reset"
    directory = root / "robots" / "robot_0"
    (directory / "requests").mkdir(parents=True)
    (directory / "protocol.lock").touch(mode=0o600)
    unrelated = tmp_path / "private-session"
    unrelated.mkdir(mode=0o700)
    secret = root / "deployment.env"
    secret.write_text("PRIVATE=value")
    secret.chmod(0o600)
    prepare_reset_directories(root, ["robot_0"])
    assert directory.stat().st_mode & 0o777 == 0o777
    assert (directory / "requests").stat().st_mode & 0o777 == 0o777
    assert (directory / "protocol.lock").stat().st_mode & 0o777 == 0o644
    assert unrelated.stat().st_mode & 0o777 == 0o700
    assert secret.stat().st_mode & 0o777 == 0o600


class SupervisorCrash(BaseException):
    pass


@pytest.fixture
def deployment(tmp_path, monkeypatch):
    root, maps, env = tmp_path / "reset", tmp_path / "maps", tmp_path / "deployment.env"
    write_deployment_env(env, MISSION, 173)
    names = ["robot_0", "robot_1", "robot_2"]
    epochs = {name: claim_map_epoch(maps, MISSION, name) for name in names}
    running = set(names)
    planners = set(names)
    operations = []
    cleanups = []
    crash = {}

    class Harness(Supervisor):
        def robot_command(self, arguments, environment, deadline):
            self.remaining(deadline)
            if arguments[:2] == ["exec", "-T"]:
                cleanups.append((arguments, deadline, list(operations)))
                if "cleanup_error" in crash:
                    raise crash["cleanup_error"]
                return SimpleNamespace(stdout="0 zombie segments cleaned")
            robot = names[int(arguments[-1].removeprefix("peer"))]
            assert environment["SWARMDECK_MISSION_ID"] == MISSION
            assert environment["SWARMDECK_TEST_DOMAIN"] == "173"
            operations.append((arguments[0], robot))
            if arguments[0] == "stop":
                running.discard(robot)
            elif arguments[0] == "up" and robot not in running:
                request = json.loads(
                    (root / "robots" / robot / "request.json").read_text()
                )
                epochs[robot] = claim_map_epoch(
                    maps, MISSION, robot, request["map_epoch"]
                )
                running.add(robot)
            return SimpleNamespace(stdout="")

        def planner_state(self, request, state, deadline):
            self.remaining(deadline)
            robot = request["robot_id"]
            if state == "stopped":
                planners.discard(robot)
            else:
                planners.add(robot)
            if crash.pop(state, False):
                raise SupervisorCrash()

        def robot_ros(self, request, operation, environment, deadline):
            self.remaining(deadline)
            robot = request["robot_id"]
            if operation == "verify":
                assert robot in planners, "cannot verify an absent planner"
            operations.append((operation, robot))
            return {
                "map_epoch": epochs[robot],
                "run_id": robot_run_id(MISSION, robot, epochs[robot]),
            }

    def fleet_response(*args, **kwargs):
        return io.BytesIO(
            json.dumps(
                {
                    "robots": [
                        {
                            "robot_id": name,
                            "online": name in running,
                            "navigation_ready": name in planners,
                            "live_mapping": {
                                "mission_id": MISSION,
                                "robot_map_epoch": epochs[name],
                                "run_id": robot_run_id(MISSION, name, epochs[name]),
                                "mapping_graph_revision": 1,
                            },
                        }
                        for name in names
                    ]
                }
            ).encode()
        )

    monkeypatch.setattr(
        "deploy.simulation_reset.urllib.request.urlopen", fleet_response
    )

    def supervisor():
        return Harness(
            root,
            env,
            ["docker", "compose"],
            ["sim", "mgg", "mapping", "peer0", "peer1", "peer2"],
            "http://server",
            robot_ids=names,
        )

    def submit(robot="robot_1", request_id=None):
        request = {
            "version": 1,
            "request_id": request_id or str(uuid4()),
            "robot_id": robot,
            "mission_id": MISSION,
            "map_epoch": epochs.get(robot, 0) + 1,
            "requested_at_ns": time.time_ns(),
        }
        atomic_json(root / "robots" / robot / "request.json", request)
        return request

    def status(robot="robot_1"):
        return json.loads((root / "robots" / robot / "status.json").read_text())

    return SimpleNamespace(
        root=root,
        env=env,
        epochs=epochs,
        running=running,
        planners=planners,
        operations=operations,
        cleanups=cleanups,
        crash=crash,
        supervisor=supervisor,
        submit=submit,
        status=status,
    )


@pytest.mark.parametrize(
    "failure",
    [
        None,
        OSError("docker unavailable"),
        subprocess.CalledProcessError(1, "clean"),
        subprocess.TimeoutExpired("clean", 3),
    ],
)
def test_robot_reset_cleanup_is_bounded_and_never_fails_reset(
    deployment, failure, capsys
):
    d = deployment
    d.submit()
    if failure is not None:
        d.crash["cleanup_error"] = failure
    before = time.monotonic()
    assert d.supervisor().poll_robots()
    assert d.status()["ok"] is True
    assert len(d.cleanups) == 2
    first, second = d.cleanups
    for arguments, deadline, _ in d.cleanups:
        assert arguments[:4] == ["exec", "-T", "sim", "timeout"]
        assert arguments[4:7] == ["--kill-after=1", "2", "bash"]
        assert "exec fastdds shm clean" in arguments[-1]
        assert 0 < deadline - before <= 3.5
    assert first[2] == []  # MGG stopped, before ROS quiesce.
    assert second[2][-1] == ("stop", "robot_1")
    output = capsys.readouterr().err
    assert output.count("DDS SHM cleanup") == 1  # Rate-limited per supervisor.
    assert ("failed" if failure else "completed") in output


def test_robot_reset_probe_forces_udp_only(deployment, monkeypatch):
    supervisor = deployment.supervisor()
    calls = []

    def command(arguments, environment, deadline):
        calls.append(arguments)
        return SimpleNamespace(stdout='{"ok": true}')

    monkeypatch.setattr(supervisor, "robot_command", command)
    request = deployment.submit()
    Supervisor.robot_ros(supervisor, request, "verify", {}, time.monotonic() + 10)
    assert calls[0][:5] == [
        "exec",
        "-T",
        "-e",
        "FASTRTPS_DEFAULT_PROFILES_FILE=/app/deploy/dds/fastdds_udp_only.xml",
        "mgg",
    ]


def test_only_target_restarts_and_completed_request_survives_later_resets(deployment):
    d = deployment
    first = d.submit()
    original_env = d.env.read_bytes()
    assert d.supervisor().poll_robots()
    assert d.epochs == {"robot_0": 0, "robot_1": 1, "robot_2": 0}
    assert all(robot == "robot_1" for _, robot in d.operations)
    assert d.running == d.planners == {"robot_0", "robot_1", "robot_2"}
    assert d.status()["ok"] is True
    assert d.env.read_bytes() == original_env
    d.submit()
    assert d.supervisor().poll_robots()
    assert d.epochs["robot_1"] == 2
    before = list(d.operations)
    atomic_json(d.root / "robots" / "robot_1" / "request.json", first)
    assert not d.supervisor().poll_robots()
    assert d.status()["request_id"] == first["request_id"]
    assert d.status()["map_epoch"] == 1
    assert d.operations == before
    assert d.epochs["robot_1"] == 2


@pytest.mark.parametrize("crash_phase", ["stopped", "running"])
def test_supervisor_crash_resumes_without_restarting_a_started_peer(
    deployment, crash_phase
):
    d = deployment
    d.submit()
    d.crash[crash_phase] = True
    with pytest.raises(SupervisorCrash):
        d.supervisor().poll_robots()
    assert d.supervisor().poll_robots()
    assert d.status()["phase"] == "done"
    assert d.epochs == {"robot_0": 0, "robot_1": 1, "robot_2": 0}
    assert d.operations.count(("quiesce", "robot_1")) == 1
    if crash_phase == "running":
        assert d.operations.count(("stop", "robot_1")) == 1


def test_recovery_after_deadline_fails_without_another_reset(deployment):
    d = deployment
    request = d.submit()
    atomic_json(
        d.root / "robots" / "robot_1" / "requests" / f"{request['request_id']}.json",
        {
            **request,
            "phase": "starting",
            "deadline_at_ns": time.time_ns() - 1,
        },
    )
    assert d.supervisor().poll_robots()
    assert d.status()["phase"] == "failed"
    assert "60 second" in d.status()["error"]
    assert not d.operations
    assert d.epochs["robot_1"] == 0


def test_unconfigured_robot_boundary_is_unavailable(deployment):
    d = deployment
    d.submit("unknown_robot")
    assert d.supervisor().poll_robots()
    assert d.status("unknown_robot")["ok"] is False
    assert "unavailable" in d.status("unknown_robot")["error"]
    assert not d.operations


def test_planner_control_and_epoch_change_preserve_other_process_groups(
    tmp_path, monkeypatch
):
    names = ["robot_0", "robot_1", "robot_2"]
    launched, killed = [], []

    def launch(arguments, **kwargs):
        process = SimpleNamespace(
            pid=100 + len(launched), poll=lambda: None, wait=lambda **kw: 0
        )
        launched.append((kwargs["env"]["SWARMDECK_MGG_ROBOT"], process))
        return process

    monkeypatch.setattr("deploy.mgg.supervisor.subprocess.Popen", launch)
    monkeypatch.setattr(
        "deploy.mgg.supervisor.os.killpg", lambda pid, sig: killed.append(pid)
    )
    root, maps = tmp_path / "reset", tmp_path / "maps"
    for robot in names:
        claim_map_epoch(maps, MISSION, robot)
    supervisor = PlannerSupervisor(root, MISSION, names, maps, acknowledge_source)
    supervisor.step()
    initial = dict(supervisor.processes)
    command = {
        "request_id": str(uuid4()),
        "mission_id": MISSION,
        "state": "stopped",
        "map_epoch": 1,
    }
    atomic_json(root / "robots" / "robot_1" / "mgg-request.json", command)
    supervisor.step()
    assert set(killed) == {initial["robot_1"].pid}
    assert supervisor.processes == {
        name: initial[name] for name in ("robot_0", "robot_2")
    }
    supervisor.step()
    assert len(launched) == 3
    # Durable stopped state is respected by a replacement MGG supervisor too.
    recovered = PlannerSupervisor(root, MISSION, names, maps, acknowledge_source)
    recovered.step()
    assert set(recovered.processes) == {"robot_0", "robot_2"}
    command["state"] = "running"
    atomic_json(root / "robots" / "robot_1" / "mgg-request.json", command)
    supervisor.step()
    # Compose starts the container before its frontend claims the new run.
    # Never launch or acknowledge a planner for the still-present old epoch.
    assert "robot_1" not in supervisor.processes
    assert supervisor.processes == {
        name: initial[name] for name in ("robot_0", "robot_2")
    }
    claim_map_epoch(maps, MISSION, "robot_1")
    supervisor.step()
    restarted = supervisor.processes["robot_1"]
    claim_map_epoch(maps, MISSION, "robot_1")
    supervisor.step()
    assert supervisor.processes["robot_1"] is not restarted
    assert supervisor.processes["robot_0"] is initial["robot_0"]
    assert supervisor.processes["robot_2"] is initial["robot_2"]


def test_local_source_ack_is_durable_and_interrupted_ack_never_replays(
    tmp_path, monkeypatch
):
    maps, root = tmp_path / "maps", tmp_path / "reset"
    claim_map_epoch(maps, MISSION, "robot_0")
    claim_map_epoch(maps, MISSION, "robot_0")
    epoch = json.loads((maps / MISSION / "robot_0" / "map-epoch.json").read_text())
    calls = []

    def reset(*args, **kwargs):
        calls.append(args[0])
        return {"source_reset_stamp": {"sec": 123, "nanosec": 456}}

    supervisor = PlannerSupervisor(root, MISSION, ["robot_0"], maps, reset)
    first = supervisor.prepare_source("robot_0", epoch)
    recovered = PlannerSupervisor(root, MISSION, ["robot_0"], maps, reset)
    assert recovered.prepare_source("robot_0", epoch) == first
    assert first["source_reset_run_id"] == epoch["run_id"]
    assert first["source_reset_stamp"] == {"sec": 123, "nanosec": 456}
    assert len(calls) == 1
    # Service side effects are not replayable if its acknowledgement was lost.
    atomic_json(
        root / "robots" / "robot_0" / "source-reset.json",
        {
            "run_id": epoch["run_id"],
            "phase": "starting",
        },
    )
    with pytest.raises(RuntimeError, match="interrupted"):
        recovered.prepare_source("robot_0", epoch)
    assert len(calls) == 1


def test_local_costmap_reset_uses_actual_service_ack_and_ros_clock(monkeypatch):
    from deploy.mgg import robot_reset

    clear = SimpleNamespace(Request=SimpleNamespace)
    for name, attributes in {
        "rclpy": {"spin_once": lambda *args, **kwargs: None},
        "nav2_msgs.srv": {"ClearEntireCostmap": clear},
    }.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    quiescences = []
    monkeypatch.setattr(
        robot_reset, "quiesce", lambda node, robot, deadline: quiescences.append(robot)
    )
    calls = []

    def call(node, client, request, deadline, *, idempotent=False):
        calls.append((client.srv_name, request))
        return SimpleNamespace()

    monkeypatch.setattr(robot_reset, "call", call)
    node = SimpleNamespace(
        create_client=lambda kind, name: SimpleNamespace(srv_name=name),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=4_000_000_005)
        ),
    )
    source = robot_reset.SourceResetter(node, "robot_1")
    result = source.reset(time.monotonic() + 1)
    assert result == {"source_reset_stamp": {"sec": 4, "nanosec": 5}}
    assert quiescences == ["robot_1"]
    assert [name for name, request in calls] == [
        "/robot_1/local_costmap/clear_entirely_local_costmap",
    ]
    bad_node = SimpleNamespace(
        create_client=lambda kind, name: SimpleNamespace(srv_name=name),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0)),
    )
    with pytest.raises(RuntimeError, match="simulation clock unavailable"):
        robot_reset.SourceResetter(bad_node, "robot_1").reset(
            time.monotonic() + 1, quiesced=True
        )


@pytest.mark.parametrize("idempotent", [False, True])
def test_lost_ros_reply_replays_only_idempotent_operations(monkeypatch, idempotent):
    from deploy.mgg import robot_reset

    clock, effects = [0.0], []
    response = SimpleNamespace(ok=True)

    def send(request):
        effects.append(request)
        future = Future()
        if len(effects) > 1:
            future.set_result(response)
        return future

    client = SimpleNamespace(
        srv_name="/clear" if idempotent else "/reset",
        wait_for_service=lambda **kwargs: True,
        call_async=send,
        remove_pending_request=lambda future: None,
    )
    node = SimpleNamespace()
    rclpy = ModuleType("rclpy")
    rclpy.spin_until_future_complete = (
        lambda node, future, timeout_sec: clock.__setitem__(
            0, clock[0] + (timeout_sec if not future.done() else 0)
        )
    )
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setattr(robot_reset.time, "monotonic", lambda: clock[0])
    if idempotent:
        assert robot_reset.call(node, client, {}, 5, idempotent=True).ok
        assert clock[0] < 5
    else:
        with pytest.raises(TimeoutError):
            robot_reset.call(node, client, {}, 5)
        # A lost destructive reset acknowledgement must never reset twice.
        assert len(effects) == 1


def test_persistent_local_costmap_client_keeps_replies_across_resets(monkeypatch):
    from deploy.mgg import robot_reset

    effects = []
    clear = SimpleNamespace(Request=SimpleNamespace)
    rclpy = ModuleType("rclpy")
    rclpy.spin_once = lambda *args, **kwargs: None
    rclpy.spin_until_future_complete = lambda node, future, timeout_sec: None
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)

    def create_client(kind, name):
        def send(request):
            effects.append(name)
            future = Future()
            future.set_result(SimpleNamespace())
            return future

        return SimpleNamespace(
            srv_name=name,
            wait_for_service=lambda **kwargs: True,
            call_async=send,
            remove_pending_request=lambda future: None,
        )

    node = SimpleNamespace(
        create_client=create_client,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=9_000_000_001)
        ),
    )
    nav2 = ModuleType("nav2_msgs.srv")
    nav2.ClearEntireCostmap = clear
    monkeypatch.setitem(sys.modules, "nav2_msgs.srv", nav2)
    source = robot_reset.SourceResetter(node, "robot_1")
    for _ in range(2):
        assert source.reset(time.monotonic() + 1, quiesced=True) == {
            "source_reset_stamp": {"sec": 9, "nanosec": 1},
        }
    assert (
        effects
        == [
            "/robot_1/local_costmap/clear_entirely_local_costmap",
        ]
        * 2
    )


def test_quiesce_waits_for_trajectory_controller_before_cancelling_target(
    monkeypatch,
):
    from deploy.mgg import robot_reset

    clock, effects = [0.0], []
    cancel = SimpleNamespace(
        Request=SimpleNamespace, Response=SimpleNamespace(ERROR_NONE=0)
    )
    rclpy = ModuleType("rclpy")
    rclpy.spin_once = lambda node, timeout_sec: clock.__setitem__(
        0, clock[0] + timeout_sec
    )
    rclpy.spin_until_future_complete = lambda *args, **kwargs: None
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    for name, attributes in {
        "action_msgs.srv": {"CancelGoal": cancel},
        "geometry_msgs.msg": {"Twist": SimpleNamespace},
    }.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(robot_reset.time, "monotonic", lambda: clock[0])
    controller = "/robot_1/follow_path/_action/cancel_goal"

    def services():
        return [(controller, ["action_msgs/srv/CancelGoal"])]

    def create_client(kind, name):
        def send(request):
            effects.append(name)
            future = Future()
            future.set_result(SimpleNamespace(return_code=0))
            return future

        return SimpleNamespace(
            srv_name=name,
            wait_for_service=lambda **kwargs: True,
            call_async=send,
        )

    node = SimpleNamespace(
        get_service_names_and_types=services,
        create_client=create_client,
        destroy_client=lambda client: None,
        create_publisher=lambda kind, name, depth: SimpleNamespace(
            get_subscription_count=lambda: 1,
            publish=lambda message: effects.append(name),
        ),
        destroy_publisher=lambda publisher: None,
    )
    robot_reset.quiesce(node, "robot_1", 1)
    assert effects == [controller, "/robot_1/cmd_vel"]
