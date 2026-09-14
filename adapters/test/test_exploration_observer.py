import argparse
import asyncio
import sys
from types import SimpleNamespace

from adapters.test.ros import onboard_exploration_observer as observer
from adapters.test.ros.onboard_exploration_observer import (
    summarize,
    summarize_post_stop,
    validate_base_url,
)


def test_observer_refuses_production_or_nonloopback_api():
    assert validate_base_url("http://127.0.0.1:18080/") == "http://127.0.0.1:18080"
    for value in (
        "http://127.0.0.1:8080",
        "http://192.168.1.161:18080",
        "https://127.0.0.1:18080",
        "http://127.0.0.1:18080/api",
    ):
        try:
            validate_base_url(value)
        except ValueError:
            pass
        else:
            raise AssertionError(f"unsafe observation URL accepted: {value}")


def test_summary_reports_motion_status_and_revision_without_inferring_verification():
    def sample(x, status, revision):
        return {
            "fleet": {
                "ok": True,
                "data": {
                    "robots": [
                        {
                            "robot_id": "robot_0",
                            "pose": {"x": x, "y": 0.0},
                            "online": True,
                            "mode": (
                                "explore"
                                if status in {"exploring", "waiting"}
                                else "idle"
                            ),
                            "exploration_status": status,
                            "fleet_exploration_status": "incomplete",
                            "nav_status": (
                                "active"
                                if status in {"exploring", "waiting"}
                                else "idle"
                            ),
                            "global_planned_path": [{"x": 1, "y": 0}] * revision,
                            "local_planned_path": [],
                        }
                    ]
                },
            },
            "replica_index": {
                "ok": True,
                "data": {
                    "replicas": [
                        {
                            "robot_id": "robot_0",
                            "session_id": "12345678-1234-4234-8234-123456789abc",
                            "revision": revision,
                        },
                        {
                            "robot_id": "robot_0",
                            "session_id": "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
                            "revision": 99,
                        },
                    ]
                },
            },
        }

    manifest = {
        "ok": True,
        "data": {
            "robot_id": "robot_0",
            "session_id": "12345678-1234-4234-8234-123456789abc",
            "revision": 8,
            "solution_order": [2, 7, 0, 0],
            "snapshot": {
                "snapshot_id": "snapshot-8",
                "manifests": [
                    {
                        "graph_revision": {
                            "component_id": "component:anchor",
                            "epoch": 2,
                            "revision": 7,
                        },
                        "geometry_revision": "a" * 64,
                    }
                ],
            },
        },
    }
    result = summarize(
        sample(0.0, "starting", 0),
        [sample(0.3, "waiting", 3), sample(1.0, "blocked", 1)],
        [manifest],
        "12345678-1234-4234-8234-123456789abc",
    )

    robot = result["robots"]["robot_0"]
    assert robot["server_frame_displacement_m"] == 1.0
    assert not robot["ever_reported_exploring"]
    assert robot["ever_reported_executing"]
    assert robot["ever_reported_blocked"]
    assert robot["max_global_path_points"] == 3
    replica = result["replicas"]["robot_0/12345678-1234-4234-8234-123456789abc"]
    assert replica["first_observed_revision"] == 0
    assert replica["last_observed_revision"] == 1
    assert replica["observed_revision_delta"] == 1
    assert replica["final_manifest_revision"] == 8
    assert all("aaaaaaaa" not in key for key in result["replicas"])
    assert result["reported_components"][0]["component_id"] == "component:anchor"
    assert result["verified_components"] == []


def test_setup_failure_still_sends_stop_all_and_writes_evidence(tmp_path, monkeypatch):
    commands = []

    async def sample(*_args):
        return {
            "fleet": {"ok": True, "data": {"robots": []}},
            "map_status": {"ok": True, "data": {}},
            "replica_index": {"ok": True, "data": {"replicas": []}},
        }

    async def command(_base_url, payload, _timeout):
        commands.append(payload)
        return {"sent": True, "delivery_confirmed": False, "payload": payload}

    monkeypatch.setattr(observer, "collect_sample", sample)
    monkeypatch.setattr(observer, "send_gui_command", command)
    args = argparse.Namespace(
        base_url="http://127.0.0.1:18080",
        duration=1.0,
        interval=0.2,
        timeout=0.2,
        mission_id=None,
        max_manifests=4,
        output=tmp_path / "evidence.json",
    )

    report, fatal = asyncio.run(observer.observe(args))

    assert fatal
    assert commands == [{"type": "stop_all"}]
    assert report["commands"]["stop_all"]["sent"]
    assert args.output.exists()


def test_final_manifest_fetch_is_mission_filtered_and_count_bounded(monkeypatch):
    fetched = []
    active = 0
    peak_active = 0

    async def get(_base_url, path, _timeout):
        nonlocal active, peak_active
        active += 1
        peak_active = max(peak_active, active)
        await asyncio.sleep(0)
        fetched.append(path)
        active -= 1
        return {"ok": True, "data": {"path": path}}

    monkeypatch.setattr(observer, "endpoint", get)
    index = {
        "ok": True,
        "data": {
            "replicas": [
                {"robot_id": f"robot_{i}", "session_id": "mission-a", "revision": i}
                for i in range(10)
            ]
            + [{"robot_id": "old", "session_id": "mission-old", "revision": 99}]
        },
    }

    result = asyncio.run(
        observer.final_manifests("http://127.0.0.1:18080", index, 0.2, "mission-a", 6)
    )

    assert len(result) == len(fetched) == 6
    assert peak_active == 4
    assert all("mission-a" in path for path in fetched)
    assert all("mission-old" not in path for path in fetched)


def test_command_frame_written_survives_close_timeout(monkeypatch):
    class Transport:
        aborted = False

        def abort(self):
            self.aborted = True

    class Socket:
        def __init__(self):
            self.transport = Transport()
            self.frames = []

        def __aiter__(self):
            return self

        async def __anext__(self):
            await asyncio.Event().wait()

        async def send(self, frame):
            self.frames.append(frame)

        async def close(self):
            await asyncio.Event().wait()

    socket = Socket()

    async def connect(*_args, **_kwargs):
        return socket

    monkeypatch.setitem(sys.modules, "websockets", SimpleNamespace(connect=connect))
    result = asyncio.run(
        observer.send_gui_command("http://127.0.0.1:18080", {"type": "stop_all"}, 0.01)
    )

    assert result["sent"] is True
    assert result["frame_written"] is True
    assert result["delivery_confirmed"] is False
    assert result["cleanup_complete"] is False
    assert result["cleanup_errors"] == ["close: TimeoutError"]
    assert socket.transport.aborted
    assert socket.frames == ['{"type":"stop_all"}']


def test_post_stop_summary_requires_every_eligible_robot_stopped_and_pathless():
    snapshot = {
        "fleet": {
            "ok": True,
            "data": {
                "robots": [
                    {
                        "robot_id": robot_id,
                        "online": True,
                        "mode": "estop",
                        "exploration_status": "stopped",
                        "nav_status": "idle",
                        "goal": None,
                        "global_planned_path": [],
                        "local_planned_path": [],
                    }
                    for robot_id in ("robot_0", "robot_1")
                ]
            },
        }
    }

    result = summarize_post_stop(snapshot, ["robot_0", "robot_1"])

    assert result["protocol_acknowledgement"] is False
    assert result["all_eligible_reported_stopped_and_inactive"] is True
    assert all(
        state["no_active_paths"] for state in result["eligible_robot_states"].values()
    )

    snapshot["fleet"]["data"]["robots"][1]["global_planned_path"] = [{"x": 1}]
    result = summarize_post_stop(snapshot, ["robot_0", "robot_1"])
    assert result["all_eligible_reported_stopped_and_inactive"] is False
