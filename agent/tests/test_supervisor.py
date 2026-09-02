import asyncio
import json

from agent_cortex.contracts import PlannerDecision
from agent_cortex.providers import ProviderRequest
from agent_cortex.supervisor import (
    CortexSupervisor,
    SupervisorRequest,
    build_supervisor,
    classify_job,
)


class Provider:
    name = "test"

    def status(self):
        return {"name": self.name, "available": True}

    async def start(self, _request):
        events = [
            {"type": "init", "conversation_id": "opaque-provider-id"},
            {"type": "token", "delta": "answer"},
            {"type": "done", "response": "answer", "status": "SUCCESS"},
        ]

        class Reader:
            def __init__(self, lines):
                self.lines = iter(lines)

            async def readline(self):
                return next(self.lines, b"")

            async def read(self, _size=-1):
                return b""

        class Process:
            stdout = Reader([(json.dumps(event) + "\n").encode() for event in events])
            stderr = Reader([])

            async def wait(self):
                return 0

            def kill(self):
                pass

        return Process()

    def parse_event(self, line):
        return json.loads(line)


def test_observe_mode_is_transparent_and_durable(tmp_path, monkeypatch):
    monkeypatch.setenv("CORTEX_SHADOW_PLANNER", "false")
    supervisor = CortexSupervisor(mode="observe", store_path=tmp_path / "state.db")
    request = SupervisorRequest(
        provider_request=ProviderRequest(
            prompt="system + operator prompt",
            workspace=".",
            conversation_id="opaque-provider-id",
        ),
        operator_prompt="diagnose @tars_0",
        provider_name="test",
        selected_robot="tars_0",
    )

    async def collect():
        return [event async for event in supervisor.run(Provider(), request)]

    events = asyncio.run(collect())
    assert events == [
        {"type": "init", "conversation_id": "opaque-provider-id"},
        {"type": "token", "delta": "answer"},
        {"type": "done", "response": "answer", "status": "SUCCESS"},
    ]
    jobs = supervisor.store.list_jobs()
    assert jobs[0]["kind"] == "diagnosis"
    assert jobs[0]["phase"] == "completed"
    assert jobs[0]["conversation_id"] == "opaque-provider-id"
    assert [event["event_type"] for event in supervisor.store.get_events(jobs[0]["job_id"])] == [
        "provider.init",
        "provider.token",
        "provider.done",
    ]


def test_store_failure_does_not_break_provider_stream(tmp_path, monkeypatch):
    monkeypatch.setenv("CORTEX_SHADOW_PLANNER", "false")
    supervisor = CortexSupervisor(mode="observe", store_path=tmp_path / "state.db")

    class BrokenStore:
        def __getattr__(self, _name):
            def fail(*_args, **_kwargs):
                raise OSError("disk unavailable")

            return fail

    supervisor.store = BrokenStore()
    request = SupervisorRequest(
        provider_request=ProviderRequest(prompt="hello", workspace="."),
        operator_prompt="hello",
        provider_name="test",
    )

    async def collect():
        return [event async for event in supervisor.run(Provider(), request)]

    assert asyncio.run(collect())[-1]["type"] == "done"
    assert supervisor.last_store_error == "disk unavailable"


def test_slow_shadow_plan_finishes_after_fast_provider(tmp_path, monkeypatch):
    monkeypatch.setenv("CORTEX_SHADOW_PLANNER", "false")
    supervisor = CortexSupervisor(mode="observe", store_path=tmp_path / "state.db")

    class SlowPlanner:
        async def plan(self, _request):
            await asyncio.sleep(0.02)
            return PlannerDecision(
                action="diagnose",
                target_robots=["tars_0"],
                instructions="Run the consolidated doctor check.",
                confidence=0.9,
                requires_approval=False,
            )

    supervisor.shadow_planner = SlowPlanner()
    request = SupervisorRequest(
        provider_request=ProviderRequest(prompt="context", workspace="."),
        operator_prompt="diagnose @tars_0",
        provider_name="test",
        selected_robot="tars_0",
    )

    async def collect_and_wait_for_shadow():
        events = [event async for event in supervisor.run(Provider(), request)]
        assert events[-1]["type"] == "done"
        await asyncio.sleep(0.04)

    asyncio.run(collect_and_wait_for_shadow())
    job = supervisor.store.list_jobs()[0]
    event_types = [
        event["event_type"] for event in supervisor.store.get_events(job["job_id"])
    ]
    assert "supervisor.shadow_plan" in event_types


def test_job_classification_is_observational_only():
    assert classify_job("/doctor @spot_0") == "diagnosis"
    assert classify_job("please fix and restart scout") == "repair"
    assert classify_job("/code refactor provider") == "code_change"
    assert classify_job("/stop all") == "robot_action"
    assert classify_job("hello") == "conversation"


def test_bad_rollout_mode_fails_open_to_legacy(tmp_path, monkeypatch):
    monkeypatch.setenv("CORTEX_SUPERVISOR_MODE", "typo")
    monkeypatch.setenv("CORTEX_STATE_DB", str(tmp_path / "state.db"))

    supervisor = build_supervisor(tmp_path / "history")

    assert supervisor.mode == "legacy"
    assert "must be 'legacy' or 'observe'" in supervisor.last_store_error
