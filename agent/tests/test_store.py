from agent_cortex.store import CortexStore, SCHEMA_VERSION


def test_store_is_idempotent_and_keeps_ordered_events(tmp_path):
    path = tmp_path / "cortex" / "state.db"
    store = CortexStore(path)
    job_id = store.create_job(
        kind="diagnosis",
        provider="agy",
        request_text="diagnose scout",
        selected_robot="tars_0",
    )
    assert store.append_event(job_id, "provider.init", {"value": 1}) == 1
    assert store.append_event(job_id, "provider.done", {"value": 2}) == 2
    store.set_phase(job_id, "completed")

    reopened = CortexStore(path)
    assert reopened.get_job(job_id)["phase"] == "completed"
    assert [event["payload"] for event in reopened.get_events(job_id)] == [
        {"value": 1},
        {"value": 2},
    ]
    with reopened._connect() as connection:
        assert connection.execute("SELECT version FROM schema_info").fetchone()[0] == (
            SCHEMA_VERSION
        )


def test_memory_is_candidate_until_explicitly_reviewed(tmp_path):
    store = CortexStore(tmp_path / "state.db")
    memory_id = store.propose_memory(
        scope="operator",
        memory_key="response_style",
        value={"preference": "brief"},
        confidence=0.8,
    )

    candidates = store.list_memories(status="candidate")
    assert candidates[0]["memory_id"] == memory_id
    assert candidates[0]["value"] == {"preference": "brief"}

    store.review_memory(memory_id, "confirmed")
    assert store.list_memories(status="candidate") == []
    assert store.list_memories(status="confirmed")[0]["memory_id"] == memory_id


def test_startup_recovery_closes_jobs_from_a_previous_process(tmp_path):
    store = CortexStore(tmp_path / "state.db")
    job_id = store.create_job(
        kind="repair", provider="agy", request_text="restart interrupted"
    )

    assert store.interrupt_running_jobs() == 1
    assert store.get_job(job_id)["phase"] == "interrupted"
    assert "Cortex restarted" in store.get_job(job_id)["error_text"]


def test_compaction_reduces_replay_without_deleting_evidence(tmp_path):
    store = CortexStore(tmp_path / "state.db")
    job_id = store.create_job(
        kind="conversation", provider="agy", request_text="long conversation"
    )
    store.append_event(job_id, "provider.token", {"delta": "old context"})
    store.append_event(job_id, "provider.tool_output", {"output": "old evidence"})
    store.add_compaction(
        job_id=job_id,
        through_sequence=2,
        summary="The robot passed its initial checks.",
    )
    store.append_event(job_id, "provider.token", {"delta": "new context"})

    replay = store.get_replay_context(job_id)

    assert replay["summary"] == "The robot passed its initial checks."
    assert [event["sequence"] for event in replay["events"]] == [3]
    assert len(store.get_events(job_id)) == 3
