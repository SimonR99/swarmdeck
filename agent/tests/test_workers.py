from agent_cortex.providers import AgyProvider, OpenCodeProvider
from agent_cortex.workers import RuntimeCodingWorker, get_coding_worker


def test_worker_selection_is_separate_from_active_provider(monkeypatch):
    monkeypatch.setenv("CORTEX_CODING_WORKER", "opencode")
    monkeypatch.setenv("CORTEX_OPENCODE_COMMAND", "/bin/echo")

    worker = get_coding_worker()

    assert isinstance(worker, RuntimeCodingWorker)
    assert isinstance(worker.provider, OpenCodeProvider)
    assert worker.status()["role"] == "coding_worker"
    assert worker.status()["active"] is False


def test_default_worker_remains_agy(monkeypatch):
    monkeypatch.delenv("CORTEX_CODING_WORKER", raising=False)
    assert isinstance(get_coding_worker().provider, AgyProvider)
