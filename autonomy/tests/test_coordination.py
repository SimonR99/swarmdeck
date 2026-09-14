from dataclasses import replace
import uuid
from autonomy.coordination import (
    CompletionTracker,
    ExplorationReport,
    Intention,
    LeaseArbiter,
)


def test_components_leases_reorder_partition_and_rejoin():
    now = [0.0]
    clock = lambda: now[0]
    session = str(uuid.uuid4())
    a = LeaseArbiter("a", session, {"a", "b"}, clock=clock)
    b = LeaseArbiter("b", session, {"a", "b"}, clock=clock)
    a.set_component("first")
    b.set_component("unrelated")
    ca, cb = a.propose((0, 0, 0)), b.propose((0, 0, 0))
    a.receive(cb)
    b.receive(ca)
    now[0] = 1
    assert a.decision() == b.decision() == "granted"
    # A verified component merge enables deterministic arbitration.
    b.set_component("first")
    cb = b.propose((0, 0, 0))
    ca = a.propose((0, 0, 0))
    a.receive(cb)
    b.receive(ca)
    now[0] = 2
    assert a.decision() == "granted"
    assert b.decision() == "conflict"
    assert not b.receive(replace(ca, sequence=ca.sequence - 1))
    # A partition expires a peer reservation on the receiver's clock.
    now[0] = 5
    cb = b.propose((0, 0, 0))
    assert b.decision() == "pending"
    now[0] = 6
    assert b.decision() == "granted"
    stop = a.release()
    assert b.receive(stop)
    assert not b.receive(ca)
    assert not b.receive(replace(cb, session_id=str(uuid.uuid4()), sequence=100))


def test_decision_winner_uses_one_clock_sample_and_verified_component():
    now = [0.0]
    session = str(uuid.uuid4())
    arbiter = LeaseArbiter("a", session, {"a", "b", "c"}, clock=lambda: now[0])
    arbiter.set_component("verified")
    arbiter.propose((0, 0, 0), cost=2.0)
    arbiter.receive(
        Intention("b", session, 1, "verified", (0, 0, 0), 2.0, 1.0, 3.0)
    )
    # Defend the evaluation even if a lease from another component is present
    # because of corrupted legacy state or an implementation regression.
    unrelated = Intention(
        "c", session, 1, "other", (0, 0, 0), 2.0, 0.0, 3.0
    )
    arbiter.leases["c"] = (unrelated, 3.0)

    calls = []

    def crossing_clock():
        calls.append(None)
        return 2.9 if len(calls) == 1 else 3.1

    arbiter.clock = crossing_clock
    assert arbiter.decision_with_winner() == ("conflict", "b")
    assert len(calls) == 1


def test_completion_requires_fresh_all_participants_and_coverage():
    now = [0.0]
    tracker = CompletionTracker({"a", "b"}, clock=lambda: now[0])
    tracker.receive(ExplorationReport("a", "locally_exhausted", 0, True))
    assert tracker.state() == "unknown"
    tracker.receive(ExplorationReport("b", "blocked", 0, False))
    assert tracker.state() == "incomplete"
    tracker.receive(ExplorationReport("b", "locally_exhausted", 0, True))
    assert tracker.state() == "complete"
    now[0] = 6
    assert tracker.state() == "unknown"
