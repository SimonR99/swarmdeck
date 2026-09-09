from dataclasses import replace
import uuid
from autonomy.coordination import LeaseArbiter, CompletionTracker, ExplorationReport


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
