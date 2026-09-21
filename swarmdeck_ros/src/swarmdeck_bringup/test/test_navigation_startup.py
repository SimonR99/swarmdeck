"""Lifecycle response loss must not leave a partially configured Nav2 stack."""

import importlib.util
from pathlib import Path

import pytest

SCRIPT = (
    Path(__file__).resolve().parents[2] / "swarmdeck_nav/scripts/lifecycle_startup.py"
)
spec = importlib.util.spec_from_file_location("navigation_startup", SCRIPT)
startup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(startup)


class Clock:
    now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def test_lost_configuration_response_recovers_without_double_transition():
    clock = Clock()
    states = {"controller": 1, "planner": 1, "smoother": 1}
    changes = []

    def change(name, transition, deadline):
        assert deadline > clock()
        changes.append((name, transition))
        states[name] = 2 if transition == 1 else 3
        if (name, transition) == ("controller", 1):
            clock.sleep(1)
            raise TimeoutError("DDS response lost after configure completed")
        return True

    startup.bringup(
        states, lambda name, _: states[name], change, clock=clock, sleep=clock.sleep
    )
    assert set(states.values()) == {3}
    assert all(
        changes.count((name, transition)) == 1
        for name in states
        for transition in (1, 3)
    )


def test_mixed_active_and_inactive_nodes_are_not_reset_or_reconfigured():
    clock = Clock()
    states = {"controller": 3, "planner": 2}
    changes = []

    def change(name, transition, _):
        changes.append((name, transition))
        states[name] = 3
        return True

    startup.bringup(
        states, lambda name, _: states[name], change, clock=clock, sleep=clock.sleep
    )
    assert states == {"controller": 3, "planner": 3}
    assert changes.count(("planner", 3)) == 1
    assert ("controller", 1) not in changes
    assert ("controller", 3) not in changes


def test_transitional_or_missing_service_exhausts_shared_deadline():
    for state in (None, 10):
        clock = Clock()
        changes = []

        def query(name, deadline):
            if state is None:
                clock.sleep(deadline - clock())
                raise TimeoutError("service absent")
            return state

        with pytest.raises(TimeoutError):
            startup.bringup(
                ["controller", "planner"],
                query,
                lambda *args: changes.append(args),
                deadline_s=3,
                clock=clock,
                sleep=clock.sleep,
            )
        assert changes == []
        assert clock() == 3


def test_active_node_loss_during_later_activation_is_not_reported_ready():
    clock = Clock()
    states = {"controller": 2, "planner": 2}

    def change(name, transition, _):
        states[name] = 3
        if name == "planner":
            states["controller"] = 2
        return True

    with pytest.raises(RuntimeError):
        startup.bringup(
            states, lambda name, _: states[name], change, clock=clock, sleep=clock.sleep
        )


@pytest.mark.parametrize("query_fails", [False, True])
def test_shutdown_during_query_prevents_transitions_and_retry(query_fails):
    clock = Clock()
    stopping = False
    queries = []
    changes = []

    def query(name, _):
        nonlocal stopping
        queries.append(name)
        stopping = True
        if query_fails:
            raise RuntimeError("ROS context shut down during discovery")
        return 2

    with pytest.raises(startup.BringupCancelled):
        startup.bringup(
            ["controller"],
            query,
            lambda *args: changes.append(args),
            clock=clock,
            sleep=clock.sleep,
            cancelled=lambda: stopping,
        )
    assert queries == ["controller"]
    assert changes == []
    assert clock() == 0
