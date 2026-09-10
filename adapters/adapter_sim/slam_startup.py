"""Bounded startup recovery for one SLAM Toolbox lifecycle node."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

STATE_UNCONFIGURED = 1
STATE_INACTIVE = 2
STATE_ACTIVE = 3

TRANSITION_CONFIGURE = 1
TRANSITION_ACTIVATE = 3


def uses_slam_toolbox(backend: str | None) -> bool:
    """Match the simulation launch default without touching other back ends."""
    return (backend or "toolbox").strip().lower() == "toolbox"


@dataclass(frozen=True)
class StartupResult:
    ready: bool
    detail: str


class SlamToolboxStartup:
    """Run at most one recovery episode, with one caller doing the work.

    ``query_state`` and ``change_state`` receive the episode's absolute
    wall-clock deadline. The ROS adapter uses that to bound both service
    discovery and the response wait without spinning its node twice.
    """

    def __init__(
        self,
        query_state: Callable[[float], int],
        change_state: Callable[[int, float], bool],
        report_failure: Callable[[str], None],
        *,
        deadline_s: float = 30.0,
        backoff_s: float = 0.25,
        query_timeout_s: float = 2.0,
        transition_timeout_s: float = 10.0,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._query_state = query_state
        self._change_state = change_state
        self._report_failure = report_failure
        self._deadline_s = max(0.0, float(deadline_s))
        self._backoff_s = max(0.0, float(backoff_s))
        self._query_timeout_s = max(0.0, float(query_timeout_s))
        self._transition_timeout_s = max(0.0, float(transition_timeout_s))
        self._clock = clock
        self._sleep = sleep
        self._claim_lock = threading.Lock()
        self._claimed = False

    def run_once(self) -> StartupResult | None:
        """Recover once; concurrent and later calls leave the episode alone."""
        with self._claim_lock:
            if self._claimed:
                return None
            self._claimed = True

        result = self._run()
        if not result.ready:
            self._report_failure(result.detail)
        return result

    def _run(self) -> StartupResult:
        deadline = self._clock() + self._deadline_s
        detail = "state service did not become ready"

        while self._clock() < deadline:
            try:
                call_deadline = min(deadline, self._clock() + self._query_timeout_s)
                state = int(self._query_state(call_deadline))
            except Exception as exc:
                detail = f"state query failed: {exc}"
            else:
                if self._clock() >= deadline:
                    detail = "state query exceeded the startup deadline"
                    break
                if state == STATE_ACTIVE:
                    return StartupResult(True, "active")

                transition = None
                if state == STATE_UNCONFIGURED:
                    transition = TRANSITION_CONFIGURE
                elif state == STATE_INACTIVE:
                    transition = TRANSITION_ACTIVATE
                else:
                    # Unknown and transitional states may still be progressing.
                    # Observe them again; never guess a transition or reset.
                    detail = f"lifecycle state {state} did not settle"

                if transition is not None:
                    name = (
                        "configure"
                        if transition == TRANSITION_CONFIGURE
                        else "activate"
                    )
                    try:
                        call_deadline = min(
                            deadline, self._clock() + self._transition_timeout_s
                        )
                        accepted = self._change_state(transition, call_deadline)
                    except Exception as exc:
                        detail = f"{name} failed: {exc}"
                    else:
                        if not accepted:
                            detail = f"{name} transition was rejected"
                        else:
                            detail = (
                                f"{name} completed but active state was not confirmed"
                            )

            remaining = deadline - self._clock()
            if remaining <= 0.0:
                break
            self._sleep(min(self._backoff_s, remaining))

        return StartupResult(False, detail)
