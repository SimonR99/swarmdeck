"""Unit tests for pure parsing functions in profile_stack.py.

These tests exercise only functions that operate on strings and dicts, with
no docker, ROS, or browser dependency. Browser JavaScript helper tests use
Node when available (otherwise skip). No running stack is needed.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from profile_stack import (  # noqa: E402
    PLAN_LINE,
    parse_timestamped_plan_lines,
    replan_cadence,
    short_command,
)


# ---------------------------------------------------------------------------
# PLAN_LINE regex
# ---------------------------------------------------------------------------

_SAMPLE_PLAN = (
    "[robot_0.mgg.mggplanner_node]: "
    "planning cycle: grid graph: 1423 free cells, 312 vertices, 8734 edges"
    " (global 0 m); 987 ms (global 12, grid 901, gain 63, select 11)"
)

_SAMPLE_PLAN_LONGER = (
    "[robot_1.mgg.mggplanner_node]: extra text grid graph: 500 free cells, "
    "100 vertices, 2000 edges (global 0.5 m); 400 ms (global 5, grid 370, gain 20, select 5)"
)


def test_plan_line_matches_sample():
    m = PLAN_LINE.search(_SAMPLE_PLAN)
    assert m is not None
    robot, cells, verts, edges, total, _g, grid, gain, sel = m.groups()
    assert robot == "robot_0"
    assert cells == "1423"
    assert verts == "312"
    assert edges == "8734"
    assert total == "987"
    assert grid == "901"
    assert gain == "63"
    assert sel == "11"


def test_plan_line_matches_robot_1():
    m = PLAN_LINE.search(_SAMPLE_PLAN_LONGER)
    assert m is not None
    assert m.group(1) == "robot_1"


def test_plan_line_no_match_on_unrelated():
    assert PLAN_LINE.search("[robot_0.mgg.mggplanner_node]: heartbeat sent") is None


# ---------------------------------------------------------------------------
# parse_timestamped_plan_lines
# ---------------------------------------------------------------------------

# Docker logs --timestamps output: one UTC timestamp per line
_DOCKER_LOG_SNIPPET = """\
2026-09-23T12:04:01.123456789Z [INFO] [timestamp] [robot_0.mgg.mggplanner_node]: planning cycle: grid graph: 1423 free cells, 312 vertices, 8734 edges (global 0 m); 987 ms (global 12, grid 901, gain 63, select 11)
2026-09-23T12:04:03.456000000Z [INFO] [timestamp] [robot_0.mgg.mggplanner_node]: planning cycle: grid graph: 1430 free cells, 315 vertices, 8800 edges (global 0 m); 750 ms (global 10, grid 700, gain 35, select 5)
2026-09-23T12:04:02.000000000Z [INFO] [timestamp] [robot_1.mgg.mggplanner_node]: planning cycle: grid graph: 500 free cells, 100 vertices, 2000 edges (global 0 m); 400 ms (global 5, grid 370, gain 20, select 5)
2026-09-23T12:04:01.000000000Z not a plan line; should be ignored
"""


def test_parse_timestamped_plan_lines_count():
    events = parse_timestamped_plan_lines(_DOCKER_LOG_SNIPPET)
    assert len(events) == 3


def test_parse_timestamped_plan_lines_sorted_by_time():
    events = parse_timestamped_plan_lines(_DOCKER_LOG_SNIPPET)
    times = [e[0] for e in events]
    assert times == sorted(times)


def test_parse_timestamped_plan_lines_robot_ids():
    events = parse_timestamped_plan_lines(_DOCKER_LOG_SNIPPET)
    robots = [e[1]["robot"] for e in events]
    # robot_0 first plan, then robot_1, then robot_0 second plan
    assert robots[0] == "robot_0"
    assert robots[1] == "robot_1"
    assert robots[2] == "robot_0"


def test_parse_timestamped_plan_lines_fields():
    events = parse_timestamped_plan_lines(_DOCKER_LOG_SNIPPET)
    _, cycle = events[0]
    assert cycle["total"] == 987
    assert cycle["grid"] == 901
    assert cycle["gain"] == 63
    assert cycle["vertices"] == 312


def test_parse_timestamped_plan_lines_epoch():
    """The epoch should match 2026-09-23T12:04:01.123456Z UTC."""
    import datetime as dt
    events = parse_timestamped_plan_lines(_DOCKER_LOG_SNIPPET)
    epoch = events[0][0]
    # reconstruct expected epoch
    expected = dt.datetime(2026, 9, 23, 12, 4, 1, 123456, tzinfo=dt.timezone.utc).timestamp()
    assert abs(epoch - expected) < 0.001


def test_parse_timestamped_plan_lines_empty():
    assert parse_timestamped_plan_lines("") == []


def test_parse_timestamped_plan_lines_no_plan_lines():
    snippet = "2026-09-23T12:04:01.123456789Z just a log line\n"
    assert parse_timestamped_plan_lines(snippet) == []


# ---------------------------------------------------------------------------
# replan_cadence
# ---------------------------------------------------------------------------

def test_replan_cadence_single_robot():
    # Two events 2.5 s apart for robot_0
    events = [
        (1000.0, {"robot": "robot_0", "total": 500, "grid": 400, "gain": 80,
                  "select": 20, "cells": 100, "vertices": 50, "edges": 500}),
        (1002.5, {"robot": "robot_0", "total": 480, "grid": 380, "gain": 85,
                  "select": 15, "cells": 100, "vertices": 50, "edges": 500}),
        (1005.0, {"robot": "robot_0", "total": 510, "grid": 420, "gain": 75,
                  "select": 15, "cells": 100, "vertices": 50, "edges": 500}),
    ]
    cad = replan_cadence(events)
    assert "robot_0" in cad
    assert len(cad["robot_0"]) == 2
    assert abs(cad["robot_0"][0] - 2.5) < 1e-9
    assert abs(cad["robot_0"][1] - 2.5) < 1e-9


def test_replan_cadence_multiple_robots():
    events = [
        (1000.0, {"robot": "robot_0", "total": 500, "grid": 400, "gain": 80,
                  "select": 20, "cells": 100, "vertices": 50, "edges": 500}),
        (1001.0, {"robot": "robot_1", "total": 300, "grid": 250, "gain": 40,
                  "select": 10, "cells": 80, "vertices": 40, "edges": 400}),
        (1003.0, {"robot": "robot_0", "total": 490, "grid": 390, "gain": 85,
                  "select": 15, "cells": 100, "vertices": 50, "edges": 500}),
        (1004.5, {"robot": "robot_1", "total": 310, "grid": 260, "gain": 38,
                  "select": 12, "cells": 80, "vertices": 40, "edges": 400}),
    ]
    cad = replan_cadence(events)
    assert abs(cad["robot_0"][0] - 3.0) < 1e-9
    assert abs(cad["robot_1"][0] - 3.5) < 1e-9


def test_replan_cadence_single_event_no_interval():
    events = [
        (1000.0, {"robot": "robot_0", "total": 500, "grid": 400, "gain": 80,
                  "select": 20, "cells": 100, "vertices": 50, "edges": 500}),
    ]
    cad = replan_cadence(events)
    # single event: no interval
    assert cad == {} or cad.get("robot_0", []) == []


def test_replan_cadence_empty():
    assert replan_cadence([]) == {}


# ---------------------------------------------------------------------------
# latency_trace window filtering (fix 1)
# ---------------------------------------------------------------------------

# Build a docker-timestamps log snippet spanning two time ranges.
# The plan line at T=1000 is BEFORE our probe window; T=2000 is inside it.
_PLAN_CONTENT = (
    "[robot_0.mgg.mggplanner_node]: planning cycle: grid graph: 400 free cells, "
    "100 vertices, 2000 edges (global 0 m); 500 ms (global 5, grid 450, gain 35, select 10)"
)


def _make_ts_line(epoch_s: float, content: str) -> str:
    """Build a fake docker --timestamps log line for a given UTC epoch."""
    import datetime as dt
    ts = dt.datetime.fromtimestamp(epoch_s, tz=dt.timezone.utc)
    frac = f"{ts.microsecond:06d}000"  # pad to nanoseconds
    return f"{ts.strftime('%Y-%m-%dT%H:%M:%S')}.{frac}Z {content}"


def test_latency_window_filter_drops_pre_window_plans():
    """Plan events before start_epoch must be excluded from the filtered list."""
    start_epoch = 2000.0
    end_epoch   = 2100.0

    # one plan well before the window, one inside
    old_line    = _make_ts_line(1000.0, _PLAN_CONTENT)
    inside_line = _make_ts_line(2050.0, _PLAN_CONTENT)
    log_text = old_line + "\n" + inside_line + "\n"

    all_events = parse_timestamped_plan_lines(log_text)
    assert len(all_events) == 2, "both raw lines must parse"

    # apply the same filter latency_trace uses
    filtered = [(t, c) for t, c in all_events if start_epoch <= t <= end_epoch]
    assert len(filtered) == 1, "only the inside-window plan must survive"
    assert abs(filtered[0][0] - 2050.0) < 1.0


def test_latency_window_filter_drops_post_window_plans():
    """Plan events after end_epoch must also be excluded."""
    start_epoch = 2000.0
    end_epoch   = 2100.0

    inside_line = _make_ts_line(2050.0, _PLAN_CONTENT)
    after_line  = _make_ts_line(2200.0, _PLAN_CONTENT)
    log_text = inside_line + "\n" + after_line + "\n"

    all_events = parse_timestamped_plan_lines(log_text)
    filtered = [(t, c) for t, c in all_events if start_epoch <= t <= end_epoch]
    assert len(filtered) == 1
    assert abs(filtered[0][0] - 2050.0) < 1.0


def test_latency_window_filter_empty_when_all_outside():
    """All plans outside the window → empty filtered list."""
    start_epoch = 5000.0
    end_epoch   = 5100.0

    log_text = (
        _make_ts_line(1000.0, _PLAN_CONTENT) + "\n"
        + _make_ts_line(9000.0, _PLAN_CONTENT) + "\n"
    )
    all_events = parse_timestamped_plan_lines(log_text)
    filtered = [(t, c) for t, c in all_events if start_epoch <= t <= end_epoch]
    assert filtered == []


# ---------------------------------------------------------------------------
# Reference-pose selection (fix 2): last pre-plan pose, not first
# ---------------------------------------------------------------------------

def _last_pre_plan_pos(events, plan_epoch):
    """Reimplementation of the fixed reference-pose logic for unit testing.

    events: sorted list of (utc_s, x, y)
    Returns (x, y) of the latest sample at or before plan_epoch.
    """
    ref_pos = None
    for utc, x, y in reversed(events):
        if utc <= plan_epoch:
            ref_pos = (x, y)
            break
    if ref_pos is None:
        ref_pos = (events[0][1], events[0][2])
    return ref_pos


def test_ref_pos_uses_last_pre_plan_sample():
    """The reference pose must be the latest sample at-or-before the plan."""
    # Robot moved from (0,0) to (1,0) at t=100, then the plan arrives at t=200.
    # Correct ref = (1,0) -- the pose just before the plan -- NOT (0,0).
    events = [
        (50.0,  0.0, 0.0),
        (100.0, 1.0, 0.0),  # robot moved here before the plan
        (150.0, 1.0, 0.0),
        (250.0, 1.1, 0.0),  # tiny post-plan movement (0.1 m from 1.0)
    ]
    plan_epoch = 200.0
    ref = _last_pre_plan_pos(events, plan_epoch)
    assert ref == (1.0, 0.0), f"expected last pre-plan pos (1,0), got {ref}"


def test_ref_pos_old_first_logic_gives_wrong_answer():
    """Show that the old 'first pre-plan' logic produces the wrong reference."""
    events = [
        (50.0,  0.0, 0.0),
        (100.0, 1.0, 0.0),
        (150.0, 1.0, 0.0),
        (250.0, 1.1, 0.0),
    ]
    plan_epoch = 200.0
    # OLD behaviour: next((x,y) for utc,x,y in events if utc <= plan_epoch)
    old_ref = next(
        ((x, y) for utc, x, y in events if utc <= plan_epoch),
        (events[0][1], events[0][2]),
    )
    # Old logic returns the FIRST pre-plan pose (0, 0), not the last (1, 0)
    assert old_ref == (0.0, 0.0), f"old logic should give (0,0), got {old_ref}"

    # With old ref (0,0): displacement at t=250 is ~1.1 m already exceeded,
    # so the latency would be falsely short (250-200=50 s) even though the
    # 1.0 m displacement happened BEFORE the plan.
    # With new ref (1,0): displacement at t=250 is 0.1 m -- correct.
    new_ref = _last_pre_plan_pos(events, plan_epoch)
    assert new_ref == (1.0, 0.0)


def test_ref_pos_fallback_when_no_pre_plan_events():
    """When all events are after the plan, fall back to the first known pose."""
    events = [
        (300.0, 2.0, 3.0),
        (400.0, 2.5, 3.0),
    ]
    plan_epoch = 100.0  # earlier than all events
    ref = _last_pre_plan_pos(events, plan_epoch)
    assert ref == (2.0, 3.0), "should fall back to events[0] when none precede the plan"


# ---------------------------------------------------------------------------
# short_command (existing helper; regression guard)
# ---------------------------------------------------------------------------

def test_short_command_py_script():
    assert short_command("python3 /opt/ros/jazzy/lib/swarmdeck_sim/swarmdeck_argos_bridge.py __ns:=/sim") \
        == "swarmdeck_argos_bridge.py /sim"


def test_short_command_executable():
    cmd = "/usr/bin/argos3 -c scenario.argos"
    result = short_command(cmd)
    assert "argos3" in result


def _browser_helpers_check(assertions):
    import shutil
    import subprocess
    import pytest
    import profile_stack

    if not shutil.which('node'):
        pytest.skip('browser helper tests require node')
    helpers = getattr(profile_stack, '_BROWSER_HELPERS_JS', '')
    result = subprocess.run(
        ['node', '-e', helpers + '\n' + assertions], capture_output=True, text=True
    )
    assert result.returncode == 0, result.stderr


def test_browser_proc_stat_ticks_with_spaces_and_parentheses():
    _browser_helpers_check('''
const assert = require('node:assert/strict');
const fields = ['S', '42', ...Array(9).fill('0'), '120', '30', ...Array(6).fill('0'), '999'];
assert.deepEqual(parseProcStat('123 (chrome (worker)) ' + fields.join(' ')),
                 {ppid: 42, ticks: 150, start: 999});
assert.throws(() => parseProcStat('malformed'));
''')


def test_browser_cpu_percent_multicore_and_clock_rate():
    _browser_helpers_check('''
const assert = require('node:assert/strict');
assert.equal(cpuPercent(2500, 2, 100), 1250);
assert.equal(cpuPercent(500, 2, 250), 100);
assert.equal(cpuPercent(0, 2, 100), 0);
assert.throws(() => cpuPercent(10, 0, 100));
assert.throws(() => cpuPercent(10, 1, 0));
''')
