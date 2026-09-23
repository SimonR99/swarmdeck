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


def test_latency_window_filter_uses_probe_window(monkeypatch):
    result = _trace(monkeypatch, [_pose(2049, 0), _pose(2051, .2)],
                    plans=(1000, 2050, 2200), start=2000, end=2100)
    assert result['plan_count'] == 1
    assert result['latency_s'] == {'robot_0': [1]}


def test_latency_window_filter_empty_when_all_outside(monkeypatch):
    result = _trace(monkeypatch, [_pose(2050, 0)],
                    plans=(1000, 9000), start=2000, end=2100)
    assert result['plan_count'] == 0


# ---------------------------------------------------------------------------
# Reference-pose selection (fix 2): last pre-plan pose, not first
# ---------------------------------------------------------------------------

def test_ref_pos_uses_last_pre_plan_sample(monkeypatch):
    result = _trace(monkeypatch, [_pose(1, 0), _pose(9, 1),
                                _pose(11, 1.01), _pose(12, 1.2)])
    assert result['latency_s'] == {'robot_0': [2]}


def test_ref_pos_fallback_when_no_pre_plan_events(monkeypatch):
    result = _trace(monkeypatch, [_pose(11, 2), _pose(12, 2.2)])
    assert result['latency_s'] == {'robot_0': [2]}


def test_latency_superseded_plan_cannot_borrow_later_motion(monkeypatch):
    result = _trace(monkeypatch, [_pose(9, 0), _pose(19, 0), _pose(21, .2)],
                    plans=(10, 20))
    assert result['latency_s'] == {'robot_0': [1]}
    assert result['cut_off_plan_count'] == 1


def test_latency_reset_cuts_off_plan_even_without_later_telemetry(monkeypatch):
    result = _trace(monkeypatch, [_pose(9, 0), {'utc': 11, 'reset': None}])
    assert result['latency_s'] == {}
    assert result['cut_off_plan_count'] == 1


def test_latency_no_reset_spanning_reference_for_a_new_plan(monkeypatch):
    result = _trace(monkeypatch, [_pose(9, 0), {'utc': 11, 'reset': ['robot_0']},
                                _pose(21, 1), _pose(22, 1.2)], plans=(20,))
    assert result['latency_s'] == {'robot_0': [2]}
    assert result['cut_off_plan_count'] == 0


def test_latency_displacement_exactly_at_next_plan_is_cut_off(monkeypatch):
    result = _trace(monkeypatch, [_pose(9, 0), _pose(20, .2)], plans=(10, 20))
    assert result['latency_s'] == {}
    assert result['cut_off_plan_count'] == 1


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


def _trace(monkeypatch, events, plans=(10,), start=0, end=30):
    """Run the actual collector/correlator with captured external command output."""
    import json
    import subprocess
    import profile_stack

    clock = iter((start, end))
    monkeypatch.setattr(profile_stack.time, 'time', lambda: next(clock))
    results = iter((
        subprocess.CompletedProcess([], 0, stdout=json.dumps(events), stderr=''),
        subprocess.CompletedProcess([], 0, stdout='\n'.join(
            _make_ts_line(t, _PLAN_CONTENT) for t in plans), stderr=''),
    ))
    monkeypatch.setattr(profile_stack, 'sh', lambda *a, **kw: next(results))
    return profile_stack.latency_trace(end - start)


def _pose(utc, x, y=0, transform=None):
    return {'utc': utc, 'robot_id': 'robot_0', 'pose': {'x': x, 'y': y},
            'navigation_transform': transform}


def test_latency_registration_only_change_is_not_motion(monkeypatch):
    result = _trace(monkeypatch, [
        _pose(9, 0, transform={'x': 0, 'y': 0, 'yaw': 0}),
        _pose(11, .2, transform={'x': .2, 'y': 0, 'yaw': 0}),
        _pose(12, .4, transform={'x': .2, 'y': 0, 'yaw': 0}),
    ])
    assert result['latency_s'] == {}


def test_latency_inverts_small_registration_rotation_before_comparing(monkeypatch):
    import math
    # Below the transform-change tolerance but enough world movement at range
    # to exceed the displacement threshold. The robot is stationary locally.
    yaw = .0009
    result = _trace(monkeypatch, [
        _pose(9, 200, transform={'x': 0, 'y': 0, 'yaw': 0}),
        _pose(11, 200 * math.cos(yaw), 200 * math.sin(yaw),
              transform={'x': 0, 'y': 0, 'yaw': yaw}),
    ])
    assert result['latency_s'] == {}


def test_latency_reset_cannot_be_motion(monkeypatch):
    result = _trace(monkeypatch, [
        _pose(9, 0), {'utc': 10.5, 'reset': ['robot_0']}, _pose(11, 1),
    ])
    assert result['latency_s'] == {}


def test_latency_real_motion_in_navigation_frame(monkeypatch):
    transform = {'x': 5, 'y': 4, 'yaw': 1.57}
    result = _trace(monkeypatch, [_pose(9, 5, 4, transform),
                                _pose(11, 5, 4.2, transform)])
    assert result['latency_s'] == {'robot_0': [1]}


def test_browser_timeout_returns_unavailable(monkeypatch, tmp_path):
    import subprocess
    import profile_stack

    monkeypatch.setattr(profile_stack, '_find_playwright_modules', lambda: tmp_path)
    def timeout(*args, **kwargs):
        raise subprocess.TimeoutExpired(args[0], kwargs['timeout'])
    monkeypatch.setattr(profile_stack, 'sh', timeout)
    result = profile_stack.browser_cpu('http://localhost:5173', idle_s=1, pans=0)
    assert 'timed out' in result['error']
