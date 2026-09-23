"""Unit tests for pure parsing functions in profile_stack.py.

These tests exercise only functions that operate on strings and dicts, with
no docker, ROS, or browser dependency. They must pass with a plain ``pytest``
invocation without any running stack.
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
# short_command (existing helper; regression guard)
# ---------------------------------------------------------------------------

def test_short_command_py_script():
    assert short_command("python3 /opt/ros/jazzy/lib/swarmdeck_sim/swarmdeck_argos_bridge.py __ns:=/sim") \
        == "swarmdeck_argos_bridge.py /sim"


def test_short_command_executable():
    cmd = "/usr/bin/argos3 -c scenario.argos"
    result = short_command(cmd)
    assert "argos3" in result
