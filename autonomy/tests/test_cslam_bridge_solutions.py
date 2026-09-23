"""The bridge's optimizer-result statistics follow the core's adoption rule.

``deploy/autonomy/cslam_bridge.py`` imports the ROS stack at module scope,
but ``Bridge.optimized`` is pure bookkeeping over ``CslamMapper.solution``
and deserves to run under pytest: it drifted once already, counting
acceptance by the adopted order after the core moved acceptance to
``solver_order``, so ``solution_results_unchanged`` could never advance.
"""

from __future__ import annotations

import importlib
import sys
import types
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import MagicMock
import uuid

import pytest

from autonomy.contracts import IDENTITY_SE3
from autonomy.cslam import CslamMapper
from autonomy.mapping import CorrectionAwareMapper, SubmapStore

REPO = Path(__file__).resolve().parents[2]
BRIDGE_SOURCE = REPO / "deploy" / "autonomy" / "cslam_bridge.py"

STUBBED_ROS_MODULES = [
    "rclpy",
    "rclpy.context",
    "rclpy.clock",
    "rclpy.executors",
    "rclpy.node",
    "rclpy.parameter",
    "rclpy.time",
    "rclpy.qos",
    "nav_msgs",
    "nav_msgs.msg",
    "sensor_msgs",
    "sensor_msgs.msg",
    "sensor_msgs_py",
    "sensor_msgs_py.point_cloud2",
    "std_msgs",
    "std_msgs.msg",
    "tf2_ros",
    "cslam_common_interfaces",
    "cslam_common_interfaces.msg",
]


@pytest.fixture(scope="module")
def bridge_module():
    """The imported bridge module, with its ROS imports stubbed."""

    saved = {name: sys.modules.get(name) for name in STUBBED_ROS_MODULES}
    for name in STUBBED_ROS_MODULES:
        sys.modules[name] = MagicMock()
    # `class Bridge(Node)` needs a real base class, and `except
    # TransformException` a real exception class.
    node = types.ModuleType("rclpy.node")
    node.Node = type("Node", (), {})
    sys.modules["rclpy.node"] = node
    tf2 = types.ModuleType("tf2_ros")
    tf2.Buffer, tf2.TransformListener = MagicMock(), MagicMock()
    tf2.TransformException = type("TransformException", (Exception,), {})
    sys.modules["tf2_ros"] = tf2
    sys.modules.pop("deploy.autonomy.cslam_bridge", None)
    try:
        yield importlib.import_module("deploy.autonomy.cslam_bridge")
    finally:
        sys.modules.pop("deploy.autonomy.cslam_bridge", None)
        for name, value in saved.items():
            if value is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = value


def value(robot, seq, x):
    return NS(
        key=NS(robot_id=robot, keyframe_id=seq),
        pose=NS(position=NS(x=x, y=0.0, z=0.0), orientation=NS(x=0, y=0, z=0, w=1)),
    )


def result(mission, clock, x):
    return NS(
        success=True,
        mission_id=mission,
        map_epoch=0,
        robot_map_epochs=[0],
        participant_robot_ids=[0],
        solution_clock=clock,
        optimizer_robot_id=0,
        origin_robot_id=0,
        estimates=[value(0, 0, x)],
        anchor_estimates=[value(0, 0, x)],
    )


def counters(bridge):
    return (
        bridge.solution_results_received,
        bridge.solution_results_accepted,
        bridge.solution_results_unchanged,
        bridge.solution_results_deferred,
        bridge.solution_count,
    )


def test_optimized_counts_adopted_deferred_and_unchanged_separately(
    tmp_path, bridge_module
):
    now = [100.0]
    core = CslamMapper(
        CorrectionAwareMapper(SubmapStore(tmp_path / "maps")),
        "robot_0",
        0,
        str(uuid.uuid4()),
        {0: "robot_0"},
        clock=lambda: now[0],
    )
    core.capture(0, 1, IDENTITY_SE3, [[1.0, 0.0, 0.0]])
    bridge = NS(
        core=core,
        solution_results_received=0,
        solution_results_accepted=0,
        solution_results_unchanged=0,
        solution_results_deferred=0,
        solution_count=0,
    )
    optimized = bridge_module.Bridge.optimized

    # Another mission: received, nothing else.
    optimized(bridge, result(str(uuid.uuid4()), 10, 0.0))
    assert counters(bridge) == (1, 0, 0, 0, 0)
    assert core.solver_order == (0, -1)

    # The first accepted solution is adopted whatever it moves: it names the
    # frame every publisher of a merged component shares.
    optimized(bridge, result(core.mission_id, 1, 0.0))
    assert counters(bridge) == (2, 1, 0, 0, 1)
    assert core.solver_order == (1, 0)
    assert core.solution_order == (1, 0)

    # A replayed clock is not accepted again.
    optimized(bridge, result(core.mission_id, 1, 0.0))
    assert counters(bridge) == (3, 1, 0, 0, 1)

    # Accepted and unchanged, after the interval: the solver clock advances,
    # the frame does not.
    now[0] += 60.0
    optimized(bridge, result(core.mission_id, 2, 0.0))
    assert counters(bridge) == (4, 2, 1, 0, 1)
    assert core.solution_order == (1, 0)

    # Adopted.
    optimized(bridge, result(core.mission_id, 3, 0.06))
    assert counters(bridge) == (5, 3, 1, 0, 2)
    assert core.solution_order == (3, 0)
    assert core.correction_revision == 2

    # A refinement inside the interval: accepted and deferred, not adopted.
    now[0] += 3.0
    optimized(bridge, result(core.mission_id, 4, 0.12))
    assert counters(bridge) == (6, 4, 1, 1, 2)
    assert core.solver_order == (4, 0)
    assert core.solution_order == (3, 0)
    assert core.deferred_solution is not None

    # After the interval the next report is adopted with the refinement.
    now[0] += 7.0
    optimized(bridge, result(core.mission_id, 5, 0.12))
    assert counters(bridge) == (7, 5, 1, 1, 3)
    assert core.solution_order == (5, 0)
    assert core.deferred_solution is None


def test_parked_scans_repeat_only_at_the_scene_change_period(bridge_module):
    import math

    parked_since = bridge_module.parked_since
    rest = (1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0)

    def yawed(rad):
        return (1.0, 2.0, 0.0, 0.0, 0.0, math.sin(rad / 2), math.cos(rad / 2))

    assert not parked_since(None, rest, 10.0, 0.0)  # nothing normalized yet
    assert parked_since(rest, rest, 10.0, 9.0)
    assert parked_since(rest, yawed(0.04), 10.0, 9.0)
    assert not parked_since(rest, yawed(0.06), 10.0, 9.0)
    assert not parked_since(rest, (1.03, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0), 10.0, 9.0)
    # A parked robot still feeds Swarm-SLAM's scene-change rule at its period.
    assert not parked_since(rest, rest, 9.0 + bridge_module.PARKED_REPUBLISH_S, 9.0)


def test_write_text_if_changed_skips_the_write_when_nothing_changed(tmp_path, bridge_module):
    write_text_if_changed = bridge_module.write_text_if_changed
    path = tmp_path / "status.json"

    last = write_text_if_changed(path, "a", None)
    assert last == "a" and path.read_text() == "a"
    mtime = path.stat().st_mtime_ns

    # Unchanged text: no replace, the file is untouched.
    last = write_text_if_changed(path, "a", last)
    assert last == "a" and path.stat().st_mtime_ns == mtime

    # Changed text: written and returned as the new cache.
    last = write_text_if_changed(path, "b", last)
    assert last == "b" and path.read_text() == "b"


def _authority(mission_id, run_id, epoch, revision):
    return {
        "robot_id": "robot_0",
        "mission_id": mission_id,
        "robot_map_epoch": epoch,
        "run_id": run_id,
        "component_id": "component:merged",
        "solution_order": [0, -1],
        "correction_revision": 0,
        "map_epoch": epoch,
        "mapping_graph_revision": revision,
        "geometry_revision": f"{revision:064x}",
        "map_source_stamp": {"sec": 0, "nanosec": 0},
        "navigation_frame": "robot_0/odom",
        "T_component_navigation": IDENTITY_SE3,
        "planning_frame": "robot_0/odom",
        "T_component_planning": IDENTITY_SE3,
    }


def test_authority_heartbeat_republishes_the_cache_verbatim_while_gated(bridge_module):
    """The heartbeat keeps MGG's snapshot alive across a transient stall.

    ``cslam_bridge._snapshot`` gates a *fresh* authority on sensor freshness,
    the reset ACK and a paired product; none of that used to gate the
    heartbeat, so a single slow tick sent ``resetting`` and expired MGG's
    snapshot (~3 s TTL). ``authority_heartbeat`` is the fix: keep the last
    good authority in service until a fresher one lands or the run itself
    changes.
    """
    from autonomy.map_epochs import robot_run_id

    heartbeat = bridge_module.authority_heartbeat
    mission = str(uuid.uuid4())
    run_id = robot_run_id(mission, "robot_0", 0)
    good = _authority(mission, run_id, 0, 7)
    kwargs = dict(robot_id="robot_0", mission_id=mission, robot_map_epoch=0, run_id=run_id)

    # A fresh authority is published and cached as-is.
    message, cache = heartbeat(good, None, **kwargs)
    assert message is good and cache is good

    # No fresh authority this tick (a gated build, or none attempted): the
    # cached one is re-sent byte-identical, never rebuilt or mutated.
    message, cache = heartbeat(None, cache, **kwargs)
    assert message is good and cache is good

    # A newer fresh authority replaces the cache, and stays authoritative on
    # the next gap; the bridge never regresses to a revision already
    # superseded.
    newer = _authority(mission, run_id, 0, 9)
    message, cache = heartbeat(newer, cache, **kwargs)
    assert message is newer and cache is newer
    message, cache = heartbeat(None, cache, **kwargs)
    assert message is newer

    # A run change (mission id, map epoch or run id) is a real reset: the
    # stale authority is never re-sent for a run it no longer names.
    reset_run_id = robot_run_id(mission, "robot_0", 1)
    message, cache = heartbeat(
        None,
        cache,
        robot_id="robot_0",
        mission_id=mission,
        robot_map_epoch=1,
        run_id=reset_run_id,
    )
    assert message == {
        "robot_id": "robot_0",
        "mission_id": mission,
        "robot_map_epoch": 1,
        "run_id": reset_run_id,
        "state": "resetting",
    }
    assert cache is None

    # With nothing cached and nothing fresh, resetting is reported.
    message, cache = heartbeat(None, None, **kwargs)
    assert message["state"] == "resetting" and cache is None


def test_authority_heartbeat_resend_is_a_valid_causal_update(bridge_module):
    """The re-sent authority passes the contract that rejects causal rollback."""

    from adapters.mapping_authority import accepts_authority_update
    from autonomy.map_epochs import robot_run_id

    heartbeat = bridge_module.authority_heartbeat
    mission = str(uuid.uuid4())
    run_id = robot_run_id(mission, "robot_0", 0)
    good = _authority(mission, run_id, 0, 7)
    kwargs = dict(robot_id="robot_0", mission_id=mission, robot_map_epoch=0, run_id=run_id)

    assert accepts_authority_update(good, None)
    resent, _ = heartbeat(None, good, **kwargs)
    # Identical heartbeats are accepted, never treated as rollback.
    assert accepts_authority_update(resent, good)
