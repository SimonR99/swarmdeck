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


def test_authority_heartbeat_publisher_ticks_regardless_of_slow_or_stalled_updates(
    bridge_module,
):
    """`tick` never waits on `update`: a `_snapshot` tick that is slow, or
    that never runs at all (the executor starved by other callbacks), delays
    only the next *fresh* authority, never the wire heartbeat itself.
    """

    Publisher = bridge_module.AuthorityHeartbeatPublisher
    published, logged, now = [], [], [0.0]
    publisher = Publisher(
        published.append,
        period_s=1.0,
        log=logged.append,
        epoch_identity=lambda: None,
        clock=lambda: now[0],
    )

    publisher.update("authority-v1", "epoch-0")
    # `update` is never called again here, simulating a `_snapshot` timer
    # that stalls for minutes; every scheduled tick still republishes the
    # last cached message, on schedule, independent of that stall.
    for _ in range(5):
        now[0] += 1.0
        publisher.tick()
    assert published == ["authority-v1"] * 5
    # Every tick published exactly on `period_s`: no actual gap, nothing
    # logged, despite five ticks.
    assert logged == []


def test_authority_heartbeat_publisher_never_logs_a_gap_while_resending_a_stale_cached_message(
    bridge_module,
):
    """Stale sensor input with uninterrupted heartbeats must never log a gap.

    `_snapshot` calls `update()` every tick regardless of whether it could
    build a *fresh* authority; when it could not (stale sensor input, no
    product yet, ...) it re-sends the cached one instead of nothing. That is
    a `_snapshot`-side build-failure diagnostic (`status.json`'s
    `authority_gap_reason`), never a heartbeat gap: the wire heartbeat
    itself never missed a beat.
    """

    Publisher = bridge_module.AuthorityHeartbeatPublisher
    published, logged, now = [], [], [0.0]
    publisher = Publisher(
        published.append,
        period_s=1.0,
        log=logged.append,
        epoch_identity=lambda: None,
        clock=lambda: now[0],
    )

    for _ in range(10):
        # `_snapshot` re-sends the same cached authority every tick because
        # a fresh one could not be built (e.g. sensor input stale), exactly
        # as `authority_heartbeat` decides.
        publisher.update("authority-v1", "epoch-0")
        now[0] += 1.0
        publisher.tick()

    assert published == ["authority-v1"] * 10
    assert logged == []


def test_authority_heartbeat_publisher_logs_no_authority_yet_before_any_update(
    bridge_module,
):
    Publisher = bridge_module.AuthorityHeartbeatPublisher
    published, logged, now = [], [], [0.0]
    publisher = Publisher(
        published.append,
        period_s=1.0,
        log=logged.append,
        epoch_identity=lambda: None,
        clock=lambda: now[0],
    )

    # Nothing published yet, and not a full period since construction: no
    # gap reported prematurely.
    publisher.tick()
    assert published == [] and logged == []

    now[0] += 1.5
    publisher.tick()
    assert published == []
    assert logged == ["no authority yet"]


def test_authority_heartbeat_publisher_logs_when_the_thread_itself_is_late(
    bridge_module,
):
    """A message was ready the whole time; only the thread's own schedule
    slipped (GIL/OS contention on the heartbeat thread itself, never the
    ROS executor `_snapshot` runs on, since the heartbeat thread does not
    touch it).
    """

    Publisher = bridge_module.AuthorityHeartbeatPublisher
    published, logged, now = [], [], [0.0]
    publisher = Publisher(
        published.append,
        period_s=1.0,
        log=logged.append,
        epoch_identity=lambda: None,
        clock=lambda: now[0],
    )

    publisher.update("authority-v1", "epoch-0")
    now[0] += 1.0
    publisher.tick()
    assert logged == []

    # `run()` should have ticked at now=2.0; it does not until now=2.5, a
    # missed schedule slot attributable only to the thread itself, since a
    # message was cached and ready the entire time.
    now[0] += 1.5
    publisher.tick()
    assert published == ["authority-v1"] * 2
    assert logged == ["the thread was late"]


def test_authority_heartbeat_publisher_reset_replaces_the_cached_authority(
    bridge_module,
):
    """A map-epoch reset must never resend the old authority: `update`
    replaces the cache with whatever `authority_heartbeat` decided, and the
    next tick publishes exactly that, matching its own contract end to end.
    """

    from autonomy.map_epochs import robot_run_id

    Publisher = bridge_module.AuthorityHeartbeatPublisher
    heartbeat = bridge_module.authority_heartbeat
    published, now = [], [0.0]
    # This test exercises `authority_heartbeat`'s own cache invalidation (a
    # mission/run/epoch field mismatch inside the message), not the
    # publisher's stat-level epoch fencing (covered separately below), so
    # the durable epoch identity is held constant throughout.
    publisher = Publisher(
        published.append,
        period_s=1.0,
        log=lambda reason: None,
        epoch_identity=lambda: "epoch-0",
        clock=lambda: now[0],
    )

    mission = str(uuid.uuid4())
    run_id0 = robot_run_id(mission, "robot_0", 0)
    good = _authority(mission, run_id0, 0, 7)
    kwargs0 = dict(
        robot_id="robot_0", mission_id=mission, robot_map_epoch=0, run_id=run_id0
    )

    message, cache = heartbeat(good, None, **kwargs0)
    publisher.update(message, "epoch-0")
    publisher.tick()
    assert published[-1] == good

    # The build is momentarily gated (e.g. sensor stale): the cache is
    # resent verbatim, never silently dropped.
    message, cache = heartbeat(None, cache, **kwargs0)
    publisher.update(message, "epoch-0")
    publisher.tick()
    assert published[-1] == good

    # A real epoch reset: the cache no longer names the current run, so the
    # decision falls through to `resetting`, and the publisher immediately
    # starts sending that instead, never the stale authority again.
    run_id1 = robot_run_id(mission, "robot_0", 1)
    kwargs1 = dict(
        robot_id="robot_0", mission_id=mission, robot_map_epoch=1, run_id=run_id1
    )
    message, cache = heartbeat(None, cache, **kwargs1)
    assert message["state"] == "resetting" and cache is None
    publisher.update(message, "epoch-0")
    publisher.tick()
    assert published[-1] == message
    assert published[-1] != good


def test_authority_heartbeat_publisher_invalidate_clears_the_cache_immediately(
    bridge_module,
):
    Publisher = bridge_module.AuthorityHeartbeatPublisher
    published, logged, now = [], [], [0.0]
    publisher = Publisher(
        published.append,
        period_s=1.0,
        log=logged.append,
        epoch_identity=lambda: "epoch-0",
        clock=lambda: now[0],
    )

    publisher.update("authority-v1", "epoch-0")
    now[0] += 1.0
    publisher.tick()
    assert published == ["authority-v1"]

    publisher.invalidate()
    now[0] += 1.0
    publisher.tick()
    # Nothing published this tick: the cache was cleared, not merely stale.
    assert published == ["authority-v1"]

    now[0] += 1.0
    publisher.tick()
    assert logged[-1] == "epoch retired"


def test_authority_heartbeat_stops_resending_after_the_durable_epoch_changes(
    tmp_path, bridge_module
):
    """The reviewer's exact scenario: change the on-disk epoch while the old
    bridge is alive. `tick`'s own stat-level check of map-epoch.json
    (`_file_identity`, the real function, and `claim_map_epoch`, the real
    durable-epoch writer) stops the heartbeat from resending the old
    authority, with no `Bridge.snapshot()` call involved at all: an executor
    stall on the old bridge, or simply the old process not yet having
    exited, must never let it keep broadcasting a retired epoch.
    """

    from autonomy.map_epochs import claim_map_epoch

    file_identity = bridge_module._file_identity
    mission = str(uuid.uuid4())
    claim_map_epoch(tmp_path, mission, "robot_0")  # epoch 0, the old bridge's own
    epoch_path = tmp_path / mission / "robot_0" / "map-epoch.json"

    Publisher = bridge_module.AuthorityHeartbeatPublisher
    published, logged, now = [], [], [0.0]
    publisher = Publisher(
        published.append,
        period_s=1.0,
        log=logged.append,
        epoch_identity=lambda: file_identity(epoch_path),
        clock=lambda: now[0],
    )

    # The old bridge built and cached its authority under epoch 0, exactly
    # as `Bridge._snapshot` does (the identity it validated `record` under).
    publisher.update("authority-epoch-0", file_identity(epoch_path))
    now[0] += 1.0
    publisher.tick()
    assert published == ["authority-epoch-0"]

    # A fresh launch claims a new epoch on disk (`peer.launch.py`'s own
    # startup path, `garbage_collect_missions`/`checkpoint_wal`'s neighbour).
    claim_map_epoch(tmp_path, mission, "robot_0")  # epoch 1

    now[0] += 1.0
    publisher.tick()
    # Never resent: the durable epoch moved on since this authority was
    # built, caught by `tick`'s own check, not by `snapshot()` (never
    # called here at all).
    assert published == ["authority-epoch-0"]

    now[0] += 1.0
    publisher.tick()
    assert published == ["authority-epoch-0"]
    assert logged[-1] == "epoch retired"


def test_authority_heartbeat_thread_join_waits_for_an_in_flight_tick(bridge_module):
    """`Bridge.close()`'s own shutdown sequence: `self.closed.set()` then
    `self.authority_heartbeat_thread.join(timeout=...)`, reproduced here with
    a real thread. Setting `closed` must never let the join return while a
    tick is mid-publish, and no publish may happen once the join has
    returned: `close()` runs entirely before the sensor node (which owns
    `authority_pub`) or the bridge node itself is destroyed.
    """

    import threading

    Publisher = bridge_module.AuthorityHeartbeatPublisher
    entered = threading.Event()
    release = threading.Event()
    published = []

    def slow_publish(message):
        # Simulates a tick already past `Event.wait()` and inside
        # `self._publish(message)` -- the DDS write itself, or anything else
        # that briefly blocks -- at the exact moment shutdown begins.
        entered.set()
        release.wait(timeout=5)
        published.append(message)

    publisher = Publisher(
        slow_publish, period_s=0.01, log=lambda reason: None, epoch_identity=lambda: None
    )
    publisher.update("authority-v1", None)

    closed = threading.Event()
    thread = threading.Thread(target=publisher.run, args=(closed,), daemon=True)
    thread.start()
    assert entered.wait(timeout=5)  # the first tick is now inside publish(), blocked

    # Shutdown begins while that tick is in flight, exactly as Bridge.close()
    # does: signal closed, then join with a bound.
    closed.set()
    thread.join(timeout=0.05)
    # The short join must not have succeeded: close() has not "returned"
    # while the in-flight tick is still inside publish().
    assert thread.is_alive()
    assert published == []

    release.set()  # let the in-flight tick's publish() finish
    thread.join(timeout=bridge_module.AUTHORITY_HEARTBEAT_JOIN_TIMEOUT_S)
    assert not thread.is_alive()
    assert published == ["authority-v1"]

    # Nothing publishes again after the thread (and so close()'s join) has
    # stopped: run()'s wait() saw `closed` already set and the loop exited
    # without a second tick.
    assert published == ["authority-v1"]
