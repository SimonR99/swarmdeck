"""The bridge's optimizer-result statistics follow the core's adoption rule.

``deploy/autonomy/cslam_bridge.py`` imports the ROS stack at module scope,
but ``Bridge.optimized`` is pure bookkeeping over ``CslamMapper.solution``
and deserves to run under pytest: it drifted once already, counting
acceptance by the adopted order after the core moved acceptance to
``solver_order``, so ``solution_results_unchanged`` could never advance.
"""

from __future__ import annotations

import importlib
import json
import sys
import threading
import time
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


def test_write_text_if_changed_skips_the_write_when_nothing_changed(
    tmp_path, bridge_module
):
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


def test_bridge_start_removes_only_stale_unique_temporaries(tmp_path, bridge_module):
    """A crash between write and rename leaves a uniquely named temporary
    that no later writer reuses; the next bridge start deletes it, and
    never a temporary another process may still rename.
    """
    import os

    from autonomy.cslam import (
        STALE_TEMPORARY_AGE_S,
        remove_stale_unique_temporaries,
        write_unique_temporary,
    )

    names = bridge_module.BRIDGE_WRITTEN_FILES
    assert set(names) == {"snapshot.json", "status.json", "graph_solution.json"}
    stale = [write_unique_temporary(tmp_path / name, "{}") for name in names]
    fresh = write_unique_temporary(tmp_path / "status.json", "{}")
    unrelated = tmp_path / ".status.json.tmp"
    other_file = write_unique_temporary(tmp_path / "worker.json", "{}")
    (tmp_path / "mola").mkdir()
    nested = write_unique_temporary(tmp_path / "mola" / "status.json", "{}")
    unrelated.write_text("{}")
    now = fresh.stat().st_mtime
    old = now - STALE_TEMPORARY_AGE_S - 1.0
    for path in (*stale, unrelated, other_file, nested):
        os.utime(path, (old, old))

    removed = remove_stale_unique_temporaries(tmp_path, names, now=now)

    assert sorted(removed) == sorted(stale)
    assert not any(path.exists() for path in stale)
    assert all(path.exists() for path in (fresh, unrelated, other_file, nested))


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
    kwargs = dict(
        robot_id="robot_0",
        mission_id=mission,
        robot_map_epoch=0,
        run_id=run_id,
        cache_age_s=0.0,
    )

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
        cache_age_s=0.0,
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
    kwargs = dict(
        robot_id="robot_0",
        mission_id=mission,
        robot_map_epoch=0,
        run_id=run_id,
        cache_age_s=0.0,
    )

    assert accepts_authority_update(good, None)
    resent, _ = heartbeat(None, good, **kwargs)
    # Identical heartbeats are accepted, never treated as rollback.
    assert accepts_authority_update(resent, good)


def test_authority_heartbeat_stops_resending_after_the_resend_bound(bridge_module):
    """A stall that outlasts `AUTHORITY_RESEND_MAX_S` loses map authority.

    Dead sensors or TF used to publish ``resetting`` so MGG's snapshot and
    the server's live mapping expired; the re-send must not keep a robot
    with no fresh map authoritative forever.
    """
    from autonomy.map_epochs import robot_run_id

    heartbeat = bridge_module.authority_heartbeat
    bound = bridge_module.AUTHORITY_RESEND_MAX_S
    assert bound == 10.0
    mission = str(uuid.uuid4())
    run_id = robot_run_id(mission, "robot_0", 0)
    good = _authority(mission, run_id, 0, 7)
    identity = dict(
        robot_id="robot_0", mission_id=mission, robot_map_epoch=0, run_id=run_id
    )

    message, cache = heartbeat(None, good, **identity, cache_age_s=bound - 0.1)
    assert message is good and cache is good

    message, cache = heartbeat(None, good, **identity, cache_age_s=bound)
    assert message == {**identity, "state": "resetting"}
    assert cache is None

    # A fresh build is always published, whatever the age of the old cache.
    message, cache = heartbeat(good, None, **identity, cache_age_s=bound * 5)
    assert message is good and cache is good


def test_authority_heartbeat_publisher_sends_resetting_once_snapshot_stops_updating(
    bridge_module,
):
    """A `_snapshot` that never runs again cannot keep authority alive."""
    from autonomy.map_epochs import robot_run_id

    Publisher = bridge_module.AuthorityHeartbeatPublisher
    bound = bridge_module.AUTHORITY_RESEND_MAX_S
    published, now = [], [100.0]
    publisher = Publisher(
        published.append, period_s=1.0, log=lambda reason: None, clock=lambda: now[0]
    )
    mission = str(uuid.uuid4())
    run_id = robot_run_id(mission, "robot_0", 0)
    good = _authority(mission, run_id, 0, 7)
    publisher.update(good, fresh_at=now[0])

    now[0] += bound - 1.0
    publisher.tick()
    assert published[-1] is good

    now[0] += 1.0
    publisher.tick()
    assert published[-1] == {
        "robot_id": "robot_0",
        "mission_id": mission,
        "robot_map_epoch": 0,
        "run_id": run_id,
        "state": "resetting",
    }


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
        clock=lambda: now[0],
    )

    publisher.update("authority-v1")
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
        clock=lambda: now[0],
    )

    for _ in range(10):
        # `_snapshot` re-sends the same cached authority every tick because
        # a fresh one could not be built (e.g. sensor input stale), exactly
        # as `authority_heartbeat` decides.
        publisher.update("authority-v1")
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
        clock=lambda: now[0],
    )

    # Nothing published yet, and not a full period since construction: no
    # gap reported prematurely.
    publisher.tick()
    assert published == [] and logged == []

    now[0] += 2.0
    publisher.tick()
    assert published == []
    assert logged == ["2.0s between sends: no authority yet"]


def test_authority_heartbeat_publisher_stamps_completion_after_publish_returns(
    bridge_module,
):
    """The reviewer's counterexample: a publish call that itself blocks for
    a long time must not corrupt the *next* tick's gap measurement by being
    sampled before the call (which would show a falsely small gap for the
    tick that actually blocked, and a falsely large one for the unrelated
    tick right after it); the completion timestamp is sampled only after
    `_publish` returns.
    """

    Publisher = bridge_module.AuthorityHeartbeatPublisher
    published, logged, now = [], [], [0.0]
    calls = []

    def fake_publish(message):
        calls.append(message)
        published.append(message)
        if len(calls) == 2:
            # The second tick's own publish call blocks for 10 s, as a slow
            # write or a blocked DDS call might, advancing the clock while
            # still inside `_publish`.
            now[0] += 10.0

    publisher = Publisher(
        fake_publish,
        period_s=1.0,
        log=logged.append,
        clock=lambda: now[0],
    )
    publisher.update("authority-v1")

    now[0] += 1.0
    publisher.tick()  # completes at t=1
    assert published == ["authority-v1"]
    assert logged == []

    now[0] += 1.0
    publisher.tick()  # starts at t=2; `_publish` blocks until t=12
    assert published == ["authority-v1"] * 2
    # The true 11 s gap since the first completion (t=1) is caught here,
    # exactly because completion is stamped after `_publish` returns.
    assert logged == ["11.0s between sends: send blocked 10.0s (map epoch lock or DDS)"]

    now[0] += 1.0
    publisher.tick()  # starts at t=13; `_publish` is instant, completes at t=13
    assert published == ["authority-v1"] * 3
    # No new log entry: the true interval since the second tick's own
    # completion (t=12) is 1 s. A pre-publish timestamp would have compared
    # this tick's start (t=13) against the second tick's *pre-call* sample
    # (t=2) instead, falsely reporting an ~11 s gap that never happened.
    assert logged == ["11.0s between sends: send blocked 10.0s (map epoch lock or DDS)"]


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
        clock=lambda: now[0],
    )

    mission = str(uuid.uuid4())
    run_id0 = robot_run_id(mission, "robot_0", 0)
    good = _authority(mission, run_id0, 0, 7)
    kwargs0 = dict(
        robot_id="robot_0",
        mission_id=mission,
        robot_map_epoch=0,
        run_id=run_id0,
        cache_age_s=0.0,
    )

    message, cache = heartbeat(good, None, **kwargs0)
    publisher.update(message)
    publisher.tick()
    assert published[-1] == good

    # The build is momentarily gated (e.g. sensor stale): the cache is
    # resent verbatim, never silently dropped.
    message, cache = heartbeat(None, cache, **kwargs0)
    publisher.update(message)
    publisher.tick()
    assert published[-1] == good

    # A real epoch reset: the cache no longer names the current run, so the
    # decision falls through to `resetting`, and the publisher immediately
    # starts sending that instead, never the stale authority again.
    run_id1 = robot_run_id(mission, "robot_0", 1)
    kwargs1 = dict(
        robot_id="robot_0",
        mission_id=mission,
        robot_map_epoch=1,
        run_id=run_id1,
        cache_age_s=0.0,
    )
    message, cache = heartbeat(None, cache, **kwargs1)
    assert message["state"] == "resetting" and cache is None
    publisher.update(message)
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
        clock=lambda: now[0],
    )

    publisher.update("authority-v1")
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
    assert logged[-1].endswith(": map epoch retired")


def _claimed_bridge(tmp_path, bridge_module, monkeypatch):
    """A namespace standing in for `Bridge`, on a claimed epoch 0 under
    `tmp_path`, whose `authority_pub` records every published payload.
    Returns (bridge, mission, message) where `message` names epoch 0's run.
    """

    from autonomy.map_epochs import claim_map_epoch, robot_run_id

    monkeypatch.setattr(bridge_module, "String", lambda data: NS(data=data))
    mission = str(uuid.uuid4())
    claim_map_epoch(tmp_path, mission, "robot_0")  # epoch 0
    published = []
    bridge = NS(
        root=tmp_path / mission / "robot_0",
        authority_pub=NS(publish=lambda message: published.append(message.data)),
        published=published,
        _authority_send_lock=threading.Lock(),
        _authority_closed=False,
    )
    message = _authority(mission, robot_run_id(mission, "robot_0", 0), 0, 3)
    return bridge, mission, message


def test_send_authority_stops_once_a_new_epoch_is_claimed(
    tmp_path, bridge_module, monkeypatch
):
    """The real `Bridge._send_authority` and the real `claim_map_epoch`:
    no `Bridge.snapshot()` involved, so an old bridge whose executor is
    stalled still stops sending the moment the durable epoch moves on.
    """

    from autonomy.map_epochs import claim_map_epoch

    bridge, mission, message = _claimed_bridge(tmp_path, bridge_module, monkeypatch)
    send = bridge_module.Bridge._send_authority

    assert send(bridge, message) is None
    assert bridge.published == [json.dumps(message, allow_nan=False)]

    claim_map_epoch(tmp_path, mission, "robot_0")  # epoch 1 retires epoch 0
    assert send(bridge, message) == "map epoch retired"
    assert len(bridge.published) == 1


def test_send_authority_fails_closed_when_the_epoch_cannot_be_read(
    tmp_path, bridge_module, monkeypatch
):
    """A corrupt epoch record raises (the heartbeat reports it as a failed
    send), and an absent one is declined: never "probably still current".
    """

    bridge, _, message = _claimed_bridge(tmp_path, bridge_module, monkeypatch)
    send = bridge_module.Bridge._send_authority

    (bridge.root / "map-epoch.json").write_text("{not json")
    with pytest.raises(ValueError):
        send(bridge, message)
    (bridge.root / "map-epoch.json").unlink()
    assert send(bridge, message) == "no map epoch claimed"
    assert bridge.published == []

    logged, now = [], [0.0]
    publisher = bridge_module.AuthorityHeartbeatPublisher(
        lambda m: send(bridge, m), period_s=1.0, log=logged.append, clock=lambda: now[0]
    )
    (bridge.root / "map-epoch.json").write_text("{not json")
    publisher.update(message)
    now[0] += 2.0
    publisher.tick()
    assert bridge.published == []
    assert logged and logged[-1].startswith("2.0s between sends: send failed (")


def test_heartbeat_never_sends_a_message_selected_before_a_new_epoch_claim(
    tmp_path, bridge_module, monkeypatch
):
    """The heartbeat reads its cached epoch-0 message, then another thread
    claims epoch 1 before the send: barriers pin exactly that interleaving,
    and the send is declined because it re-reads the epoch under the lock.
    """

    from autonomy.map_epochs import claim_map_epoch

    bridge, mission, message = _claimed_bridge(tmp_path, bridge_module, monkeypatch)
    selected, claimed = threading.Event(), threading.Event()
    logged = []

    def send(selected_message):
        selected.set()  # tick() has read the cache
        assert claimed.wait(timeout=5)
        return bridge_module.Bridge._send_authority(bridge, selected_message)

    now = [0.0]
    publisher = bridge_module.AuthorityHeartbeatPublisher(
        send, period_s=1.0, log=logged.append, clock=lambda: now[0]
    )
    publisher.update(message)
    now[0] += 2.0
    thread = threading.Thread(target=publisher.tick, daemon=True)
    thread.start()
    assert selected.wait(timeout=5)
    claim_map_epoch(tmp_path, mission, "robot_0")  # epoch 1
    claimed.set()
    thread.join(timeout=5)

    assert not thread.is_alive()
    assert bridge.published == []
    assert logged == ["2.0s between sends: map epoch retired"]


def test_epoch_claim_waits_for_an_in_flight_send_then_retires_it(
    tmp_path, bridge_module, monkeypatch
):
    """A send in flight holds map_epoch_lock from its epoch check through
    its publish: a concurrent `claim_map_epoch()` blocks until the publish
    returns, and the next send is declined. The claim can never land
    between the check and the publish.
    """

    from autonomy.map_epochs import claim_map_epoch

    bridge, mission, message = _claimed_bridge(tmp_path, bridge_module, monkeypatch)
    publishing, release = threading.Event(), threading.Event()

    def blocking_publish(payload):
        publishing.set()
        assert release.wait(timeout=5)
        bridge.published.append(payload.data)

    bridge.authority_pub = NS(publish=blocking_publish)
    send = bridge_module.Bridge._send_authority
    sender = threading.Thread(target=send, args=(bridge, message), daemon=True)
    sender.start()
    assert publishing.wait(timeout=5)

    claimer = threading.Thread(
        target=claim_map_epoch, args=(tmp_path, mission, "robot_0"), daemon=True
    )
    claimer.start()
    claimer.join(timeout=0.2)
    assert claimer.is_alive()  # blocked on map_epoch_lock behind the send

    release.set()
    sender.join(timeout=5)
    claimer.join(timeout=5)
    assert not sender.is_alive() and not claimer.is_alive()
    assert len(bridge.published) == 1
    assert send(bridge, message) == "map epoch retired"
    assert len(bridge.published) == 1


def _map_epoch_lock_is_free(peer_root):
    """Probe `map_epoch_lock` from a separate open file, without waiting."""

    import fcntl

    with (peer_root / "map-epoch.lock").open("a") as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return False
        fcntl.flock(stream, fcntl.LOCK_UN)
        return True


def test_snapshot_builds_outside_map_epoch_lock_and_fences_each_write(
    tmp_path, bridge_module, monkeypatch
):
    """`Bridge.snapshot()` no longer holds map_epoch_lock while `_snapshot`
    reads the product and builds the authority (the heartbeat needs that
    lock before every send); each file write re-validates the epoch under
    the lock instead, and a retired epoch writes nothing.
    """

    from autonomy.map_epochs import claim_map_epoch

    bridge, mission, message = _claimed_bridge(tmp_path, bridge_module, monkeypatch)
    Bridge = bridge_module.Bridge
    bridge.core = NS(run_id=message["run_id"])
    bridge._authority_heartbeat = bridge_module.AuthorityHeartbeatPublisher(
        lambda m: None, period_s=1.0, log=lambda reason: None
    )
    status = bridge.root / "status.json"
    free_during_build, claim_mid_tick = [], []

    def _snapshot():
        free_during_build.append(_map_epoch_lock_is_free(bridge.root))
        bridge._authority_heartbeat.update(message)
        if claim_mid_tick:
            claim_map_epoch(tmp_path, mission, "robot_0")  # epoch 1, mid-tick
        bridge_module.write_text_if_changed(
            status,
            "tick",
            None,
            replace=lambda temporary, path: Bridge._replace_if_current(
                bridge, temporary, path
            ),
        )

    bridge._snapshot = _snapshot
    Bridge.snapshot(bridge)
    assert free_during_build == [True]
    assert status.read_text() == "tick"

    # A claim landing after the early epoch check: the fenced write
    # refuses, leaves no temporary behind, and the cache is invalidated.
    claim_mid_tick.append(True)
    Bridge.snapshot(bridge)
    assert free_during_build == [True, True]
    assert not status.exists()  # the claim deleted it; nothing recreated it
    assert list(bridge.root.glob(".status.json.*.tmp")) == []
    assert bridge._authority_heartbeat._message is None


def test_retired_and_current_bridge_writers_never_share_a_temporary(
    tmp_path, bridge_module
):
    """The review's interleaving: an epoch-0 bridge passed its early check,
    a claim moved to epoch 1, the epoch-1 bridge wrote its snapshot
    temporary and paused before its fenced rename, then the epoch-0 bridge
    wrote its own. Whichever rename runs first, snapshot.json holds only
    the current bytes, and the retired writer never deletes the current
    writer's temporary.
    """

    from autonomy.cslam import publish_snapshot_if_new
    from autonomy.map_epochs import claim_map_epoch, robot_run_id

    Bridge = bridge_module.Bridge
    mission = str(uuid.uuid4())
    root = tmp_path / mission / "robot_0"
    claim_map_epoch(tmp_path, mission, "robot_0")  # epoch 0
    claim_map_epoch(tmp_path, mission, "robot_0")  # epoch 1
    writers = {
        "current": NS(root=root, core=NS(run_id=robot_run_id(mission, "robot_0", 1))),
        "retired": NS(root=root, core=NS(run_id=robot_run_id(mission, "robot_0", 0))),
    }
    snapshot = root / "snapshot.json"

    def write(name, written, go, errors):
        def replace(temporary, path):
            written.set()  # temporary written, rename pending
            assert go.wait(timeout=5)
            Bridge._replace_if_current(writers[name], temporary, path)

        try:
            publish_snapshot_if_new(snapshot, {"writer": name}, 1, -1, replace=replace)
        except Exception as exc:
            errors.append(exc)

    for renames in (("current", "retired"), ("retired", "current")):
        snapshot.unlink(missing_ok=True)
        written = {name: threading.Event() for name in writers}
        go = {name: threading.Event() for name in writers}
        errors = {name: [] for name in writers}
        threads = {
            name: threading.Thread(
                target=write,
                args=(name, written[name], go[name], errors[name]),
                daemon=True,
            )
            for name in writers
        }
        threads["current"].start()
        assert written["current"].wait(timeout=5)
        threads["retired"].start()  # writes after the current writer did
        assert written["retired"].wait(timeout=5)
        for name in renames:
            go[name].set()
            threads[name].join(timeout=5)
            assert not threads[name].is_alive()

        assert errors["current"] == [], renames
        assert [type(error) for error in errors["retired"]] == [
            bridge_module.MapEpochRetired
        ]
        assert json.loads(snapshot.read_text()) == {"writer": "current"}, renames
        assert list(root.glob(".snapshot.json.*.tmp")) == []


def _running_bridge(tmp_path, bridge_module, monkeypatch, *, period_s=0.01):
    """`_claimed_bridge` plus what the real `Bridge.close()` touches: a
    heartbeat thread running the real `_send_authority`/`_log_authority_gap`,
    a recording logger, and stubbed sensor-domain teardown. `bridge.events`
    records publishes, gap logs, errors and teardown in order.
    """

    bridge, mission, message = _claimed_bridge(tmp_path, bridge_module, monkeypatch)
    Bridge = bridge_module.Bridge
    events = []
    bridge.events = events
    bridge.core = NS(run_id=message["run_id"])
    bridge.closed = threading.Event()
    bridge._authority_send_lock = threading.Lock()
    bridge._authority_closed = False
    bridge.authority_pub = NS(publish=lambda payload: events.append("publish"))
    bridge.get_logger = lambda: NS(
        warn=lambda text, **kwargs: events.append(("warn", text, kwargs)),
        error=lambda text: events.append(("error", text)),
    )
    bridge.sensor_executor = NS(shutdown=lambda timeout_sec: events.append("shutdown"))
    bridge.sensor_thread = None
    bridge.sensor_context = object()
    bridge.sensor_node = NS(destroy_node=lambda: events.append("destroy"))
    bridge._authority_heartbeat = bridge_module.AuthorityHeartbeatPublisher(
        lambda m: Bridge._send_authority(bridge, m),
        period_s=period_s,
        log=lambda reason: Bridge._log_authority_gap(bridge, reason),
    )
    bridge._authority_heartbeat.update(message)
    bridge.authority_heartbeat_thread = threading.Thread(
        target=bridge._authority_heartbeat.run, args=(bridge.closed,), daemon=True
    )
    return bridge, mission, message


def test_close_waits_for_an_in_flight_send_and_nothing_is_sent_after(
    tmp_path, bridge_module, monkeypatch
):
    """The real `Bridge.close()` with shutdown during a publish in flight:
    close() does not return until that publish does, and after it returns
    nothing is published or logged, before or after ROS teardown.
    """

    bridge, _, _ = _running_bridge(tmp_path, bridge_module, monkeypatch)
    publishing, release = threading.Event(), threading.Event()

    def blocking_publish(payload):
        publishing.set()
        assert release.wait(timeout=5)
        bridge.events.append("publish")

    bridge.authority_pub = NS(publish=blocking_publish)
    bridge.authority_heartbeat_thread.start()
    assert publishing.wait(timeout=5)

    closer = threading.Thread(
        target=bridge_module.Bridge.close, args=(bridge,), daemon=True
    )
    closer.start()
    closer.join(timeout=0.2)
    assert closer.is_alive()  # waiting on the publish in flight
    assert "destroy" not in bridge.events

    release.set()
    closer.join(timeout=5)
    assert not closer.is_alive()
    assert not bridge.authority_heartbeat_thread.is_alive()
    returned = list(bridge.events)
    assert returned.index("publish") < returned.index("destroy")

    time.sleep(0.1)  # ten heartbeat periods: nothing more happens
    assert bridge.events == returned
    assert [event for event in returned if event == "publish"] == ["publish"]


def test_close_after_a_join_timeout_leaves_the_thread_fenced(
    tmp_path, bridge_module, monkeypatch
):
    """The join-timeout path of the real `Bridge.close()`: the heartbeat is
    blocked on map_epoch_lock (held here, as a slow claim would), close()
    gives up joining, logs, and tears ROS down. Once the lock is released
    the thread passes its epoch check but publishes and logs nothing.
    """

    from autonomy.map_epochs import map_epoch_lock

    bridge, _, _ = _running_bridge(tmp_path, bridge_module, monkeypatch)
    monkeypatch.setattr(bridge_module, "AUTHORITY_HEARTBEAT_JOIN_TIMEOUT_S", 0.05)
    sending = threading.Event()
    heartbeat = bridge._authority_heartbeat
    real_send = heartbeat._send

    def send(message):
        sending.set()
        return real_send(message)

    heartbeat._send = send
    with map_epoch_lock(bridge.root):
        bridge.authority_heartbeat_thread.start()
        assert sending.wait(timeout=5)
        time.sleep(0.05)  # now blocked on map_epoch_lock
        bridge_module.Bridge.close(bridge)
        assert bridge.authority_heartbeat_thread.is_alive()
        assert bridge.events[0][0] == "error"
        assert bridge.events[1:] == ["shutdown", "destroy"]
    bridge.authority_heartbeat_thread.join(timeout=5)
    assert not bridge.authority_heartbeat_thread.is_alive()
    assert "publish" not in bridge.events
    assert not any(
        event[0] == "warn" for event in bridge.events if type(event) is tuple
    )


def test_cache_lock_send_lock_and_map_epoch_lock_never_deadlock(
    tmp_path, bridge_module, monkeypatch
):
    """Every path that takes more than one of the three locks, hammered at
    once: the heartbeat (cache lock, then map_epoch_lock -> send lock, then
    send lock to log), the main executor (cache lock via update/invalidate,
    map_epoch_lock via `_replace_if_current`), other processes' claims
    (map_epoch_lock) and `close()` (send lock). All finish within bounds.
    """

    from autonomy.map_epochs import claim_map_epoch

    bridge, mission, message = _running_bridge(
        tmp_path, bridge_module, monkeypatch, period_s=0.0005
    )
    Bridge = bridge_module.Bridge
    stop = threading.Event()
    target = bridge.root / "status.json"
    # Any thread dying on an exception would also leave it "not alive".
    thread_errors = []
    monkeypatch.setattr(
        threading, "excepthook", lambda args: thread_errors.append(args.exc_value)
    )

    def executor():
        while not stop.is_set():
            bridge._authority_heartbeat.update(message)
            temporary = bridge_module.write_unique_temporary(target, "tick")
            try:
                Bridge._replace_if_current(bridge, temporary, target)
            except bridge_module.MapEpochRetired:
                bridge._authority_heartbeat.invalidate()

    def claimer():
        for _ in range(20):
            claim_map_epoch(tmp_path, mission, "robot_0")

    workers = [threading.Thread(target=executor, daemon=True) for _ in range(2)]
    bridge.authority_heartbeat_thread.start()
    for worker in workers:
        worker.start()
    deadline = time.monotonic() + 5
    while "publish" not in bridge.events and time.monotonic() < deadline:
        time.sleep(0.001)
    workers.append(threading.Thread(target=claimer, daemon=True))
    workers[-1].start()
    time.sleep(0.2)
    closer = threading.Thread(target=Bridge.close, args=(bridge,), daemon=True)
    closer.start()
    closer.join(timeout=5)
    stop.set()
    for worker in workers:
        worker.join(timeout=5)
    assert not closer.is_alive()
    assert not any(worker.is_alive() for worker in workers)
    assert not bridge.authority_heartbeat_thread.is_alive()
    assert "publish" in bridge.events
    assert thread_errors == []
    # The heartbeat turns a failing send into a logged reason, not a crash.
    warnings = [event[1] for event in bridge.events if type(event) is tuple]
    assert not any("send failed" in text for text in warnings), warnings


def test_heartbeat_on_a_real_clock_logs_no_gap_while_sends_complete_on_time(
    bridge_module,
):
    """`run()` with the real monotonic clock and an instant send: waiting
    one period after each send makes every interval slightly longer than
    the period, which is scheduling jitter, not a gap MGG can observe.
    """

    sent, logged = [], []
    publisher = bridge_module.AuthorityHeartbeatPublisher(
        lambda message: sent.append(message), period_s=0.02, log=logged.append
    )
    publisher.update("authority-v1")
    closed = threading.Event()
    thread = threading.Thread(target=publisher.run, args=(closed,), daemon=True)
    thread.start()
    time.sleep(0.3)
    closed.set()
    thread.join(timeout=5)
    assert len(sent) >= 5
    assert logged == []


def test_heartbeat_gap_reason_separates_a_blocked_send_from_a_late_tick(
    bridge_module,
):
    """A real gap is logged once the interval between completed sends
    exceeds 1.5 periods, naming whether the send itself blocked (on
    map_epoch_lock or DDS) or the tick started late.
    """

    now, logged = [0.0], []
    blocking = []

    def send(message):
        if blocking:
            now[0] += blocking.pop()

    publisher = bridge_module.AuthorityHeartbeatPublisher(
        send, period_s=1.0, log=logged.append, clock=lambda: now[0]
    )
    publisher.update("authority-v1")
    now[0] += 1.0
    publisher.tick()
    now[0] += 1.4  # jitter below 1.5 periods: not a gap
    publisher.tick()
    assert logged == []

    now[0] += 1.0
    blocking.append(2.0)
    publisher.tick()
    assert logged == ["3.0s between sends: send blocked 2.0s (map epoch lock or DDS)"]

    now[0] += 4.0
    publisher.tick()
    assert (
        logged[-1]
        == "4.0s between sends: heartbeat tick started 4.0s after the last send"
    )


def test_gap_log_is_rate_limited_and_silent_after_close(
    tmp_path, bridge_module, monkeypatch
):
    bridge, _, _ = _running_bridge(tmp_path, bridge_module, monkeypatch)
    log = bridge_module.Bridge._log_authority_gap

    log(bridge, "3.0s between sends: map epoch retired")
    assert bridge.events == [
        (
            "warn",
            "map authority heartbeat gap (3.0s between sends: map epoch retired)",
            {"throttle_duration_sec": 5.0},
        )
    ]
    bridge._authority_closed = True
    log(bridge, "anything")
    assert len(bridge.events) == 1


def test_heartbeat_skips_a_tick_instead_of_stalling_behind_a_slow_lock_holder(
    tmp_path, bridge_module, monkeypatch
):
    """Another thread holds map_epoch_lock (a slow claim, worker fsync or
    watermark write): the tick gives up after `AUTHORITY_EPOCH_LOCK_WAIT_S`
    without sending, and the gap log names the cause.
    """

    from autonomy.map_epochs import map_epoch_lock

    bridge, _, message = _claimed_bridge(tmp_path, bridge_module, monkeypatch)
    monkeypatch.setattr(bridge_module, "AUTHORITY_EPOCH_LOCK_WAIT_S", 0.1)
    logged = []
    publisher = bridge_module.AuthorityHeartbeatPublisher(
        lambda m: bridge_module.Bridge._send_authority(bridge, m),
        period_s=0.2,
        log=logged.append,
    )
    publisher.update(message)
    holding, release = threading.Event(), threading.Event()

    def slow_holder():
        with map_epoch_lock(bridge.root):
            holding.set()
            release.wait(timeout=5)

    holder = threading.Thread(target=slow_holder, daemon=True)
    holder.start()
    assert holding.wait(timeout=5)
    time.sleep(0.3)  # more than 1.5 periods since the publisher started
    started = time.monotonic()
    publisher.tick()
    elapsed = time.monotonic() - started
    release.set()
    holder.join(timeout=5)

    assert 0.1 <= elapsed < 0.2  # bounded by the lock wait, not the holder
    assert bridge.published == []
    assert len(logged) == 1 and logged[0].endswith("s between sends: epoch lock busy")


def test_an_unserializable_candidate_never_evicts_the_last_good_authority(
    bridge_module,
):
    """A freshly built authority that cannot be sent (a NaN, which the wire
    JSON forbids) is rejected before it can replace the cache: the last
    good authority keeps being re-sent, or `resetting` when there is none.
    """

    heartbeat = bridge_module.authority_heartbeat
    mission = str(uuid.uuid4())
    run_id = str(uuid.uuid4())
    kwargs = dict(
        robot_id="robot_0",
        mission_id=mission,
        robot_map_epoch=0,
        run_id=run_id,
        cache_age_s=0.0,
    )
    good = _authority(mission, run_id, 0, 3)
    invalid = {
        **_authority(mission, run_id, 0, 4),
        "T_component_planning": [float("nan")],
    }

    message, cache = heartbeat(invalid, good, **kwargs)
    assert message is good and cache is good

    message, cache = heartbeat(invalid, None, **kwargs)
    assert message["state"] == "resetting" and cache is None


def event_bridge(module, monkeypatch):
    bridge = object.__new__(module.Bridge)
    bridge.pending_cloud = None
    bridge.pending_cloud_since = 0.0
    bridge.normalize_retry_s = 0.05
    bridge.normalize_retry_timer = None
    bridge.capture_retry_timer = None
    bridge.sensor_node = NS(destroy_timer=lambda timer: timer.cancel())
    bridge.destroy_timer = lambda timer: timer.cancel()
    bridge.timers = []

    def timer(node, period, callback):
        handle = NS(period=period, callback=callback, cancelled=False)
        handle.cancel = lambda: setattr(handle, "cancelled", True)
        bridge.timers.append(handle)
        return None, handle

    monkeypatch.setattr(module, "create_steady_timer", timer)
    bridge.get_logger = lambda: NS(warn=lambda *args, **kwargs: None)
    return bridge


def test_raw_cloud_normalizes_without_periodic_poll(bridge_module, monkeypatch):
    bridge = event_bridge(bridge_module, monkeypatch)
    normalized = []
    bridge.normalize = lambda: normalized.append(bridge.pending_cloud)
    cloud = object()
    bridge.raw_cloud(cloud)
    assert normalized == [cloud]
    assert bridge.timers == []


def test_missing_tf_retries_only_while_pending_and_expires(bridge_module, monkeypatch):
    bridge = event_bridge(bridge_module, monkeypatch)
    now = [10.0]
    monkeypatch.setattr(bridge_module.time, "monotonic", lambda: now[0])
    bridge.base, bridge.odom_frame = "base", "odom"

    def missing(*args):
        raise bridge_module.TransformException()

    bridge.tf = NS(lookup_transform=missing)
    cloud = NS(header=NS(stamp=NS(sec=0, nanosec=0), frame_id="lidar"))
    bridge.raw_cloud(cloud)
    assert bridge.normalize_retry_timer.period == 0.05
    first = bridge.normalize_retry_timer
    now[0] += 0.05
    first.callback()
    assert first.cancelled
    assert bridge.normalize_retry_timer.period == 0.1
    now[0] += bridge_module.AUTHORITY_SENSOR_TTL_S
    bridge.normalize_retry_timer.callback()
    assert bridge.pending_cloud is None
    assert bridge.normalize_retry_timer is None
    assert all(timer.cancelled for timer in bridge.timers)


def test_keyframe_join_flushes_on_events_and_arms_only_pending_deadline(
    bridge_module, monkeypatch
):
    bridge = event_bridge(bridge_module, monkeypatch)
    now = [10.0]
    monkeypatch.setattr(bridge_module.time, "monotonic", lambda: now[0])
    bridge.core = NS(mission_id="m", map_epoch=0)
    bridge.clouds, bridge.odoms, bridge.pending_capture_since = {}, {}, {}
    ready = [False]
    consumed = []

    def consume(seq):
        if seq in bridge.clouds and seq in bridge.odoms and ready[0]:
            bridge.clouds.pop(seq)
            bridge.odoms.pop(seq)
            bridge.pending_capture_since.pop(seq)
            consumed.append(seq)

    bridge.consume = consume
    bridge.key_cloud(NS(mission_id="m", map_epoch=0, id=1, pointcloud="cloud"))
    assert bridge.timers == []
    bridge.key_odom(NS(mission_id="m", map_epoch=0, id=1, odom="odom"))
    assert bridge.capture_retry_timer.period == bridge_module.RAW_CAPTURE_JOIN_GRACE_S
    waiting = bridge.capture_retry_timer
    ready[0] = True
    # This is also invoked by the raw-capture guard condition on the peer executor.
    bridge.flush_captures()
    assert consumed == [1]
    assert waiting.cancelled
    assert bridge.capture_retry_timer is None
    bridge.flush_captures()
    assert len(bridge.timers) == 1


def test_tf_recovery_cancels_retry_even_for_parked_cloud(bridge_module, monkeypatch):
    bridge = event_bridge(bridge_module, monkeypatch)
    now = [10.0]
    monkeypatch.setattr(bridge_module.time, "monotonic", lambda: now[0])
    bridge.base, bridge.odom_frame = "base", "odom"
    ready = [False]
    transform = NS(translation=NS(x=0.0, y=0.0, z=0.0), rotation=NS(x=0, y=0, z=0, w=1))

    def lookup(*args):
        if not ready[0]:
            raise bridge_module.TransformException()
        return NS(transform=transform)

    bridge.tf = NS(lookup_transform=lookup)
    bridge.last_normalized_pose = (0, 0, 0, 0, 0, 0, 1)
    bridge.last_normalized_at = 9.0
    bridge.captures_skipped_parked = 0
    bridge._shared_lock = threading.Lock()
    cloud = NS(header=NS(stamp=NS(sec=0, nanosec=0), frame_id="lidar"))
    bridge.raw_cloud(cloud)
    retry = bridge.normalize_retry_timer
    ready[0] = True
    now[0] += 0.05
    retry.callback()
    assert bridge.pending_cloud is None
    assert bridge.normalize_retry_timer is None
    assert retry.cancelled
    assert bridge.captures_skipped_parked == 1
    assert bridge.last_sensor_at == now[0]


def test_raw_capture_join_deadline_preserves_grace_and_cleans_timer(
    bridge_module, monkeypatch
):
    bridge = event_bridge(bridge_module, monkeypatch)
    now = [10.0]
    monkeypatch.setattr(bridge_module.time, "monotonic", lambda: now[0])
    bridge._shared_lock = threading.Lock()
    bridge.raw_capture_enabled = True
    bridge.raw_captures, bridge.raw_capture_metadata = {}, {}
    bridge.raw_capture_collisions = {}
    bridge.raw_capture_source_reset = False
    bridge.capture_calibrations = {}
    bridge.core = NS(mission_id="m", map_epoch=0)
    bridge.clouds, bridge.odoms, bridge.pending_capture_since = {}, {}, {}
    bridge.dropped = 0
    bridge.key_cloud(NS(mission_id="m", map_epoch=0, id=1, pointcloud="cloud"))
    odom = NS(header=NS(stamp=NS(sec=3, nanosec=0)))
    bridge.key_odom(NS(mission_id="m", map_epoch=0, id=1, odom=odom))
    assert bridge.clouds and bridge.odoms
    deadline = bridge.capture_retry_timer
    now[0] += bridge_module.RAW_CAPTURE_JOIN_GRACE_S
    deadline.callback()
    assert not bridge.clouds and not bridge.odoms
    assert not bridge.pending_capture_since
    assert bridge.dropped == 1  # no calibration: existing fail-closed behavior
    assert deadline.cancelled
    assert bridge.capture_retry_timer is None


def test_tf_retry_never_schedules_beyond_sensor_freshness_limit(
    bridge_module, monkeypatch
):
    bridge = event_bridge(bridge_module, monkeypatch)
    monkeypatch.setattr(bridge_module.time, "monotonic", lambda: 12.8)
    bridge.pending_cloud_since = 10.0
    bridge.normalize_retry_s = 1.0
    bridge._retry_normalize()
    assert bridge.normalize_retry_timer.period == pytest.approx(0.2)
