"""Exploration command/state races without ROS or robot motion."""

import asyncio
from concurrent.futures import Future
from types import SimpleNamespace as NS
from unittest.mock import Mock
import time

from adapters.exploration import MggExploration
from adapters.session import dispatch_command, _stop_on_disconnect


def rig():
    bridge = Mock()
    bridge.id = "r"
    bridge.cfg = {"link_timeout_s": 5}
    bridge.node.get_clock().now().nanoseconds = 1000000000
    explorer = MggExploration.__new__(MggExploration)
    explorer.bridge = bridge
    explorer.active = False
    explorer.generation = 0
    explorer.started_ns = 0
    explorer.pending = None
    explorer.pending_stop = None
    explorer.stop_deadline = 0
    explorer.deadline = 0
    explorer.last_link = time.monotonic()
    explorer.frame = "map"
    explorer.request_type = lambda: None
    explorer.start_client = Mock()
    explorer.stop_client = Mock()
    explorer.start_client.service_is_ready.return_value = True
    explorer.stop_client.service_is_ready.return_value = True
    explorer.start_client.call_async.return_value = Future()
    explorer.stop_client.call_async.return_value = Future()
    bridge.exploration = explorer
    return bridge, explorer


def path(frame="map", stamp=2, empty=False):
    pose = NS(position=NS(x=1.0, y=2.0), orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))
    return NS(
        header=NS(frame_id=frame, stamp=NS(sec=stamp, nanosec=0)),
        poses=[] if empty else [NS(pose=pose)],
    )


def test_start_stop_and_late_paths():
    bridge, explorer = rig()
    explorer.start()
    explorer.start()  # Idempotent while starting.
    explorer.start_client.call_async.assert_called_once()
    explorer.on_path(path())
    bridge.navigate_to.assert_called_once_with({"x": 1.0, "y": 2.0, "yaw": 0.0})
    explorer.stop()
    explorer.on_path(path(stamp=3))
    explorer.start_client.call_async.return_value.set_result(NS(success=True))
    assert not explorer.active
    bridge.navigate_to.assert_called_once()
    bridge.drive.assert_called_with(0.0, 0.0)
    explorer.stop_client.call_async.assert_called_once()


def test_latched_wrong_frame_empty_and_lost_link():
    bridge, explorer = rig()
    explorer.on_path(path())
    bridge.navigate_to.assert_not_called()
    explorer.start()
    explorer.on_path(path(stamp=0))
    bridge.navigate_to.assert_not_called()
    explorer.on_path(path(frame="other_map"))
    assert not explorer.active
    for failure in ("empty", "link", "timeout"):
        bridge, explorer = rig()
        explorer.start()
        if failure == "empty":
            explorer.on_path(path(empty=True))
        elif failure == "link":
            explorer.last_link = time.monotonic() - 10
            explorer.tick()
        else:
            explorer.deadline = 0
            explorer.tick()
        assert not explorer.active
        bridge.cancel_goal.assert_called()


def test_stop_all_dispatch_disables_exploration_first():
    bridge, explorer = rig()
    explorer.start()

    async def run():
        await dispatch_command(bridge, {"type": "stop"}, asyncio.get_running_loop())

    asyncio.run(run())
    assert not explorer.active
    bridge.stop.assert_called_once()


def test_failed_start_and_disconnect_do_not_leave_autonomy_active():
    bridge, explorer = rig()
    explorer.start()
    explorer.pending.set_result(NS(success=False, message="no path"))
    assert not explorer.active
    bridge, explorer = rig()
    explorer.start()
    _stop_on_disconnect(bridge)
    assert not explorer.active


def test_stop_wins_during_navigation_dispatch():
    bridge, explorer = rig()
    explorer.start()
    bridge.navigate_to.side_effect = lambda goal: explorer.stop()
    explorer.on_path(path())
    assert not explorer.active
    bridge.drive.assert_called_with(0.0, 0.0)


def test_start_waits_for_previous_requests_and_service_availability():
    bridge, explorer = rig()
    explorer.stop_client.service_is_ready.return_value = False
    explorer.start()
    assert not explorer.active
    explorer.stop_client.service_is_ready.return_value = True
    explorer.start()
    explorer.stop()
    explorer.start()
    assert not explorer.active
    explorer.start_client.call_async.assert_called_once()


def test_stop_service_failure_still_cancels_motion():
    bridge, explorer = rig()
    explorer.start()
    explorer.stop_client.call_async.side_effect = RuntimeError("DDS unavailable")
    explorer.stop()
    assert not explorer.active
    bridge.cancel_goal.assert_called()
    bridge.drive.assert_called_with(0.0, 0.0)


def test_manual_goal_preempts_exploration():
    bridge, explorer = rig()
    explorer.start()
    goal = {"x": 3, "y": 4}

    async def run():
        await dispatch_command(
            bridge, {"type": "navigate_to", "goal": goal}, asyncio.get_running_loop()
        )

    asyncio.run(run())
    assert not explorer.active
    bridge.navigate_to.assert_called_once()
    assert bridge.navigate_to.call_args.args[0] == goal


def test_orphaned_requests_expire_after_planner_restart():
    bridge, explorer = rig()
    explorer.start()
    abandoned_start = explorer.pending
    explorer.stop()
    abandoned_stop = explorer.pending_stop
    explorer.deadline = explorer.stop_deadline = 0
    explorer.tick()
    assert abandoned_start.cancelled() and abandoned_stop.cancelled()
    assert explorer.pending is None and explorer.pending_stop is None
    explorer.on_path(path())
    bridge.navigate_to.assert_not_called()
    explorer.start_client.call_async.return_value = Future()
    explorer.start()
    assert explorer.active


def test_terminal_status_survives_either_dds_delivery_order():
    import json

    for status_first in (True, False):
        bridge, explorer = rig()
        explorer.start()
        message = NS(data=json.dumps({"state": "blocked", "stamp_ns": 2000000000}))
        calls = [
            lambda: explorer.on_status(message),
            lambda: explorer.on_path(path(empty=True)),
        ]
        for call in calls if status_first else reversed(calls):
            call()
        assert not explorer.active
        assert explorer.status == "blocked"
        explorer.stop_client.call_async.assert_not_called()
        bridge.cancel_goal.assert_called()


def test_manual_stop_and_old_status_cannot_relabel_a_session():
    import json

    _, explorer = rig()
    explorer.start()
    explorer.on_status(NS(data=json.dumps({"state": "complete", "stamp_ns": 1})))
    assert explorer.active
    explorer.stop()
    explorer.on_status(
        NS(data=json.dumps({"state": "complete", "stamp_ns": 2000000000}))
    )
    assert explorer.status == "stopped"
