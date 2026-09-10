"""MGG objective planning boundary without a ROS installation."""

from concurrent.futures import Future
import math
import sys
import threading
import time
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from adapters.objective_planning import MggObjectivePlanning


class Request:
    NAVIGATE = 1
    RETURN_HOME = 2

    def __init__(self):
        self.goal = pose(0, 0)
        self.mission_id = self.component_id = self.goal_landmark_id = ""
        self.graph_revision = self.map_revision = self.objective = 0
        self.map_epoch = self.mapping_graph_revision = 0
        self.geometry_revision = ""
        self.map_source_stamp = NS(sec=0, nanosec=0)


class Response:
    SUCCEEDED = 0


class Service:
    Request = Request
    Response = Response


def pose(x, y, z=0.4, yaw=0.0):
    return NS(
        position=NS(x=x, y=y, z=z),
        orientation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)),
    )


def rig(monkeypatch, response):
    package = ModuleType("mgg_msgs")
    services = ModuleType("mgg_msgs.srv")
    services.PlanObjective = Service
    monkeypatch.setitem(sys.modules, "mgg_msgs", package)
    monkeypatch.setitem(sys.modules, "mgg_msgs.srv", services)
    future = Future()
    future.set_result(response)
    client = Mock()
    client.wait_for_service.return_value = True
    client.call_async.return_value = future
    bridge = Mock()
    bridge.id = "robot_1"
    bridge.map_frame = "robot_1/map_frame"
    bridge.cfg = {}
    bridge._mapping_authority = NS(current=lambda: None)
    bridge._goal_generation = 4
    bridge._goal_lock = threading.RLock()
    bridge.node.create_client.return_value = client
    bridge.node.get_clock().now().to_msg.return_value = NS(sec=12, nanosec=34)

    def cancel():
        with bridge._goal_lock:
            bridge._goal_generation += 1
            bridge.nav_status = "cancelled"
            return bridge._goal_generation

    bridge.cancel_goal.side_effect = cancel

    def cancel_if_current(expected, *, pending=False):
        with bridge._goal_lock:
            if expected != bridge._goal_generation:
                return None
            generation = cancel()
            bridge.nav_status = "active" if pending else "cancelled"
            return generation

    def set_status(expected, status):
        with bridge._goal_lock:
            if expected != bridge._goal_generation:
                return False
            bridge.nav_status = status
            return True

    bridge.cancel_goal_if_current.side_effect = cancel_if_current
    bridge.set_nav_status_if_current.side_effect = set_status
    bridge.set_goal_pending_if_current.side_effect = lambda expected: set_status(
        expected, "active"
    )
    bridge.wait_goal_quiet.side_effect = (
        lambda expected, _deadline: expected == bridge._goal_generation
    )

    def follow(_plan, *, expected_generation=None, not_after=None, pre_submit=None):
        with bridge._goal_lock:
            if not_after is not None and time.monotonic() >= not_after:
                return None
            if (
                expected_generation is not None
                and expected_generation != bridge._goal_generation
            ):
                return None
            if pre_submit is not None and not pre_submit():
                return None
            bridge._goal_generation += 1
            bridge.nav_status = "active"
            return bridge._goal_generation

    bridge.follow_path.side_effect = follow
    planner = MggObjectivePlanning(
        bridge,
        {"component_id": "component-a", "objective_timeout_s": 0.1},
    )
    return bridge, planner, client


def success_path():
    return NS(
        status=Response.SUCCEEDED,
        component_id="component-a",
        graph_revision=7,
        map_revision=9,
        map_epoch=0,
        mapping_graph_revision=0,
        geometry_revision="",
        map_source_stamp=NS(sec=0, nanosec=0),
        reason="",
        path=[pose(0, 0, 0.4), pose(1, 0.5, 0.41), pose(2, 1, 0.39)],
    )


def test_navigate_requests_snapshot_and_executes_complete_path(monkeypatch):
    bridge, planner, client = rig(monkeypatch, success_path())
    assert planner.navigate({"x": 2, "y": 1, "yaw": 0.3})

    request = client.call_async.call_args.args[0]
    assert request.objective == Request.NAVIGATE
    assert request.component_id == "component-a"
    assert request.goal.position.x == 2
    assert request.goal.orientation.z == pytest.approx(math.sin(0.15))
    plan = bridge.follow_path.call_args.args[0]
    assert plan.revision_ns == 12_000_000_034
    assert [(p.x, p.y, p.z) for p in plan.poses] == [
        (0, 0, 0.4),
        (1, 0.5, 0.41),
        (2, 1, 0.39),
    ]


def test_return_home_uses_configured_goal(monkeypatch):
    bridge, planner, client = rig(monkeypatch, success_path())
    planner.home = {"x": -1, "y": -2}
    bridge.follow_path.return_value = True
    assert planner.return_home()
    request = client.call_async.call_args.args[0]
    assert request.objective == Request.RETURN_HOME
    assert request.goal.position.x == -1


def test_return_home_uses_corrected_authority_anchor(monkeypatch):
    response = success_path()
    response.geometry_revision = "a" * 64
    bridge, planner, client = rig(monkeypatch, response)
    bridge.follow_path.return_value = True
    authority = {
        "mission_id": "mission-1",
        "component_id": "component-a",
        "map_epoch": 0,
        "navigation_frame": "robot_1/map_frame",
        "mapping_graph_revision": 0,
        "geometry_revision": "a" * 64,
        "map_source_stamp": {"sec": 0, "nanosec": 0},
        "home": {
            "keyframe_id": "kf-home",
            "T_navigation_home": [
                [0.0, -1.0, 0.0, 3.0],
                [1.0, 0.0, 0.0, -2.0],
                [0.0, 0.0, 1.0, 0.4],
                [0.0, 0.0, 0.0, 1.0],
            ],
        },
    }
    planner.authority_reader = NS(current=lambda: authority)
    bridge.exploration = None
    # A stale caller pose must not override the corrected authority anchor.
    assert planner.return_home({"x": 99.0, "y": 99.0})
    request = client.call_async.call_args.args[0]
    assert request.mission_id == "mission-1"
    assert request.goal_landmark_id == "kf-home"
    assert request.goal.position.x == 3.0
    assert request.goal.position.y == -2.0
    assert request.goal.orientation.z == pytest.approx(math.sin(math.pi / 4.0))


def test_objective_binds_and_validates_indexed_map_snapshot(monkeypatch):
    response = success_path()
    digest = "a" * 64
    response.map_epoch = 11
    response.mapping_graph_revision = 12
    response.geometry_revision = digest
    response.map_source_stamp = NS(sec=13, nanosec=14)
    bridge, planner, client = rig(monkeypatch, response)
    bridge.follow_path.return_value = True
    planner.authority_reader = NS(
        current=lambda: {
            "component_id": "component-a",
            "map_epoch": 11,
            "mapping_graph_revision": 12,
            "geometry_revision": digest,
            "map_source_stamp": {"sec": 13, "nanosec": 14},
        }
    )
    assert planner.navigate({"x": 2, "y": 1})
    request = client.call_async.call_args.args[0]
    assert (request.map_epoch, request.mapping_graph_revision) == (11, 12)
    assert request.geometry_revision == digest
    assert (request.map_source_stamp.sec, request.map_source_stamp.nanosec) == (
        13,
        14,
    )

    response.geometry_revision = "b" * 64
    assert not planner.navigate({"x": 2, "y": 1})


def test_stop_generation_fences_late_service_result(monkeypatch):
    response = success_path()
    bridge, planner, client = rig(monkeypatch, response)

    class StoppingFuture:
        def done(self):
            bridge._goal_generation += 1
            return False

        def cancel(self):
            self.cancelled = True

    pending = StoppingFuture()
    client.call_async.return_value = pending
    assert not planner.navigate({"x": 1, "y": 2})
    assert pending.cancelled
    bridge.follow_path.assert_not_called()


def test_manual_command_while_initial_home_plan_is_pending_wins(monkeypatch):
    response = success_path()
    response.geometry_revision = "a" * 64
    bridge, planner, client = rig(monkeypatch, response)
    planner.authority_reader = NS(current=lambda: correction_authority())

    class ManualCommandFuture:
        replaced = False

        def done(self):
            if not self.replaced:
                self.replaced = True
                bridge.cancel_goal()
                bridge.nav_status = "active"
                bridge.mode = "teleop"
            return False

        def cancel(self):
            self.cancelled = True

    pending = ManualCommandFuture()
    client.call_async.return_value = pending

    assert not planner.return_home()
    assert pending.cancelled
    bridge.follow_path.assert_not_called()
    assert bridge.nav_status == "active"
    assert bridge.mode == "teleop"


def test_rejects_stale_component_and_nonplanar_result(monkeypatch):
    response = success_path()
    response.component_id = "component-b"
    bridge, planner, _ = rig(monkeypatch, response)
    assert not planner.navigate({"x": 1, "y": 2})
    bridge.follow_path.assert_not_called()


def correction_authority(
    revision=1,
    x=0.0,
    geometry="a" * 64,
    mapping_graph_revision=0,
    home_x=-2.0,
    mission_id="mission-1",
    landmark_id="kf-home",
    map_epoch=0,
    navigation_frame="robot_1/map_frame",
):
    return {
        "mission_id": mission_id,
        "component_id": "component-a",
        "navigation_frame": navigation_frame,
        "correction_revision": revision,
        "map_epoch": map_epoch,
        "mapping_graph_revision": mapping_graph_revision,
        "geometry_revision": geometry,
        "map_source_stamp": {"sec": 0, "nanosec": 0},
        "home": {
            "keyframe_id": landmark_id,
            "T_navigation_home": [
                [1.0, 0.0, 0.0, home_x],
                [0.0, 1.0, 0.0, 1.0],
                [0.0, 0.0, 1.0, 0.4],
                [0.0, 0.0, 0.0, 1.0],
            ],
        },
        "T_component_navigation": [
            [1.0, 0.0, 0.0, x],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
    }


def wait_until(predicate, timeout=0.5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return predicate()


def return_home_rig(monkeypatch):
    response = success_path()
    response.geometry_revision = "a" * 64
    bridge, planner, client = rig(monkeypatch, response)
    authority = [correction_authority()]
    planner.authority_reader = NS(current=lambda: authority[0])
    planner.replan_backoff_s = 0.0
    planner.replan_deadline_s = 0.25
    assert planner.return_home()
    bridge.nav_status = "active"
    return bridge, planner, client, authority


def test_correction_while_planning_blocks_route_execution(monkeypatch):
    response = success_path()
    response.geometry_revision = "a" * 64
    bridge, planner, client = rig(monkeypatch, response)
    authority = [correction_authority()]
    planner.authority_reader = NS(current=lambda: authority[0])

    class CorrectingFuture:
        corrected = False

        def done(self):
            if not self.corrected:
                authority[0] = correction_authority(revision=2, x=0.1)
                self.corrected = True
            return True

        def result(self):
            return response

    client.call_async.return_value = CorrectingFuture()
    assert not planner.navigate({"x": 1, "y": 2})
    bridge.follow_path.assert_not_called()


def test_active_objective_cancels_on_correction_or_authority_loss(monkeypatch):
    response = success_path()
    response.geometry_revision = "a" * 64
    bridge, planner, _ = rig(monkeypatch, response)
    bridge.follow_path.return_value = True
    authority = [correction_authority()]
    planner.authority_reader = NS(current=lambda: authority[0])
    assert planner.navigate({"x": 1, "y": 2})
    bridge.nav_status = "active"

    # correction_revision is authoritative even when this robot's anchor did
    # not move; another corrected submap can still invalidate the route.
    authority[0] = correction_authority(revision=2, x=0.0)
    planner._check_active_authority()
    bridge.cancel_goal_if_current.assert_called_once()
    assert bridge.nav_status == "failed"

    # A second route also fails closed when the once-valid authority expires.
    bridge.nav_status = "idle"
    authority[0] = correction_authority(revision=3, x=0.1)
    assert planner.navigate({"x": 1, "y": 2})
    bridge.nav_status = "active"
    authority[0] = None
    planner._check_active_authority()
    assert bridge.cancel_goal_if_current.call_count == 2


def test_active_objective_ignores_geometry_and_noop_correction_updates(monkeypatch):
    response = success_path()
    response.geometry_revision = "a" * 64
    bridge, planner, _ = rig(monkeypatch, response)
    bridge.follow_path.return_value = True
    authority = [correction_authority()]
    planner.authority_reader = NS(current=lambda: authority[0])
    assert planner.navigate({"x": 1, "y": 2})
    bridge.nav_status = "active"

    authority[0] = correction_authority(
        revision=1, geometry="b" * 64, mapping_graph_revision=99
    )
    planner._check_active_authority()

    assert bridge.cancel_goal.call_count == 1
    assert bridge.nav_status == "active"
    assert planner._active_route is not None


def test_new_goal_generation_wins_over_a_stale_authority_timer(monkeypatch):
    response = success_path()
    response.geometry_revision = "a" * 64
    bridge, planner, _ = rig(monkeypatch, response)
    bridge.follow_path.return_value = True
    initial = correction_authority()
    authority = [initial]
    planner.authority_reader = NS(current=lambda: authority[0])
    assert planner.navigate({"x": 1, "y": 2})
    bridge.nav_status = "active"

    replacement_generation = bridge._goal_generation + 1

    def replaced_while_timer_checked():
        bridge._goal_generation = replacement_generation
        return correction_authority(revision=2, x=0.1)

    planner.authority_reader = NS(current=replaced_while_timer_checked)
    prior_cancels = bridge.cancel_goal.call_count
    planner._check_active_authority()

    assert bridge.cancel_goal.call_count == prior_cancels
    assert bridge._goal_generation == replacement_generation


def test_reused_correction_revision_still_enforces_transform_tolerance(monkeypatch):
    response = success_path()
    response.geometry_revision = "a" * 64
    bridge, planner, _ = rig(monkeypatch, response)
    bridge.follow_path.return_value = True
    authority = [correction_authority()]
    planner.authority_reader = NS(current=lambda: authority[0])
    assert planner.navigate({"x": 1, "y": 2})
    bridge.nav_status = "active"

    authority[0] = correction_authority(revision=1, x=0.01)
    planner._check_active_authority()
    assert bridge.cancel_goal.call_count == 1
    authority[0] = correction_authority(revision=1, x=0.03)
    planner._check_active_authority()
    bridge.cancel_goal_if_current.assert_called_once()

    response = success_path()
    response.path[-1].position.x = 1.0
    response.path[-1].position.y = 0.5
    response.path[-1].position.z = 1.0
    bridge, planner, _ = rig(monkeypatch, response)
    assert not planner.navigate({"x": 1, "y": 2})
    bridge.follow_path.assert_not_called()


def test_return_home_replans_to_latest_authority_home_without_terminal_gap(
    monkeypatch,
):
    bridge, planner, client, authority = return_home_rig(monkeypatch)
    authority[0] = correction_authority(revision=2, x=0.03, home_x=-1.8)

    planner._check_active_authority()

    assert wait_until(lambda: client.call_async.call_count == 2)
    assert wait_until(lambda: planner._active_route is not None)
    retry = client.call_async.call_args.args[0]
    assert retry.goal.position.x == pytest.approx(-1.8)
    assert retry.mission_id == "mission-1"
    assert retry.goal_landmark_id == "kf-home"
    assert bridge.nav_status == "active"
    assert bridge.cancel_goal_if_current.call_count == 1


def test_return_home_coalesces_corrections_before_the_worker_plans(monkeypatch):
    bridge, planner, client, authority = return_home_rig(monkeypatch)
    planner.replan_backoff_s = 0.05
    authority[0] = correction_authority(revision=2, x=0.03, home_x=-1.8)

    planner._check_active_authority()
    worker = planner._recovery_thread
    authority[0] = correction_authority(revision=3, x=0.06, home_x=-1.5)
    # Enqueuing the same retained intent cannot create a second worker.
    assert planner._start_recovery(
        planner._recovery_generation, planner._home_intent, cancel_route=False
    )
    assert planner._recovery_thread is worker

    assert wait_until(lambda: client.call_async.call_count == 2)
    assert client.call_async.call_args.args[0].goal.position.x == pytest.approx(-1.5)
    bridge.cancel_goal()


def test_return_home_recovery_retries_then_fails_with_a_bound(monkeypatch):
    bridge, planner, client, authority = return_home_rig(monkeypatch)
    planner.replan_max_attempts = 2
    client.wait_for_service.side_effect = [False, False]
    authority[0] = correction_authority(revision=2, x=0.03)

    planner._check_active_authority()

    assert wait_until(lambda: bridge.nav_status == "failed")
    assert client.wait_for_service.call_count == 3  # initial plan plus two retries
    assert client.call_async.call_count == 1
    warning = bridge.node.get_logger().warning.call_args.args[0]
    assert "exhausted after 2 attempts" in warning


def test_return_home_recovery_retries_synchronous_service_submission(monkeypatch):
    bridge, planner, client, authority = return_home_rig(monkeypatch)
    retry_response = success_path()
    retry_response.geometry_revision = "a" * 64
    ready = Future()
    ready.set_result(retry_response)
    client.call_async.side_effect = [RuntimeError("transport reset"), ready]
    authority[0] = correction_authority(revision=2, x=0.03)

    planner._check_active_authority()

    assert wait_until(lambda: client.call_async.call_count == 3)
    assert wait_until(lambda: planner._active_route is not None)
    assert bridge.nav_status == "active"


@pytest.mark.parametrize(
    "replacement",
    [
        {"mission_id": "mission-2"},
        {"landmark_id": "kf-other"},
        {"map_epoch": 1},
        {"navigation_frame": "robot_1/replacement_map"},
    ],
)
def test_return_home_authority_identity_change_fails_closed(monkeypatch, replacement):
    bridge, planner, client, authority = return_home_rig(monkeypatch)
    authority[0] = correction_authority(
        mission_id=replacement.get("mission_id", "mission-1"),
        landmark_id=replacement.get("landmark_id", "kf-home"),
        map_epoch=replacement.get("map_epoch", 0),
        navigation_frame=replacement.get("navigation_frame", "robot_1/map_frame"),
    )

    planner._check_active_authority()

    assert bridge.nav_status == "failed"
    assert client.call_async.call_count == 1
    assert planner._recovery_thread is None


def test_stop_during_return_home_recovery_backoff_wins_without_status_write(
    monkeypatch,
):
    bridge, planner, client, authority = return_home_rig(monkeypatch)
    planner.replan_backoff_s = 0.08
    authority[0] = correction_authority(revision=2, x=0.03)
    planner._check_active_authority()

    bridge.cancel_goal()
    bridge.nav_status = "idle"  # Stop's public terminal state.

    assert wait_until(lambda: planner._recovery_thread is None)
    assert client.call_async.call_count == 1
    assert bridge.nav_status == "idle"


def test_manual_navigate_during_recovery_backoff_replaces_home_intent(monkeypatch):
    bridge, planner, client, authority = return_home_rig(monkeypatch)
    planner.replan_backoff_s = 0.08
    authority[0] = correction_authority(revision=2, x=0.03)
    planner._check_active_authority()

    assert planner.navigate({"x": 4.0, "y": 5.0})

    assert wait_until(lambda: planner._recovery_thread is None)
    assert client.call_async.call_count == 2
    assert client.call_async.call_args.args[0].objective == Request.NAVIGATE
    assert bridge.nav_status == "active"
    assert planner._home_intent is None


def test_stop_while_recovery_service_future_is_pending_wins(monkeypatch):
    bridge, planner, client, authority = return_home_rig(monkeypatch)

    class StoppingFuture:
        stopped = False

        def done(self):
            if not self.stopped:
                self.stopped = True
                bridge.cancel_goal()
                bridge.nav_status = "idle"
            return False

        def cancel(self):
            self.cancelled = True

    pending = StoppingFuture()
    client.call_async.return_value = pending
    authority[0] = correction_authority(revision=2, x=0.03)

    planner._check_active_authority()

    assert wait_until(lambda: getattr(pending, "cancelled", False))
    assert wait_until(lambda: planner._recovery_thread is None)
    assert bridge.follow_path.call_count == 1
    assert bridge.nav_status == "idle"


def test_recovery_retries_when_indexed_snapshot_changes_during_planning(monkeypatch):
    bridge, planner, client, authority = return_home_rig(monkeypatch)
    response_a = success_path()
    response_a.geometry_revision = "a" * 64
    response_b = success_path()
    response_b.geometry_revision = "b" * 64

    class SnapshotChangingFuture:
        changed = False

        def done(self):
            if not self.changed:
                self.changed = True
                authority[0] = correction_authority(
                    revision=2, x=0.03, geometry="b" * 64
                )
            return True

        def result(self):
            return response_a

    ready_b = Future()
    ready_b.set_result(response_b)
    client.call_async.side_effect = [SnapshotChangingFuture(), ready_b]
    authority[0] = correction_authority(revision=2, x=0.03)

    planner._check_active_authority()

    assert wait_until(lambda: client.call_async.call_count == 3)
    assert wait_until(lambda: planner._active_route is not None)
    assert client.call_async.call_args.args[0].geometry_revision == "b" * 64
    assert bridge.nav_status == "active"


def test_pre_submit_authority_validation_blocks_snapshot_churn(monkeypatch):
    response = success_path()
    response.geometry_revision = "a" * 64
    bridge, planner, _ = rig(monkeypatch, response)
    authority = [correction_authority()]
    planner.authority_reader = NS(current=lambda: authority[0])
    original_follow = bridge.follow_path.side_effect

    def churn_before_submit(plan, **kwargs):
        authority[0] = correction_authority(geometry="b" * 64)
        return original_follow(plan, **kwargs)

    bridge.follow_path.side_effect = churn_before_submit

    assert not planner.navigate({"x": 1.0, "y": 2.0})
    assert bridge._goal_generation == 5
    assert bridge.nav_status == "failed"
