"""MGG objective planning boundary without a ROS installation."""

from concurrent.futures import Future
import math
import sys
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
        orientation=NS(
            x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
        ),
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
    bridge.node.create_client.return_value = client
    bridge.node.get_clock().now().to_msg.return_value = NS(sec=12, nanosec=34)

    def cancel():
        bridge._goal_generation += 1

    bridge.cancel_goal.side_effect = cancel
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
    bridge, planner, client = rig(monkeypatch, success_path())
    bridge.follow_path.return_value = True
    authority = {
        "mission_id": "mission-1",
        "component_id": "component-a",
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


def test_rejects_stale_component_and_nonplanar_result(monkeypatch):
    response = success_path()
    response.component_id = "component-b"
    bridge, planner, _ = rig(monkeypatch, response)
    assert not planner.navigate({"x": 1, "y": 2})
    bridge.follow_path.assert_not_called()


def correction_authority(
    revision=1, x=0.0, geometry="a" * 64, mapping_graph_revision=0
):
    return {
        "mission_id": "mission-1",
        "component_id": "component-a",
        "correction_revision": revision,
        "map_epoch": 0,
        "mapping_graph_revision": mapping_graph_revision,
        "geometry_revision": geometry,
        "map_source_stamp": {"sec": 0, "nanosec": 0},
        "T_component_navigation": [
            [1.0, 0.0, 0.0, x],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
    }


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
    assert bridge.cancel_goal.call_count == 2
    assert bridge.nav_status == "failed"

    # A second route also fails closed when the once-valid authority expires.
    bridge.nav_status = "idle"
    authority[0] = correction_authority(revision=3, x=0.1)
    assert planner.navigate({"x": 1, "y": 2})
    bridge.nav_status = "active"
    authority[0] = None
    planner._check_active_authority()
    assert bridge.cancel_goal.call_count == 4


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
    assert bridge.cancel_goal.call_count == 2

    response = success_path()
    response.path[-1].position.x = 1.0
    response.path[-1].position.y = 0.5
    response.path[-1].position.z = 1.0
    bridge, planner, _ = rig(monkeypatch, response)
    assert not planner.navigate({"x": 1, "y": 2})
    bridge.follow_path.assert_not_called()
