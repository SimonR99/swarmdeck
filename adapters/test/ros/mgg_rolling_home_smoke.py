"""Actual rclpy/DDS contract smoke for rolling Return Home.

Run with the generated mgg_msgs overlay on an isolated ROS domain. The goal
sink is inert: this verifies service serialization, adapter ownership, and
local-chunk handoff without publishing a robot command.
"""

from __future__ import annotations

import threading
import time

import rclpy
from geometry_msgs.msg import Pose
from rclpy.executors import MultiThreadedExecutor

from adapters.objective_planning import MggObjectivePlanning
from mgg_msgs.srv import PlanObjective, RefineObjectiveRoute


class Authority:
    def __init__(self):
        self.value = {
            "mission_id": "mission-contract",
            "component_id": "shared-component",
            "navigation_frame": "map",
            "planning_frame": "map",
            "solution_order": [1, 0],
            "correction_revision": 1,
            "map_epoch": 3,
            "mapping_graph_revision": 11,
            "geometry_revision": "a" * 64,
            "map_source_stamp": {"sec": 12, "nanosec": 34},
            "T_component_navigation": identity(),
            "T_component_planning": identity(),
            "home": {
                "keyframe_id": "kf-home",
                "T_navigation_home": [
                    [1.0, 0.0, 0.0, -2.0],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0],
                ],
            },
        }

    def current(self):
        return self.value


def identity():
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def append_poses(target, xs):
    for x in xs:
        pose = Pose()
        pose.position.x = x
        pose.orientation.w = 1.0
        target.append(pose)


class InertBridge:
    def __init__(self, node, authority):
        self.node = node
        self.id = "probe"
        self.cfg = {}
        self.navigation_frame = "map"
        self._mapping_authority = authority
        self._goal_lock = threading.RLock()
        self._goal_generation = 0
        self.nav_status = "idle"
        self.paths = []

    def cancel_goal(self):
        with self._goal_lock:
            self._goal_generation += 1
            self.nav_status = "cancelled"
            return self._goal_generation

    def cancel_goal_if_current(self, expected_generation, *, pending=False):
        with self._goal_lock:
            if expected_generation != self._goal_generation:
                return None
            generation = self.cancel_goal()
            self.nav_status = "active" if pending else "cancelled"
            return generation

    def set_goal_pending_if_current(self, expected_generation):
        return self.set_nav_status_if_current(expected_generation, "active")

    def set_nav_status_if_current(self, expected_generation, status):
        with self._goal_lock:
            if expected_generation != self._goal_generation:
                return False
            self.nav_status = status
            return True

    def wait_goal_quiet(self, expected_generation, not_after):
        return (
            expected_generation == self._goal_generation
            and time.monotonic() < not_after
        )

    def follow_path(
        self, plan, *, expected_generation=None, not_after=None, pre_submit=None
    ):
        with self._goal_lock:
            if expected_generation != self._goal_generation:
                return None
            if not_after is not None and time.monotonic() >= not_after:
                return None
            if pre_submit is not None and not pre_submit():
                return None
            self.paths.append(plan)
            self.nav_status = "active"
            return self._goal_generation


def main():
    rclpy.init()
    node = rclpy.create_node("swarmdeck_rolling_home_contract")
    authority = Authority()
    requests = []
    errors = []

    def plan(request, response):
        try:
            assert request.objective == PlanObjective.Request.RETURN_HOME
            assert request.mission_id == authority.value["mission_id"]
            assert request.component_id == authority.value["component_id"]
            response.status = PlanObjective.Response.SUCCEEDED
            response.component_id = request.component_id
            response.graph_revision = 7
            response.map_revision = 9
            response.map_epoch = request.map_epoch
            response.mapping_graph_revision = request.mapping_graph_revision
            response.geometry_revision = request.geometry_revision
            response.map_source_stamp = request.map_source_stamp
            response.indexed_map_validated = False
            response.partial = True
            response.route_id = "route-contract-1"
            append_poses(response.path, (0.0, -0.5))
            append_poses(response.global_path, (0.0, -0.5, -1.0, -2.0))
        except Exception as exc:
            errors.append(exc)
        return response

    def refine(request, response):
        try:
            requests.append(request)
            assert request.mission_id == authority.value["mission_id"]
            assert request.component_id == authority.value["component_id"]
            assert request.route_id == "route-contract-1"
            assert request.graph_revision == request.map_revision == 0
            assert request.map_epoch == authority.value["map_epoch"]
            assert (
                request.mapping_graph_revision
                == authority.value["mapping_graph_revision"]
            )
            assert request.geometry_revision == authority.value["geometry_revision"]
            assert request.map_source_stamp.sec == 12
            assert request.map_source_stamp.nanosec == 34
            response.status = RefineObjectiveRoute.Response.SUCCEEDED
            response.component_id = request.component_id
            response.graph_revision = 7 + len(requests)
            response.map_revision = 9 + len(requests)
            response.map_epoch = request.map_epoch
            response.mapping_graph_revision = request.mapping_graph_revision
            response.geometry_revision = request.geometry_revision
            response.map_source_stamp = request.map_source_stamp
            response.indexed_map_validated = False
            response.partial = len(requests) == 1
            append_poses(
                response.path,
                (-0.5, -1.2) if response.partial else (-1.2, -2.0),
            )
        except Exception as exc:
            errors.append(exc)
        return response

    plan_service = node.create_service(PlanObjective, "/probe/mgg/plan_objective", plan)
    refine_service = node.create_service(
        RefineObjectiveRoute, "/probe/mgg/refine_objective_route", refine
    )
    bridge = InertBridge(node, authority)
    planner = MggObjectivePlanning(
        bridge,
        {
            "namespace": "/probe/mgg",
            "frame": "map",
            "component_id": "local-component",
            "objective_timeout_s": 5.0,
            "authority_replan_backoff_s": 0.0,
        },
    )
    # Drive continuation explicitly below; avoid racing the background timer.
    planner._authority_timer.cancel()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def wait_until(predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        raise AssertionError("rolling Home DDS smoke timed out")

    try:
        wait_until(
            lambda: planner.client.service_is_ready()
            and planner.refine_client.service_is_ready()
        )
        assert planner.return_home()
        assert len(bridge.paths) == 1
        fixed_global = planner.global_display_plan()
        assert fixed_global is not None and fixed_global.poses[-1].x == -2.0
        fixed_points = tuple((pose.x, pose.y) for pose in fixed_global.poses)
        state = planner.decorate_state({"nav_status": "active"})
        assert state["objective_continuation"] == {
            "objective": "return_home",
            "evidence_source": "mgg_native",
            "phase": "following_local",
        }

        bridge.nav_status = "succeeded"
        planner._check_active_authority()
        wait_until(lambda: len(requests) == 1 and len(bridge.paths) == 2)
        assert bridge.paths[-1].poses[-1].x == -1.2
        assert (
            tuple((pose.x, pose.y) for pose in planner.global_display_plan().poses)
            == fixed_points
        )

        bridge.nav_status = "succeeded"
        planner._check_active_authority()
        wait_until(lambda: len(requests) == 2 and len(bridge.paths) == 3)
        assert bridge.paths[-1].poses[-1].x == -2.0
        assert (
            tuple((pose.x, pose.y) for pose in planner.global_display_plan().poses)
            == fixed_points
        )
        assert (
            planner.decorate_state({"nav_status": "active"})["objective_continuation"][
                "phase"
            ]
            == "following_final"
        )

        bridge.nav_status = "succeeded"
        planner._check_active_authority()
        assert planner.global_display_plan() is None
        assert not errors, errors
        print(
            "PASS: actual DDS PlanObjective + two RefineObjectiveRoute calls, "
            "native evidence, shared component, and fixed global Home route"
        )
    finally:
        planner._authority_timer.cancel()
        executor.shutdown(timeout_sec=5.0)
        thread.join(timeout=5.0)
        executor.remove_node(node)
        plan_service.destroy()
        refine_service.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
