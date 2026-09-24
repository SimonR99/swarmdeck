"""Actual MGG PCI + DDS + SwarmDeck controller, with an inert navigation sink.

Run in the MGG image on an isolated ROS domain. No robot commands are published.
"""

import subprocess
import threading
import time
from concurrent.futures import ThreadPoolExecutor
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from nav2_msgs.action import FollowPath
from mgg_msgs.srv import PlanObjective, PlannerSrv
from adapters.exploration import MggExploration, follow_path_goal
from adapters.objective_planning import MggObjectivePlanning


class FollowPathSink:
    """Production-format Nav2 action boundary without a velocity output."""

    def __init__(self, node, paths, cancellations):
        self.node = node
        self.id = "probe"
        self.cfg = {"link_timeout_s": 60}
        self.navigation_frame = "map"
        self.paths = paths
        self.cancellations = cancellations
        self._goal_generation = 0
        self._goal_lock = threading.RLock()
        self.nav_status = "idle"
        self.client = ActionClient(node, FollowPath, "/probe/follow_path")
        self.handle = None
        self.cancel_futures = []
        self.goal_errors = []

    def follow_path(
        self, plan, *, expected_generation=None, not_after=None, pre_submit=None
    ):
        request = follow_path_goal(plan)
        with self._goal_lock:
            if (
                expected_generation is not None
                and expected_generation != self._goal_generation
            ) or (not_after is not None and time.monotonic() >= not_after):
                return None
            if pre_submit is not None and not pre_submit():
                return None
            generation = (
                self.cancel_goal()
                if expected_generation is None
                else expected_generation
            )
            self.nav_status = "active"
            future = self.client.send_goal_async(request)

            def accepted(done):
                try:
                    handle = done.result()
                    with self._goal_lock:
                        if not handle.accepted:
                            self.goal_errors.append(
                                AssertionError(
                                    "native FollowPath sink rejected the path"
                                )
                            )
                        elif generation != self._goal_generation:
                            self._cancel_handle(handle)
                        else:
                            self.handle = handle
                except Exception as exc:
                    # Consume teardown races in rclpy's callback Tasks; the
                    # contract still fails errors observed before teardown.
                    self.goal_errors.append(exc)

            future.add_done_callback(accepted)
            return generation

    def _cancel_handle(self, handle):
        self.cancellations.append(True)
        self.cancel_futures.append(handle.cancel_goal_async())

    def cancel_goal(self):
        with self._goal_lock:
            self._goal_generation += 1
            if self.handle is not None:
                self._cancel_handle(self.handle)
                self.handle = None
            self.nav_status = "cancelled"
            return self._goal_generation

    def cancel_goal_if_current(self, expected_generation, *, pending=False):
        with self._goal_lock:
            if expected_generation != self._goal_generation:
                return None
            generation = self.cancel_goal()
            if pending:
                self.nav_status = "active"
            return generation

    def set_nav_status_if_current(self, expected_generation, status):
        with self._goal_lock:
            if expected_generation != self._goal_generation:
                return False
            self.nav_status = status
            return True

    def set_goal_pending_if_current(self, expected_generation):
        return self.set_nav_status_if_current(expected_generation, "active")

    def drive(self, *_):
        pass


def main():
    process = subprocess.Popen(
        [
            "ros2",
            "run",
            "mgg_pci",
            "mgg_pci_node",
            "--ros-args",
            "-r",
            "__ns:=/probe/mgg",
            "-p",
            "world_frame:=map",
            "-p",
            "bootstrap_distance:=0.0",
            "-p",
            "stuck_timeout_sec:=0.5",
            "-p",
            "auto_period_sec:=0.1",
        ],
        stdout=subprocess.DEVNULL,
    )
    rclpy.init()
    node = rclpy.create_node("swarmdeck_mgg_contract_probe")
    paths = []
    cancellations = []
    completed_paths = []
    action_group = ReentrantCallbackGroup()

    def execute(goal_handle):
        paths.append(goal_handle.request.path)
        while not goal_handle.is_cancel_requested:
            time.sleep(0.01)
        goal_handle.canceled()
        completed_paths.append(True)
        return FollowPath.Result()

    action_server = ActionServer(
        node,
        FollowPath,
        "/probe/follow_path",
        execute,
        callback_group=action_group,
        cancel_callback=lambda _: CancelResponse.ACCEPT,
    )
    bridge = FollowPathSink(node, paths, cancellations)
    explorer = MggExploration(bridge, {})

    outcome = {"status": 0, "delay": 0, "entered": False}

    def plan(request, response):
        outcome["entered"] = True
        time.sleep(outcome["delay"])
        response.status = outcome["status"]
        if outcome["status"] < 0:
            return response
        response.path = []
        for x, y in ((0.5, 0.25), (1.0, 0.5), (2.0, 1.0)):
            pose = Pose()
            pose.position.x, pose.position.y = x, y
            pose.orientation.w = 1.0
            response.path.append(pose)
        return response

    service = node.create_service(PlannerSrv, "/probe/mgg/mggplanner", plan)

    def objective_plan(request, response):
        response.status = PlanObjective.Response.SUCCEEDED
        response.component_id = request.component_id
        response.graph_revision = request.graph_revision or 7
        response.map_revision = request.map_revision or 9
        for x, y in ((0.0, 0.0), (0.75, 0.5), (1.5, 1.0)):
            point = Pose()
            point.position.x, point.position.y = x, y
            point.orientation.w = 1.0
            response.path.append(point)
        return response

    objective_service = node.create_service(
        PlanObjective, "/probe/mgg/plan_objective", objective_plan
    )
    publisher = node.create_publisher(Odometry, "/probe/mgg/odometry", 10)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    def wait_until(predicate, timeout=20):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            odom = Odometry()
            odom.header.frame_id = "map"
            odom.pose.pose.orientation.w = 1.0
            publisher.publish(odom)
            executor.spin_once(timeout_sec=0.1)
            if predicate():
                return
        raise AssertionError("MGG contract probe timed out")

    try:
        wait_until(
            lambda: explorer.start_client.service_is_ready()
            and explorer.stop_client.service_is_ready()
        )
        explorer.start()
        wait_until(
            lambda: bool(paths)
            and bridge.handle is not None
            and explorer.pending is None
        )
        assert explorer.active
        assert [
            (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
            for pose in paths[-1].poses
        ] == [(0.5, 0.25, 0.0), (1.0, 0.5, 0.0), (2.0, 1.0, 0.0)]
        explorer.stop()
        wait_until(lambda: explorer.pending_stop.done())
        assert not explorer.active and cancellations
        count = len(paths)
        end = time.monotonic() + 2
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.1)
        assert len(paths) == count
        for result, label in ((-2, "blocked"), (-3, "complete")):
            outcome["status"] = result
            explorer.start()
            wait_until(lambda: explorer.status == label and explorer.pending is None)
            assert not explorer.active
        objective = MggObjectivePlanning(
            bridge,
            {
                "namespace": "/probe/mgg",
                "frame": "map",
                "component_id": "component-a",
                "objective_timeout_s": 5.0,
            },
        )
        count = len(paths)
        with ThreadPoolExecutor(max_workers=1) as pool:
            planned = pool.submit(objective.navigate, {"x": 1.5, "y": 1.0})
            wait_until(
                lambda: planned.done()
                and len(paths) > count
                and bridge.handle is not None
                and bridge.handle.accepted
            )
            assert planned.result()
        assert bridge.handle is not None and bridge.handle.accepted
        assert len(paths[-1].poses) == 3
        bridge.cancel_goal()
        # The inert action server deliberately supplies no controller feedback
        # or odometry. PCI no-progress timing belongs to its native unit tests;
        # waiting for that wall-clock watchdog here made this transport smoke
        # depend on scheduler timing and concurrent accepted goals.
        outcome.update(status=0, delay=1.0, entered=False)
        count = len(paths)
        explorer.start()
        wait_until(lambda: outcome["entered"])
        explorer.stop()
        wait_until(lambda: explorer.pending is None and explorer.pending_stop.done())
        end = time.monotonic() + 1
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.1)
        assert len(paths) == count, "a late planner response escaped the stop gate"
        wait_until(
            lambda: all(cancel.done() for cancel in bridge.cancel_futures)
            and len(completed_paths) == len(paths)
        )
        for cancel in bridge.cancel_futures:
            cancel.result()
        assert not bridge.goal_errors, bridge.goal_errors
        end = time.monotonic() + 0.2
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.05)
        print(
            "PASS: native full-path PCI start/stop, planner terminal states, "
            "and late-response fencing"
        )
    finally:
        explorer.timer.cancel()
        if "objective" in locals():
            objective._authority_timer.cancel()
        # Stop the external client before destroying the local services/actions
        # it is using; otherwise rclpy can complete an in-flight response with
        # a Destroyable exception during interpreter teardown.
        process.terminate()
        process.wait(timeout=5)
        end = time.monotonic() + 0.2
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.05)
        # Drain executor threads before destroying handles that an already
        # scheduled callback can still be using.
        executor.shutdown()
        action_server.destroy()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
