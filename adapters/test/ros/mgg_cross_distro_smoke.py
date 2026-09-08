"""Jazzy native PCI / Humble adapter interoperability, using an inert goal sink.

Run `planner` in swarmdeck-mgg:local, `adapter` in ros:humble-perception,
sharing an isolated network namespace and ROS_DOMAIN_ID. No robot is involved.
"""

import subprocess
import sys
import time
from types import SimpleNamespace

import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose


def main():
    rclpy.init()
    node = rclpy.create_node("mgg_cross_distro_" + sys.argv[1])
    if sys.argv[1] == "planner":
        from mgg_msgs.srv import PlannerSrv

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
            ]
        )

        def plan(request, response):
            pose = Pose()
            pose.position.x, pose.position.y = 2.0, 1.0
            pose.orientation.w = 1.0
            response.path = [pose]
            return response

        service = node.create_service(PlannerSrv, "/probe/mgg/mggplanner", plan)
        pub = node.create_publisher(Odometry, "/probe/mgg/odometry", 10)
        timer = node.create_timer(0.1, lambda: pub.publish(Odometry()))
        try:
            rclpy.spin(node)
        finally:
            process.terminate()
            process.wait(timeout=5)
    else:
        from adapters.exploration import MggExploration

        goals, stops = [], []
        bridge = SimpleNamespace(
            node=node,
            id="probe",
            cfg={"link_timeout_s": 60},
            map_frame="map",
            navigate_to=goals.append,
            cancel_goal=lambda: stops.append(True),
            drive=lambda *_: None,
        )
        explorer = MggExploration(bridge, {})

        def wait_until(predicate):
            end = time.monotonic() + 25
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
                if predicate():
                    return
            raise AssertionError("cross-distribution request timed out")

        try:
            wait_until(
                lambda: explorer.start_client.service_is_ready()
                and explorer.stop_client.service_is_ready()
            )
            explorer.start()
            wait_until(lambda: bool(goals) and explorer.pending is None)
            assert explorer.active and goals[-1] == {"x": 2.0, "y": 1.0, "yaw": 0.0}
            explorer.stop()
            wait_until(lambda: explorer.pending_stop.done())
            count = len(goals)
            end = time.monotonic() + 2
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            assert not explorer.active and stops and len(goals) == count
            print("PASS: Humble adapter / Jazzy PCI start, path and stop")
        finally:
            explorer.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
