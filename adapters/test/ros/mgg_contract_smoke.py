"""Actual MGG PCI + DDS + SwarmDeck controller, with an inert navigation sink.

Run in the MGG image on an isolated ROS domain. No robot commands are published.
"""

import subprocess
import time
from types import SimpleNamespace
import rclpy
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from mgg_msgs.srv import PlannerSrv
from adapters.exploration import MggExploration


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
        ],
        stdout=subprocess.DEVNULL,
    )
    rclpy.init()
    node = rclpy.create_node("swarmdeck_mgg_contract_probe")
    goals = []
    cancellations = []
    bridge = SimpleNamespace(
        node=node,
        id="probe",
        cfg={"link_timeout_s": 60},
        map_frame="map",
        navigate_to=goals.append,
        cancel_goal=lambda: cancellations.append(True),
        drive=lambda *_: None,
    )
    explorer = MggExploration(bridge, {})

    def plan(request, response):
        pose = Pose()
        pose.position.x, pose.position.y = 2.0, 1.0
        pose.orientation.w = 1.0
        response.path = [pose]
        return response

    service = node.create_service(PlannerSrv, "/probe/mgg/mggplanner", plan)
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
        wait_until(lambda: bool(goals) and explorer.pending is None)
        assert explorer.active and goals[-1] == {"x": 2.0, "y": 1.0, "yaw": 0.0}
        explorer.stop()
        wait_until(lambda: explorer.pending_stop.done())
        assert not explorer.active and cancellations
        count = len(goals)
        end = time.monotonic() + 2
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.1)
        assert len(goals) == count
        print(
            "PASS: actual PCI start, goal delivery, stop acknowledgment, and no post-stop goals"
        )
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        process.wait(timeout=5)


if __name__ == "__main__":
    main()
