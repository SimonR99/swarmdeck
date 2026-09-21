#!/usr/bin/env python3
"""Epoch fencing regression against the installed native Swarm-SLAM package.

Run in the rebuilt cslam image on an unused ROS_DOMAIN_ID:
  python3 adapters/test/ros/cslam_map_epoch_smoke.py

Exercises real descriptor stores, durable watermarks, native graph retirement,
MAC selection after early-index retirement, late closures/results, wrong
missions, and backend crash/restart replay. No server, simulation, pretrained
descriptor network, or registration mock needed.
"""

import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid
from unittest.mock import patch

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from cslam.algebraic_connectivity_maximization import (
    AlgebraicConnectivityMaximization,
    EdgeInterRobot,
)
from cslam.epoch_fence import EpochFence
from cslam.loop_closure_sparse_matching import LoopClosureSparseMatching
from cslam.mac.mac import MAC
from cslam_common_interfaces.msg import (
    InterRobotLoopClosure,
    KeyframeOdom,
    OptimizationResult,
    PoseGraph,
    PoseGraphValue,
    RobotHeartbeat,
    RobotIds,
)


def envelope(message, mission, publisher=0, epochs=(2, 0, 0)):
    message.mission_id = mission
    message.publisher_robot_id = publisher
    message.map_epoch = epochs[publisher]
    message.robot_map_epochs = list(epochs)
    return message


def frontend_regression(directory, mission):
    retired = []
    path = str(directory / "frontend-watermarks.json")
    fence = EpochFence(mission, 0, 2, 3, path, retired.append)
    fresh = envelope(RobotHeartbeat(), mission, 1, (2, 1, 0))
    fresh.robot_id = fresh.origin_robot_id = 1
    assert fence.heartbeat(fresh, 1)
    assert retired == [1]
    reopened = EpochFence(mission, 0, 3, 3, path)
    old = envelope(InterRobotLoopClosure(), mission, 2, (3, 0, 0))
    assert not reopened.accept(old, (0, 1))
    assert reopened.accept(old, (0, 2)), "unrelated closures must survive"
    mixed = envelope(InterRobotLoopClosure(), mission, 2, (3, 0, 4))
    assert not reopened.accept(mixed, (0, 1, 2))
    assert reopened.epochs == [3, 1, 0], "rejected compound input advanced a fence"
    wrong = copy.deepcopy(fresh)
    wrong.mission_id = str(uuid.uuid4())
    assert not reopened.heartbeat(wrong, 1)
    assert json.loads(Path(path).read_text())["robot_map_epochs"] == [3, 1, 0]

    matcher = LoopClosureSparseMatching(
        {
            "robot_id": 0,
            "max_nb_robots": 3,
            "frontend.sensor_type": "stereo",
            "frontend.enable_sparsification": False,
            "evaluation.enable_sparsification_comparison": False,
        }
    )
    vector = np.array([1.0, 0.0, 0.0])
    matcher.local_nnsm.add_item(vector, 7)
    matcher.other_robots_nnsm[1].add_item(vector, 8)
    matcher.other_robots_nnsm[2].add_item(vector, 9)
    selector = matcher.candidate_selector
    retired_edge = EdgeInterRobot(0, 7, 1, 8, 1.0)
    retained_edge = EdgeInterRobot(0, 7, 2, 9, 1.0)
    selector.candidate_edges_to_fixed([retired_edge, retained_edge])
    selector.add_candidate_edge(EdgeInterRobot(1, 10, 2, 9, 0.9))
    matcher.invalidate_robot(1)
    assert matcher.other_robots_nnsm[1].search_best(vector) == (None, None)
    assert matcher.local_nnsm.search_best(vector)[0] == 7
    assert matcher.other_robots_nnsm[2].search_best(vector)[0] == 9
    assert selector.fixed_edges == [retained_edge]
    selector.add_candidate_edge(retired_edge)
    assert list(selector.candidate_edges.values()) == [
        retired_edge
    ], "fresh run reused a retired reservation"


def algebraic_regression(directory, mission):
    selector = AlgebraicConnectivityMaximization(robot_id=1, max_nb_robots=4)
    fence = EpochFence(
        mission,
        1,
        0,
        4,
        str(directory / "selector-watermarks.json"),
        selector.invalidate_robot,
    )

    def receive(edge, epochs):
        message = envelope(InterRobotLoopClosure(), mission, 2, epochs)
        if not fence.accept(message, (edge.robot0_id, edge.robot1_id)):
            return False
        # The real closure callback also admits successful remote/replayed
        # measurements here, without requiring a locally seen candidate.
        selector.candidate_edges_to_fixed([edge])
        return True

    retired = EdgeInterRobot(0, 12, 1, 2, 1.0)
    retained = [EdgeInterRobot(1, 2, 2, 3, 1.0), EdgeInterRobot(2, 3, 3, 7, 1.0)]
    for edge in [retired, *retained]:
        assert receive(edge, (0, 0, 0, 0))
    selector.add_candidate_edge(EdgeInterRobot(0, 13, 2, 3, 0.9))
    candidates = [EdgeInterRobot(1, 0, 3, 1, 0.8), EdgeInterRobot(2, 0, 3, 0, 0.7)]
    for edge in candidates:
        selector.add_candidate_edge(edge)

    advanced = envelope(RobotHeartbeat(), mission, 0, (1, 0, 0, 0))
    advanced.robot_id = advanced.origin_robot_id = 0
    assert fence.heartbeat(advanced, 0)
    assert not receive(retired, (0, 0, 0, 0)), "late closure revived retired geometry"
    assert selector.fixed_edges == retained
    assert list(selector.candidate_edges.values()) == candidates

    solver_graphs = []
    original_solve = MAC.fw_subset

    def observe_solve(solver, *args, **kwargs):
        result = original_solve(solver, *args, **kwargs)
        solver_graphs.append(solver.L_odom.toarray())
        return result

    def select(considered, expected_poses):
        previous = len(solver_graphs)
        available = [
            edge
            for edge in selector.candidate_edges.values()
            if considered[edge.robot0_id] and considered[edge.robot1_id]
        ]
        selected = selector.select_candidates(1, considered)
        assert len(selected) == 1 and selected[0] in available
        # A greedy fallback or a swallowed solver failure must not pass.
        assert len(solver_graphs) == previous + 1, "real MAC solve did not complete"
        laplacian = solver_graphs[-1]
        assert laplacian.shape == (expected_poses, expected_poses)
        assert (
            np.linalg.eigvalsh(laplacian)[1] > 1e-6
        ), "phantom or disconnected solver rows"
        return selected

    with patch.object(MAC, "fw_subset", observe_solve):
        # Robot 0's old fourteen-pose chain is gone; later robots retain
        # 3 + 4 + 8 poses, including closures beyond every candidate index.
        included = dict.fromkeys(range(4), True)
        select(included, 15)

        # Reusing keyframe zero in the new epoch must not recover an old ID.
        assert receive(EdgeInterRobot(0, 0, 1, 0, 1.0), (1, 0, 0, 0))
        selector.add_candidate_edge(EdgeInterRobot(0, 0, 3, 2, 0.9))
        selected = select(included, 16)
        for edge in selected:
            assert edge.robot0_id != 0 or edge.robot0_keyframe_id == 0
            assert edge.robot1_id != 0 or edge.robot1_keyframe_id == 0

        # An out-of-range peer remains stored, but contributes no solver
        # rows or inferred odometry until it is included again.
        selector.add_candidate_edge(EdgeInterRobot(0, 0, 2, 2, 0.9))
        included[3] = False
        assert select(included, 8) == [EdgeInterRobot(0, 0, 2, 2, 0.9)]
        assert selector.fixed_edges[:2] == retained


def native_regression(directory, mission):
    rclpy.init()
    node = rclpy.create_node("map_epoch_regression")
    graphs, visualizations, heartbeats = [], [], []
    node.create_subscription(PoseGraph, "/cslam/pose_graph", graphs.append, 100)
    node.create_subscription(
        PoseGraph, "/cslam/viz/pose_graph", visualizations.append, 100
    )
    node.create_subscription(
        RobotHeartbeat, "/r0/cslam/heartbeat", heartbeats.append, 10
    )
    odom = node.create_publisher(KeyframeOdom, "/r0/cslam/keyframe_odom", 100)
    requests = node.create_publisher(RobotIds, "/r0/cslam/get_pose_graph", 100)
    closures = node.create_publisher(
        InterRobotLoopClosure,
        "/cslam/inter_robot_loop_closure",
        QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    peer = node.create_publisher(RobotHeartbeat, "/r1/cslam/heartbeat", 10)
    results = node.create_publisher(
        OptimizationResult, "/r0/cslam/optimized_estimates", 100
    )
    process = None
    log = (directory / "backend.log").open("w+")

    def spin_until(predicate, seconds=10):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if process.poll() is not None:
                log.flush()
                raise AssertionError((directory / "backend.log").read_text())
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError("native epoch condition timed out")

    def start():
        return subprocess.Popen(
            [
                "ros2",
                "run",
                "cslam",
                "pose_graph_manager",
                "--ros-args",
                "-r",
                "__ns:=/r0",
                "-p",
                "robot_id:=0",
                "-p",
                "max_nb_robots:=3",
                "-p",
                f"swarmdeck.mission_id:={mission}",
                "-p",
                "swarmdeck.map_epoch:=2",
                "-p",
                f"swarmdeck.epoch_state_path:={directory / 'native-watermarks'}",
                "-p",
                "neighbor_management.heartbeat_period_sec:=0.1",
                "-p",
                "backend.pose_graph_optimization_start_period_ms:=600000",
                "-p",
                "backend.enable_broadcast_tf_frames:=false",
                "-p",
                "visualization.enable:=true",
                "-p",
                "visualization.publishing_period_ms:=50",
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    def stop():
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)

    def keyframe(index, epochs=(2, 0, 0), message_mission=mission):
        msg = envelope(KeyframeOdom(), message_mission, 0, epochs)
        msg.id = index
        msg.odom.pose.pose.orientation.w = 1.0
        msg.odom.pose.pose.position.x = float(index)
        odom.publish(msg)

    def graph(epochs):
        previous = len(graphs)
        msg = envelope(RobotIds(), mission, 0, epochs)
        msg.ids = [0, 1, 2]
        requests.publish(msg)
        spin_until(lambda: len(graphs) > previous)
        return graphs[-1]

    def pairs(msg):
        return {(edge.key_from.robot_id, edge.key_to.robot_id) for edge in msg.edges}

    try:
        process = start()
        spin_until(
            lambda: odom.get_subscription_count() > 0
            and requests.get_subscription_count() > 0
            and closures.get_subscription_count() > 0
            and peer.get_subscription_count() > 0
            and results.get_subscription_count() > 0
        )
        keyframe(0)
        keyframe(1)
        for robot in (1, 2):
            msg = envelope(InterRobotLoopClosure(), mission, robot)
            msg.robot0_id, msg.robot1_id = 0, robot
            msg.success = True
            msg.transform.rotation.w = 1.0
            closures.publish(msg)
        spin_until(lambda: visualizations and len(visualizations[-1].edges) == 3)
        initial = graph((2, 0, 0))
        assert [value.key.keyframe_id for value in initial.values] == [0, 1]
        assert pairs(initial) == {(0, 0), (0, 1), (0, 2)}

        advanced = envelope(RobotHeartbeat(), mission, 1, (2, 1, 0))
        advanced.robot_id = advanced.origin_robot_id = 1
        peer.publish(advanced)
        spin_until(
            lambda: heartbeats and list(heartbeats[-1].robot_map_epochs) == [2, 1, 0]
        )
        current = graph((2, 1, 0))
        assert [value.key.keyframe_id for value in current.values] == [0, 1]
        assert pairs(current) == {(0, 0), (0, 2)}

        late = envelope(InterRobotLoopClosure(), mission, 1)
        late.robot0_id, late.robot1_id = 0, 1
        late.success = True
        late.transform.rotation.w = 1.0
        closures.publish(late)
        delayed = envelope(OptimizationResult(), mission, 1)
        delayed.success = True
        delayed.solution_clock = 1000000
        delayed.optimizer_robot_id = delayed.origin_robot_id = 1
        delayed.participant_robot_ids = [0, 1]
        value = PoseGraphValue()
        value.key.robot_id = 0
        value.pose.orientation.w = 1.0
        value.pose.position.x = 999.0
        delayed.estimates = [value]
        anchor = PoseGraphValue()
        anchor.key.robot_id = 1
        anchor.pose.orientation.w = 1.0
        delayed.anchor_estimates = [anchor]
        results.publish(delayed)
        keyframe(2, message_mission=str(uuid.uuid4()))
        keyframe(2, epochs=(1, 1, 0))
        previous = len(visualizations)
        spin_until(lambda: len(visualizations) > previous + 2)
        rejected = graph((2, 1, 0))
        assert [value.key.keyframe_id for value in rejected.values] == [0, 1]
        assert pairs(rejected) == {(0, 0), (0, 2)}
        assert visualizations[-1].values[0].pose.position.x == 0.0
        keyframe(2, epochs=(2, 1, 0))
        spin_until(lambda: len(visualizations[-1].edges) == 3)
        assert [value.key.keyframe_id for value in graph((2, 1, 0)).values] == [0, 1, 2]

        # A native process crash must not forget the observed peer watermark.
        # The launcher normally also advances its own epoch; keeping it fixed
        # here deliberately isolates and exercises durable peer fencing.
        stop()
        heartbeats.clear()
        process = start()
        spin_until(
            lambda: heartbeats and list(heartbeats[-1].robot_map_epochs) == [2, 1, 0]
        )
        spin_until(lambda: odom.get_subscription_count() > 0)
        keyframe(0, epochs=(2, 1, 0))
        previous = len(visualizations)
        spin_until(lambda: len(visualizations) > previous + 2)
        restored = graph((2, 1, 0))
        assert [value.key.keyframe_id for value in restored.values] == [0]
        assert pairs(restored) == {
            (0, 2)
        }, "stale closure replay resurrected a retired peer"
    finally:
        stop()
        log.close()
        node.destroy_node()
        rclpy.shutdown()


def main():
    mission = str(uuid.uuid4())
    with tempfile.TemporaryDirectory(prefix="cslam-map-epoch-") as temporary:
        directory = Path(temporary)
        frontend_regression(directory, mission)
        algebraic_regression(directory, mission)
        native_regression(directory, mission)
    print(
        "map epoch fencing: descriptor retirement, MAC reindexing, durable restart, native stale rejection passed"
    )


if __name__ == "__main__":
    main()
