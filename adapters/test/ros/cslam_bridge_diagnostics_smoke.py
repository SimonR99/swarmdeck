#!/usr/bin/env python3
"""Focused native regression for bridge optimizer-result diagnostics."""

from pathlib import Path
from types import SimpleNamespace as NS
import tempfile
from threading import Event, Lock
import time
import uuid

from builtin_interfaces.msg import Time as TimeMsg
import numpy as np
import rclpy

from adapters import reconstruction  # cslam_bridge imports colorize from here
from autonomy.contracts import IDENTITY_SE3
from autonomy.cslam import CslamMapper
from autonomy.mapping import CorrectionAwareMapper, SubmapStore
from deploy.autonomy import cslam_bridge
from deploy.autonomy.cslam_bridge import Bridge
from rclpy.clock import ClockType
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter


def value(robot: int, seq: int, x: float):
    return NS(
        key=NS(robot_id=robot, keyframe_id=seq),
        pose=NS(
            position=NS(x=x, y=0.0, z=0.0),
            orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def result(mission: str, clock: int, x: float):
    return NS(
        success=True,
        mission_id=mission,
        publisher_robot_id=0,
        map_epoch=4,
        robot_map_epochs=[4, 0, 0],
        participant_robot_ids=[0],
        solution_clock=clock,
        optimizer_robot_id=0,
        origin_robot_id=0,
        estimates=[value(0, 0, x)],
        anchor_estimates=[value(0, 0, x)],
    )


def closure(mission: str, first: int, second: int, success: bool):
    epochs = [4, 0, 0]
    return NS(
        mission_id=mission,
        publisher_robot_id=first,
        map_epoch=epochs[first],
        robot_map_epochs=epochs,
        robot0_id=first,
        robot1_id=second,
        success=success,
    )


def verify_capture_time_color_transform() -> None:
    cloud_stamp = TimeMsg(sec=10, nanosec=0)
    image_stamp = TimeMsg(sec=10, nanosec=100_000_000)
    camera_header = NS(stamp=image_stamp, frame_id="camera")
    tf_calls = []

    class FakeTf:
        def lookup_transform_full(self, *args):
            tf_calls.append(args)
            return NS(
                transform=NS(
                    translation=NS(x=1.0, y=2.0, z=3.0),
                    rotation=NS(x=0.0, y=0.0, z=0.0, w=1.0),
                )
            )

    image = NS(header=camera_header)
    depth = NS(header=camera_header)
    bridge = NS(
        color_images={10_100_000_000: image},
        depth_images={10_100_000_000: depth},
        color_info=NS(header=camera_header),
        color_frame="",
        color_frame_convention="body",
        color_capture_attempts=0,
        color_pairs_selected=0,
        color_pair_rejections=0,
        color_tf_rejections=0,
        color_projection_rejections=0,
        colored_captures=0,
        _shared_lock=Lock(),
        tf=FakeTf(),
        base="base_link",
        odom_frame="odom",
    )
    observed = {}

    def fake_colorize(points, image, depth, info, camera_from_points):
        observed["points"] = points
        observed["transform"] = camera_from_points
        return np.asarray([[9, 8, 7, 255]], dtype=np.uint8)

    original = reconstruction.colorize_ros_rgbd
    reconstruction.colorize_ros_rgbd = fake_colorize
    try:
        points = np.asarray([[0.0, 0.0, 1.0]], dtype=np.float32)
        colors = Bridge._capture_colors(
            bridge,
            points,
            NS(stamp=cloud_stamp, frame_id="lidar"),
        )
    finally:
        reconstruction.colorize_ros_rgbd = original

    assert colors.tolist() == [[9, 8, 7, 255]]
    assert observed["points"] is points
    assert len(tf_calls) == 1
    target, target_time, source, source_time, fixed = tf_calls[0]
    assert (target, source, fixed) == ("camera", "base_link", "odom")
    assert target_time.nanoseconds == 10_100_000_000
    assert source_time.nanoseconds == 10_000_000_000
    expected = np.asarray(
        [
            [0.0, -1.0, 0.0, -2.0],
            [0.0, 0.0, -1.0, -3.0],
            [1.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    assert np.allclose(observed["transform"], expected)
    assert bridge.color_capture_attempts == 1
    assert bridge.color_pairs_selected == 1
    assert bridge.colored_captures == 1
    assert bridge.color_pair_rejections == 0
    assert bridge.color_tf_rejections == 0
    assert bridge.color_projection_rejections == 0


def verify_consume_uses_paired_odom_capture_stamp() -> None:
    """Swarm-SLAM recreates its cloud with Header(); odometry keeps scan time."""
    zero_header = NS(stamp=TimeMsg(), frame_id="")
    odom_header = NS(stamp=TimeMsg(sec=10), frame_id="odom")
    observed = {}
    bridge = NS(
        clouds={7: NS(header=zero_header)},
        odoms={
            7: NS(
                header=odom_header,
                pose=NS(
                    pose=NS(
                        position=NS(x=0.0, y=0.0, z=0.0),
                        orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0),
                    )
                ),
            )
        },
        pending_capture_since={7: 0.0},
        raw_capture_enabled=False,
        capture_calibrations={10_000_000_000: (np.eye(4), "lidar")},
        _shared_lock=Lock(),
        dropped=0,
        capture_count=0,
        core=NS(
            key=lambda seq: ("r0", seq),
            capture=lambda *args, **kwargs: False,
        ),
        _take_raw_capture=lambda stamp, key: None,
        _capture_colors=lambda points, header: observed.setdefault("header", header),
    )
    original = cslam_bridge.point_cloud2.read_points_numpy
    cslam_bridge.point_cloud2.read_points_numpy = lambda *args, **kwargs: np.asarray(
        [[1.0, 0.0, 0.0]],
        dtype=np.float32,
    )
    try:
        Bridge.consume(bridge, 7)
    finally:
        cslam_bridge.point_cloud2.read_points_numpy = original
    assert observed["header"] is odom_header
    assert observed["header"] is not zero_header


def verify_authority_heartbeat_uses_steady_time() -> None:
    context = Context()
    rclpy.init(context=context)
    node = Node(
        "cslam_authority_steady_timer_smoke",
        context=context,
        # Keep this clock frozen even if a simulator shares the test's domain.
        use_global_arguments=False,
        cli_args=[
            "--ros-args",
            "-r",
            f"/clock:=/cslam_timer_smoke_{uuid.uuid4().hex}/clock",
        ],
        parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, True),
        ],
        automatically_declare_parameters_from_overrides=True,
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    fired = Event()
    clock, timer = cslam_bridge.create_steady_timer(node, 0.02, fired.set)
    try:
        assert node.get_clock().clock_type == ClockType.ROS_TIME
        assert node.get_clock().now().nanoseconds == 0
        assert clock.clock_type == ClockType.STEADY_TIME
        deadline = time.monotonic() + 1.0
        while not fired.is_set() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert fired.is_set(), "steady timer did not fire with frozen simulated time"
        assert node.get_clock().now().nanoseconds == 0
    finally:
        node.destroy_timer(timer)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)

    assert cslam_bridge.sensor_input_is_fresh(98.0, now=100.0)
    assert not cslam_bridge.sensor_input_is_fresh(97.0, now=100.0)
    assert not cslam_bridge.sensor_input_is_fresh(0.0, now=1.0)


def main() -> None:
    verify_authority_heartbeat_uses_steady_time()
    verify_capture_time_color_transform()
    verify_consume_uses_paired_odom_capture_stamp()
    mission = str(uuid.uuid4())
    now = [100.0]
    with tempfile.TemporaryDirectory(prefix="cslam-diagnostics-") as directory:
        core = CslamMapper(
            CorrectionAwareMapper(SubmapStore(Path(directory))),
            "robot_0",
            0,
            mission,
            {0: "robot_0", 1: "r1", 2: "r2"},
            clock=lambda: now[0],
            map_epoch=4,
        )
        core.capture(0, 1, IDENTITY_SE3, [[1.0, 0.0, 0.0]])
        bridge = NS(
            core=core,
            solution_results_received=0,
            solution_results_accepted=0,
            solution_results_unchanged=0,
            solution_results_deferred=0,
            solution_count=0,
            closure_candidates=0,
            verified_closures=0,
            rejected_closures=0,
            closures_by_peer={},
        )

        Bridge.inter_robot_closure(bridge, closure(mission, 0, 3, False))
        assert bridge.closure_candidates == 0  # unknown robot is outside this fleet
        Bridge.inter_robot_closure(bridge, closure(mission, 0, 1, False))
        Bridge.inter_robot_closure(bridge, closure(mission, 1, 0, True))
        Bridge.inter_robot_closure(bridge, closure(mission, 1, 2, True))
        assert bridge.closure_candidates == 2
        assert bridge.rejected_closures == 1
        assert bridge.verified_closures == 1
        assert bridge.closures_by_peer == {"r1": 1}

        wrong_mission = result(str(uuid.uuid4()), 10, 0.0)
        Bridge.optimized(bridge, wrong_mission)
        assert bridge.solution_results_received == 1
        assert bridge.solution_results_accepted == 0
        assert bridge.solution_results_unchanged == 0
        assert bridge.solution_results_deferred == 0
        assert bridge.solution_count == 0
        assert core.solver_order == (0, -1)

        # Accepted and unchanged: the solver clock is remembered for
        # ordering, the adopted frame keeps its order.
        unchanged = result(mission, 1, 0.0)
        Bridge.optimized(bridge, unchanged)
        assert bridge.solution_results_received == 2
        assert bridge.solution_results_accepted == 1
        assert bridge.solution_results_unchanged == 1
        assert bridge.solution_count == 0
        assert core.solver_order == (1, 0)
        assert core.solution_order == (0, -1)

        Bridge.optimized(bridge, unchanged)
        assert bridge.solution_results_received == 3
        assert bridge.solution_results_accepted == 1
        assert bridge.solution_results_unchanged == 1
        assert bridge.solution_count == 0
        assert core.solver_order == (1, 0)

        changed = result(mission, 2, 1.0)
        Bridge.optimized(bridge, changed)
        assert bridge.solution_results_received == 4
        assert bridge.solution_results_accepted == 2
        assert bridge.solution_results_unchanged == 1
        assert bridge.solution_count == 1
        assert core.solution_order == (2, 0)
        assert core.correction_revision == 1

        # A 6 cm refinement three seconds after an adoption is held by the
        # adoption interval: accepted and deferred, not adopted.
        now[0] += 3.0
        Bridge.optimized(bridge, result(mission, 3, 1.06))
        assert bridge.solution_results_received == 5
        assert bridge.solution_results_accepted == 3
        assert bridge.solution_results_unchanged == 1
        assert bridge.solution_results_deferred == 1
        assert bridge.solution_count == 1
        assert core.solver_order == (3, 0)
        assert core.solution_order == (2, 0)
        assert core.deferred_solution is not None

        now[0] += 7.0
        Bridge.optimized(bridge, result(mission, 4, 1.06))
        assert bridge.solution_results_accepted == 4
        assert bridge.solution_results_deferred == 1
        assert bridge.solution_count == 2
        assert core.solution_order == (4, 0)
        assert core.deferred_solution is None

    print(
        "PASS: wrong-mission/stale results rejected; accepted unchanged, "
        "deferred and adopted results counted separately"
    )


if __name__ == "__main__":
    main()
