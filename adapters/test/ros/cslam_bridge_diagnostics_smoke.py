#!/usr/bin/env python3
"""Focused native regression for bridge optimizer-result diagnostics."""

from pathlib import Path
from types import SimpleNamespace as NS
import tempfile
from threading import Lock
import uuid

from builtin_interfaces.msg import Time as TimeMsg
import numpy as np

from adapters import reconstruction
from autonomy.contracts import IDENTITY_SE3
from autonomy.cslam import CslamMapper
from autonomy.mapping import CorrectionAwareMapper, SubmapStore
from deploy.autonomy import cslam_bridge
from deploy.autonomy.cslam_bridge import Bridge


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
        solution_clock=clock,
        optimizer_robot_id=0,
        origin_robot_id=0,
        estimates=[value(0, 0, x)],
        anchor_estimates=[value(0, 0, x)],
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


def main() -> None:
    verify_capture_time_color_transform()
    verify_consume_uses_paired_odom_capture_stamp()
    mission = str(uuid.uuid4())
    with tempfile.TemporaryDirectory(prefix="cslam-diagnostics-") as directory:
        core = CslamMapper(
            CorrectionAwareMapper(SubmapStore(Path(directory))),
            "robot_0",
            0,
            mission,
            {0: "robot_0"},
        )
        core.capture(0, 1, IDENTITY_SE3, [[1.0, 0.0, 0.0]])
        bridge = NS(
            core=core,
            solution_results_received=0,
            solution_results_accepted=0,
            solution_results_unchanged=0,
            solution_count=0,
            closure_candidates=0,
            verified_closures=0,
            rejected_closures=0,
            closures_by_peer={},
        )

        Bridge.inter_robot_closure(bridge, NS(robot0_id=0, robot1_id=1, success=False))
        assert bridge.closure_candidates == 0  # this single-peer bridge is uninvolved
        core.robot_names[1] = "r1"
        Bridge.inter_robot_closure(bridge, NS(robot0_id=0, robot1_id=1, success=False))
        Bridge.inter_robot_closure(bridge, NS(robot0_id=1, robot1_id=0, success=True))
        Bridge.inter_robot_closure(bridge, NS(robot0_id=1, robot1_id=2, success=True))
        assert bridge.closure_candidates == 2
        assert bridge.rejected_closures == 1
        assert bridge.verified_closures == 1
        assert bridge.closures_by_peer == {"r1": 1}

        wrong_mission = result(str(uuid.uuid4()), 10, 0.0)
        Bridge.optimized(bridge, wrong_mission)
        assert bridge.solution_results_received == 1
        assert bridge.solution_results_accepted == 0
        assert bridge.solution_results_unchanged == 0
        assert bridge.solution_count == 0
        assert core.solution_order == (0, -1)

        unchanged = result(mission, 1, 0.0)
        Bridge.optimized(bridge, unchanged)
        assert bridge.solution_results_received == 2
        assert bridge.solution_results_accepted == 1
        assert bridge.solution_results_unchanged == 1
        assert bridge.solution_count == 0
        assert core.solution_order == (1, 0)

        Bridge.optimized(bridge, unchanged)
        assert bridge.solution_results_received == 3
        assert bridge.solution_results_accepted == 1
        assert bridge.solution_results_unchanged == 1
        assert bridge.solution_count == 0
        assert core.solution_order == (1, 0)

        changed = result(mission, 2, 1.0)
        Bridge.optimized(bridge, changed)
        assert bridge.solution_results_received == 4
        assert bridge.solution_results_accepted == 2
        assert bridge.solution_results_unchanged == 1
        assert bridge.solution_count == 1
        assert core.solution_order == (2, 0)
        assert core.correction_revision == 1

    print(
        "PASS: wrong-mission/stale results rejected; accepted unchanged and "
        "pose-changing results counted separately"
    )


if __name__ == "__main__":
    main()
