import json
import struct

import numpy as np

from swarmdeck_protocol import decode_keyframe, encode_keyframe
from swarmdeck_slam.fault_injection import (
    generate_faulty_odometry,
    replace_wire_odometry,
    wire_body,
)


class _Packet:
    def __init__(self, robot_id: str, seq: int) -> None:
        self.robot_id = robot_id
        self.session = "run"
        self.seq = seq
        self.stamp = float(seq)
        yaw = 0.02 * seq
        self.t_odom_base = np.array(
            [0.2 * seq, 0.0, 0.0, 0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)]
        )


def test_fault_history_is_deterministic_independent_and_severe() -> None:
    packets = [
        _Packet(robot, seq)
        for robot in ("robot_0", "robot_1")
        for seq in range(40)
    ]

    first, report = generate_faulty_odometry(packets, seed=19)
    second, _ = generate_faulty_odometry(packets, seed=19)

    assert all(np.array_equal(a, b) for a, b in zip(first, second))
    assert not np.array_equal(first[0], first[40])
    assert all(abs(np.linalg.norm(pose[3:]) - 1.0) < 1e-12 for pose in first)
    assert all(len(item["events"]) == 4 for item in report["trajectories"])
    assert all(item["max_reported_step_m"] > 2.0 for item in report["trajectories"])
    assert all(
        item["max_reported_step_yaw_deg"] > 20.0
        for item in report["trajectories"]
    )


def test_wire_rewrite_preserves_cloud_and_descriptor_body_exactly() -> None:
    points = np.array([[1.0, 2.0, 0.3], [-2.0, 0.5, 0.7]], dtype=np.float64)
    original = encode_keyframe(
        robot_id="bot",
        session="run",
        seq=2,
        stamp=3.0,
        points=points,
        t_odom_base=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    )
    replacement = np.array([9.0, -8.0, 0.0, 0.0, 0.0, 0.5, np.sqrt(0.75)])

    corrupted = replace_wire_odometry(original, replacement)
    packet = decode_keyframe(corrupted)

    assert wire_body(corrupted) == wire_body(original)
    assert np.array_equal(packet.points, decode_keyframe(original).points)
    assert np.allclose(packet.t_odom_base, replacement)

    header_struct = struct.Struct("<4sHI")
    header_length = header_struct.unpack_from(corrupted)[2]
    header = json.loads(
        corrupted[header_struct.size:header_struct.size + header_length]
    )
    assert header["robot_id"] == "bot"
