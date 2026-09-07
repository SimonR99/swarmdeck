"""Cloud color and coordinate contracts without running a solver."""

from types import SimpleNamespace as NS
import zlib
import numpy as np
from swarmdeck_protocol import encode_keyframe, decode_keyframe
from swarmdeck_slam.backend import keyframe_from_packet
import swarmdeck_slam.service as svc


def test_cloud_keeps_camera_rgb_and_world_frame_for_local_scope(monkeypatch):
    colors = np.array([[250, 20, 10, 255]], dtype=np.uint8)
    frames = {}
    for seq, rgba in enumerate([None, colors]):
        packet = decode_keyframe(
            encode_keyframe(
                robot_id="r",
                seq=seq,
                stamp=seq,
                points=np.array([[1, 2, 0.5]]),
                t_odom_base=[0, 0, 0, 0, 0, 0, 1],
                colors=rgba,
            )
        )
        frame = keyframe_from_packet(packet)
        frames[frame.id] = frame
    world = np.array(
        [[0, -1, 0, 10], [1, 0, 0, 20], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float
    )
    backend = NS(_keyframes=frames, render=NS(odometry_as_pose=True))
    monkeypatch.setattr(svc, "backend", backend)
    graph = NS(
        poses={k: np.eye(4) for k in frames},
        t_world_trajectory={},
        t_world_map={"r": world},
    )
    body, headers = svc._build_cloud_payload(NS(optimized=graph), robot_id="r")
    raw = zlib.decompress(body)
    assert headers["X-Cloud-Points"] == "1"
    assert headers["X-Cloud-RGB"] == "1"
    assert headers["X-Cloud-Frame"] == "world"
    np.testing.assert_allclose(np.frombuffer(raw[:6], dtype="<i2") * 0.01, [8, 21, 0.5])
    assert list(raw[7:]) == [250, 20, 10]
