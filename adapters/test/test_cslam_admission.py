"""Behavioral regressions for native peer admission and scene-change gates."""

from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]


def added_lines(patch: str) -> list[str]:
    return [
        line[1:]
        for line in patch.splitlines()
        if line.startswith("+") and not line.startswith("+++")
    ]


SCENE_PATCH = REPO / "deploy/patches/cslam-scene-change-keyframe.patch"


def _scene_functions():
    """Load the pure helpers straight out of the patch's added lines.

    Returns `(scene_signature, scene_changed, odom_yaw)`.
    """
    import numpy as np

    lines = added_lines(SCENE_PATCH.read_text())
    start = next(i for i, l in enumerate(lines) if "def odom_yaw" in l) - 1
    # The helpers end where the patch starts editing generate_new_keyframe.
    end = next(i for i, l in enumerate(lines) if "stamp = msg[1].header.stamp" in l)
    body = "\n".join(l[4:] if l.startswith("    ") else l for l in lines[start:end])
    namespace = {"np": np}
    exec(body.replace("@staticmethod\n", ""), namespace)
    return (
        namespace["scene_signature"],
        namespace["scene_changed"],
        namespace["odom_yaw"],
    )


def _rotated(points, yaw):
    """The same returns seen from a sensor turned by `yaw` about the vertical."""
    import numpy as np

    c, s = np.cos(yaw), np.sin(yaw)
    rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    # A robot that turns by +yaw sees the world rotated by -yaw in its own frame.
    return points @ rotation


def _cluttered_room():
    """A far wall with a few near obstacles, the range profile of a bistro.

    Adjacent sectors differ by metres wherever a table leg or a pillar sits in
    front of the wall, which is what made a small yaw look like a new scene.
    """
    import numpy as np

    # Returns every quarter degree, offset so none sits on a sector boundary.
    angles = np.linspace(-np.pi, np.pi, 1440, endpoint=False) + np.radians(0.125)
    wall = np.stack([15 * np.cos(angles), 15 * np.sin(angles), 0 * angles], axis=1)
    pieces = [wall]
    for centre, radius in ((0.4, 1.5), (1.9, 2.5), (-2.4, 3.0), (-1.0, 4.0)):
        span = np.abs((angles - centre + np.pi) % (2 * np.pi) - np.pi) < np.radians(4)
        pieces.append(
            np.stack(
                [
                    radius * np.cos(angles[span]),
                    radius * np.sin(angles[span]),
                    0 * angles[span],
                ],
                axis=1,
            )
        )
    return np.vstack(pieces)


def test_a_neighbour_leaving_changes_the_scene_but_noise_does_not():
    import numpy as np

    signature, changed, _ = _scene_functions()
    angles = np.linspace(-np.pi, np.pi, 720, endpoint=False)
    wall = np.stack([15 * np.cos(angles), 15 * np.sin(angles), 0 * angles], axis=1)
    # A neighbour two metres ahead: its returns are masked, so those sectors
    # hold only the far wall. After it leaves, the low rings reach the floor.
    ahead = np.abs(angles) < np.radians(14)
    floor = np.stack(
        [3 * np.cos(angles[ahead]), 3 * np.sin(angles[ahead]), 0 * angles[ahead]],
        axis=1,
    )
    before = signature(wall, 72)
    after = signature(np.vstack([wall, floor]), 72)
    assert changed(before, after, 0.5, 0.05)
    noisy = wall + np.random.default_rng(1).normal(0, 0.02, wall.shape)
    assert not changed(before, signature(noisy, 72), 0.5, 0.05)
    assert not changed(None, after, 0.5, 0.05)
    assert not changed(before, after, 0.5, 0.0)  # disabled


def test_turning_in_place_is_not_a_scene_change():
    """The signature is binned in the odometry frame, not the sensor frame.

    Measured 2026-09-17 on Benchbot (mission a72b1c3d, robot_3): a robot
    dithering inside 0.7 m by 0.4 m with its yaw swinging by 5 to 39 degrees
    took a keyframe every 5.0 to 5.8 s, the scene-change minimum period, for
    160 s; 27 of its 51 keyframe steps were under 0.2 m. On one of its stored
    keyframes a pure 5 degree yaw already moved four of 72 sectors by more than
    0.5 m in the sensor frame, which is the 0.05 fraction.
    """
    import numpy as np

    signature, changed, _ = _scene_functions()
    room = _cluttered_room()
    reference = signature(room, 72)
    for yaw in (np.radians(5), 0.5, -0.5, np.radians(170), np.radians(-190)):
        seen = _rotated(room, yaw)
        # In the sensor frame the same room passes as a scene change...
        assert changed(reference, signature(seen, 72), 0.5, 0.05), yaw
        # ...aligned with the heading it is the same signature, sector for
        # sector, including the sectors that wrapped past the +-pi seam.
        aligned = signature(seen, 72, yaw)
        assert np.allclose(aligned, reference, atol=1e-9), yaw
        assert not changed(reference, aligned, 0.5, 0.05), yaw


def test_a_neighbour_leaving_is_still_seen_after_the_robot_has_turned():
    import numpy as np

    signature, changed, _ = _scene_functions()
    room = _cluttered_room()
    angles = np.arctan2(room[:, 1], room[:, 0])
    # The neighbour's masked body hid the floor three metres ahead; the robot
    # then turned by half a radian before the neighbour drove off.
    ahead = np.abs(angles) < np.radians(14)
    floor = np.stack(
        [3 * np.cos(angles[ahead]), 3 * np.sin(angles[ahead]), 0 * angles[ahead]],
        axis=1,
    )
    before = signature(room, 72, 0.0)
    after = signature(_rotated(np.vstack([room, floor]), 0.5), 72, 0.5)
    assert changed(before, after, 0.5, 0.05)


def test_odom_yaw_reads_the_heading_out_of_the_odometry_quaternion():
    from types import SimpleNamespace
    import numpy as np

    _, _, odom_yaw = _scene_functions()
    for heading in (0.0, 0.5, -0.5, 3.0, -3.0):
        orientation = SimpleNamespace(
            x=0.0, y=0.0, z=np.sin(heading / 2), w=np.cos(heading / 2)
        )
        odom = SimpleNamespace(
            pose=SimpleNamespace(pose=SimpleNamespace(orientation=orientation))
        )
        assert odom_yaw(odom) == pytest.approx(heading)


PAIR_CAP_PATCH = REPO / "deploy/patches/cslam-inter-robot-pair-cap.patch"


class _FakeSelector:
    """The two selector members the patch touches, with upstream's semantics."""

    def __init__(self, edges):
        self.candidate_edges = {self._key(e): e for e in edges}
        self.already_considered_matches = set()

    @staticmethod
    def _key(e):
        return (e.robot0_id, e.robot0_keyframe_id, e.robot1_id, e.robot1_keyframe_id)

    def remove_candidate_edges(self, edges, failed=False):
        for k in list(self.candidate_edges):
            if self.candidate_edges[k] in edges:
                del self.candidate_edges[k]
        for e in edges:
            self.already_considered_matches.add(self._key(e))


def _pair_cap_class():
    """Build a class from the patch's own added lines.

    The three helpers are whole methods; the counting hunk is the body added
    inside `receive_inter_robot_loop_closure`'s success branch, wrapped here
    in a method of its own.
    """
    lines = added_lines(PAIR_CAP_PATCH.read_text())
    start = next(i for i, l in enumerate(lines) if l.startswith("    def robot_pair("))
    end = next(i for i, l in enumerate(lines) if "remove_candidate_edges(capped)" in l)
    helpers = "\n".join(l[4:] for l in lines[start : end + 1])
    first = next(i for i, l in enumerate(lines) if "pair = self.robot_pair(msg" in l)
    last = next(
        i for i, l in enumerate(lines) if "no further candidates will be selected" in l
    )
    counting = "def receive_inter_robot_loop_closure(self, msg):\n" + "\n".join(
        l[8:] for l in lines[first : last + 1]
    )
    namespace = {}
    exec(helpers + "\n" + counting, namespace)
    return type(
        "PairCap",
        (),
        {k: v for k, v in namespace.items() if callable(v) and not k.startswith("__")},
    )


def test_pair_cap_withdraws_only_the_capped_pair_and_logs_once():
    from types import SimpleNamespace

    Edge = SimpleNamespace
    cap = 3
    edges = [
        Edge(robot0_id=a, robot0_keyframe_id=k, robot1_id=b, robot1_keyframe_id=k + 100)
        for a, b in ((0, 1), (0, 2), (1, 2))
        for k in range(4)
    ]
    selector = _FakeSelector(edges)
    logs = []
    obj = _pair_cap_class()()
    obj.params = {"frontend.inter_robot_closure_pair_cap": cap}
    obj.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(info=logs.append))
    obj.lcm = SimpleNamespace(candidate_selector=selector)
    obj.inter_robot_closures_per_pair = {}
    closure = lambda r0, k, r1: SimpleNamespace(  # noqa: E731
        robot0_id=r0, robot0_keyframe_id=k, robot1_id=r1, robot1_keyframe_id=k + 100
    )
    # Closures on (0,1) from either side count for the one unordered pair.
    obj.receive_inter_robot_loop_closure(closure(0, 0, 1))
    obj.receive_inter_robot_loop_closure(closure(1, 1, 0))
    assert not obj.is_pair_capped(0, 1)
    obj.drop_capped_pair_candidates()
    assert len(selector.candidate_edges) == 12  # below the cap, nothing withdrawn
    obj.receive_inter_robot_loop_closure(closure(0, 2, 1))
    assert obj.inter_robot_closures_per_pair == {(0, 1): cap}
    assert obj.is_pair_capped(0, 1) and obj.is_pair_capped(1, 0)
    assert not obj.is_pair_capped(0, 2) and not obj.is_pair_capped(1, 2)
    assert [m for m in logs if "cap reached for robots (0, 1)" in m] == logs
    assert len(logs) == 1
    # Over the cap: counted, not logged again.
    obj.receive_inter_robot_loop_closure(closure(0, 3, 1))
    assert obj.inter_robot_closures_per_pair[(0, 1)] == cap + 1
    assert len(logs) == 1
    # The prune withdraws every (0,1) candidate, marks it considered, and
    # leaves the other pairs alone.
    obj.drop_capped_pair_candidates()
    assert not any(k[0] == 0 and k[2] == 1 for k in selector.candidate_edges)
    assert len(selector.candidate_edges) == 8
    assert len(selector.already_considered_matches) == 4
    # Zero or less: no cap, nothing withdrawn, nothing logged.
    obj.params["frontend.inter_robot_closure_pair_cap"] = 0
    selector.candidate_edges[(0, 9, 1, 109)] = Edge(
        robot0_id=0, robot0_keyframe_id=9, robot1_id=1, robot1_keyframe_id=109
    )
    assert not obj.is_pair_capped(0, 1)
    obj.drop_capped_pair_candidates()
    assert (0, 9, 1, 109) in selector.candidate_edges
    obj.params["frontend.inter_robot_closure_pair_cap"] = -1
    obj.receive_inter_robot_loop_closure(closure(2, 0, 3))
    assert obj.inter_robot_closures_per_pair[(2, 3)] == 1
    assert len(logs) == 1


def test_scene_signature_ignores_returns_beyond_its_range():
    import numpy as np

    scene_signature, scene_changed, _ = _scene_functions()
    # The robot's own surroundings fill half the circle at 2 m; the other
    # half is open street where the fleet drives 15 to 25 m away.
    near = [(2.0 * np.cos(a), 2.0 * np.sin(a), 0.0) for a in np.linspace(0.1, 3.0, 40)]
    far = [(20.0, -5.0, 0.0), (18.0, -9.0, 0.0), (8.0, -22.0, 0.0), (-7.0, -25.0, 0.0)]
    with_fleet = scene_signature(near + far, 72, 0.0, 8.0)
    without = scene_signature(near, 72, 0.0, 8.0)
    assert np.array_equal(with_fleet, without)
    # Without the range those four bodies occupy four otherwise empty sectors,
    # enough for the rule (3.6 of 72) each time one of them moves on.
    assert scene_changed(
        scene_signature(near, 72), scene_signature(near + far, 72), 0.5, 0.05
    )


CONSISTENCY_PATCH = REPO / "deploy/patches/cslam-closure-consistency.patch"


def _consistency_function():
    import numpy as np

    added = added_lines(CONSISTENCY_PATCH.read_text())
    stop = added.index("        self.local_odom_map = {}")
    namespace = {"np": np}
    exec("\n".join(added[:stop]), namespace)
    return namespace["registration_odom_consistent"]


def _fake_odom(x, y, yaw, covariance=None, quaternion_scale=1.0):
    from types import SimpleNamespace

    q = SimpleNamespace(
        x=0.0,
        y=0.0,
        z=quaternion_scale * np.sin(yaw / 2.0),
        w=quaternion_scale * np.cos(yaw / 2.0),
    )
    pose = SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=0.0),
            orientation=q,
        ),
        covariance=covariance,
    )
    return SimpleNamespace(pose=pose)


def _fake_transform(x, y, yaw, z=0.0, roll=0.0, pitch=0.0):
    from types import SimpleNamespace

    cr, sr = np.cos(roll / 2.0), np.sin(roll / 2.0)
    cp, sp = np.cos(pitch / 2.0), np.sin(pitch / 2.0)
    cy, sy = np.cos(yaw / 2.0), np.sin(yaw / 2.0)
    return SimpleNamespace(
        translation=SimpleNamespace(x=x, y=y, z=z),
        rotation=SimpleNamespace(
            x=sr * cp * cy - cr * sp * sy,
            y=cr * sp * cy + sr * cp * sy,
            z=cr * cp * sy - sr * sp * cy,
            w=cr * cp * cy + sr * sp * sy,
        ),
    )


def test_repetitive_tunnel_180_degree_branch_is_rejected():
    # These are the graph solution's optimized relative pose for captured
    # 231/303, not a claim about the raw registration output. The deployment
    # replay compares the raw compute_transform result with the same gate.
    """The captured 303/231 pair collapses 9.55 m to 0.41 m and flips yaw."""
    consistent = _consistency_function()
    raw1 = _fake_odom(
        41.3450846551,
        -19.9715816336,
        np.radians(98.27),
        quaternion_scale=1.005,
    )
    raw0 = _fake_odom(32.0603208355, -22.2104779974, np.radians(165.75))
    false_registration = _fake_transform(0.406, 0.015, np.radians(112.5))
    assert not consistent(raw0, raw1, false_registration, path_length_m=9.55)


def test_consistency_gate_compares_translation_in_the_tilted_body_frame():
    consistent = _consistency_function()
    pitch = np.radians(16.0)
    raw0 = _fake_odom(0.0, 0.0, 0.0)
    raw1 = _fake_odom(4.0, 0.0, 0.0)
    raw1.pose.pose.position.z = -4.0 * np.tan(pitch)
    attitude = _fake_transform(0.0, 0.0, 0.0, pitch=pitch).rotation
    raw0.pose.pose.orientation = attitude
    raw1.pose.pose.orientation = attitude
    correct = _fake_transform(4.0 / np.cos(pitch), 0.0, 0.0)
    world_delta = _fake_transform(4.0, 0.0, 0.0, z=raw1.pose.pose.position.z)
    bounds = dict(max_sigma=1.0, translation_floor_m=0.05, translation_sigma_per_m=0.0)
    assert consistent(raw0, raw1, correct, **bounds)
    assert not consistent(raw0, raw1, world_delta, **bounds)


def test_consistency_gate_keeps_a_plausible_accumulated_drift_correction():
    consistent = _consistency_function()
    raw0 = _fake_odom(0.0, 0.0, 0.0)
    raw1 = _fake_odom(10.0, 0.0, 0.0)
    drift_correction = _fake_transform(
        8.8,
        0.2,
        np.radians(9.0),
        z=0.3,
        roll=np.radians(5.0),
        pitch=np.radians(-4.0),
    )
    assert consistent(raw0, raw1, drift_correction, path_length_m=10.0)


def test_consistency_gate_rejects_degenerate_quaternions():
    consistent = _consistency_function()
    invalid = _fake_odom(0.0, 0.0, 0.0, quaternion_scale=0.0)
    valid = _fake_odom(1.0, 0.0, 0.0)
    assert not consistent(invalid, valid, _fake_transform(1.0, 0.0, 0.0))
