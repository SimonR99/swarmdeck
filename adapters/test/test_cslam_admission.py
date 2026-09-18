"""Guard the native peer path's inter-robot admission contract.

`cslam_lidar.yaml` is the only place the peer front end's acceptance thresholds
are set, and its own header records why that is dangerous: ROS 2 accepts a
parameter name no node declares, in silence. These tests hold the three things
that a measured relaxation of those thresholds depends on:

* the overlap gate exists in the patch chain, is declared, is read, and is
  passed to both registration call sites;
* the shipped config never relaxes the inlier threshold without that gate,
  because the two were measured together and the relaxation is only safe with
  it (see the measurement block in `cslam_lidar.yaml`);
* the image actually applies the patch, after the two patches it builds on.
"""

from pathlib import Path
import re

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
CONFIG = REPO / "swarmdeck_ros/src/swarmdeck_cslam/config/cslam_lidar.yaml"
OVERLAP_PATCH = REPO / "deploy/patches/cslam-registration-overlap.patch"
DOCKERFILE = REPO / "deploy/docker/Dockerfile.cslam"
# MISTLab's own lidar default. Anything at or below it is a relaxation of the
# only geometric gate upstream has.
UPSTREAM_MIN_INLIERS = 100


@pytest.fixture(scope="module")
def frontend():
    document = yaml.safe_load(CONFIG.read_text())
    return document["/**"]["ros__parameters"]["frontend"]


def added_lines(patch: str) -> list[str]:
    return [line[1:] for line in patch.splitlines() if line.startswith("+")]


def test_overlap_patch_declares_reads_and_applies_the_new_parameter():
    patch = OVERLAP_PATCH.read_text()
    added = "\n".join(added_lines(patch))
    assert (
        "('frontend.registration_min_overlap', 0.0)," in added
    ), "the parameter must be declared, or the node rejects it at startup"
    assert "params['frontend.registration_min_overlap'] = node.get_parameter(" in added
    # Both registration call sites, inter-robot and intra-robot, must pass it;
    # a gate applied to only one of them is a silent asymmetry in the graph.
    call_sites = [
        line
        for line in added_lines(patch)
        if "icp_utils.compute_transform(" in line
        and 'self.params["frontend.registration_min_overlap"]' in line
    ]
    assert len(call_sites) == 2, call_sites
    assert (
        "def compute_transform(src, dst, voxel_size, min_inliers, min_overlap=0.0):"
        in added
    )
    # Defaulting to 0.0 keeps upstream behaviour for anyone who does not set it.
    assert "min_overlap=0.0" in added


def test_short_overlap_refuses_instead_of_publishing_the_refinement():
    added = added_lines(OVERLAP_PATCH.read_text())
    body = "\n".join(added)
    assert "if overlap < min_overlap:" in body
    assert "valid = False" in body
    # The refusal must also keep the refined transform out of the solution,
    # otherwise a rejected pair still mutates what the caller reads back.
    refused = body.index("if overlap < min_overlap:")
    assert "else:" in body[refused:]
    assert body.index("solution.translation = T_icp[:3, 3]") > refused


def test_relaxed_inlier_threshold_never_ships_without_the_overlap_gate(frontend):
    inliers = frontend["registration_min_inliers"]
    overlap = frontend.get("registration_min_overlap")
    if inliers > UPSTREAM_MIN_INLIERS:
        return  # stricter than upstream; the overlap gate is then optional
    assert overlap is not None, (
        "registration_min_inliers was relaxed to or below upstream's default "
        "without setting registration_min_overlap; the measured result is "
        "false inter-robot merges"
    )
    assert 0.0 < overlap < 1.0, overlap


def test_similarity_threshold_stays_within_the_scan_context_metric(frontend):
    # `similarity` is ScanContext's mean column cosine similarity, which the
    # measurement in cslam_lidar.yaml shows peaks near 0.9 on Bistro keyframes.
    # A threshold above that admits nothing, which is how this shipped broken.
    assert 0.0 < frontend["similarity_threshold"] <= 0.9


def test_image_applies_the_overlap_patch_after_the_patches_it_builds_on():
    text = DOCKERFILE.read_text()
    assert "COPY deploy/patches/cslam-registration-overlap.patch" in text
    order = [
        text.index(f"git apply /tmp/cslam-{name}.patch")
        for name in (
            "keyframe-readiness",
            "registration-budget",
            "registration-overlap",
        )
    ]
    assert order == sorted(order), "the overlap patch must be applied last"


def test_config_sets_no_frontend_key_the_patch_chain_leaves_undeclared(frontend):
    """Keys this repo invented must be declared by this repo's patches.

    Upstream's own keys are declared in upstream; the failure this catches is a
    SwarmDeck key that only exists in the yaml, which ROS 2 would accept and
    then ignore.
    """
    patched = set()
    for patch in (REPO / "deploy/patches").glob("cslam-*.patch"):
        patched.update(
            re.findall(
                r"\('(frontend\.[a-z_]+)'", "\n".join(added_lines(patch.read_text()))
            )
        )
    invented = {
        "frontend.registration_min_overlap",
        "frontend.keyframe_min_subscribers",
    }
    for key in invented:
        if key.split(".", 1)[1] in frontend:
            assert key in patched, f"{key} is set but no cslam patch declares it"


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


def test_scene_change_patch_declares_and_reads_its_parameters(frontend):
    added = "\n".join(added_lines(SCENE_PATCH.read_text()))
    for name in (
        "keyframe_scene_change_fraction",
        "keyframe_scene_change_range_m",
        "keyframe_scene_change_bins",
        "keyframe_scene_change_min_period_s",
    ):
        assert f"('frontend.{name}'," in added, name
        assert name in frontend, name
    # Off by default upstream; the shipped config turns it on.
    assert "('frontend.keyframe_scene_change_fraction', 0.0)," in added
    assert 0.0 < frontend["keyframe_scene_change_fraction"] < 0.5
    text = DOCKERFILE.read_text()
    assert text.index("git apply /tmp/cslam-registration-overlap.patch") < text.index(
        "git apply /tmp/cslam-scene-change-keyframe.patch"
    )


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


def test_generate_new_keyframe_aligns_the_signature_with_the_odometry_heading():
    added = "\n".join(added_lines(SCENE_PATCH.read_text()))
    assert (
        'def scene_signature(points, bins, yaw=0.0, max_range_m=float("inf")):' in added
    )
    assert "np.arctan2(xy[finite, 1], xy[finite, 0]) + yaw" in added
    # The call site must pass the heading of the same odometry message the
    # distance rule reads, or the alignment is only a default argument.
    call = added.index("signature = self.scene_signature(")
    assert "self.odom_yaw(msg[1])," in added[call : call + 300]


def test_inter_robot_closures_default_off_in_the_peer_launch():
    text = (REPO / "deploy/autonomy/peer.launch.py").read_text()
    assert '"SWARMDECK_INTER_ROBOT_CLOSURES", "false"' in text
    assert '"frontend.inter_robot_loop_closure_budget": 0' in text


def test_zero_budget_is_an_explicit_inter_robot_off_switch():
    patch = (REPO / "deploy/patches/cslam-inter-robot-switch.patch").read_text()
    added = "\n".join(added_lines(patch))
    assert 'if self.params["frontend.inter_robot_loop_closure_budget"] <= 0:' in added
    assert "return" in added
    assert "git apply /tmp/cslam-inter-robot-switch.patch" in DOCKERFILE.read_text()


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


def test_a_parked_robot_earns_a_bounded_number_of_scene_keyframes():
    added = "\n".join(added_lines(SCENE_PATCH.read_text()))
    # Initialised once, and reset when the distance rule fires: the robot
    # really moved, so the allowance starts again.
    assert added.count("self.scene_keyframes_since_motion = 0") == 2
    assert (
        'self.scene_keyframes_since_motion <\n                int(self.params["frontend.keyframe_scene_change_max_stationary"])'
        in added
    )
    assert "self.scene_keyframes_since_motion += 1" in added
    assert "keyframe_scene_change_max_range_m" in added
    yaml_text = (
        REPO / "swarmdeck_ros/src/swarmdeck_cslam/config/cslam_lidar.yaml"
    ).read_text()
    assert "keyframe_scene_change_max_range_m: 8.0" in yaml_text
    assert "keyframe_scene_change_max_stationary: 3" in yaml_text
