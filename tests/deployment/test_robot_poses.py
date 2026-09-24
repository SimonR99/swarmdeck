"""The inter-robot transforms MGG's roadmap merge receives, without ROS."""

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

PATH = Path(__file__).parents[2] / "deploy/mgg/robot_poses.py"
SPEC = importlib.util.spec_from_file_location("robot_poses_under_test", PATH)
robot_poses = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(robot_poses)


def planar(x, y, yaw, z=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, z], [0, 0, 0, 1.0]])


def authority(robot, component, transform, **extra):
    return {
        "robot_id": robot,
        "component_id": component,
        "planning_frame": f"{robot}/odom",
        "T_component_planning": transform.tolist(),
        **extra,
    }


def test_cslam_places_peers_of_the_same_component_only():
    poses = robot_poses.CslamPoses()
    poses.update("robot_0", authority("robot_0", "c1", planar(1.0, 0.0, 0.0)))
    poses.update("robot_1", authority("robot_1", "c1", planar(3.0, 2.0, math.pi / 2)))
    poses.update("robot_2", authority("robot_2", "c2", planar(0.0, 0.0, 0.0)))
    frame, placed = poses.transforms("robot_0")
    assert frame == "robot_0/odom"
    assert list(placed) == ["robot_1"]
    peer_frame, transform = placed["robot_1"]
    assert peer_frame == "robot_1/odom"
    # robot_1's origin sits 2 m east and 2 m north of robot_0's, turned 90 deg.
    np.testing.assert_allclose(transform, planar(2.0, 2.0, math.pi / 2), atol=1e-12)
    # Until C-SLAM links robot_2, it is placed for nobody.
    assert poses.transforms("robot_2")[1] == {}


@pytest.mark.parametrize(
    "bad",
    [
        None,
        {"state": "resetting", "robot_id": "robot_1"},
        authority("robot_9", "c1", np.eye(4)),
        authority("robot_1", "c1", np.eye(4) * 2.0),
        authority("robot_1", "", np.eye(4)),
    ],
)
def test_cslam_forgets_a_peer_without_a_valid_authority(bad):
    poses = robot_poses.CslamPoses()
    poses.update("robot_0", authority("robot_0", "c1", np.eye(4)))
    poses.update("robot_1", authority("robot_1", "c1", np.eye(4)))
    assert "robot_1" in poses.transforms("robot_0")[1]
    poses.update("robot_1", bad)
    assert poses.transforms("robot_0")[1] == {}


def test_ground_truth_places_odometry_frames_in_the_world():
    poses = robot_poses.GroundTruthPoses()
    # robot_0 spawned at (0, 0); robot_1 at (5, 1) facing north, and its
    # odometry has since drifted: it believes it is at (2, 0) when it is at
    # (5, 3.1) in the world.
    poses.add_truth("robot_0", 100, planar(1.0, 0.0, 0.0))
    assert poses.add_odometry("robot_0", 100, "robot_0/odom", planar(1.0, 0.0, 0.0))
    poses.add_truth("robot_1", 200, planar(5.0, 3.1, math.pi / 2))
    # Paired with the ground truth of its own tick, or the nearest in 50 ms.
    assert not poses.add_odometry(
        "robot_1", 200 + 60_000_000, "robot_1/odom", planar(2.0, 0.0, 0.0)
    )
    assert poses.add_odometry(
        "robot_1", 200 + 40_000_000, "/robot_1/odom", planar(2.0, 0.0, 0.0)
    )
    frame, placed = poses.transforms("robot_0")
    assert frame == "robot_0/odom"
    peer_frame, transform = placed["robot_1"]
    assert peer_frame == "robot_1/odom"
    np.testing.assert_allclose(transform, planar(5.0, 1.1, math.pi / 2), atol=1e-12)


def test_ground_truth_history_is_bounded():
    poses = robot_poses.GroundTruthPoses()
    for stamp in range(robot_poses.MAX_TRUTH_SAMPLES + 10):
        poses.add_truth("robot_0", stamp * 10**9, np.eye(4))
    assert len(poses.truth["robot_0"]) == robot_poses.MAX_TRUTH_SAMPLES
    assert 0 not in poses.truth["robot_0"]


@pytest.mark.parametrize("yaw", [0.0, 0.7, math.pi / 2, math.pi, -2.5])
def test_quaternion_round_trip(yaw):
    rotation = planar(0.0, 0.0, yaw)[:3, :3]
    q = robot_poses.quaternion(rotation)
    np.testing.assert_allclose(
        robot_poses.pose_matrix((0, 0, 0), q)[:3, :3], rotation, atol=1e-12
    )


def test_missing_and_found_transforms_are_logged_once():
    lines = []
    announcements = robot_poses.Announcements("robot_0", "cslam", lines.append)
    for _ in range(3):
        announcements.update(["robot_1", "robot_2"], {"robot_2": None})
    assert len(lines) == 2
    assert "no transform to robot_1" in lines[0]
    assert "C-SLAM map component" in lines[0]
    assert "sharing roadmaps with robot_2" in lines[1]
    announcements.update(["robot_1", "robot_2"], {"robot_1": None, "robot_2": None})
    assert len(lines) == 3


def test_source_defaults_to_cslam(monkeypatch):
    monkeypatch.delenv("SWARMDECK_ROBOT_POSES", raising=False)
    assert robot_poses.selected_source() == "cslam"
    monkeypatch.setenv("SWARMDECK_ROBOT_POSES", "ground_truth")
    assert robot_poses.selected_source() == "ground_truth"
    with pytest.raises(ValueError):
        robot_poses.selected_source("odometry")
