from __future__ import annotations

import math
import uuid

import pytest

from autonomy.contracts import (
    IDENTITY_SE3,
    ZERO_COVARIANCE,
    Calibration,
    CalibratedCapture,
    ComponentRevision,
    DeskewStatus,
    GraphSolution,
    PayloadRef,
    KeyframeId,
    component_id_for_anchor,
    validate_covariance,
    validate_se3,
)


def session() -> str:
    return str(uuid.UUID("82c12e9a-ad97-48bd-85a7-f8e3abfb8491"))


def test_identity_survives_robot_restart_without_key_collision() -> None:
    first = KeyframeId("robot_0", session(), 0)
    restarted = KeyframeId("robot_0", str(uuid.uuid4()), 0)
    assert first != restarted
    assert len({first.stable_id, restarted.stable_id}) == 2
    with pytest.raises(ValueError, match="UUID"):
        KeyframeId("robot_0", "boot-1", 0)
    with pytest.raises(ValueError, match="non-negative integer"):
        KeyframeId.from_dict(
            {"robot_id": "robot_0", "session_id": session(), "seq": 1.0}
        )
    with pytest.raises(ValueError, match="must be a string"):
        KeyframeId(7, session(), 0)  # type: ignore[arg-type]


def test_capture_accepts_namespaced_frame_and_preserves_calibration() -> None:
    calibration = Calibration(
        "cal-v3",
        "robot_0/camera/depth_optical_frame",
        "REP-103 optical: +x right, +y down, +z forward",
        (500.0, 500.0, 320.0, 240.0),
        "plumb_bob",
        (0.0,) * 5,
        IDENTITY_SE3,
        0.001,
    )
    capture = CalibratedCapture(
        KeyframeId("robot_0", session(), 4),
        100,
        120,
        calibration.sensor_frame,
        calibration.version,
        IDENTITY_SE3,
        ZERO_COVARIANCE,
        DeskewStatus.DESKEWED,
        rgb_timestamp_ns=110,
        depth_timestamp_ns=112,
    )
    assert capture.capture_end_ns == 120

    unknown_covariance = CalibratedCapture(
        KeyframeId("robot_0", session(), 5),
        121,
        122,
        calibration.sensor_frame,
        calibration.version,
        IDENTITY_SE3,
        None,
        DeskewStatus.UNKNOWN,
    )
    assert unknown_covariance.covariance is None


def test_nonfinite_and_nonrigid_poses_are_rejected() -> None:
    nonfinite = [list(row) for row in IDENTITY_SE3]
    nonfinite[0][3] = math.nan
    with pytest.raises(ValueError, match="nonfinite"):
        validate_se3(nonfinite)
    scaled = [list(row) for row in IDENTITY_SE3]
    scaled[0][0] = 2
    with pytest.raises(ValueError, match="orthonormal"):
        validate_se3(scaled)


def test_covariance_must_be_symmetric_positive_semidefinite() -> None:
    covariance = [list(row) for row in ZERO_COVARIANCE]
    covariance[0][0] = covariance[1][1] = 1.0
    covariance[0][1] = covariance[1][0] = 2.0
    with pytest.raises(ValueError, match="positive semidefinite"):
        validate_covariance(covariance)


def test_solution_requires_exact_membership_and_canonical_digest() -> None:
    keyframe = KeyframeId("robot_0", session(), 1)
    component = component_id_for_anchor(keyframe)
    solution = GraphSolution(
        ComponentRevision(component, 2, 0),
        keyframe,
        (keyframe,),
        {keyframe: IDENTITY_SE3},
        retracted_constraints=("loop:17",),
    )
    assert solution.digest == solution.digest
    with pytest.raises(TypeError):
        solution.poses[keyframe] = IDENTITY_SE3  # type: ignore[index]
    with pytest.raises(ValueError, match="exactly"):
        GraphSolution(ComponentRevision(component, 2, 1), keyframe, (keyframe,), {})


def test_payload_sizes_preserve_strict_integer_typing() -> None:
    with pytest.raises(ValueError, match="non-negative"):
        PayloadRef("a" * 64, "application/octet-stream", False)
    with pytest.raises(ValueError, match="non-negative"):
        PayloadRef("a" * 64, "application/octet-stream", 1.5)  # type: ignore[arg-type]
