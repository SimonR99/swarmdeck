import json
import uuid

import numpy as np
import pytest

from autonomy.capture_providers import (
    CAPTURE_PROVIDER_SPECS,
    CaptureClock,
    CaptureGeometry,
    CaptureProvenance,
    CovarianceProvenance,
    RawCaptureMetadata,
    capture_provider,
    endpoint_preserving_sample,
)
from autonomy.contracts import (
    Calibration,
    DeskewStatus,
    IDENTITY_SE3,
    KeyframeId,
    RayReturnSemantics,
)


def key(seq=0):
    return KeyframeId("robot_0", str(uuid.uuid4()), seq)


def calibration():
    return Calibration(
        "calibration:v1",
        "robot_0/lidar",
        "x-forward/y-left/z-up",
        (),
        "none",
        (),
        IDENTITY_SE3,
    )


def provenance(keyframe, provider, geometry, *, start=10, end=10, transform=10):
    spec = CAPTURE_PROVIDER_SPECS[provider]
    source_contract = (
        spec.registered_source_contract
        if geometry is CaptureGeometry.REGISTERED_CLOUD
        else spec.raw_source_contract
    )
    return CaptureProvenance(
        keyframe,
        geometry,
        start,
        end,
        transform,
        spec.clock,
        keyframe.session_id,
        True,
        source_contract=source_contract,
    )


def make_capture(provider, evidence, covariance=None):
    return capture_provider(provider).capture(
        evidence.keyframe_id,
        evidence,
        calibration(),
        IDENTITY_SE3,
        covariance,
    )


def test_provider_selection_is_explicit_and_validated():
    assert capture_provider().spec.name == "unknown"
    assert capture_provider(" Simulation ").spec.name == "simulation"
    with pytest.raises(ValueError, match="valid providers"):
        capture_provider("automatic")


def test_simulation_raw_tick_preserves_first_return_and_no_deskew_needed():
    item = provenance(key(), "simulation", CaptureGeometry.RAW_RAY_CAPTURE)
    capture = make_capture("simulation", item)
    assert capture.capture_start_ns == capture.capture_end_ns == 10
    assert capture.ray_return_semantics is RayReturnSemantics.FIRST_RETURN
    assert capture.deskew_status is DeskewStatus.NOT_REQUIRED


def test_provider_name_alone_never_qualifies_a_slam_keyframe_cloud():
    keyframe = key()
    item = CaptureProvenance.unqualified(keyframe, 10)
    capture = make_capture("simulation", item)
    assert capture.ray_return_semantics is RayReturnSemantics.UNKNOWN
    assert capture.deskew_status is DeskewStatus.UNKNOWN


@pytest.mark.parametrize("provider", ["superodometry", "fast_livo2"])
def test_hardware_raw_first_returns_remain_unqualified_without_deskew(provider):
    item = provenance(
        key(), provider, CaptureGeometry.RAW_RAY_CAPTURE, start=10, end=20, transform=20
    )
    capture = make_capture(provider, item)
    assert capture.ray_return_semantics is RayReturnSemantics.FIRST_RETURN
    assert capture.deskew_status is DeskewStatus.NOT_DESKEWED


def test_fast_livo_registered_cloud_is_deskewed_but_not_a_first_return_set():
    item = provenance(
        key(),
        "fast_livo2",
        CaptureGeometry.REGISTERED_CLOUD,
        start=10,
        end=20,
        transform=20,
    )
    capture = make_capture("fast_livo2", item)
    assert capture.deskew_status is DeskewStatus.DESKEWED
    assert capture.ray_return_semantics is RayReturnSemantics.UNKNOWN


def test_raw_capture_requires_one_origin_before_preserving_return_semantics():
    item = provenance(key(), "simulation", CaptureGeometry.RAW_RAY_CAPTURE)
    item = CaptureProvenance(
        item.keyframe_id,
        item.geometry,
        item.capture_start_ns,
        item.capture_end_ns,
        item.transform_timestamp_ns,
        item.clock,
        item.estimator_session_id,
        False,
        source_contract=item.source_contract,
    )
    capture = make_capture("simulation", item)
    assert capture.ray_return_semantics is RayReturnSemantics.UNKNOWN
    assert capture.deskew_status is DeskewStatus.UNKNOWN


def test_clock_keyframe_reset_and_capture_time_transform_are_checked():
    keyframe = key()
    good = provenance(keyframe, "simulation", CaptureGeometry.RAW_RAY_CAPTURE)
    wrong_key = provenance(key(), "simulation", CaptureGeometry.RAW_RAY_CAPTURE)
    with pytest.raises(ValueError, match="another keyframe"):
        capture_provider("simulation").capture(
            keyframe, wrong_key, calibration(), IDENTITY_SE3, None
        )

    wrong_session = CaptureProvenance(
        keyframe,
        good.geometry,
        10,
        10,
        10,
        good.clock,
        str(uuid.uuid4()),
        True,
        source_contract=good.source_contract,
    )
    with pytest.raises(ValueError, match="reset requires a new mission"):
        make_capture("simulation", wrong_session)

    wrong_clock = CaptureProvenance(
        keyframe,
        good.geometry,
        10,
        10,
        10,
        CaptureClock.ROS_TIME,
        keyframe.session_id,
        True,
        source_contract=good.source_contract,
    )
    with pytest.raises(ValueError, match="ros_sim_time"):
        make_capture("simulation", wrong_clock)

    with pytest.raises(ValueError, match="outside the capture interval"):
        CaptureProvenance(
            keyframe,
            good.geometry,
            10,
            20,
            21,
            good.clock,
            keyframe.session_id,
            True,
        )


def test_simulation_requires_one_tick_and_exact_end_transform_for_ray_evidence():
    with pytest.raises(ValueError, match="one simulator tick"):
        make_capture(
            "simulation",
            provenance(
                key(),
                "simulation",
                CaptureGeometry.RAW_RAY_CAPTURE,
                start=10,
                end=20,
                transform=20,
            ),
        )
    with pytest.raises(ValueError, match="match capture end"):
        make_capture(
            "simulation",
            provenance(
                key(),
                "simulation",
                CaptureGeometry.RAW_RAY_CAPTURE,
                start=10,
                end=20,
                transform=15,
            ),
        )


def test_unverified_covariance_cannot_be_smuggled_through_a_provider():
    keyframe = key()
    item = CaptureProvenance(
        keyframe,
        CaptureGeometry.RAW_RAY_CAPTURE,
        10,
        10,
        10,
        CaptureClock.ROS_SIM_TIME,
        keyframe.session_id,
        True,
        CovarianceProvenance.SIMULATOR_MODEL,
        CAPTURE_PROVIDER_SPECS["simulation"].raw_source_contract,
    )
    with pytest.raises(ValueError, match="not qualified"):
        make_capture("simulation", item, np.eye(6))


def raw_metadata(**changes):
    value = {
        "schema": "swarmdeck.raw-capture.v1",
        "provider": "simulation",
        "source_contract": ("argos.photorealistic_lidar.hit_endpoints.single_tick.v1"),
        "geometry": "raw_ray_capture",
        "stamp_ns": 123,
        "frame_id": "robot_0/base_link/lidar",
        "clock": "ros_sim_time",
        "first_return": True,
        "instantaneous": True,
        "single_sensor_origin": True,
        "producer_id": "a" * 32,
        "sensor_epoch": 1,
        "point_count": 3,
        "points_sha256": "b" * 64,
    }
    value.update(changes)
    return json.dumps(value)


def test_raw_metadata_requires_the_fixed_producer_contract():
    metadata = RawCaptureMetadata.from_json(raw_metadata())
    assert metadata.stamp_ns == 123
    assert metadata.frame_id == "robot_0/base_link/lidar"
    evidence = metadata.provenance(key())
    assert evidence.geometry is CaptureGeometry.RAW_RAY_CAPTURE
    assert evidence.capture_start_ns == evidence.capture_end_ns == 123

    for change in (
        {"source_contract": "generic-lidar"},
        {"first_return": False},
        {"first_return": 1},
        {"instantaneous": False},
        {"single_sensor_origin": False},
        {"clock": "ros_time"},
    ):
        with pytest.raises(ValueError):
            RawCaptureMetadata.from_json(raw_metadata(**change))


def test_raw_metadata_is_bounded_and_rejects_ambiguous_identity():
    with pytest.raises(ValueError, match="oversized"):
        RawCaptureMetadata.from_json(" " * 2049)
    with pytest.raises(ValueError, match="producer"):
        RawCaptureMetadata.from_json(raw_metadata(producer_id="restarted"))
    with pytest.raises(ValueError, match="oversized"):
        RawCaptureMetadata.from_json(raw_metadata(point_count=250_001))


def test_raw_geometry_without_the_provider_source_contract_stays_unknown():
    item = provenance(key(), "simulation", CaptureGeometry.RAW_RAY_CAPTURE)
    unbound = CaptureProvenance(
        item.keyframe_id,
        item.geometry,
        item.capture_start_ns,
        item.capture_end_ns,
        item.transform_timestamp_ns,
        item.clock,
        item.estimator_session_id,
        True,
        source_contract="generic-lidar",
    )
    capture = make_capture("simulation", unbound)
    assert capture.ray_return_semantics is RayReturnSemantics.UNKNOWN
    assert capture.deskew_status is DeskewStatus.UNKNOWN


def test_endpoint_sampling_is_bounded_deterministic_and_never_averages_rays():
    points = np.arange(30, dtype=np.float64).reshape(10, 3)
    sampled = endpoint_preserving_sample(points, 4)
    np.testing.assert_array_equal(sampled, points[[0, 3, 6, 9]])
    assert len(sampled) == 4
    assert all(
        any(np.array_equal(point, source) for source in points) for point in sampled
    )
    np.testing.assert_array_equal(endpoint_preserving_sample(points, 20), points)
    with pytest.raises(ValueError, match="positive integer"):
        endpoint_preserving_sample(points, 0)
