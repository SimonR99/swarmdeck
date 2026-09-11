"""Calibrated capture provenance, independent of ROS and SLAM transports.

The provider name identifies the sensor/estimator path.  It does not bless a
cloud by itself: every capture also describes the concrete geometry source,
clock, capture interval, capture-time transform, and estimator lifetime.  This
keeps a voxelized or accumulated SLAM keyframe occupied-only even when it was
ultimately produced from a first-return lidar.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
import re
from typing import Sequence

import numpy as np

from .contracts import (
    CalibratedCapture,
    Calibration,
    DeskewStatus,
    KeyframeId,
    Matrix4,
    RayReturnSemantics,
)


class CaptureGeometry(str, Enum):
    """Relationship between stored points and one physical sensor capture."""

    RAW_RAY_CAPTURE = "raw_ray_capture"
    SLAM_KEYFRAME_CLOUD = "slam_keyframe_cloud"
    REGISTERED_CLOUD = "registered_cloud"


class CaptureClock(str, Enum):
    UNKNOWN = "unknown"
    ROS_TIME = "ros_time"
    ROS_SIM_TIME = "ros_sim_time"


class CovarianceProvenance(str, Enum):
    UNKNOWN = "unknown"
    ESTIMATOR_MESSAGE = "estimator_message"
    SIMULATOR_MODEL = "simulator_model"


class ResetSemantics(str, Enum):
    NEW_MISSION = "new_mission"


@dataclass(frozen=True)
class CaptureProvenance:
    """Evidence attached to the exact point set stored for a keyframe.

    ``keyframe_id`` is the explicit raw-capture/keyframe join.  A provider
    refuses evidence from another keyframe or estimator lifetime.  The pose
    timestamp must lie inside the capture interval; qualifying rays require it
    at the provider's deskew reference, currently the capture end.
    """

    keyframe_id: KeyframeId
    geometry: CaptureGeometry
    capture_start_ns: int
    capture_end_ns: int
    transform_timestamp_ns: int
    clock: CaptureClock
    estimator_session_id: str
    single_sensor_origin: bool
    covariance: CovarianceProvenance = CovarianceProvenance.UNKNOWN
    source_contract: str | None = None

    def __post_init__(self) -> None:
        try:
            object.__setattr__(self, "geometry", CaptureGeometry(self.geometry))
            object.__setattr__(self, "clock", CaptureClock(self.clock))
            object.__setattr__(
                self, "covariance", CovarianceProvenance(self.covariance)
            )
        except ValueError as exc:
            raise ValueError("invalid capture provenance") from exc
        for name in (
            "capture_start_ns",
            "capture_end_ns",
            "transform_timestamp_ns",
        ):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")
        if self.capture_end_ns < self.capture_start_ns:
            raise ValueError("capture interval is invalid")
        if not (
            self.capture_start_ns <= self.transform_timestamp_ns <= self.capture_end_ns
        ):
            raise ValueError(
                "capture transform timestamp is outside the capture interval"
            )
        if (
            not isinstance(self.estimator_session_id, str)
            or not self.estimator_session_id
        ):
            raise ValueError("estimator_session_id is required")
        if not isinstance(self.single_sensor_origin, bool):
            raise ValueError("single_sensor_origin must be boolean")
        if self.source_contract is not None and (
            not isinstance(self.source_contract, str) or not self.source_contract
        ):
            raise ValueError("source_contract must be a non-empty string")

    @classmethod
    def unqualified(cls, keyframe_id: KeyframeId, stamp_ns: int) -> "CaptureProvenance":
        """Describe the legacy Swarm-SLAM keyframe cloud conservatively."""

        return cls(
            keyframe_id=keyframe_id,
            geometry=CaptureGeometry.SLAM_KEYFRAME_CLOUD,
            capture_start_ns=stamp_ns,
            capture_end_ns=stamp_ns,
            transform_timestamp_ns=stamp_ns,
            clock=CaptureClock.UNKNOWN,
            estimator_session_id=keyframe_id.session_id,
            single_sensor_origin=False,
        )


@dataclass(frozen=True)
class CaptureProviderSpec:
    name: str
    clock: CaptureClock
    reset_semantics: ResetSemantics
    covariance: CovarianceProvenance
    raw_deskew: DeskewStatus
    raw_source_contract: str | None = None
    registered_source_contract: str | None = None


CAPTURE_PROVIDER_SPECS = {
    "unknown": CaptureProviderSpec(
        "unknown",
        CaptureClock.UNKNOWN,
        ResetSemantics.NEW_MISSION,
        CovarianceProvenance.UNKNOWN,
        DeskewStatus.UNKNOWN,
    ),
    # ARGoS ray traces one complete cloud at one simulation tick.  A raw point
    # set paired to that tick therefore has no within-scan motion to remove.
    "simulation": CaptureProviderSpec(
        "simulation",
        CaptureClock.ROS_SIM_TIME,
        ResetSemantics.NEW_MISSION,
        CovarianceProvenance.UNKNOWN,
        DeskewStatus.NOT_REQUIRED,
        "argos.photorealistic_lidar.hit_endpoints.single_tick.v1",
    ),
    # The raw Ouster scan contains first returns, but this boundary has no proof
    # that the stored raw points were deskewed.  /registered_scan is processed
    # estimator output and is not assumed to preserve one endpoint per ray.
    "superodometry": CaptureProviderSpec(
        "superodometry",
        CaptureClock.ROS_TIME,
        ResetSemantics.NEW_MISSION,
        CovarianceProvenance.UNKNOWN,
        DeskewStatus.NOT_DESKEWED,
        "superodometry.input_ouster.first_returns.v1",
    ),
    # FAST-LIVO2 internally undistorts its registered cloud, but may downsample
    # it; raw input retains first-return meaning but is still motion-skewed.
    "fast_livo2": CaptureProviderSpec(
        "fast_livo2",
        CaptureClock.ROS_TIME,
        ResetSemantics.NEW_MISSION,
        CovarianceProvenance.UNKNOWN,
        DeskewStatus.NOT_DESKEWED,
        "fast_livo2.input_lidar.first_returns.v1",
        "fast_livo2.feats_undistort.registered.v1",
    ),
}


_FRAME = re.compile(r"^/?[A-Za-z0-9][A-Za-z0-9_./-]{0,254}$")
_PRODUCER = re.compile(r"^[0-9a-f]{32}$")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
MAX_RAW_METADATA_BYTES = 2048
MAX_RAW_CAPTURE_POINTS = 250_000


@dataclass(frozen=True)
class RawCaptureMetadata:
    """Bounded producer attestation joined to one raw PointCloud2 by stamp."""

    provider: str
    source_contract: str
    stamp_ns: int
    frame_id: str
    clock: CaptureClock
    producer_id: str
    sensor_epoch: int
    point_count: int
    points_sha256: str

    @classmethod
    def from_json(cls, payload: str) -> "RawCaptureMetadata":
        if (
            not isinstance(payload, str)
            or len(payload.encode()) > MAX_RAW_METADATA_BYTES
        ):
            raise ValueError("raw capture metadata is oversized")
        try:
            value = json.loads(payload)
        except (json.JSONDecodeError, TypeError) as exc:
            raise ValueError("raw capture metadata is not valid JSON") from exc
        if (
            not isinstance(value, dict)
            or value.get("schema") != "swarmdeck.raw-capture.v1"
        ):
            raise ValueError("raw capture metadata has an unsupported schema")
        if value.get("geometry") != CaptureGeometry.RAW_RAY_CAPTURE.value or any(
            value.get(name) is not True
            for name in ("first_return", "instantaneous", "single_sensor_origin")
        ):
            raise ValueError("raw capture metadata does not attest the source contract")
        provider = value.get("provider")
        source = value.get("source_contract")
        stamp_ns = value.get("stamp_ns")
        frame_id = value.get("frame_id")
        producer_id = value.get("producer_id")
        epoch = value.get("sensor_epoch")
        point_count = value.get("point_count")
        points_sha256 = value.get("points_sha256")
        if not isinstance(provider, str) or provider not in CAPTURE_PROVIDER_SPECS:
            raise ValueError("raw capture metadata has an unknown provider")
        if source != CAPTURE_PROVIDER_SPECS[provider].raw_source_contract:
            raise ValueError("raw capture metadata source contract is not qualified")
        if not isinstance(stamp_ns, int) or isinstance(stamp_ns, bool) or stamp_ns < 0:
            raise ValueError("raw capture metadata stamp is invalid")
        if (
            not isinstance(frame_id, str)
            or not _FRAME.fullmatch(frame_id)
            or "//" in frame_id
            or frame_id.endswith("/")
        ):
            raise ValueError("raw capture metadata frame is invalid")
        if not isinstance(producer_id, str) or not _PRODUCER.fullmatch(producer_id):
            raise ValueError("raw capture metadata producer is invalid")
        if not isinstance(epoch, int) or isinstance(epoch, bool) or epoch < 0:
            raise ValueError("raw capture metadata sensor epoch is invalid")
        if (
            not isinstance(point_count, int)
            or isinstance(point_count, bool)
            or point_count <= 0
            or point_count > MAX_RAW_CAPTURE_POINTS
        ):
            raise ValueError("raw capture metadata point count is invalid or oversized")
        if not isinstance(points_sha256, str) or not _SHA256.fullmatch(points_sha256):
            raise ValueError("raw capture metadata point digest is invalid")
        try:
            clock = CaptureClock(value.get("clock"))
        except (TypeError, ValueError) as exc:
            raise ValueError("raw capture metadata clock is invalid") from exc
        if clock is not CAPTURE_PROVIDER_SPECS[provider].clock:
            raise ValueError("raw capture metadata clock does not match its provider")
        return cls(
            provider,
            source,
            stamp_ns,
            frame_id,
            clock,
            producer_id,
            epoch,
            point_count,
            points_sha256,
        )

    def provenance(self, keyframe_id: KeyframeId) -> CaptureProvenance:
        return CaptureProvenance(
            keyframe_id,
            CaptureGeometry.RAW_RAY_CAPTURE,
            self.stamp_ns,
            self.stamp_ns,
            self.stamp_ns,
            self.clock,
            keyframe_id.session_id,
            True,
            source_contract=self.source_contract,
        )


def endpoint_preserving_sample(
    points: Sequence[Sequence[float]] | np.ndarray, max_points: int
) -> np.ndarray:
    """Return a deterministic uniform subset without creating ray centroids."""

    cloud = np.asarray(points)
    if cloud.ndim != 2 or cloud.shape[1] != 3:
        raise ValueError("raw capture points must have shape Nx3")
    if (
        not isinstance(max_points, int)
        or isinstance(max_points, bool)
        or max_points <= 0
    ):
        raise ValueError("max_points must be a positive integer")
    if len(cloud) <= max_points:
        return cloud.copy()
    if max_points == 1:
        return cloud[[0]].copy()
    indices = np.arange(max_points, dtype=np.int64) * (len(cloud) - 1)
    indices //= max_points - 1
    return cloud[indices].copy()


class CaptureProvider:
    """Validate one selected capture path and construct its durable contract."""

    def __init__(self, spec: CaptureProviderSpec):
        self.spec = spec

    def capture(
        self,
        keyframe_id: KeyframeId,
        provenance: CaptureProvenance,
        calibration: Calibration,
        T_local_base: Matrix4,
        covariance: Sequence[Sequence[float]] | None,
    ) -> CalibratedCapture:
        if provenance.keyframe_id != keyframe_id:
            raise ValueError("capture provenance is paired to another keyframe")
        if provenance.estimator_session_id != keyframe_id.session_id:
            raise ValueError("estimator reset requires a new mission/session")

        concrete_source = provenance.geometry is not CaptureGeometry.SLAM_KEYFRAME_CLOUD
        if concrete_source and provenance.clock is not self.spec.clock:
            raise ValueError(
                f"{self.spec.name} capture requires {self.spec.clock.value} timestamps"
            )
        if covariance is None:
            if provenance.covariance is not CovarianceProvenance.UNKNOWN:
                raise ValueError(
                    "covariance provenance was supplied without covariance"
                )
        elif (
            provenance.covariance is CovarianceProvenance.UNKNOWN
            or provenance.covariance is not self.spec.covariance
        ):
            raise ValueError("capture covariance is not qualified by this provider")

        deskew = DeskewStatus.UNKNOWN
        returns = RayReturnSemantics.UNKNOWN
        if (
            provenance.geometry is CaptureGeometry.RAW_RAY_CAPTURE
            and provenance.single_sensor_origin
            and provenance.source_contract == self.spec.raw_source_contract
            and self.spec.raw_source_contract is not None
        ):
            returns = RayReturnSemantics.FIRST_RETURN
            deskew = self.spec.raw_deskew
            if (
                deskew in (DeskewStatus.DESKEWED, DeskewStatus.NOT_REQUIRED)
                and provenance.transform_timestamp_ns != provenance.capture_end_ns
            ):
                raise ValueError("qualified capture transform must match capture end")
            if (
                self.spec.name == "simulation"
                and provenance.capture_start_ns != provenance.capture_end_ns
            ):
                raise ValueError(
                    "simulation ray capture must come from one simulator tick"
                )
        elif (
            self.spec.name == "fast_livo2"
            and provenance.geometry is CaptureGeometry.REGISTERED_CLOUD
            and provenance.source_contract == self.spec.registered_source_contract
        ):
            # This is useful capture truth even though the missing first-return
            # property deliberately keeps the planner result occupied-only.
            deskew = DeskewStatus.DESKEWED

        return CalibratedCapture(
            keyframe_id=keyframe_id,
            capture_start_ns=provenance.capture_start_ns,
            capture_end_ns=provenance.capture_end_ns,
            sensor_frame=calibration.sensor_frame,
            calibration_version=calibration.version,
            T_local_base=T_local_base,
            covariance=covariance,
            deskew_status=deskew,
            ray_return_semantics=returns,
        )


def capture_provider(name: str | CaptureProvider | None = None) -> CaptureProvider:
    if isinstance(name, CaptureProvider):
        return name
    selected = "unknown" if name is None else str(name).strip().lower()
    try:
        spec = CAPTURE_PROVIDER_SPECS[selected]
    except KeyError as exc:
        choices = ", ".join(sorted(CAPTURE_PROVIDER_SPECS))
        raise ValueError(
            f"unknown capture provider {name!r}; valid providers: {choices}"
        ) from exc
    return CaptureProvider(spec)
