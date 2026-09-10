"""Versioned, ROS-free autonomy contracts.

Transforms are homogeneous 4x4 matrices named ``T_a_b`` and map coordinates in
frame ``b`` into frame ``a``. Present covariances use tangent order
``(rx, ry, rz, tx, ty, tz)``; ``None`` means the source did not measure one.
Time values are integer nanoseconds in the capture clock domain; watchdog code
must use a local monotonic clock instead.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
import uuid
from dataclasses import asdict, dataclass
from enum import Enum
from types import MappingProxyType
from typing import Any, Mapping, Sequence

SCHEMA_VERSION = "swarmdeck.autonomy.v1"
_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{0,127}$")
_FRAME_RE = re.compile(r"^/?[A-Za-z0-9][A-Za-z0-9_./-]{0,254}$")

Matrix4 = tuple[tuple[float, float, float, float], ...]
Matrix6 = tuple[tuple[float, float, float, float, float, float], ...]
Bounds3 = tuple[tuple[float, float, float], tuple[float, float, float]]


def _text_id(value: str, field_name: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{field_name} must be a string")
    if not _ID_RE.fullmatch(value):
        raise ValueError(f"invalid {field_name}: {value!r}")
    return value


def _session_id(value: str) -> str:
    if not isinstance(value, str):
        raise ValueError("session_id must be a UUID string")
    try:
        parsed = uuid.UUID(str(value))
    except (ValueError, AttributeError) as exc:
        raise ValueError("session_id must be a UUID") from exc
    if str(parsed) != str(value).lower():
        raise ValueError("session_id must use canonical UUID spelling")
    return str(parsed)


def _frame_id(value: str, field_name: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{field_name} must be a string")
    if not _FRAME_RE.fullmatch(value) or "//" in value or value.endswith("/"):
        raise ValueError(f"invalid {field_name}: {value!r}")
    return value


def _matrix(
    value: Sequence[Sequence[float]], rows: int, cols: int, field_name: str
) -> tuple[tuple[float, ...], ...]:
    if len(value) != rows or any(len(row) != cols for row in value):
        raise ValueError(f"{field_name} must have shape {rows}x{cols}")
    result = tuple(tuple(float(v) for v in row) for row in value)
    if not all(math.isfinite(v) for row in result for v in row):
        raise ValueError(f"{field_name} contains a nonfinite value")
    return result


def validate_se3(value: Sequence[Sequence[float]], field_name: str = "pose") -> Matrix4:
    """Return an immutable, validated rigid SE(3) matrix."""

    result = _matrix(value, 4, 4, field_name)
    if any(
        abs(result[3][i] - expected) > 1e-9 for i, expected in enumerate((0, 0, 0, 1))
    ):
        raise ValueError(f"{field_name} has an invalid homogeneous row")
    rotation = [row[:3] for row in result[:3]]
    for i in range(3):
        for j in range(3):
            dot = sum(rotation[k][i] * rotation[k][j] for k in range(3))
            expected = 1.0 if i == j else 0.0
            if abs(dot - expected) > 1e-5:
                raise ValueError(f"{field_name} rotation is not orthonormal")
    det = (
        rotation[0][0]
        * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1]
        * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2]
        * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )
    if abs(det - 1.0) > 1e-5:
        raise ValueError(f"{field_name} rotation determinant is not +1")
    return result  # type: ignore[return-value]


def validate_covariance(
    value: Sequence[Sequence[float]], field_name: str = "covariance"
) -> Matrix6:
    result = _matrix(value, 6, 6, field_name)
    for i in range(6):
        if result[i][i] < 0:
            raise ValueError(f"{field_name} has a negative variance")
        for j in range(i):
            if abs(result[i][j] - result[j][i]) > 1e-9:
                raise ValueError(f"{field_name} must be symmetric")
    # Cholesky-like PSD check that also admits zero eigenvalues. If a pivot is
    # zero, every remaining entry in that factor column must also be zero.
    factor = [[0.0] * 6 for _ in range(6)]
    for i in range(6):
        for j in range(i + 1):
            residual = result[i][j] - sum(factor[i][k] * factor[j][k] for k in range(j))
            if i == j:
                if residual < -1e-10:
                    raise ValueError(f"{field_name} must be positive semidefinite")
                factor[i][j] = math.sqrt(max(0.0, residual))
            elif factor[j][j] > 1e-12:
                factor[i][j] = residual / factor[j][j]
            elif abs(residual) > 1e-9:
                raise ValueError(f"{field_name} must be positive semidefinite")
    return result  # type: ignore[return-value]


@dataclass(frozen=True, order=True)
class KeyframeId:
    robot_id: str
    session_id: str
    seq: int

    def __post_init__(self) -> None:
        object.__setattr__(self, "robot_id", _text_id(self.robot_id, "robot_id"))
        object.__setattr__(self, "session_id", _session_id(self.session_id))
        if not isinstance(self.seq, int) or isinstance(self.seq, bool) or self.seq < 0:
            raise ValueError("seq must be a non-negative integer")

    @property
    def stable_id(self) -> str:
        return f"{self.robot_id}/{self.session_id}/{self.seq}"

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "KeyframeId":
        return cls(value["robot_id"], value["session_id"], value["seq"])


@dataclass(frozen=True, order=True)
class SubmapId:
    robot_id: str
    session_id: str
    seq: int

    def __post_init__(self) -> None:
        KeyframeId(self.robot_id, self.session_id, self.seq)

    @property
    def stable_id(self) -> str:
        return f"{self.robot_id}/{self.session_id}/submap/{self.seq}"

    @classmethod
    def from_keyframe(cls, keyframe_id: KeyframeId) -> "SubmapId":
        return cls(keyframe_id.robot_id, keyframe_id.session_id, keyframe_id.seq)

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "SubmapId":
        return cls(value["robot_id"], value["session_id"], value["seq"])


def component_id_for_anchor(anchor: KeyframeId) -> str:
    digest = hashlib.sha256(anchor.stable_id.encode()).hexdigest()[:24]
    return f"component:{digest}"


class DeskewStatus(str, Enum):
    DESKEWED = "deskewed"
    NOT_REQUIRED = "not_required"
    NOT_DESKEWED = "not_deskewed"
    UNKNOWN = "unknown"


class RayReturnSemantics(str, Enum):
    UNKNOWN = "unknown"
    FIRST_RETURN = "first_return"


class RayOriginAssociation(str, Enum):
    UNKNOWN = "unknown"
    SINGLE_CAPTURE = "single_capture"


@dataclass(frozen=True)
class RayEvidence:
    """Compact certificate for endpoint-to-origin free-space evidence.

    A qualified value means every endpoint in the submap came from the one
    associated capture and its sole stored sensor origin. Missing or partial
    evidence remains occupied-endpoint-only.
    """

    return_semantics: RayReturnSemantics = RayReturnSemantics.UNKNOWN
    deskew: DeskewStatus = DeskewStatus.UNKNOWN
    origin_association: RayOriginAssociation = RayOriginAssociation.UNKNOWN

    def __post_init__(self) -> None:
        try:
            object.__setattr__(
                self, "return_semantics", RayReturnSemantics(self.return_semantics)
            )
            object.__setattr__(self, "deskew", DeskewStatus(self.deskew))
            object.__setattr__(
                self,
                "origin_association",
                RayOriginAssociation(self.origin_association),
            )
        except ValueError as exc:
            raise ValueError("invalid ray evidence") from exc

    @property
    def certifies_free_space(self) -> bool:
        return (
            self.return_semantics is RayReturnSemantics.FIRST_RETURN
            and self.deskew is DeskewStatus.DESKEWED
            and self.origin_association is RayOriginAssociation.SINGLE_CAPTURE
        )


@dataclass(frozen=True)
class PayloadRef:
    sha256: str
    media_type: str
    size_bytes: int

    def __post_init__(self) -> None:
        digest = self.sha256.lower()
        if not re.fullmatch(r"[0-9a-f]{64}", digest):
            raise ValueError("payload sha256 must be 64 lowercase hex characters")
        object.__setattr__(self, "sha256", digest)
        if (
            not isinstance(self.size_bytes, int)
            or isinstance(self.size_bytes, bool)
            or self.size_bytes < 0
        ):
            raise ValueError("payload size_bytes must be non-negative")
        if not self.media_type:
            raise ValueError("payload media_type is required")


@dataclass(frozen=True)
class Calibration:
    version: str
    sensor_frame: str
    optical_frame_convention: str
    intrinsics: tuple[float, ...]
    distortion_model: str
    distortion: tuple[float, ...]
    T_base_sensor: Matrix4
    depth_units_m: float = 1.0

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "version", _text_id(self.version, "calibration version")
        )
        object.__setattr__(
            self, "sensor_frame", _frame_id(self.sensor_frame, "sensor_frame")
        )
        intrinsics = tuple(float(v) for v in self.intrinsics)
        distortion = tuple(float(v) for v in self.distortion)
        object.__setattr__(self, "intrinsics", intrinsics)
        object.__setattr__(self, "distortion", distortion)
        if len(intrinsics) not in (0, 4, 9):
            raise ValueError(
                "intrinsics must be empty, (fx,fy,cx,cy), or a 3x3 row-major matrix"
            )
        if not all(math.isfinite(v) for v in intrinsics + distortion):
            raise ValueError("calibration contains a nonfinite value")
        object.__setattr__(
            self, "T_base_sensor", validate_se3(self.T_base_sensor, "T_base_sensor")
        )
        if not math.isfinite(self.depth_units_m) or self.depth_units_m <= 0:
            raise ValueError("depth_units_m must be finite and positive")


@dataclass(frozen=True)
class CalibratedCapture:
    keyframe_id: KeyframeId
    capture_start_ns: int
    capture_end_ns: int
    sensor_frame: str
    calibration_version: str
    T_local_base: Matrix4
    covariance: Matrix6 | None
    deskew_status: DeskewStatus
    payloads: tuple[PayloadRef, ...] = ()
    rgb_timestamp_ns: int | None = None
    depth_timestamp_ns: int | None = None
    ray_return_semantics: RayReturnSemantics = RayReturnSemantics.UNKNOWN

    def __post_init__(self) -> None:
        if (
            not isinstance(self.capture_start_ns, int)
            or isinstance(self.capture_start_ns, bool)
            or not isinstance(self.capture_end_ns, int)
            or isinstance(self.capture_end_ns, bool)
            or self.capture_start_ns < 0
            or self.capture_end_ns < self.capture_start_ns
        ):
            raise ValueError("capture interval is invalid")
        object.__setattr__(
            self, "sensor_frame", _frame_id(self.sensor_frame, "sensor_frame")
        )
        object.__setattr__(
            self,
            "calibration_version",
            _text_id(self.calibration_version, "calibration_version"),
        )
        object.__setattr__(
            self, "T_local_base", validate_se3(self.T_local_base, "T_local_base")
        )
        if self.covariance is not None:
            object.__setattr__(self, "covariance", validate_covariance(self.covariance))
        try:
            object.__setattr__(self, "deskew_status", DeskewStatus(self.deskew_status))
            object.__setattr__(
                self,
                "ray_return_semantics",
                RayReturnSemantics(self.ray_return_semantics),
            )
        except ValueError as exc:
            raise ValueError("invalid capture ray or deskew semantics") from exc
        object.__setattr__(self, "payloads", tuple(self.payloads))
        for name, value in (
            ("rgb_timestamp_ns", self.rgb_timestamp_ns),
            ("depth_timestamp_ns", self.depth_timestamp_ns),
        ):
            if value is not None and (
                not isinstance(value, int) or isinstance(value, bool) or value < 0
            ):
                raise ValueError(f"{name} must be non-negative")


@dataclass(frozen=True, order=True)
class ComponentRevision:
    component_id: str
    epoch: int
    revision: int

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "component_id", _text_id(self.component_id, "component_id")
        )
        if (
            not isinstance(self.epoch, int)
            or isinstance(self.epoch, bool)
            or not isinstance(self.revision, int)
            or isinstance(self.revision, bool)
            or self.epoch < 0
            or self.revision < 0
        ):
            raise ValueError("component epoch and revision must be non-negative")


@dataclass(frozen=True)
class GraphSolution:
    revision: ComponentRevision
    anchor: KeyframeId
    membership: tuple[KeyframeId, ...]
    poses: Mapping[KeyframeId, Matrix4]
    retracted_constraints: tuple[str, ...] = ()
    retracted_keyframes: tuple[KeyframeId, ...] = ()

    def __post_init__(self) -> None:
        if len(set(self.membership)) != len(self.membership):
            raise ValueError("graph membership contains a duplicate keyframe")
        members = tuple(sorted(self.membership))
        object.__setattr__(self, "membership", members)
        if self.anchor not in members:
            raise ValueError("graph anchor must be a component member")
        if set(self.poses) != set(members):
            raise ValueError("poses must contain exactly the component membership")
        clean = {
            key: validate_se3(value, f"pose[{key.stable_id}]")
            for key, value in self.poses.items()
        }
        object.__setattr__(self, "poses", MappingProxyType(clean))
        if set(self.retracted_keyframes) & set(members):
            raise ValueError("a keyframe cannot be both a member and retracted")
        if len(set(self.retracted_constraints)) != len(self.retracted_constraints):
            raise ValueError("retracted constraints contain a duplicate ID")
        if len(set(self.retracted_keyframes)) != len(self.retracted_keyframes):
            raise ValueError("retracted keyframes contain a duplicate ID")
        for constraint_id in self.retracted_constraints:
            _text_id(constraint_id, "constraint_id")

    def canonical_dict(self) -> dict[str, Any]:
        return {
            "schema": SCHEMA_VERSION,
            "revision": asdict(self.revision),
            "anchor": asdict(self.anchor),
            "membership": [asdict(k) for k in self.membership],
            "poses": [
                {"keyframe_id": asdict(k), "T_component_keyframe": self.poses[k]}
                for k in self.membership
            ],
            "retracted_constraints": sorted(self.retracted_constraints),
            "retracted_keyframes": [
                asdict(k) for k in sorted(self.retracted_keyframes)
            ],
        }

    @property
    def digest(self) -> str:
        raw = json.dumps(
            self.canonical_dict(), sort_keys=True, separators=(",", ":")
        ).encode()
        return hashlib.sha256(raw).hexdigest()


@dataclass(frozen=True)
class ChunkRef:
    sha256: str
    encoding: str
    size_bytes: int
    bounds: Bounds3
    point_count: int

    def __post_init__(self) -> None:
        payload = PayloadRef(self.sha256, self.encoding, self.size_bytes)
        object.__setattr__(self, "sha256", payload.sha256)
        if (
            not isinstance(self.point_count, int)
            or isinstance(self.point_count, bool)
            or self.point_count < 0
        ):
            raise ValueError("point_count must be non-negative")
        clean_bounds = _matrix(self.bounds, 2, 3, "chunk bounds")
        if any(clean_bounds[0][axis] > clean_bounds[1][axis] for axis in range(3)):
            raise ValueError("chunk bounds minima exceed maxima")
        object.__setattr__(self, "bounds", clean_bounds)


@dataclass(frozen=True)
class SubmapRevision:
    submap_id: SubmapId
    geometry_revision: int
    pose_revision: ComponentRevision
    T_component_submap: Matrix4
    keyframes: tuple[KeyframeId, ...]
    chunks: tuple[ChunkRef, ...]
    bounds: Bounds3
    resolution_m: float
    replaces_geometry_revision: int | None = None
    observed_at_ns: int = 0
    sensor_origins: tuple[tuple[float, float, float], ...] = ()
    ray_evidence: RayEvidence = RayEvidence()

    def __post_init__(self) -> None:
        if (
            not isinstance(self.geometry_revision, int)
            or isinstance(self.geometry_revision, bool)
            or self.geometry_revision < 0
        ):
            raise ValueError("geometry_revision must be non-negative")
        object.__setattr__(
            self,
            "T_component_submap",
            validate_se3(self.T_component_submap, "T_component_submap"),
        )
        if not self.keyframes:
            raise ValueError("submap revision requires at least one keyframe")
        if not math.isfinite(self.resolution_m) or self.resolution_m <= 0:
            raise ValueError("resolution_m must be finite and positive")
        if self.replaces_geometry_revision is not None:
            if (
                not isinstance(self.replaces_geometry_revision, int)
                or isinstance(self.replaces_geometry_revision, bool)
                or self.replaces_geometry_revision < 0
                or self.replaces_geometry_revision >= self.geometry_revision
            ):
                raise ValueError(
                    "a geometry revision may replace only an older revision"
                )
        if (
            not isinstance(self.observed_at_ns, int)
            or isinstance(self.observed_at_ns, bool)
            or self.observed_at_ns < 0
        ):
            raise ValueError("observed_at_ns must be a non-negative integer")
        origins = tuple(
            tuple(float(coordinate) for coordinate in origin)
            for origin in self.sensor_origins
        )
        if any(
            len(origin) != 3 or not all(math.isfinite(value) for value in origin)
            for origin in origins
        ):
            raise ValueError("sensor_origins must contain finite XYZ triples")
        object.__setattr__(self, "sensor_origins", origins)
        evidence = self.ray_evidence
        if isinstance(evidence, Mapping):
            evidence = RayEvidence(**evidence)
        if not isinstance(evidence, RayEvidence):
            raise ValueError("ray_evidence must be RayEvidence")
        if evidence.certifies_free_space and len(origins) != 1:
            raise ValueError(
                "qualified ray evidence requires exactly one sensor origin"
            )
        object.__setattr__(self, "ray_evidence", evidence)


@dataclass(frozen=True)
class MapManifest:
    map_id: str
    layer_id: str
    frame_id: str
    graph_revision: ComponentRevision
    geometry_revision: str
    submaps: tuple[SubmapRevision, ...]
    chunks: tuple[ChunkRef, ...]
    tombstones: tuple[str, ...]

    def __post_init__(self) -> None:
        _text_id(self.map_id, "map_id")
        _text_id(self.layer_id, "layer_id")
        _frame_id(self.frame_id, "frame_id")
        if not re.fullmatch(r"[0-9a-f]{64}", self.geometry_revision):
            raise ValueError("geometry_revision must be lowercase SHA-256")
        if any(submap.pose_revision != self.graph_revision for submap in self.submaps):
            raise ValueError("all submaps must belong to the manifest graph revision")
        if len({submap.submap_id for submap in self.submaps}) != len(self.submaps):
            raise ValueError("manifest contains a duplicate submap")
        expected_chunks = {
            chunk.sha256: chunk for submap in self.submaps for chunk in submap.chunks
        }
        actual_chunks = {chunk.sha256: chunk for chunk in self.chunks}
        if len(actual_chunks) != len(self.chunks) or actual_chunks != expected_chunks:
            raise ValueError("manifest chunk table must exactly cover its submaps")

    def to_dict(self) -> dict[str, Any]:
        return _jsonable({"schema": SCHEMA_VERSION, **asdict(self)})


@dataclass(frozen=True)
class MapSnapshot:
    snapshot_id: str
    generated_at_ns: int
    manifests: tuple[MapManifest, ...]

    def __post_init__(self) -> None:
        if (
            not isinstance(self.generated_at_ns, int)
            or isinstance(self.generated_at_ns, bool)
            or self.generated_at_ns < 0
        ):
            raise ValueError("generated_at_ns must be a non-negative integer")
        expected = hashlib.sha256(
            json.dumps(
                [manifest.to_dict() for manifest in self.manifests],
                sort_keys=True,
                separators=(",", ":"),
            ).encode()
        ).hexdigest()
        if self.snapshot_id != expected:
            raise ValueError("snapshot_id does not match the canonical manifests")

    def to_dict(self) -> dict[str, Any]:
        return _jsonable({"schema": SCHEMA_VERSION, **asdict(self)})

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"))


def _jsonable(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, Mapping):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(v) for v in value]
    return value


IDENTITY_SE3: Matrix4 = (
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.0),
    (0.0, 0.0, 0.0, 1.0),
)

ZERO_COVARIANCE: Matrix6 = tuple(tuple(0.0 for _ in range(6)) for _ in range(6))  # type: ignore[assignment]
