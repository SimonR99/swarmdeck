"""Keyframe upload format.

A keyframe is one node of the collaborative pose graph: a voxel-downsampled
cloud, the odometry pose it was captured at, and a place-recognition descriptor.
Robots stream these instead of occupancy grids, because the back-end optimizes
*trajectories* and renders occupancy from the result -- it never registers grids
against each other.

Wire layout (one self-describing blob, one POST body)::

    b"SDKF"              4  magic
    uint16 le            2  KEYFRAME_WIRE_VERSION
    uint32 le            4  header length
    <header>             n  UTF-8 JSON, fields below
    <body>               *  zlib(int16 points ++ uint8 descriptor)

The header carries only scalars, so a reader can size and validate every array
before allocating anything. The body is a single zlib stream holding, in order:

    int16 le  points[n_points, 3]        metres / scale, base frame at capture
    uint8     descriptor[rings, sectors] omitted entirely when descriptor is null

Why int16: at the default 0.01 m scale it spans +/-327 m, comfortably beyond any
single keyframe's extent, at a quarter the size of float64 and half of float32.
Quantization noise is 5 mm RMS, an order of magnitude below the voxel sizes these
clouds are downsampled to, so it is not a meaningful error source.

Conventions that the rest of the system depends on:

* ``T_odom_base`` is ``[x, y, z, qx, qy, qz, qw]`` -- ROS quaternion order,
  scalar LAST. It maps points in ``base`` into ``odom``.
* Points are in the **base frame at capture time**, already de-skewed and with
  the sensor extrinsic applied. Rendering a keyframe is then exactly
  ``T_world_base @ points``, with no per-robot extrinsic lookup at render time.
* ``stamp`` is UNIX seconds as a float.
"""

from __future__ import annotations

import json
import struct
import zlib
from dataclasses import dataclass
from typing import Any, Final

import numpy as np

KEYFRAME_MAGIC: Final = b"SDKF"
KEYFRAME_WIRE_VERSION: Final = 1

#: Reject anything larger before decompressing. A 0.2 m-voxel indoor keyframe is
#: 20-40k points (~60-80 KB compressed); 8 MB is far above any legitimate frame
#: and bounds the damage a malformed or hostile body can do.
MAX_KEYFRAME_BYTES: Final = 8 * 1024 * 1024

#: Upper bound on decompressed body size, enforced incrementally so a zip bomb
#: cannot be expanded in full before it is rejected.
MAX_DECOMPRESSED_BYTES: Final = 64 * 1024 * 1024

_HEADER_STRUCT: Final = struct.Struct("<4sHI")
_INT16_LIMIT: Final = 32767


class ProtocolError(ValueError):
    """Raised when a blob is not a valid keyframe. Always safe to surface."""


@dataclass(frozen=True, slots=True)
class Descriptor:
    """A place-recognition descriptor attached to a keyframe.

    ``kind`` is carried explicitly so a future descriptor can be introduced
    without a wire-version bump: readers that do not know a kind skip the
    descriptor and keep the cloud, which degrades loop closure rather than
    dropping the keyframe.
    """

    kind: str
    data: np.ndarray  # uint8 [rings, sectors]
    max_range: float

    def __post_init__(self) -> None:
        if self.data.dtype != np.uint8:
            raise ProtocolError(f"descriptor data must be uint8, got {self.data.dtype}")
        if self.data.ndim != 2:
            raise ProtocolError(f"descriptor data must be 2-D, got shape {self.data.shape}")


@dataclass(frozen=True, slots=True)
class KeyframePacket:
    """One decoded keyframe, ready to become a pose-graph node."""

    robot_id: str
    seq: int
    stamp: float
    points: np.ndarray  # float32 [n, 3], base frame at capture
    t_odom_base: np.ndarray  # float64 [7] -> x, y, z, qx, qy, qz, qw
    descriptor: Descriptor | None

    @property
    def n_points(self) -> int:
        return int(self.points.shape[0])


def _validate_points(points: np.ndarray) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ProtocolError(f"points must be [n, 3], got shape {pts.shape}")
    if not np.isfinite(pts).all():
        raise ProtocolError("points contain non-finite values")
    return pts


def _validate_pose(t_odom_base: Any) -> np.ndarray:
    pose = np.asarray(t_odom_base, dtype=np.float64).reshape(-1)
    if pose.shape != (7,):
        raise ProtocolError(f"t_odom_base must be 7 floats [x,y,z,qx,qy,qz,qw], got {pose.shape}")
    if not np.isfinite(pose).all():
        raise ProtocolError("t_odom_base contains non-finite values")
    norm = float(np.linalg.norm(pose[3:]))
    if not 0.9 < norm < 1.1:
        raise ProtocolError(f"t_odom_base quaternion is not unit length (|q| = {norm:.4f})")
    return pose


def encode_keyframe(
    *,
    robot_id: str,
    seq: int,
    stamp: float,
    points: np.ndarray,
    t_odom_base: Any,
    descriptor: Descriptor | None = None,
    scale: float = 0.01,
) -> bytes:
    """Serialize one keyframe. Raises :class:`ProtocolError` on invalid input.

    Points beyond the int16 range at ``scale`` are dropped rather than clipped:
    clipping would fabricate a return at the range limit, and a phantom surface
    at a fixed radius is precisely the kind of artifact that produces confident
    false loop closures.
    """
    if not robot_id:
        raise ProtocolError("robot_id must be non-empty")
    if scale <= 0.0:
        raise ProtocolError(f"scale must be positive, got {scale}")

    pts = _validate_points(points)
    pose = _validate_pose(t_odom_base)

    quantized = np.rint(pts / scale)
    in_range = (np.abs(quantized) <= _INT16_LIMIT).all(axis=1)
    quantized = quantized[in_range].astype("<i2", copy=False)

    header: dict[str, Any] = {
        "robot_id": robot_id,
        "seq": int(seq),
        "stamp": float(stamp),
        "scale": float(scale),
        "n_points": int(quantized.shape[0]),
        "frame": "base",
        "t_odom_base": pose.tolist(),
        "descriptor": None,
    }

    body = quantized.tobytes(order="C")
    if descriptor is not None:
        rings, sectors = descriptor.data.shape
        header["descriptor"] = {
            "kind": descriptor.kind,
            "rings": int(rings),
            "sectors": int(sectors),
            "max_range": float(descriptor.max_range),
        }
        body += descriptor.data.tobytes(order="C")

    header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
    blob = (
        _HEADER_STRUCT.pack(KEYFRAME_MAGIC, KEYFRAME_WIRE_VERSION, len(header_bytes))
        + header_bytes
        + zlib.compress(body, level=6)
    )
    if len(blob) > MAX_KEYFRAME_BYTES:
        raise ProtocolError(
            f"encoded keyframe is {len(blob)} bytes, over the {MAX_KEYFRAME_BYTES} limit; "
            "increase the voxel size before uploading"
        )
    return blob


def _decompress_bounded(payload: bytes) -> bytes:
    """zlib-decompress with a hard ceiling, so a zip bomb cannot exhaust memory."""
    decompressor = zlib.decompressobj()
    out = decompressor.decompress(payload, MAX_DECOMPRESSED_BYTES)
    if decompressor.unconsumed_tail:
        raise ProtocolError("keyframe body expands past the decompression limit")
    if not decompressor.eof:
        raise ProtocolError("keyframe body is a truncated zlib stream")
    return out


def peek_keyframe_header(blob: bytes) -> dict[str, Any]:
    """Parse only the JSON header. Does not decompress or allocate the cloud.

    The server uses this to check identity (``robot_id`` in the blob matches
    the query string) before it forwards the opaque body to the SLAM process.
    Decompressing here would put zip-bomb expansion on the FastAPI event loop
    and would make the server a second decoder of a format it is supposed to
    pipe, not interpret.
    """
    if len(blob) > MAX_KEYFRAME_BYTES:
        raise ProtocolError(f"keyframe is {len(blob)} bytes, over the {MAX_KEYFRAME_BYTES} limit")
    if len(blob) < _HEADER_STRUCT.size:
        raise ProtocolError("keyframe is too short to contain a header")

    magic, version, header_len = _HEADER_STRUCT.unpack_from(blob)
    if magic != KEYFRAME_MAGIC:
        raise ProtocolError(f"bad magic {magic!r}, expected {KEYFRAME_MAGIC!r}")
    if version != KEYFRAME_WIRE_VERSION:
        raise ProtocolError(f"unsupported keyframe wire version {version}")

    start = _HEADER_STRUCT.size
    end = start + header_len
    if header_len < 0 or end > len(blob):
        raise ProtocolError("header length runs past the end of the blob")

    try:
        header = json.loads(blob[start:end].decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"header is not valid UTF-8 JSON: {exc}") from exc
    if not isinstance(header, dict):
        raise ProtocolError("header must be a JSON object")
    return header


def decode_keyframe(blob: bytes) -> KeyframePacket:
    """Parse a keyframe blob. Raises :class:`ProtocolError` on anything malformed.

    Every length is checked against the declared header before it is used, so a
    corrupt or hostile body fails cleanly instead of producing a mis-shaped array.
    """
    header = peek_keyframe_header(blob)
    header_len = _HEADER_STRUCT.unpack_from(blob)[2]
    end = _HEADER_STRUCT.size + header_len

    try:
        body = _decompress_bounded(blob[end:])
    except zlib.error as exc:
        raise ProtocolError(f"body is not a valid zlib stream: {exc}") from exc

    try:
        robot_id = str(header["robot_id"])
        seq = int(header["seq"])
        stamp = float(header["stamp"])
        scale = float(header["scale"])
        n_points = int(header["n_points"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ProtocolError(f"header is missing or has a malformed field: {exc}") from exc

    if not robot_id:
        raise ProtocolError("robot_id must be non-empty")
    if scale <= 0.0:
        raise ProtocolError(f"scale must be positive, got {scale}")
    if n_points < 0:
        raise ProtocolError(f"n_points must be non-negative, got {n_points}")

    points_bytes = n_points * 3 * 2
    if points_bytes > len(body):
        raise ProtocolError(
            f"header declares {n_points} points ({points_bytes} bytes) "
            f"but the body holds only {len(body)}"
        )

    points = (
        np.frombuffer(body, dtype="<i2", count=n_points * 3).reshape(n_points, 3).astype(np.float32)
        * scale
    )

    descriptor = _decode_descriptor(header.get("descriptor"), body, points_bytes)
    pose = _validate_pose(header.get("t_odom_base"))

    return KeyframePacket(
        robot_id=robot_id,
        seq=seq,
        stamp=stamp,
        points=points,
        t_odom_base=pose,
        descriptor=descriptor,
    )


def _decode_descriptor(spec: Any, body: bytes, offset: int) -> Descriptor | None:
    if spec is None:
        return None
    if not isinstance(spec, dict):
        raise ProtocolError("descriptor header must be a JSON object or null")
    try:
        kind = str(spec["kind"])
        rings = int(spec["rings"])
        sectors = int(spec["sectors"])
        max_range = float(spec["max_range"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ProtocolError(f"descriptor header is malformed: {exc}") from exc

    if rings <= 0 or sectors <= 0:
        raise ProtocolError(f"descriptor dimensions must be positive, got {rings}x{sectors}")

    needed = rings * sectors
    if offset + needed > len(body):
        raise ProtocolError(
            f"descriptor declares {rings}x{sectors} = {needed} bytes but the body "
            f"holds only {len(body) - offset} after the points"
        )

    data = np.frombuffer(body, dtype=np.uint8, count=needed, offset=offset).reshape(rings, sectors)
    return Descriptor(kind=kind, data=data, max_range=max_range)
