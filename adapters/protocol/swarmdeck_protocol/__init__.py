"""SwarmDeck wire protocol — shared by adapters, server, and the SLAM back-end.

This package is the single source of truth for on-the-wire formats. Encoding and
decoding live together on purpose: a format defined in one place cannot drift
between the process that writes it and the process that reads it.
"""

from .keyframe import (
    KEYFRAME_MAGIC,
    KEYFRAME_WIRE_VERSION,
    MAX_KEYFRAME_BYTES,
    MAX_SESSION_CHARS,
    Descriptor,
    KeyframePacket,
    ProtocolError,
    decode_keyframe,
    encode_keyframe,
    peek_keyframe_header,
)
from .odometry import (
    DEFAULT_HARDWARE_ODOMETRY,
    DEFAULT_SIM_ODOMETRY,
    ODOMETRY_ALIASES,
    ODOMETRY_PROFILES,
    OdometrySpec,
    get_odometry_spec,
    resolve_odometry_types,
)

__all__ = [
    "KEYFRAME_MAGIC",
    "KEYFRAME_WIRE_VERSION",
    "MAX_KEYFRAME_BYTES",
    "MAX_SESSION_CHARS",
    "Descriptor",
    "KeyframePacket",
    "ProtocolError",
    "decode_keyframe",
    "encode_keyframe",
    "peek_keyframe_header",
    "DEFAULT_HARDWARE_ODOMETRY",
    "DEFAULT_SIM_ODOMETRY",
    "ODOMETRY_ALIASES",
    "ODOMETRY_PROFILES",
    "OdometrySpec",
    "get_odometry_spec",
    "resolve_odometry_types",
]
