"""Correction-aware submaps, coherent snapshots, and terrain queries.

The store keeps geometry immutable and content-addressed. Pose corrections only
update metadata. Replacing geometry advances a submap revision and retires its
old occupancy contribution, so replaying a loop correction cannot leave doubled
walls behind.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import sqlite3
import struct
import threading
import time
from dataclasses import asdict, dataclass
from enum import Enum
from pathlib import Path
from typing import Iterable, Mapping, Sequence

import numpy as np

from .contracts import (
    IDENTITY_SE3,
    Bounds3,
    Calibration,
    CalibratedCapture,
    ChunkRef,
    ComponentRevision,
    GraphSolution,
    KeyframeId,
    MapManifest,
    MapSnapshot,
    Matrix4,
    SubmapId,
    SubmapRevision,
    component_id_for_anchor,
    validate_se3,
)

XYZ_F32_ENCODING = "application/vnd.swarmdeck.xyz-f32.v1"
_XYZ_MAGIC = b"SDXYZ1\x00\x00"
MAX_CHUNK_BYTES = 8 * 1024 * 1024
_MAX_POINTS_PER_CHUNK = (MAX_CHUNK_BYTES - 16) // 12


class StaleSolutionError(ValueError):
    """A solution would move a component epoch/revision backwards."""


class DuplicateConflictError(ValueError):
    """An immutable identity or solution revision was reused with other data."""


class StorageBudgetExceeded(RuntimeError):
    """The durable geometry store cannot admit another immutable chunk."""


class OccupancyState(str, Enum):
    UNKNOWN = "unknown"
    FREE = "free"
    OCCUPIED = "occupied"


@dataclass(frozen=True)
class OccupancyQuery:
    state: OccupancyState
    observation_age_ns: int | None
    component_revision: ComponentRevision | None
    geometry_revision: str | None


@dataclass(frozen=True)
class TerrainQuery:
    known: bool
    ground_height_m: float | None
    ground_normal: tuple[float, float, float] | None
    step_up_m: float | None
    drop_m: float | None
    roughness_m: float | None
    clearance_m: float | None
    support_confidence: float
    observation_age_ns: int | None
    component_revision: ComponentRevision | None
    geometry_revision: str | None


@dataclass(frozen=True)
class _StoredSubmap:
    submap_id: SubmapId
    geometry_revision: int
    keyframes: tuple[KeyframeId, ...]
    local_keyframe_poses: Mapping[KeyframeId, Matrix4]
    chunks: tuple[ChunkRef, ...]
    sensor_origins: tuple[tuple[float, float, float], ...]
    bounds: Bounds3
    resolution_m: float
    observed_at_ns: int
    replaces: int | None
    component_revision: ComponentRevision
    T_component_submap: Matrix4


def _geometry_digest(submaps: Sequence[_StoredSubmap]) -> str:
    content = [
        (
            submap.submap_id.stable_id,
            submap.geometry_revision,
            [c.sha256 for c in submap.chunks],
        )
        for submap in sorted(submaps, key=lambda item: item.submap_id.stable_id)
    ]
    return hashlib.sha256(
        json.dumps(content, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()


def _encode_points(points: np.ndarray) -> bytes:
    cloud = np.ascontiguousarray(points, dtype="<f4")
    return _XYZ_MAGIC + struct.pack("<Q", cloud.shape[0]) + cloud.tobytes(order="C")


def decode_xyz_f32(payload: bytes) -> np.ndarray:
    """Decode the stable little-endian chunk format used by the native bridge."""

    if len(payload) < 16 or payload[:8] != _XYZ_MAGIC:
        raise ValueError("not a SwarmDeck XYZ-F32 chunk")
    count = struct.unpack("<Q", payload[8:16])[0]
    if len(payload) != 16 + count * 12:
        raise ValueError("XYZ-F32 chunk length does not match point count")
    return np.frombuffer(payload, dtype="<f4", offset=16).reshape((-1, 3)).copy()


def _points(value: Sequence[Sequence[float]] | np.ndarray) -> np.ndarray:
    points = np.asarray(value, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("point cloud must have shape Nx3")
    if not np.isfinite(points).all():
        raise ValueError("point cloud contains a nonfinite value")
    return points


def _bounds(points: np.ndarray) -> Bounds3:
    if len(points) == 0:
        return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    return (
        tuple(float(v) for v in points.min(axis=0)),
        tuple(float(v) for v in points.max(axis=0)),
    )  # type: ignore[return-value]


def _transform(T_a_b: Matrix4, points_b: np.ndarray) -> np.ndarray:
    rotation = np.asarray(T_a_b, dtype=np.float64)[:3, :3]
    translation = np.asarray(T_a_b, dtype=np.float64)[:3, 3]
    return points_b @ rotation.T + translation


def _inverse(T_a_b: Matrix4) -> Matrix4:
    matrix = np.asarray(T_a_b, dtype=np.float64)
    rotation = matrix[:3, :3]
    result = np.eye(4)
    result[:3, :3] = rotation.T
    result[:3, 3] = -(rotation.T @ matrix[:3, 3])
    return tuple(tuple(float(v) for v in row) for row in result)  # type: ignore[return-value]


def _compose(T_a_b: Matrix4, T_b_c: Matrix4) -> Matrix4:
    result = np.asarray(T_a_b) @ np.asarray(T_b_c)
    return tuple(tuple(float(v) for v in row) for row in result)  # type: ignore[return-value]


def _pose_distance(a: Matrix4, b: Matrix4) -> tuple[float, float]:
    delta = np.asarray(_compose(_inverse(a), b))
    translation = float(np.linalg.norm(delta[:3, 3]))
    angle = math.acos(float(np.clip((np.trace(delta[:3, :3]) - 1.0) / 2.0, -1, 1)))
    return translation, angle


def _keyframe_dict(keyframe: KeyframeId) -> dict[str, object]:
    return asdict(keyframe)


class SubmapStore:
    """Durable SQLite metadata plus immutable content-addressed chunk files."""

    def __init__(
        self,
        root: str | os.PathLike[str],
        *,
        max_chunk_bytes: int = 512 * 1024 * 1024,
    ):
        if (
            not isinstance(max_chunk_bytes, int)
            or isinstance(max_chunk_bytes, bool)
            or max_chunk_bytes <= 0
        ):
            raise ValueError("max_chunk_bytes must be a positive integer")
        self.root = Path(root)
        self.max_chunk_bytes = max_chunk_bytes
        self.chunks_dir = self.root / "chunks"
        self.root.mkdir(parents=True, exist_ok=True)
        self.chunks_dir.mkdir(parents=True, exist_ok=True)
        self._db = sqlite3.connect(
            self.root / "mapping.sqlite3", isolation_level=None, check_same_thread=False
        )
        self._db.row_factory = sqlite3.Row
        self._lock = threading.RLock()
        self._db.execute("PRAGMA journal_mode=WAL")
        self._db.execute("PRAGMA foreign_keys=ON")
        self._create_schema()

    def close(self) -> None:
        with self._lock:
            self._db.close()

    def __enter__(self) -> "SubmapStore":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _create_schema(self) -> None:
        self._db.executescript("""
            CREATE TABLE IF NOT EXISTS chunks (
              sha256 TEXT PRIMARY KEY, encoding TEXT NOT NULL, size_bytes INTEGER NOT NULL,
              point_count INTEGER NOT NULL, bounds_json TEXT NOT NULL
            );
            CREATE TABLE IF NOT EXISTS submap_revisions (
              submap_id TEXT NOT NULL, geometry_revision INTEGER NOT NULL,
              robot_id TEXT NOT NULL, session_id TEXT NOT NULL, submap_seq INTEGER NOT NULL,
              keyframes_json TEXT NOT NULL, local_poses_json TEXT NOT NULL,
              chunks_json TEXT NOT NULL, sensor_origins_json TEXT NOT NULL,
              bounds_json TEXT NOT NULL, resolution_m REAL NOT NULL,
              observed_at_ns INTEGER NOT NULL, replaces_revision INTEGER,
              PRIMARY KEY(submap_id, geometry_revision)
            );
            CREATE TABLE IF NOT EXISTS active_submaps (
              submap_id TEXT PRIMARY KEY, geometry_revision INTEGER NOT NULL,
              component_id TEXT NOT NULL, epoch INTEGER NOT NULL, solution_revision INTEGER NOT NULL,
              pose_json TEXT NOT NULL, active INTEGER NOT NULL, tombstone_reason TEXT
            );
            CREATE TABLE IF NOT EXISTS solution_heads (
              component_id TEXT PRIMARY KEY, epoch INTEGER NOT NULL,
              revision INTEGER NOT NULL, digest TEXT NOT NULL
            );
            CREATE TABLE IF NOT EXISTS calibrations (
              robot_id TEXT NOT NULL, session_id TEXT NOT NULL, version TEXT NOT NULL,
              digest TEXT NOT NULL, calibration_json TEXT NOT NULL,
              PRIMARY KEY(robot_id, session_id, version)
            );
            CREATE TABLE IF NOT EXISTS captures (
              keyframe_id TEXT PRIMARY KEY, robot_id TEXT NOT NULL,
              session_id TEXT NOT NULL, seq INTEGER NOT NULL,
              digest TEXT NOT NULL, capture_json TEXT NOT NULL,
              calibration_json TEXT NOT NULL
            );
            """)

    def record_capture(
        self, capture: CalibratedCapture, calibration: Calibration
    ) -> bool:
        """Persist calibrated source evidence; return false for an exact replay."""

        capture_json = json.dumps(
            asdict(capture), sort_keys=True, separators=(",", ":")
        )
        calibration_json = json.dumps(
            asdict(calibration), sort_keys=True, separators=(",", ":")
        )
        capture_digest = hashlib.sha256(
            (capture_json + "\n" + calibration_json).encode()
        ).hexdigest()
        calibration_digest = hashlib.sha256(calibration_json.encode()).hexdigest()
        key = capture.keyframe_id
        with self._lock:
            self._db.execute("BEGIN IMMEDIATE")
            try:
                old_calibration = self._db.execute(
                    """SELECT digest FROM calibrations
                       WHERE robot_id=? AND session_id=? AND version=?""",
                    (key.robot_id, key.session_id, calibration.version),
                ).fetchone()
                if (
                    old_calibration is not None
                    and old_calibration["digest"] != calibration_digest
                ):
                    raise DuplicateConflictError(
                        f"calibration version was reused with other values: {calibration.version}"
                    )
                self._db.execute(
                    "INSERT OR IGNORE INTO calibrations VALUES (?, ?, ?, ?, ?)",
                    (
                        key.robot_id,
                        key.session_id,
                        calibration.version,
                        calibration_digest,
                        calibration_json,
                    ),
                )
                old_capture = self._db.execute(
                    "SELECT digest FROM captures WHERE keyframe_id=?", (key.stable_id,)
                ).fetchone()
                if old_capture is not None:
                    if old_capture["digest"] != capture_digest:
                        raise DuplicateConflictError(
                            f"keyframe capture was reused with other values: {key.stable_id}"
                        )
                    self._db.execute("COMMIT")
                    return False
                self._db.execute(
                    "INSERT INTO captures VALUES (?, ?, ?, ?, ?, ?, ?)",
                    (
                        key.stable_id,
                        key.robot_id,
                        key.session_id,
                        key.seq,
                        capture_digest,
                        capture_json,
                        calibration_json,
                    ),
                )
                self._db.execute("COMMIT")
                return True
            except Exception:
                self._db.execute("ROLLBACK")
                raise

    def get_capture(self, keyframe_id: KeyframeId) -> dict[str, object]:
        """Return the canonical durable capture/calibration record."""

        with self._lock:
            row = self._db.execute(
                """SELECT digest, capture_json, calibration_json FROM captures
                   WHERE keyframe_id=?""",
                (keyframe_id.stable_id,),
            ).fetchone()
        if row is None:
            raise KeyError(keyframe_id.stable_id)
        return {
            "digest": row["digest"],
            "capture": json.loads(row["capture_json"]),
            "calibration": json.loads(row["calibration_json"]),
        }

    def put_chunk(
        self, payload: bytes, *, point_count: int, bounds: Bounds3
    ) -> ChunkRef:
        if len(payload) > MAX_CHUNK_BYTES:
            raise ValueError(
                f"map chunk exceeds {MAX_CHUNK_BYTES} byte interchange limit"
            )
        if (
            not isinstance(point_count, int)
            or isinstance(point_count, bool)
            or point_count < 0
        ):
            raise ValueError("point_count must be a non-negative integer")
        if (
            not payload.startswith(_XYZ_MAGIC)
            or len(payload) < 16
            or struct.unpack("<Q", payload[8:16])[0] != point_count
            or len(payload) != 16 + 12 * point_count
        ):
            raise ValueError("invalid XYZ-F32 chunk payload")
        digest = hashlib.sha256(payload).hexdigest()
        result = ChunkRef(digest, XYZ_F32_ENCODING, len(payload), bounds, point_count)
        path = self.chunks_dir / digest
        with self._lock:
            self._db.execute("BEGIN IMMEDIATE")
            temporary: Path | None = None
            try:
                existing = self._db.execute(
                    "SELECT size_bytes, point_count, bounds_json FROM chunks WHERE sha256=?",
                    (digest,),
                ).fetchone()
                if existing is None:
                    used = self._db.execute(
                        "SELECT COALESCE(SUM(size_bytes), 0) AS bytes FROM chunks"
                    ).fetchone()["bytes"]
                    if int(used) + len(payload) > self.max_chunk_bytes:
                        raise StorageBudgetExceeded(
                            f"map chunks need {int(used) + len(payload)} bytes; "
                            f"budget is {self.max_chunk_bytes}"
                        )
                    if path.exists():
                        present = path.read_bytes()
                        if (
                            len(present) != len(payload)
                            or hashlib.sha256(present).hexdigest() != digest
                        ):
                            raise IOError(f"corrupt orphan map chunk {digest}")
                    else:
                        temporary = self.chunks_dir / f".{digest}.{os.getpid()}.tmp"
                        with temporary.open("xb") as stream:
                            stream.write(payload)
                            stream.flush()
                            os.fsync(stream.fileno())
                        os.replace(temporary, path)
                        temporary = None
                    self._db.execute(
                        "INSERT INTO chunks VALUES (?, ?, ?, ?, ?)",
                        (
                            digest,
                            XYZ_F32_ENCODING,
                            len(payload),
                            point_count,
                            json.dumps(bounds),
                        ),
                    )
                elif int(existing["size_bytes"]) != len(payload):
                    raise IOError(f"chunk metadata collision for {digest}")
                elif (
                    not path.exists()
                    or hashlib.sha256(path.read_bytes()).hexdigest() != digest
                ):
                    raise IOError(f"corrupt map chunk {digest}")
                self._db.execute("COMMIT")
            except Exception:
                self._db.execute("ROLLBACK")
                if temporary is not None and temporary.exists():
                    temporary.unlink()
                raise
        return result

    def get_chunk(self, sha256: str) -> bytes:
        """Return immutable bytes for HTTP/peer replication by their digest."""

        if (
            not isinstance(sha256, str)
            or len(sha256) != 64
            or any(char not in "0123456789abcdef" for char in sha256)
        ):
            raise KeyError(sha256)
        with self._lock:
            row = self._db.execute(
                "SELECT size_bytes FROM chunks WHERE sha256=?", (sha256,)
            ).fetchone()
        if row is None:
            raise KeyError(sha256)
        payload = (self.chunks_dir / sha256).read_bytes()
        if (
            len(payload) != row["size_bytes"]
            or hashlib.sha256(payload).hexdigest() != sha256
        ):
            raise IOError(f"corrupt map chunk {sha256}")
        return payload

    def add_submap(
        self,
        submap_id: SubmapId,
        points_local: Sequence[Sequence[float]] | np.ndarray,
        *,
        keyframe_poses_local: Mapping[KeyframeId, Matrix4],
        sensor_origins_local: Iterable[Sequence[float]],
        resolution_m: float,
        observed_at_ns: int,
        replace: bool = False,
        initial_T_component_submap: Matrix4 = IDENTITY_SE3,
    ) -> int:
        cloud = _points(points_local)
        if len(cloud) == 0:
            raise ValueError("point cloud must not be empty")
        if not math.isfinite(resolution_m) or resolution_m <= 0:
            raise ValueError("resolution_m must be finite and positive")
        if not keyframe_poses_local:
            raise ValueError("a submap must reference at least one keyframe")
        poses = {
            key: validate_se3(pose, "T_submap_keyframe")
            for key, pose in keyframe_poses_local.items()
        }
        origins = tuple(
            tuple(float(x) for x in origin) for origin in sensor_origins_local
        )
        if any(
            len(origin) != 3 or not all(math.isfinite(x) for x in origin)
            for origin in origins
        ):
            raise ValueError("sensor origins must be finite XYZ triples")
        bounds = _bounds(cloud)
        initial_pose = validate_se3(
            initial_T_component_submap, "initial_T_component_submap"
        )
        chunks = tuple(
            self.put_chunk(
                _encode_points(part), point_count=len(part), bounds=_bounds(part)
            )
            for start in range(0, len(cloud), _MAX_POINTS_PER_CHUNK)
            for part in (cloud[start : start + _MAX_POINTS_PER_CHUNK],)
        )
        # One origin applies to every chunk generated from this cloud. Multiple
        # origins have no point/ray association in this API and therefore stay
        # conservative (occupied endpoints only) in occupancy_query().
        stored_origins = origins
        stable_id = submap_id.stable_id
        with self._lock:
            self._db.execute("BEGIN IMMEDIATE")
            try:
                current = self._db.execute(
                    "SELECT geometry_revision FROM active_submaps WHERE submap_id=?",
                    (stable_id,),
                ).fetchone()
                if current is not None and not replace:
                    existing = self._load_revision(
                        stable_id, current["geometry_revision"]
                    )
                    same = (
                        existing.chunks == chunks
                        and existing.local_keyframe_poses == poses
                        and existing.sensor_origins == stored_origins
                        and existing.resolution_m == resolution_m
                        and existing.observed_at_ns == observed_at_ns
                    )
                    if same:
                        self._db.execute("COMMIT")
                        return int(current["geometry_revision"])
                    raise DuplicateConflictError(
                        f"submap identity already exists: {stable_id}"
                    )
                revision = (
                    0 if current is None else int(current["geometry_revision"]) + 1
                )
                trajectory_anchor = KeyframeId(
                    submap_id.robot_id, submap_id.session_id, 0
                )
                initial_component = component_id_for_anchor(trajectory_anchor)
                initial_graph = ComponentRevision(initial_component, 0, 0)
                if current is None:
                    active_values = (
                        initial_graph.component_id,
                        initial_graph.epoch,
                        initial_graph.revision,
                        json.dumps(initial_pose),
                    )
                else:
                    previous = self._db.execute(
                        "SELECT component_id, epoch, solution_revision, pose_json FROM active_submaps WHERE submap_id=?",
                        (stable_id,),
                    ).fetchone()
                    active_values = (
                        previous["component_id"],
                        previous["epoch"],
                        previous["solution_revision"],
                        previous["pose_json"],
                    )
                self._db.execute(
                    """INSERT INTO submap_revisions VALUES
                       (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
                    (
                        stable_id,
                        revision,
                        submap_id.robot_id,
                        submap_id.session_id,
                        submap_id.seq,
                        json.dumps(
                            [_keyframe_dict(k) for k in sorted(poses)], sort_keys=True
                        ),
                        json.dumps(
                            [
                                {"keyframe_id": _keyframe_dict(k), "pose": poses[k]}
                                for k in sorted(poses)
                            ],
                            sort_keys=True,
                        ),
                        json.dumps([asdict(chunk) for chunk in chunks], sort_keys=True),
                        json.dumps(stored_origins),
                        json.dumps(bounds),
                        resolution_m,
                        observed_at_ns,
                        None if current is None else int(current["geometry_revision"]),
                    ),
                )
                self._db.execute(
                    """INSERT INTO active_submaps VALUES (?, ?, ?, ?, ?, ?, 1, NULL)
                       ON CONFLICT(submap_id) DO UPDATE SET
                         geometry_revision=excluded.geometry_revision,
                         component_id=excluded.component_id, epoch=excluded.epoch,
                         solution_revision=excluded.solution_revision, pose_json=excluded.pose_json,
                         active=1, tombstone_reason=NULL""",
                    (stable_id, revision, *active_values),
                )
                self._db.execute("COMMIT")
                return revision
            except Exception:
                self._db.execute("ROLLBACK")
                raise

    def apply_solution(
        self,
        solution: GraphSolution,
        *,
        rigid_translation_tolerance_m: float = 1e-4,
        rigid_rotation_tolerance_rad: float = 1e-4,
    ) -> bool:
        """Atomically adopt one solution; return false for an exact duplicate."""

        rev = solution.revision
        with self._lock:
            self._db.execute("BEGIN IMMEDIATE")
            try:
                head = self._db.execute(
                    "SELECT epoch, revision, digest FROM solution_heads WHERE component_id=?",
                    (rev.component_id,),
                ).fetchone()
                if head is not None:
                    old_pair = (int(head["epoch"]), int(head["revision"]))
                    new_pair = (rev.epoch, rev.revision)
                    if new_pair < old_pair:
                        raise StaleSolutionError(
                            f"solution {new_pair} is older than {old_pair} for {rev.component_id}"
                        )
                    if new_pair == old_pair:
                        if head["digest"] == solution.digest:
                            adopted = False
                        else:
                            raise DuplicateConflictError(
                                f"solution revision {new_pair} was reused with different content"
                            )
                    else:
                        adopted = True
                else:
                    adopted = True

                retracted = {key.stable_id for key in solution.retracted_keyframes}
                rows = self._db.execute(
                    """SELECT a.*, r.keyframes_json, r.local_poses_json
                       FROM active_submaps a JOIN submap_revisions r
                       ON r.submap_id=a.submap_id AND r.geometry_revision=a.geometry_revision"""
                ).fetchall()
                for row in rows:
                    keyframes = tuple(
                        KeyframeId.from_dict(v)
                        for v in json.loads(row["keyframes_json"])
                    )
                    stable_keyframes = {key.stable_id for key in keyframes}
                    if stable_keyframes & retracted:
                        self._deactivate(
                            row["submap_id"], "keyframe_retracted", revision=rev
                        )
                        continue
                    affected = [key for key in keyframes if key in solution.poses]
                    if not affected:
                        continue
                    current_revision = ComponentRevision(
                        row["component_id"], row["epoch"], row["solution_revision"]
                    )
                    provisional = (
                        self._db.execute(
                            "SELECT 1 FROM solution_heads WHERE component_id=?",
                            (current_revision.component_id,),
                        ).fetchone()
                        is None
                    )
                    if (
                        rev.component_id != current_revision.component_id
                        and rev.epoch <= current_revision.epoch
                        and not provisional
                    ):
                        raise StaleSolutionError(
                            "a component change must advance the component epoch"
                        )
                    if rev.component_id == current_revision.component_id and (
                        rev.epoch,
                        rev.revision,
                    ) < (current_revision.epoch, current_revision.revision):
                        raise StaleSolutionError(
                            "solution is stale for an affected submap"
                        )
                    if len(affected) != len(keyframes):
                        self._deactivate(
                            row["submap_id"],
                            "component_split_requires_rebuild",
                            revision=rev,
                        )
                        continue
                    local_poses = {
                        KeyframeId.from_dict(item["keyframe_id"]): validate_se3(
                            item["pose"]
                        )
                        for item in json.loads(row["local_poses_json"])
                    }
                    candidates = [
                        _compose(solution.poses[key], _inverse(local_poses[key]))
                        for key in keyframes
                    ]
                    reference = candidates[0]
                    if any(
                        (
                            distance > rigid_translation_tolerance_m
                            or angle > rigid_rotation_tolerance_rad
                        )
                        for distance, angle in (
                            _pose_distance(reference, value) for value in candidates[1:]
                        )
                    ):
                        self._deactivate(
                            row["submap_id"],
                            "internal_deformation_requires_rebuild",
                            revision=rev,
                        )
                        continue
                    self._db.execute(
                        """UPDATE active_submaps SET component_id=?, epoch=?, solution_revision=?,
                           pose_json=?, active=1, tombstone_reason=NULL WHERE submap_id=?""",
                        (
                            rev.component_id,
                            rev.epoch,
                            rev.revision,
                            json.dumps(reference),
                            row["submap_id"],
                        ),
                    )
                if adopted:
                    self._db.execute(
                        """INSERT INTO solution_heads VALUES (?, ?, ?, ?)
                           ON CONFLICT(component_id) DO UPDATE SET
                             epoch=excluded.epoch, revision=excluded.revision, digest=excluded.digest""",
                        (rev.component_id, rev.epoch, rev.revision, solution.digest),
                    )
                self._db.execute("COMMIT")
                return adopted
            except Exception:
                self._db.execute("ROLLBACK")
                raise

    def _deactivate(
        self,
        submap_id: str,
        reason: str,
        *,
        revision: ComponentRevision | None = None,
    ) -> None:
        if revision is None:
            self._db.execute(
                "UPDATE active_submaps SET active=0, tombstone_reason=? WHERE submap_id=?",
                (reason, submap_id),
            )
            return
        self._db.execute(
            """UPDATE active_submaps SET active=0, tombstone_reason=?,
               component_id=?, epoch=?, solution_revision=? WHERE submap_id=?""",
            (
                reason,
                revision.component_id,
                revision.epoch,
                revision.revision,
                submap_id,
            ),
        )

    def _load_revision(self, stable_id: str, revision: int) -> _StoredSubmap:
        row = self._db.execute(
            "SELECT * FROM submap_revisions WHERE submap_id=? AND geometry_revision=?",
            (stable_id, revision),
        ).fetchone()
        active = self._db.execute(
            "SELECT * FROM active_submaps WHERE submap_id=?", (stable_id,)
        ).fetchone()
        if row is None or active is None:
            raise KeyError(stable_id)
        keyframes = tuple(
            KeyframeId.from_dict(v) for v in json.loads(row["keyframes_json"])
        )
        local_poses = {
            KeyframeId.from_dict(v["keyframe_id"]): validate_se3(v["pose"])
            for v in json.loads(row["local_poses_json"])
        }
        chunks = tuple(
            ChunkRef(
                c["sha256"],
                c["encoding"],
                c["size_bytes"],
                (tuple(c["bounds"][0]), tuple(c["bounds"][1])),
                c["point_count"],
            )
            for c in json.loads(row["chunks_json"])
        )
        return _StoredSubmap(
            SubmapId(row["robot_id"], row["session_id"], row["submap_seq"]),
            row["geometry_revision"],
            keyframes,
            local_poses,
            chunks,
            tuple(tuple(v) for v in json.loads(row["sensor_origins_json"])),
            (
                tuple(json.loads(row["bounds_json"])[0]),
                tuple(json.loads(row["bounds_json"])[1]),
            ),
            row["resolution_m"],
            row["observed_at_ns"],
            row["replaces_revision"],
            ComponentRevision(
                active["component_id"], active["epoch"], active["solution_revision"]
            ),
            validate_se3(json.loads(active["pose_json"])),
        )

    def active_submaps(
        self, component_id: str | None = None
    ) -> tuple[_StoredSubmap, ...]:
        query = "SELECT submap_id, geometry_revision FROM active_submaps WHERE active=1"
        args: tuple[object, ...] = ()
        if component_id is not None:
            query += " AND component_id=?"
            args = (component_id,)
        query += " ORDER BY component_id, submap_id"
        with self._lock:
            rows = self._db.execute(query, args).fetchall()
            return tuple(
                self._load_revision(r["submap_id"], r["geometry_revision"])
                for r in rows
            )

    def snapshot(
        self, *, map_id: str = "onboard", layer_id: str = "persistent_geometry"
    ) -> MapSnapshot:
        """Read all manifests in one SQLite transaction and content-hash them."""

        with self._lock:
            self._db.execute("BEGIN")
            try:
                active = self.active_submaps()
                tombstone_rows = self._db.execute(
                    """SELECT submap_id, tombstone_reason, component_id, epoch, solution_revision
                       FROM active_submaps WHERE active=0 ORDER BY component_id, submap_id"""
                ).fetchall()
                self._db.execute("COMMIT")
            except Exception:
                self._db.execute("ROLLBACK")
                raise
        by_revision: dict[ComponentRevision, list[_StoredSubmap]] = {}
        for submap in active:
            by_revision.setdefault(submap.component_revision, []).append(submap)
        tombstones_by_revision: dict[ComponentRevision, list[str]] = {}
        for row in tombstone_rows:
            revision = ComponentRevision(
                row["component_id"], row["epoch"], row["solution_revision"]
            )
            tombstones_by_revision.setdefault(revision, []).append(
                f"{row['submap_id']}:{row['tombstone_reason']}"
            )
        manifests: list[MapManifest] = []
        revisions = sorted(set(by_revision) | set(tombstones_by_revision))
        for revision in revisions:
            members = by_revision.get(revision, [])
            submap_revisions = tuple(self._public_revision(s) for s in members)
            chunks_by_hash = {c.sha256: c for s in members for c in s.chunks}
            geometry_revision = _geometry_digest(members)
            manifests.append(
                MapManifest(
                    map_id=map_id,
                    layer_id=layer_id,
                    frame_id=revision.component_id.replace(":", "_"),
                    graph_revision=revision,
                    geometry_revision=geometry_revision,
                    submaps=submap_revisions,
                    chunks=tuple(chunks_by_hash[key] for key in sorted(chunks_by_hash)),
                    tombstones=tuple(tombstones_by_revision.get(revision, ())),
                )
            )
        generated_at_ns = time.time_ns()
        content = json.dumps(
            [manifest.to_dict() for manifest in manifests],
            sort_keys=True,
            separators=(",", ":"),
        )
        snapshot_id = hashlib.sha256(content.encode()).hexdigest()
        return MapSnapshot(snapshot_id, generated_at_ns, tuple(manifests))

    @staticmethod
    def _public_revision(submap: _StoredSubmap) -> SubmapRevision:
        return SubmapRevision(
            submap.submap_id,
            submap.geometry_revision,
            submap.component_revision,
            submap.T_component_submap,
            submap.keyframes,
            submap.chunks,
            submap.bounds,
            submap.resolution_m,
            submap.replaces,
            submap.observed_at_ns,
            submap.sensor_origins,
        )


class CorrectionAwareMapper:
    """High-level capture, correction, export, and MGG terrain-query API."""

    def __init__(self, store: SubmapStore, *, resolution_m: float = 0.1):
        if resolution_m <= 0 or not math.isfinite(resolution_m):
            raise ValueError("resolution_m must be finite and positive")
        self.store = store
        self.resolution_m = resolution_m

    def add_capture(
        self,
        capture: CalibratedCapture,
        calibration: Calibration,
        points_sensor: Sequence[Sequence[float]] | np.ndarray,
        *,
        replace: bool = False,
    ) -> int:
        if capture.calibration_version != calibration.version:
            raise ValueError(
                "capture calibration version does not match supplied calibration"
            )
        if capture.sensor_frame != calibration.sensor_frame:
            raise ValueError("capture sensor frame does not match supplied calibration")
        source_points = _points(points_sensor)
        self.store.record_capture(capture, calibration)
        points_local = _transform(calibration.T_base_sensor, source_points)
        sensor_origin = tuple(row[3] for row in calibration.T_base_sensor[:3])
        return self.store.add_submap(
            SubmapId.from_keyframe(capture.keyframe_id),
            points_local,
            keyframe_poses_local={capture.keyframe_id: IDENTITY_SE3},
            sensor_origins_local=(sensor_origin,),
            resolution_m=self.resolution_m,
            observed_at_ns=capture.capture_end_ns,
            replace=replace,
            initial_T_component_submap=capture.T_local_base,
        )

    def replace_submap_geometry(
        self,
        submap_id: SubmapId,
        points_local: Sequence[Sequence[float]] | np.ndarray,
        *,
        keyframe_poses_local: Mapping[KeyframeId, Matrix4],
        sensor_origins_local: Iterable[Sequence[float]],
        observed_at_ns: int,
    ) -> int:
        return self.store.add_submap(
            submap_id,
            points_local,
            keyframe_poses_local=keyframe_poses_local,
            sensor_origins_local=sensor_origins_local,
            resolution_m=self.resolution_m,
            observed_at_ns=observed_at_ns,
            replace=True,
        )

    def apply_solution(self, solution: GraphSolution) -> bool:
        return self.store.apply_solution(solution)

    def snapshot(self) -> MapSnapshot:
        return self.store.snapshot()

    def snapshot_dict(self) -> dict[str, object]:
        """Canonical transport object used by server and peer sync wrappers."""

        return self.snapshot().to_dict()

    def get_chunk(self, sha256: str) -> bytes:
        return self.store.get_chunk(sha256)

    def occupancy_query(
        self,
        component_id: str,
        xyz: Sequence[float],
        *,
        now_ns: int | None = None,
    ) -> OccupancyQuery:
        """Query endpoint occupancy or observed free space from retained rays.

        An absent endpoint is unknown. It becomes free only when the query voxel
        lies on a retained sensor-origin-to-return ray. This keeps missing
        returns and expired obstacles from being mistaken for clearing evidence.
        """

        query = np.asarray(xyz, dtype=np.float64)
        if query.shape != (3,) or not np.isfinite(query).all():
            raise ValueError("occupancy query point must be a finite XYZ triple")
        submaps = self.store.active_submaps(component_id)
        newest_ns = max((submap.observed_at_ns for submap in submaps), default=0)
        revision = max((submap.component_revision for submap in submaps), default=None)
        geometry_revision = _geometry_digest(submaps) if submaps else None
        visible = False
        for submap in submaps:
            pose = submap.T_component_submap
            unambiguous_origin = (
                _transform(pose, np.asarray(submap.sensor_origins, dtype=np.float64))[0]
                if len(submap.sensor_origins) == 1
                else None
            )
            for chunk_index, chunk in enumerate(submap.chunks):
                endpoints = _transform(
                    pose, decode_xyz_f32(self.store.get_chunk(chunk.sha256))
                )
                if len(endpoints) and np.any(
                    np.linalg.norm(endpoints - query, axis=1)
                    <= submap.resolution_m * 0.75
                ):
                    return OccupancyQuery(
                        OccupancyState.OCCUPIED,
                        max(0, now_ns - newest_ns) if now_ns is not None else None,
                        revision,
                        geometry_revision,
                    )
                # Free rays require an unambiguous origin for this chunk. An
                # aggregated cloud with multiple unassociated origins can still
                # report occupied endpoints, but stays unknown between them.
                if unambiguous_origin is None:
                    continue
                for origin in (unambiguous_origin,):
                    vectors = endpoints - origin
                    lengths_sq = np.einsum("ij,ij->i", vectors, vectors)
                    valid = lengths_sq > submap.resolution_m**2
                    if not np.any(valid):
                        continue
                    fractions = ((query - origin) @ vectors[valid].T) / lengths_sq[
                        valid
                    ]
                    interior = (fractions >= 0) & (
                        fractions < 1 - submap.resolution_m / np.sqrt(lengths_sq[valid])
                    )
                    if not np.any(interior):
                        continue
                    closest = (
                        origin + fractions[interior, None] * vectors[valid][interior]
                    )
                    if np.any(
                        np.linalg.norm(closest - query, axis=1)
                        <= submap.resolution_m * 0.75
                    ):
                        visible = True
        age = max(0, now_ns - newest_ns) if now_ns is not None and newest_ns else None
        return OccupancyQuery(
            OccupancyState.FREE if visible else OccupancyState.UNKNOWN,
            age,
            revision,
            geometry_revision,
        )

    def terrain_query(
        self,
        component_id: str,
        xyz: Sequence[float],
        *,
        radius_m: float = 0.35,
        body_height_m: float = 0.6,
        now_ns: int | None = None,
    ) -> TerrainQuery:
        """Query surface support and clearance without treating unknown as free."""

        query = np.asarray(xyz, dtype=np.float64)
        if query.shape != (3,) or not np.isfinite(query).all():
            raise ValueError("terrain query point must be a finite XYZ triple")
        if radius_m <= 0 or body_height_m <= 0:
            raise ValueError("query radius and body height must be positive")
        clouds: list[np.ndarray] = []
        submaps = self.store.active_submaps(component_id)
        newest_ns = max((submap.observed_at_ns for submap in submaps), default=0)
        revision = max((submap.component_revision for submap in submaps), default=None)
        geometry_revision = _geometry_digest(submaps) if submaps else None
        for submap in submaps:
            for chunk in submap.chunks:
                local = decode_xyz_f32(self.store.get_chunk(chunk.sha256))
                clouds.append(_transform(submap.T_component_submap, local))
        if not clouds:
            return TerrainQuery(
                False,
                None,
                None,
                None,
                None,
                None,
                None,
                0.0,
                None,
                revision,
                geometry_revision,
            )
        points = np.concatenate(clouds, axis=0)
        horizontal = np.linalg.norm(points[:, :2] - query[:2], axis=1)
        local = points[horizontal <= radius_m]
        if len(local) < 3:
            return TerrainQuery(
                False,
                None,
                None,
                None,
                None,
                None,
                None,
                0.0,
                None,
                revision,
                geometry_revision,
            )

        # The upper quartile is excluded so ceilings/obstacles do not become ground.
        cutoff = min(query[2] + 0.25, float(np.quantile(local[:, 2], 0.75)))
        support = local[local[:, 2] <= cutoff]
        if len(support) < 3:
            return TerrainQuery(
                False,
                None,
                None,
                None,
                None,
                None,
                None,
                0.0,
                None,
                revision,
                geometry_revision,
            )
        ground = float(np.median(support[:, 2]))
        centered = support - support.mean(axis=0)
        covariance = centered.T @ centered / max(1, len(support) - 1)
        _, vectors = np.linalg.eigh(covariance)
        normal = vectors[:, 0]
        if normal[2] < 0:
            normal = -normal
        roughness = float(np.sqrt(np.mean((centered @ normal) ** 2)))
        higher = local[local[:, 2] > ground + max(0.05, self.resolution_m)]
        # No return is not proof of free space. A measured obstacle yields a
        # finite clearance; callers must treat None as unknown overhead.
        clearance = float(higher[:, 2].min() - ground) if len(higher) else None

        support_horizontal = np.linalg.norm(support[:, :2] - query[:2], axis=1)
        inner = support[support_horizontal <= radius_m * 0.5]
        outer = support[support_horizontal > radius_m * 0.5]
        inner_h = float(np.median(inner[:, 2])) if len(inner) else ground
        if len(outer):
            deltas = outer[:, 2] - inner_h
            step_up = max(0.0, float(np.max(deltas)))
            drop = max(0.0, float(-np.min(deltas)))
        else:
            step_up = drop = 0.0
        expected = max(3.0, math.pi * radius_m * radius_m / (self.resolution_m**2))
        confidence = min(1.0, len(support) / expected)
        age = max(0, now_ns - newest_ns) if now_ns is not None else None
        return TerrainQuery(
            True,
            ground,
            tuple(float(v) for v in normal),
            step_up,
            drop,
            roughness,
            clearance,
            confidence,
            age,
            revision,
            geometry_revision,
        )
