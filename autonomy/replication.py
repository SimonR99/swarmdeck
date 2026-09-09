"""Durable, content-addressed map replication without a SLAM dependency.

Chunks are immutable; a manifest becomes visible only after all its chunks exist.
The transport envelope is deliberately independent of a mapper's internal types.
"""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
import sqlite3
import tempfile
import threading
from urllib import error, request
from uuid import UUID

MAX_CHUNK_BYTES = 8 * 1024 * 1024
MAX_MANIFEST_BYTES = 4 * 1024 * 1024
_HASH = re.compile(r"[a-f0-9]{64}\Z")
_ROBOT = re.compile(r"[A-Za-z0-9_.-]{1,128}\Z")


class RevisionConflict(ValueError):
    pass


class MissingChunks(ValueError):
    def __init__(self, hashes):
        self.hashes = hashes
        super().__init__("Manifest references unavailable chunks")


def canonical(value) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode()


def identity(robot_id: str, session_id: str) -> None:
    if (
        not isinstance(robot_id, str)
        or not _ROBOT.fullmatch(robot_id)
        or robot_id in {".", ".."}
    ):
        raise ValueError("Invalid robot ID")
    if not isinstance(session_id, str) or str(UUID(session_id)) != session_id:
        raise ValueError("Session must be a canonical UUID")


def chunk_hash(value: str) -> None:
    if not isinstance(value, str) or not _HASH.fullmatch(value):
        raise ValueError("Invalid chunk hash")


class ReplicaStore:
    def __init__(self, root: str | Path, *, max_bytes: int = 1024**3):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.chunks = self.root / "chunks"
        self.chunks.mkdir(exist_ok=True)
        self.max_bytes = max_bytes
        self.lock = threading.RLock()
        self.db = sqlite3.connect(
            self.root / "replicas.sqlite", check_same_thread=False
        )
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.execute(
            "CREATE TABLE IF NOT EXISTS chunks (hash TEXT PRIMARY KEY, size INTEGER NOT NULL)"
        )
        self.db.execute(
            "CREATE TABLE IF NOT EXISTS manifests (robot TEXT, session TEXT, revision INTEGER, body BLOB, PRIMARY KEY(robot, session))"
        )
        # Recover files committed before their metadata during a crash. Files
        # with temporary names are incomplete uploads and are never visible.
        for path in self.chunks.iterdir():
            if path.is_file() and _HASH.fullmatch(path.name):
                self.db.execute(
                    "INSERT OR IGNORE INTO chunks VALUES (?, ?)",
                    (path.name, path.stat().st_size),
                )
        self.db.commit()

    def close(self):
        with self.lock:
            self.db.close()

    def has_chunk(self, digest: str) -> bool:
        chunk_hash(digest)
        return (self.chunks / digest).is_file()

    def put_chunk(self, digest: str, data: bytes) -> bool:
        chunk_hash(digest)
        if len(data) > MAX_CHUNK_BYTES or hashlib.sha256(data).hexdigest() != digest:
            raise ValueError("Invalid chunk size or checksum")
        with self.lock:
            if self.has_chunk(digest):
                # Recover an atomic file written before a crash interrupted metadata.
                self.db.execute(
                    "INSERT OR IGNORE INTO chunks VALUES (?, ?)", (digest, len(data))
                )
                self.db.commit()
                return False
            used = self.db.execute(
                "SELECT COALESCE(SUM(size), 0) FROM chunks"
            ).fetchone()[0]
            if used + len(data) > self.max_bytes:
                raise OverflowError("Replica storage budget exhausted")
            with tempfile.NamedTemporaryFile(dir=self.chunks, delete=False) as out:
                temp = Path(out.name)
                try:
                    out.write(data)
                    out.flush()
                    os.fsync(out.fileno())
                    os.replace(temp, self.chunks / digest)
                finally:
                    temp.unlink(missing_ok=True)
            self.db.execute(
                "INSERT OR REPLACE INTO chunks VALUES (?, ?)", (digest, len(data))
            )
            self.db.commit()
            return True

    def read_chunk(self, digest: str) -> bytes:
        chunk_hash(digest)
        return (self.chunks / digest).read_bytes()

    def publish(self, envelope: dict) -> bool:
        if not isinstance(envelope, dict):
            raise ValueError("Manifest must be an object")
        robot, session = envelope["robot_id"], envelope["session_id"]
        identity(robot, session)
        revision = envelope["revision"]
        chunks = envelope["chunks"]
        if (
            envelope.get("version") != 1
            or type(revision) is not int
            or revision < 0
            or not isinstance(chunks, list)
            or len(chunks) > 4096
            or not isinstance(envelope.get("snapshot"), dict)
        ):
            raise ValueError("Invalid replica manifest")
        declared = {}
        for item in chunks:
            if not isinstance(item, dict):
                raise ValueError("Chunk declaration must be an object")
            digest, size = item["sha256"], item["size"]
            chunk_hash(digest)
            if type(size) is not int or not 0 <= size <= MAX_CHUNK_BYTES:
                raise ValueError("Invalid declared chunk size")
            if digest in declared and declared[digest] != size:
                raise ValueError("Conflicting chunk declarations")
            declared[digest] = size
        body = canonical(envelope)
        if len(body) > MAX_MANIFEST_BYTES:
            raise ValueError("Manifest too large")
        with self.lock:
            previous = self.get(robot, session)
            if previous:
                if revision < previous["revision"]:
                    raise RevisionConflict("Stale replica revision")
                if revision == previous["revision"]:
                    if body == canonical(previous):
                        return False
                    raise RevisionConflict("Conflicting replica revision")
            missing = [
                h
                for h, size in declared.items()
                if not self.has_chunk(h) or (self.chunks / h).stat().st_size != size
            ]
            if missing:
                raise MissingChunks(missing)
            self.db.execute(
                "INSERT OR REPLACE INTO manifests VALUES (?, ?, ?, ?)",
                (robot, session, revision, body),
            )
            self.db.commit()
            return True

    def get(self, robot: str, session: str):
        identity(robot, session)
        with self.lock:
            row = self.db.execute(
                "SELECT body FROM manifests WHERE robot=? AND session=?",
                (robot, session),
            ).fetchone()
        return json.loads(row[0]) if row else None

    def index(self):
        with self.lock:
            rows = self.db.execute(
                "SELECT robot, session, revision FROM manifests ORDER BY robot, session"
            ).fetchall()
        return [{"robot_id": r, "session_id": s, "revision": v} for r, s, v in rows]


class ReplicaClient:
    """A bounded synchronous client; call from a worker, never a sensor callback."""

    def __init__(self, server_url: str, *, timeout_s: float = 5):
        self.url = server_url.rstrip("/") + "/api/autonomy"
        self.timeout_s = timeout_s

    def _request(self, path, method="GET", body=None):
        req = request.Request(
            self.url + path,
            data=body,
            method=method,
            headers={
                "Content-Type": (
                    "application/json"
                    if path.startswith("/replicas")
                    else "application/octet-stream"
                )
            },
        )
        return request.urlopen(req, timeout=self.timeout_s)

    def sync(self, envelope: dict, read_chunk) -> dict:
        identity(envelope["robot_id"], envelope["session_id"])
        body = canonical(envelope)
        # One manifest negotiation replaces N HEAD requests on every update.
        # The receiver's durable store remains the acknowledgement after
        # either process restarts or loses its transient cache.
        try:
            with self._request("/replicas", "POST", body) as reply:
                return {**json.load(reply), "uploaded_bytes": 0}
        except error.HTTPError as exc:
            if exc.code != 409:
                raise
            response = json.loads(exc.read(MAX_MANIFEST_BYTES + 1))
            missing = response.get("missing")
            if not isinstance(missing, list):
                raise
        declared = {item["sha256"]: item["size"] for item in envelope["chunks"]}
        if len(missing) > len(declared) or any(
            digest not in declared for digest in missing
        ):
            raise ValueError("Receiver requested undeclared geometry")
        uploaded = 0
        for digest in missing:
            chunk_hash(digest)
            data = read_chunk(digest)
            if len(data) != declared[digest]:
                raise ValueError("Local chunk no longer matches its manifest")
            with self._request("/chunks/" + digest, "PUT", data):
                uploaded += len(data)
        with self._request("/replicas", "POST", body) as reply:
            result = json.load(reply)
        return {**result, "uploaded_bytes": uploaded}
