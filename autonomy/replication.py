"""Durable, content-addressed map replication without a SLAM dependency.

Chunks are immutable; a manifest becomes visible only after all its chunks exist.
The transport envelope is deliberately independent of a mapper's internal types.
"""

from __future__ import annotations

import hashlib
from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import re
import sqlite3
import tempfile
import threading
import time
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
    def __init__(
        self,
        root: str | Path,
        *,
        max_bytes: int = 1024**3,
        retention_s: float = 3600,
        clock=time.time,
    ):
        if type(max_bytes) is not int or max_bytes < 0:
            raise ValueError("Invalid replica storage budget")
        if not math.isfinite(retention_s) or retention_s < 0:
            raise ValueError("Invalid unreferenced chunk retention")
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.chunks = self.root / "chunks"
        self.chunks.mkdir(exist_ok=True)
        self.max_bytes = max_bytes
        self.retention_s, self.clock = retention_s, clock
        self.lock = threading.RLock()
        self.db = sqlite3.connect(
            self.root / "replicas.sqlite", check_same_thread=False
        )
        self.db.execute("PRAGMA journal_mode=WAL")
        with self._write():
            self.db.execute(
                "CREATE TABLE IF NOT EXISTS chunks (hash TEXT PRIMARY KEY, size INTEGER NOT NULL, touched REAL NOT NULL)"
            )
            if "touched" not in {
                row[1] for row in self.db.execute("PRAGMA table_info(chunks)")
            }:
                self.db.execute(
                    "ALTER TABLE chunks ADD COLUMN touched REAL NOT NULL DEFAULT 0"
                )
                self.db.execute("UPDATE chunks SET touched=?", (self.clock(),))
            self.db.execute(
                "CREATE TABLE IF NOT EXISTS manifests (robot TEXT, session TEXT, revision INTEGER, body BLOB, PRIMARY KEY(robot, session))"
            )
            indexed = self.db.execute(
                "SELECT 1 FROM sqlite_master WHERE type='table' AND name='chunk_refs'"
            ).fetchone()
            self.db.execute(
                "CREATE TABLE IF NOT EXISTS chunk_refs (robot TEXT, session TEXT, hash TEXT, PRIMARY KEY(robot, session, hash))"
            )
            self.db.execute(
                "CREATE INDEX IF NOT EXISTS chunk_refs_hash ON chunk_refs(hash)"
            )
            self.db.execute(
                "CREATE INDEX IF NOT EXISTS chunks_touched ON chunks(touched)"
            )
            if not indexed:
                for robot, session, body in self.db.execute(
                    "SELECT robot, session, body FROM manifests"
                ).fetchall():
                    self._set_references(robot, session, json.loads(body)["chunks"])
            # Recover files committed before their metadata, and deletions
            # interrupted before metadata commit. Give recovered uploads a full
            # grace interval; temporary upload files are never visible.
            known = dict(self.db.execute("SELECT hash, size FROM chunks"))
            found = set()
            for path in self.chunks.iterdir():
                if path.is_file() and _HASH.fullmatch(path.name):
                    found.add(path.name)
                    if path.name not in known:
                        self.db.execute(
                            "INSERT INTO chunks VALUES (?, ?, ?)",
                            (path.name, path.stat().st_size, self.clock()),
                        )
            self.db.executemany(
                "DELETE FROM chunks WHERE hash=?", ((h,) for h in known.keys() - found)
            )

    @contextmanager
    def _write(self):
        # Reserve the SQLite writer before filesystem checks. An RLock alone
        # does not protect publication/collection across server worker processes.
        with self.lock:
            self.db.execute("BEGIN IMMEDIATE")
            try:
                yield
                self.db.commit()
            except BaseException:
                self.db.rollback()
                raise

    def _set_references(self, robot, session, chunks, previous=()):
        current = {item["sha256"] for item in chunks}
        old = {item["sha256"] for item in previous}
        self.db.executemany(
            "DELETE FROM chunk_refs WHERE robot=? AND session=? AND hash=?",
            ((robot, session, digest) for digest in old - current),
        )
        self.db.executemany(
            "INSERT OR IGNORE INTO chunk_refs VALUES (?, ?, ?)",
            ((robot, session, digest) for digest in current - old),
        )

    def collect_unreferenced(self, *, limit: int = 256, dry_run: bool = False) -> dict:
        """Reclaim one bounded batch, preserving all published robot sessions.

        The grace interval starts again when a chunk loses a manifest reference
        or is uploaded again. Slow uploaders can renegotiate missing hashes.
        No current or historical session manifest is retired implicitly.
        """
        if type(limit) is not int or not 1 <= limit <= 4096:
            raise ValueError("Collection limit must be between 1 and 4096")
        with self._write():
            return self._collect_unreferenced(limit, dry_run)

    def _collect_unreferenced(self, limit, dry_run=False):
        rows = self.db.execute(
            "SELECT hash, size FROM chunks WHERE touched <= ? "
            "AND NOT EXISTS (SELECT 1 FROM chunk_refs WHERE chunk_refs.hash=chunks.hash) "
            "ORDER BY touched, hash LIMIT ?",
            (self.clock() - self.retention_s, limit),
        ).fetchall()
        if not dry_run:
            for digest, _ in rows:
                (self.chunks / digest).unlink(missing_ok=True)
                self.db.execute("DELETE FROM chunks WHERE hash=?", (digest,))
        return {
            "chunks": len(rows),
            "bytes": sum(size for _, size in rows),
            "dry_run": dry_run,
        }

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
        result = self._put_chunk(digest, data)
        if result is None:
            raise OverflowError("Replica storage budget exhausted")
        return result

    def _put_chunk(self, digest, data):
        with self._write():
            if self.has_chunk(digest):
                # Recover an atomic file written before a crash interrupted metadata.
                self.db.execute(
                    "INSERT OR REPLACE INTO chunks VALUES (?, ?, ?)",
                    (digest, len(data), self.clock()),
                )
                return False
            used = self.db.execute(
                "SELECT COALESCE(SUM(size), 0) FROM chunks"
            ).fetchone()[0]
            if used + len(data) > self.max_bytes:
                reclaimed = self._collect_unreferenced(256)
                if used - reclaimed["bytes"] + len(data) > self.max_bytes:
                    # Commit collection metadata even when this batch cannot
                    # free enough room. Publication is never changed.
                    return None
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
                "INSERT OR REPLACE INTO chunks VALUES (?, ?, ?)",
                (digest, len(data), self.clock()),
            )
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
        with self._write():
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
            # Readers holding the previous manifest have a grace interval to
            # finish fetching it after replacement, even for old geometry.
            old_chunks = previous["chunks"] if previous else []
            retired = {item["sha256"] for item in old_chunks} - declared.keys()
            if retired:
                self.db.executemany(
                    "UPDATE chunks SET touched=? WHERE hash=?",
                    ((self.clock(), digest) for digest in retired),
                )
            self._set_references(robot, session, chunks, old_chunks)
            self.db.execute(
                "INSERT OR REPLACE INTO manifests VALUES (?, ?, ?, ?)",
                (robot, session, revision, body),
            )
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
