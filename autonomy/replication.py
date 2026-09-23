"""Durable, content-addressed map replication without a SLAM dependency.

Chunks are immutable; a manifest becomes visible only after all its chunks exist.
The transport envelope is deliberately independent of a mapper's internal types.
"""

from __future__ import annotations

from copy import deepcopy
from contextlib import contextmanager
import hashlib
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

from .map_epochs import robot_run_id

MAX_CHUNK_BYTES = 8 * 1024 * 1024
MAX_CHUNK_REFS = 65_536
MAX_MANIFEST_BYTES = 64 * 1024 * 1024
_HASH = re.compile(r"[a-f0-9]{64}\Z")
_ROBOT = re.compile(r"[A-Za-z0-9_.-]{1,128}\Z")


class RevisionConflict(ValueError):
    def __init__(
        self,
        message: str,
        *,
        resync: bool = False,
        base_revision: int | None = None,
        current_revision: int | None = None,
    ):
        super().__init__(message)
        self.resync = resync
        self.base_revision = base_revision
        self.current_revision = current_revision


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


def envelope_epochs(envelope: dict) -> dict[str, int]:
    """Validate robot lifetimes without confusing them with graph revisions."""
    robot, session = envelope["robot_id"], envelope["session_id"]
    identity(robot, session)
    epoch, run = envelope["map_epoch"], envelope["run_id"]
    if run != robot_run_id(session, robot, epoch):
        raise ValueError("Replica run_id does not match its robot map epoch")
    epochs = envelope["robot_map_epochs"]
    if not isinstance(epochs, dict) or epochs.get(robot) != epoch:
        raise ValueError("Replica robot map epochs do not include the publisher")
    for owner, value in epochs.items():
        robot_run_id(session, owner, value)
    participants = envelope["participant_robot_ids"]
    if (
        not isinstance(participants, list)
        or not participants
        or any(not isinstance(owner, str) for owner in participants)
        or len(set(participants)) != len(participants)
        or robot not in participants
        or not set(participants).issubset(epochs)
    ):
        raise ValueError("Invalid replica participant robot IDs")
    references = set(snapshot_runs(envelope["snapshot"]))
    references.update(snapshot_runs(envelope.get("anchor", {})))
    for owner, referenced_run in references:
        if owner not in participants or referenced_run != robot_run_id(
            session, owner, epochs[owner]
        ):
            raise ValueError("Snapshot references an undeclared robot map run")
    return epochs


def snapshot_runs(value):
    """Yield active keyframe/submap identities, including component anchors."""
    if isinstance(value, dict):
        if "robot_id" in value and "run_id" in value:
            yield value["robot_id"], value["run_id"]
        # SubmapId historically calls its keyframe namespace session_id.
        if "robot_id" in value and "session_id" in value and "seq" in value:
            yield value["robot_id"], value["session_id"]
        for key, child in value.items():
            if key != "tombstones":
                yield from snapshot_runs(child)
    elif isinstance(value, (list, tuple)):
        for child in value:
            yield from snapshot_runs(child)


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
            self.db.execute(
                "CREATE INDEX IF NOT EXISTS manifests_session ON manifests(session, robot)"
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
            # Publication recency per mission orders history retirement under
            # budget pressure. It lives beside `manifests` so that table keeps
            # its layout for stores written by earlier releases.
            self.db.execute(
                "CREATE TABLE IF NOT EXISTS sessions (session TEXT PRIMARY KEY, published REAL NOT NULL)"
            )
            self.db.execute(
                "CREATE TABLE IF NOT EXISTS map_epochs (robot TEXT, session TEXT, "
                "epoch INTEGER NOT NULL, run TEXT NOT NULL, PRIMARY KEY(robot, session))"
            )
            self.db.execute(
                "CREATE TABLE IF NOT EXISTS run_tombstones (robot TEXT, session TEXT, "
                "run TEXT, PRIMARY KEY(robot, session, run))"
            )
            # Legacy manifests have no independently fenced robot lifetime.
            # Retire them rather than interpreting their mission UUID as a run.
            for robot, session, body in self.db.execute(
                "SELECT robot, session, body FROM manifests"
            ).fetchall():
                value = json.loads(body)
                if "map_epoch" not in value or "run_id" not in value:
                    self.db.execute(
                        "INSERT OR IGNORE INTO run_tombstones VALUES (?, ?, ?)",
                        (robot, session, value.get("run_id", session)),
                    )
                    self._remove_source(robot, session)
                else:
                    self.db.execute(
                        "INSERT OR IGNORE INTO map_epochs VALUES (?, ?, ?, ?)",
                        (robot, session, value["map_epoch"], value["run_id"]),
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
            # Missions published before recency was recorded are dated by their
            # newest geometry, which was uploaded no later than the manifest.
            self.db.execute(
                "INSERT OR IGNORE INTO sessions "
                "SELECT m.session, COALESCE(MAX(c.touched), 0) FROM manifests m "
                "LEFT JOIN chunk_refs r ON r.robot=m.robot AND r.session=m.session "
                "LEFT JOIN chunks c ON c.hash=r.hash GROUP BY m.session"
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

    def _remove_source(self, robot, session):
        self.db.execute(
            "UPDATE chunks SET touched=? WHERE hash IN "
            "(SELECT hash FROM chunk_refs WHERE robot=? AND session=?)",
            (self.clock(), robot, session),
        )
        self.db.execute(
            "DELETE FROM chunk_refs WHERE robot=? AND session=?", (robot, session)
        )
        self.db.execute(
            "DELETE FROM manifests WHERE robot=? AND session=?", (robot, session)
        )

    def map_epoch(self, robot_id: str, session_id: str) -> int | None:
        identity(robot_id, session_id)
        with self.lock:
            row = self.db.execute(
                "SELECT epoch FROM map_epochs WHERE robot=? AND session=?",
                (robot_id, session_id),
            ).fetchone()
        return row[0] if row else None

    def _advance_epoch(self, robot, session, epoch):
        run = robot_run_id(session, robot, epoch)
        row = self.db.execute(
            "SELECT epoch, run FROM map_epochs WHERE robot=? AND session=?",
            (robot, session),
        ).fetchone()
        if row is not None:
            if epoch < row[0]:
                raise RevisionConflict("Stale robot map epoch")
            if epoch == row[0]:
                if run != row[1]:
                    raise RevisionConflict("Conflicting robot map run")
                return False
            self.db.execute(
                "INSERT OR IGNORE INTO run_tombstones VALUES (?, ?, ?)",
                (robot, session, row[1]),
            )
        if epoch > 0:
            # A robot may have been seen only through another peer's relay,
            # without ever committing its own source on this server.
            for publisher, body in self.db.execute(
                "SELECT robot, body FROM manifests WHERE session=?", (session,)
            ).fetchall():
                source = json.loads(body)
                retired_runs = {
                    referenced_run
                    for owner, referenced_run in snapshot_runs(source["snapshot"])
                    if owner == robot and referenced_run != run
                }
                self.db.executemany(
                    "INSERT OR IGNORE INTO run_tombstones VALUES (?, ?, ?)",
                    ((robot, session, retired_run) for retired_run in retired_runs),
                )
                if publisher == robot or not retired_runs:
                    continue
                keep, retired_hashes = set(), set()
                for manifest in source["snapshot"].get("manifests", []):
                    for submap in manifest.get("submaps", []):
                        key = submap.get("submap_id", {})
                        target = (
                            retired_hashes
                            if (
                                key.get("robot_id") == robot
                                and key.get("session_id", key.get("run_id"))
                                in retired_runs
                            )
                            else keep
                        )
                        target.update(
                            chunk["sha256"] for chunk in submap.get("chunks", [])
                        )
                self.db.executemany(
                    "DELETE FROM chunk_refs WHERE robot=? AND session=? AND hash=?",
                    ((publisher, session, digest) for digest in retired_hashes - keep),
                )
        self._remove_source(robot, session)
        self.db.execute(
            "INSERT OR REPLACE INTO map_epochs VALUES (?, ?, ?, ?)",
            (robot, session, epoch, run),
        )
        return True

    def reserve_map_epoch(self, robot_id: str, session_id: str, map_epoch: int) -> bool:
        """Atomically retire this robot's source and fence every earlier run."""
        identity(robot_id, session_id)
        robot_run_id(session_id, robot_id, map_epoch)
        with self._write():
            return self._advance_epoch(robot_id, session_id, map_epoch)

    def tombstones(self, session_id: str | None = None):
        where, args = (
            ("", ()) if session_id is None else (" WHERE session=?", (session_id,))
        )
        with self.lock:
            return tuple(
                self.db.execute(
                    "SELECT robot, session, run FROM run_tombstones"
                    + where
                    + " ORDER BY robot, session, run",
                    args,
                ).fetchall()
            )

    def collect_unreferenced(self, *, limit: int = 256, dry_run: bool = False) -> dict:
        """Reclaim one bounded batch, preserving all published robot sessions.

        The grace interval starts again when a chunk loses a manifest reference
        or is uploaded again. Slow uploaders can renegotiate missing hashes.
        This collection retires no manifest; only an upload that still does
        not fit afterwards retires history (see `_retire_history`).
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

    def _retire_history(self, needed):
        """Free `needed` bytes by retiring whole missions, oldest first.

        Historical manifests protect their geometry indefinitely, so a store
        that only collects unreferenced chunks eventually fills with history
        and rejects every upload of the mission being mapped. The most
        recently published mission is never retired, nor is one published
        within the retention interval. A retired mission's exclusive geometry
        is deleted at once: a grace interval here would keep the live map
        blocked for as long as it lasted.
        """
        rows = self.db.execute(
            "SELECT m.session, COALESCE(MAX(s.published), 0) AS published "
            "FROM manifests m LEFT JOIN sessions s ON s.session=m.session "
            "GROUP BY m.session ORDER BY published, m.session"
        ).fetchall()
        cutoff = self.clock() - self.retention_s
        freed, retired = 0, []
        for session, published in rows[:-1]:
            if freed >= needed or published > cutoff:
                break
            freed += self._retire_session(session)
            retired.append(session)
        return {"sessions": retired, "bytes": freed}

    def _retire_session(self, session):
        """Delete one mission's manifests and the geometry only it referenced."""
        owned = [
            row[0]
            for row in self.db.execute(
                "SELECT DISTINCT hash FROM chunk_refs WHERE session=?", (session,)
            )
        ]
        self.db.execute("DELETE FROM chunk_refs WHERE session=?", (session,))
        self.db.execute("DELETE FROM manifests WHERE session=?", (session,))
        self.db.execute("DELETE FROM sessions WHERE session=?", (session,))
        freed = 0
        for digest in owned:
            row = self.db.execute(
                "SELECT size FROM chunks WHERE hash=? AND NOT EXISTS "
                "(SELECT 1 FROM chunk_refs WHERE chunk_refs.hash=chunks.hash)",
                (digest,),
            ).fetchone()
            if row:
                (self.chunks / digest).unlink(missing_ok=True)
                self.db.execute("DELETE FROM chunks WHERE hash=?", (digest,))
                freed += row[0]
        return freed

    def discard_history(self, keep=()):
        """Retire every mission except those named in `keep`.

        A simulation mints a new mission at every reset and nothing reads the
        maps of the ones before it, so its server discards them at start-up.
        Missions to preserve, such as maps from real robots sharing the store,
        are named explicitly.
        """
        keep = {str(session) for session in keep}
        with self._write():
            sessions = [
                row[0]
                for row in self.db.execute(
                    "SELECT DISTINCT session FROM manifests ORDER BY session"
                )
                if row[0] not in keep
            ]
            freed = sum(self._retire_session(session) for session in sessions)
            # Uploads that never reached a manifest belong to no mission.
            freed += self._collect_unreferenced(4096)["bytes"]
        return {"sessions": sessions, "bytes": freed}

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
                used -= self._collect_unreferenced(256)["bytes"]
                if used + len(data) > self.max_bytes:
                    used -= self._retire_history(used + len(data) - self.max_bytes)[
                        "bytes"
                    ]
                if used + len(data) > self.max_bytes:
                    # Commit collection metadata even when this batch cannot
                    # free enough room. The current mission is never changed.
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

    @staticmethod
    def _submap_key(value):
        if not isinstance(value, dict):
            raise ValueError("submap must be an object")
        key = value.get("submap_id")
        if not isinstance(key, dict):
            raise ValueError("submap_id must be an object")
        try:
            result = (
                key["robot_id"],
                key["session_id"],
                key["seq"],
            )
        except KeyError as exc:
            raise ValueError("submap_id is incomplete") from exc
        if (
            not isinstance(result[0], str)
            or not isinstance(result[1], str)
            or type(result[2]) is not int
        ):
            raise ValueError("submap_id is invalid")
        return result

    @classmethod
    def _manifest_key(cls, value):
        if not isinstance(value, dict):
            raise ValueError("manifest must be an object")
        revision = value.get("graph_revision")
        if not isinstance(revision, dict) or not isinstance(
            revision.get("component_id"), str
        ):
            raise ValueError("manifest graph_revision is required")
        return revision["component_id"]

    def _expand_delta(self, delta: dict) -> dict:
        cls = type(self)
        if delta.get("version") != 2 or delta.get("kind") != "delta":
            raise ValueError("Invalid replica delta")
        robot, session = delta.get("robot_id"), delta.get("session_id")
        identity(robot, session)
        base = delta.get("base_revision")
        revision = delta.get("revision")
        if (
            type(base) is not int
            or base < 0
            or type(revision) is not int
            or revision <= base
        ):
            raise ValueError("Invalid replica delta revision")
        previous = self.get(robot, session)
        if previous is None:
            raise RevisionConflict(
                "Delta base is unavailable; full replica resync required",
                resync=True,
                base_revision=base,
                current_revision=None,
            )
        current_revision = previous.get("revision")
        if current_revision != base:
            raise RevisionConflict(
                "Delta base revision is not current; full replica resync required",
                resync=True,
                base_revision=base,
                current_revision=current_revision,
            )
        previous_snapshot = previous.get("snapshot")
        patch = delta.get("snapshot_delta")
        if not isinstance(previous_snapshot, dict) or not isinstance(patch, dict):
            raise ValueError("Invalid replica delta snapshot")
        if patch.get("schema") != previous_snapshot.get("schema"):
            raise ValueError("Replica delta schema mismatch")
        snapshot_id = patch.get("snapshot_id")
        generated_at_ns = patch.get("generated_at_ns")
        if (
            not isinstance(snapshot_id, str)
            or not _HASH.fullmatch(snapshot_id)
            or type(generated_at_ns) is not int
            or generated_at_ns < 0
        ):
            raise ValueError("Invalid replica delta snapshot identity")
        components = patch.get("components")
        removed_components = patch.get("removed_components", [])
        if not isinstance(components, list) or not isinstance(removed_components, list):
            raise ValueError("Invalid replica delta components")
        manifests = {
            cls._manifest_key(manifest): dict(manifest)
            for manifest in previous_snapshot.get("manifests", [])
        }
        if len(manifests) != len(previous_snapshot.get("manifests", [])):
            raise ValueError("Replica snapshot repeats a component")
        removed = set()
        for component in removed_components:
            if not isinstance(component, str) or component in removed:
                raise ValueError("Invalid removed component")
            removed.add(component)
            manifests.pop(component, None)
        changed = set()
        for component in components:
            if not isinstance(component, dict):
                raise ValueError("Replica component delta must be an object")
            component_id = component.get("component_id")
            metadata = component.get("manifest")
            if (
                not isinstance(component_id, str)
                or component_id in changed
                or component_id in removed
                or not isinstance(metadata, dict)
            ):
                raise ValueError("Invalid replica component delta")
            if component_id != cls._manifest_key(metadata):
                raise ValueError("Replica component delta identity mismatch")
            changed.add(component_id)
            manifest = dict(metadata)
            prior = manifests.get(component_id)
            prior_submaps = (
                {cls._submap_key(item): dict(item) for item in prior.get("submaps", [])}
                if prior is not None
                else {}
            )
            upserts = component.get("upsert_submaps", [])
            removed_submaps = component.get("removed_submaps", [])
            pose_updates = component.get("pose_updates", [])
            if (
                not isinstance(upserts, list)
                or not isinstance(removed_submaps, list)
                or not isinstance(pose_updates, list)
            ):
                raise ValueError("Invalid replica submap delta")
            removed_keys = set()
            for item in removed_submaps:
                key = cls._submap_key({"submap_id": item})
                if key in removed_keys:
                    raise ValueError("Replica delta repeats a removed submap")
                removed_keys.add(key)
                prior_submaps.pop(key, None)
            for item in upserts:
                key = cls._submap_key(item)
                if key in removed_keys:
                    raise ValueError("Replica delta both removes and upserts a submap")
                prior_submaps[key] = dict(item)
            for target in prior_submaps.values():
                target["pose_revision"] = metadata["graph_revision"]
            pose_keys = set()
            for item in pose_updates:
                if not isinstance(item, dict):
                    raise ValueError("Replica pose update must be an object")
                key = cls._submap_key({"submap_id": item.get("submap_id")})
                if key in pose_keys or key in removed_keys:
                    raise ValueError("Replica delta repeats a pose update")
                pose_keys.add(key)
                target = prior_submaps.get(key)
                if target is None:
                    raise ValueError("Replica pose update names an unknown submap")
                if "T_component_submap" not in item or "pose_revision" not in item:
                    raise ValueError("Replica pose update is incomplete")
                target["T_component_submap"] = item["T_component_submap"]
                target["pose_revision"] = item["pose_revision"]
            manifest["submaps"] = sorted(
                prior_submaps.values(),
                key=lambda item: "/".join(map(str, cls._submap_key(item))),
            )
            chunks = {
                chunk["sha256"]: chunk
                for submap in manifest["submaps"]
                for chunk in submap.get("chunks", [])
            }
            manifest["chunks"] = [chunks[key] for key in sorted(chunks)]
            manifests[component_id] = manifest
        ordered = sorted(
            manifests.values(),
            key=lambda item: (
                item["graph_revision"]["component_id"],
                item["graph_revision"]["epoch"],
                item["graph_revision"]["revision"],
            ),
        )
        canonical_manifests = [
            {"schema": previous_snapshot["schema"], **manifest} for manifest in ordered
        ]
        if hashlib.sha256(canonical(canonical_manifests)).hexdigest() != snapshot_id:
            raise ValueError("Replica delta snapshot_id does not match reconstruction")
        full = dict(delta)
        full["version"] = 1
        full.pop("kind", None)
        full.pop("base_revision", None)
        full.pop("snapshot_delta", None)
        full["snapshot"] = {
            "schema": previous_snapshot["schema"],
            "snapshot_id": snapshot_id,
            "generated_at_ns": generated_at_ns,
            "manifests": ordered,
        }
        chunk_table = {
            chunk["sha256"]: {
                "sha256": chunk["sha256"],
                "size": chunk["size_bytes"],
            }
            for manifest in ordered
            for chunk in manifest.get("chunks", [])
        }
        full["chunks"] = [chunk_table[key] for key in sorted(chunk_table)]
        return full

    def read_chunk(self, digest: str) -> bytes:
        chunk_hash(digest)
        return (self.chunks / digest).read_bytes()

    def publish(self, envelope: dict) -> bool:
        if not isinstance(envelope, dict):
            raise ValueError("Manifest must be an object")
        delta_base = None
        if envelope.get("kind") == "delta":
            delta_base = envelope.get("base_revision")
            envelope = self._expand_delta(envelope)
        robot, session = envelope["robot_id"], envelope["session_id"]
        identity(robot, session)
        epochs = envelope_epochs(envelope)
        revision = envelope["revision"]
        chunks = envelope["chunks"]
        if (
            envelope.get("version") != 1
            or type(revision) is not int
            or revision < 0
            or not isinstance(chunks, list)
            or len(chunks) > MAX_CHUNK_REFS
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
            if delta_base is not None and (
                previous is None or previous["revision"] != delta_base
            ):
                raise RevisionConflict(
                    "Delta base revision is not current; full replica resync required",
                    resync=True,
                    base_revision=delta_base,
                    current_revision=(
                        None if previous is None else previous["revision"]
                    ),
                )
            self._advance_epoch(robot, session, envelope["map_epoch"])
            previous = self.get(robot, session)
            for owner in envelope["participant_robot_ids"]:
                current = self.map_epoch(owner, session)
                if current is not None and epochs[owner] < current:
                    raise RevisionConflict(
                        "Snapshot references a stale robot map epoch"
                    )
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
            self.db.execute(
                "INSERT OR REPLACE INTO sessions VALUES (?, ?)", (session, self.clock())
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
                "SELECT m.robot, m.session, m.revision, e.epoch, e.run "
                "FROM manifests m JOIN map_epochs e USING(robot, session) "
                "ORDER BY m.robot, m.session"
            ).fetchall()
        return [
            {
                "robot_id": r,
                "session_id": s,
                "revision": v,
                "map_epoch": e,
                "run_id": run,
            }
            for r, s, v, e, run in rows
        ]

    def snapshots(self, session_id: str | None = None):
        """Read a bounded, consistent set of replica manifests for display.

        Per-robot revisions are independent. A single SQLite read transaction
        prevents the fleet viewer from assembling a mixture of database reads
        while another server worker publishes a replacement.
        """
        if session_id is not None:
            identity("replica", session_id)
        where, args = (
            ("", ()) if session_id is None else (" WHERE session=?", (session_id,))
        )
        with self.lock:
            self.db.execute("BEGIN")
            try:
                sizes = self.db.execute(
                    "SELECT length(body) FROM manifests" + where + " LIMIT 129", args
                ).fetchall()
                if len(sizes) > 128 or sum(row[0] for row in sizes) > 64 * 1024**2:
                    raise OverflowError(
                        "Replica catalogue exceeds its metadata budget; select a mission"
                    )
                rows = self.db.execute(
                    "SELECT body FROM manifests" + where + " ORDER BY robot, session",
                    args,
                ).fetchall()
                self.db.execute("COMMIT")
            except BaseException:
                self.db.execute("ROLLBACK")
                raise
        return [json.loads(row[0]) for row in rows]

    def snapshot_versions(self, session_id: str | None = None):
        """Cheap cache key: publication rejects changed bytes at one revision."""
        if session_id is not None:
            identity("replica", session_id)
        where, args = (
            ("", ()) if session_id is None else (" WHERE session=?", (session_id,))
        )
        with self.lock:
            rows = self.db.execute(
                "SELECT m.robot, m.session, m.revision, e.epoch, e.run "
                "FROM manifests m JOIN map_epochs e USING(robot, session)"
                + where
                + " ORDER BY m.robot, m.session LIMIT 129",
                args,
            ).fetchall()
        if len(rows) > 128:
            raise OverflowError(
                "Replica catalogue exceeds its source budget; select a mission"
            )
        return tuple(rows)


class ReplicaClient:
    """A bounded synchronous client; call from a worker, never a sensor callback."""

    def __init__(self, server_url: str, *, timeout_s: float = 5):
        self.url = server_url.rstrip("/") + "/api/autonomy"
        self.timeout_s = timeout_s
        self._last_envelope = None

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

    @staticmethod
    def _delta(previous: dict, current: dict) -> dict | None:
        if (
            previous.get("robot_id") != current.get("robot_id")
            or previous.get("session_id") != current.get("session_id")
            or previous.get("map_epoch") != current.get("map_epoch")
            or previous.get("run_id") != current.get("run_id")
            or previous.get("revision", -1) >= current.get("revision", 0)
        ):
            return None
        old_snapshot = previous.get("snapshot")
        new_snapshot = current.get("snapshot")
        if (
            not isinstance(old_snapshot, dict)
            or not isinstance(new_snapshot, dict)
            or not isinstance(old_snapshot.get("manifests"), list)
            or not isinstance(new_snapshot.get("manifests"), list)
            or not isinstance(new_snapshot.get("schema"), str)
            or not isinstance(new_snapshot.get("snapshot_id"), str)
            or not _HASH.fullmatch(new_snapshot["snapshot_id"])
        ):
            return None
        old_manifests = {
            item["graph_revision"]["component_id"]: item
            for item in old_snapshot["manifests"]
        }
        new_manifests = {
            item["graph_revision"]["component_id"]: item
            for item in new_snapshot["manifests"]
        }
        components = []
        for component_id, manifest in new_manifests.items():
            # Noncanonical full-envelope order is preserved via bootstrap.
            if (
                manifest.get("submaps", [])
                != sorted(
                    manifest.get("submaps", []),
                    key=lambda item: "/".join(map(str, ReplicaStore._submap_key(item))),
                )
                or manifest.get("chunks", [])
                != sorted(manifest.get("chunks", []), key=lambda item: item["sha256"])
                or any(
                    item.get("pose_revision") != manifest["graph_revision"]
                    for item in manifest.get("submaps", [])
                )
            ):
                return None
            prior = old_manifests.get(component_id)
            old_submaps = (
                {
                    ReplicaStore._submap_key(item): item
                    for item in prior.get("submaps", [])
                }
                if prior is not None
                else {}
            )
            upserts, poses = [], []
            for item in manifest.get("submaps", []):
                key = ReplicaStore._submap_key(item)
                before = old_submaps.pop(key, None)
                if before is None:
                    upserts.append(item)
                    continue
                before_geometry = dict(before)
                after_geometry = dict(item)
                for value in (before_geometry, after_geometry):
                    value.pop("T_component_submap", None)
                    value.pop("pose_revision", None)
                if before_geometry != after_geometry:
                    upserts.append(item)
                elif before.get("T_component_submap") != item.get("T_component_submap"):
                    poses.append(
                        {
                            "submap_id": item["submap_id"],
                            "T_component_submap": item["T_component_submap"],
                            "pose_revision": item["pose_revision"],
                        }
                    )
            metadata = {
                key: value
                for key, value in manifest.items()
                if key not in {"submaps", "chunks"}
            }
            prior_metadata = (
                {
                    key: value
                    for key, value in prior.items()
                    if key not in {"submaps", "chunks"}
                }
                if prior is not None
                else None
            )
            removed = [item["submap_id"] for item in old_submaps.values()]
            if (
                prior is None
                or metadata != prior_metadata
                or upserts
                or poses
                or removed
            ):
                components.append(
                    {
                        "component_id": component_id,
                        "manifest": metadata,
                        "upsert_submaps": upserts,
                        "pose_updates": poses,
                        "removed_submaps": removed,
                    }
                )
        removed_components = sorted(set(old_manifests) - set(new_manifests))
        old_chunks = {
            item["sha256"]
            for manifest in old_manifests.values()
            for item in manifest.get("chunks", [])
        }
        new_chunks = {
            item["sha256"]: item
            for manifest in new_manifests.values()
            for item in manifest.get("chunks", [])
        }
        delta = dict(current)
        delta["version"] = 2
        delta["kind"] = "delta"
        delta["base_revision"] = previous["revision"]
        delta.pop("snapshot", None)
        delta["snapshot_delta"] = {
            "schema": new_snapshot["schema"],
            "snapshot_id": new_snapshot["snapshot_id"],
            "generated_at_ns": new_snapshot["generated_at_ns"],
            "components": components,
            "removed_components": removed_components,
        }
        delta["chunks"] = [
            item
            for digest, item in sorted(new_chunks.items())
            if digest not in old_chunks
        ]
        return delta

    def _publish(self, wire: dict, full: dict, read_chunk) -> dict | None:
        body = canonical(wire)
        uploaded = 0
        while True:
            try:
                with self._request("/replicas", "POST", body) as reply:
                    result = json.load(reply)
                return {**result, "uploaded_bytes": uploaded}
            except error.HTTPError as exc:
                if exc.code != 409:
                    raise
                response = json.loads(exc.read(MAX_MANIFEST_BYTES + 1))
                missing = response.get("missing")
                if not isinstance(missing, list):
                    if wire is not full:
                        return None
                    raise
                declared = {item["sha256"]: item["size"] for item in full["chunks"]}
                if len(missing) > len(declared) or any(
                    digest not in declared for digest in missing
                ):
                    raise ValueError("Receiver requested undeclared geometry")
                for digest in missing:
                    chunk_hash(digest)
                    data = read_chunk(digest)
                    if len(data) != declared[digest]:
                        raise ValueError("Local chunk no longer matches its manifest")
                    with self._request("/chunks/" + digest, "PUT", data):
                        uploaded += len(data)

    def sync(self, envelope: dict, read_chunk) -> dict:
        identity(envelope["robot_id"], envelope["session_id"])
        previous = getattr(self, "_last_envelope", None)
        wire = envelope
        if previous is not None:
            wire = self._delta(previous, envelope) or envelope
        result = self._publish(wire, envelope, read_chunk)
        if result is None:
            result = self._publish(envelope, envelope, read_chunk)
        self._last_envelope = deepcopy(envelope)
        return result
