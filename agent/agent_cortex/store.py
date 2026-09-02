"""Durable, append-only execution state for Cortex.

Operator chat history remains in its existing JSON files.  This SQLite store is
separate because jobs, tool evidence, memory candidates, and compactions have a
different lifecycle from UI messages.
"""

from __future__ import annotations

import json
import sqlite3
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Dict, Optional


SCHEMA_VERSION = 1


class CortexStore:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self._lock = threading.RLock()
        self._initialize()

    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self.path, timeout=5.0)
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA foreign_keys=ON")
        connection.execute("PRAGMA busy_timeout=5000")
        return connection

    def _initialize(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self._lock, self._connect() as connection:
            connection.execute("PRAGMA journal_mode=WAL")
            connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS schema_info (
                    version INTEGER NOT NULL
                );
                CREATE TABLE IF NOT EXISTS jobs (
                    job_id TEXT PRIMARY KEY,
                    conversation_id TEXT,
                    kind TEXT NOT NULL,
                    phase TEXT NOT NULL,
                    provider TEXT NOT NULL,
                    selected_robot TEXT,
                    request_text TEXT NOT NULL,
                    metadata_json TEXT NOT NULL,
                    error_text TEXT,
                    created_at REAL NOT NULL,
                    updated_at REAL NOT NULL
                );
                CREATE TABLE IF NOT EXISTS events (
                    event_id INTEGER PRIMARY KEY AUTOINCREMENT,
                    job_id TEXT NOT NULL REFERENCES jobs(job_id) ON DELETE CASCADE,
                    sequence INTEGER NOT NULL,
                    event_type TEXT NOT NULL,
                    payload_json TEXT NOT NULL,
                    created_at REAL NOT NULL,
                    UNIQUE(job_id, sequence)
                );
                CREATE INDEX IF NOT EXISTS idx_events_job_sequence
                    ON events(job_id, sequence);
                CREATE INDEX IF NOT EXISTS idx_jobs_updated
                    ON jobs(updated_at DESC);
                CREATE TABLE IF NOT EXISTS memories (
                    memory_id TEXT PRIMARY KEY,
                    scope TEXT NOT NULL,
                    memory_key TEXT NOT NULL,
                    value_json TEXT NOT NULL,
                    status TEXT NOT NULL,
                    confidence REAL NOT NULL,
                    source_job_id TEXT REFERENCES jobs(job_id) ON DELETE SET NULL,
                    created_at REAL NOT NULL,
                    reviewed_at REAL
                );
                CREATE INDEX IF NOT EXISTS idx_memories_scope_status
                    ON memories(scope, status);
                CREATE TABLE IF NOT EXISTS compactions (
                    compaction_id TEXT PRIMARY KEY,
                    job_id TEXT NOT NULL REFERENCES jobs(job_id) ON DELETE CASCADE,
                    through_sequence INTEGER NOT NULL,
                    summary TEXT NOT NULL,
                    created_at REAL NOT NULL
                );
                """
            )
            row = connection.execute("SELECT version FROM schema_info LIMIT 1").fetchone()
            if row is None:
                connection.execute(
                    "INSERT INTO schema_info(version) VALUES (?)", (SCHEMA_VERSION,)
                )
            elif row["version"] != SCHEMA_VERSION:
                raise RuntimeError(
                    f"Unsupported Cortex state schema {row['version']} "
                    f"(expected {SCHEMA_VERSION})"
                )

    def create_job(
        self,
        *,
        kind: str,
        provider: str,
        request_text: str,
        conversation_id: Optional[str] = None,
        selected_robot: Optional[str] = None,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> str:
        job_id = f"job_{uuid.uuid4().hex}"
        now = time.time()
        with self._lock, self._connect() as connection:
            connection.execute(
                """
                INSERT INTO jobs(
                    job_id, conversation_id, kind, phase, provider,
                    selected_robot, request_text, metadata_json, created_at, updated_at
                ) VALUES (?, ?, ?, 'running', ?, ?, ?, ?, ?, ?)
                """,
                (
                    job_id,
                    conversation_id,
                    kind,
                    provider,
                    selected_robot,
                    request_text,
                    json.dumps(metadata or {}),
                    now,
                    now,
                ),
            )
        return job_id

    def append_event(
        self, job_id: str, event_type: str, payload: Dict[str, Any]
    ) -> int:
        now = time.time()
        with self._lock, self._connect() as connection:
            connection.execute("BEGIN IMMEDIATE")
            row = connection.execute(
                "SELECT COALESCE(MAX(sequence), 0) + 1 AS next FROM events WHERE job_id = ?",
                (job_id,),
            ).fetchone()
            sequence = int(row["next"])
            connection.execute(
                """
                INSERT INTO events(job_id, sequence, event_type, payload_json, created_at)
                VALUES (?, ?, ?, ?, ?)
                """,
                (job_id, sequence, event_type, json.dumps(payload), now),
            )
            connection.execute(
                "UPDATE jobs SET updated_at = ? WHERE job_id = ?", (now, job_id)
            )
        return sequence

    def set_phase(
        self, job_id: str, phase: str, *, error_text: Optional[str] = None
    ) -> None:
        with self._lock, self._connect() as connection:
            connection.execute(
                "UPDATE jobs SET phase = ?, error_text = ?, updated_at = ? WHERE job_id = ?",
                (phase, error_text, time.time(), job_id),
            )

    def interrupt_running_jobs(self) -> int:
        """Close jobs left running by a prior Cortex process.

        This is called once when the service's singleton supervisor starts, not
        when individual requests arrive.
        """
        with self._lock, self._connect() as connection:
            cursor = connection.execute(
                """
                UPDATE jobs
                SET phase = 'interrupted',
                    error_text = 'Cortex restarted before the provider stream completed',
                    updated_at = ?
                WHERE phase = 'running'
                """,
                (time.time(),),
            )
        return max(cursor.rowcount, 0)

    def get_job(self, job_id: str) -> Optional[Dict[str, Any]]:
        with self._lock, self._connect() as connection:
            row = connection.execute(
                "SELECT * FROM jobs WHERE job_id = ?", (job_id,)
            ).fetchone()
        return self._job_dict(row) if row else None

    def list_jobs(self, limit: int = 50) -> list[Dict[str, Any]]:
        bounded_limit = max(1, min(int(limit), 500))
        with self._lock, self._connect() as connection:
            rows = connection.execute(
                "SELECT * FROM jobs ORDER BY updated_at DESC LIMIT ?", (bounded_limit,)
            ).fetchall()
        return [self._job_dict(row) for row in rows]

    def get_events(self, job_id: str) -> list[Dict[str, Any]]:
        with self._lock, self._connect() as connection:
            rows = connection.execute(
                "SELECT * FROM events WHERE job_id = ? ORDER BY sequence", (job_id,)
            ).fetchall()
        return [
            {
                "event_id": row["event_id"],
                "job_id": row["job_id"],
                "sequence": row["sequence"],
                "event_type": row["event_type"],
                "payload": json.loads(row["payload_json"]),
                "created_at": row["created_at"],
            }
            for row in rows
        ]

    def propose_memory(
        self,
        *,
        scope: str,
        memory_key: str,
        value: Any,
        confidence: float,
        source_job_id: Optional[str] = None,
    ) -> str:
        """Store a candidate; candidates never affect prompts until reviewed."""
        memory_id = f"mem_{uuid.uuid4().hex}"
        with self._lock, self._connect() as connection:
            connection.execute(
                """
                INSERT INTO memories(
                    memory_id, scope, memory_key, value_json, status, confidence,
                    source_job_id, created_at
                ) VALUES (?, ?, ?, ?, 'candidate', ?, ?, ?)
                """,
                (
                    memory_id,
                    scope,
                    memory_key,
                    json.dumps(value),
                    max(0.0, min(float(confidence), 1.0)),
                    source_job_id,
                    time.time(),
                ),
            )
        return memory_id

    def review_memory(self, memory_id: str, status: str) -> None:
        if status not in {"confirmed", "rejected"}:
            raise ValueError("memory status must be 'confirmed' or 'rejected'")
        with self._lock, self._connect() as connection:
            connection.execute(
                "UPDATE memories SET status = ?, reviewed_at = ? WHERE memory_id = ?",
                (status, time.time(), memory_id),
            )

    def list_memories(
        self, *, scope: Optional[str] = None, status: Optional[str] = None
    ) -> list[Dict[str, Any]]:
        clauses: list[str] = []
        values: list[Any] = []
        if scope is not None:
            clauses.append("scope = ?")
            values.append(scope)
        if status is not None:
            clauses.append("status = ?")
            values.append(status)
        where = f" WHERE {' AND '.join(clauses)}" if clauses else ""
        with self._lock, self._connect() as connection:
            rows = connection.execute(
                f"SELECT * FROM memories{where} ORDER BY created_at DESC", values
            ).fetchall()
        return [
            self._memory_dict(row)
            for row in rows
        ]

    def add_compaction(
        self, *, job_id: str, through_sequence: int, summary: str
    ) -> str:
        compaction_id = f"cmp_{uuid.uuid4().hex}"
        with self._lock, self._connect() as connection:
            connection.execute(
                """
                INSERT INTO compactions(
                    compaction_id, job_id, through_sequence, summary, created_at
                ) VALUES (?, ?, ?, ?, ?)
                """,
                (compaction_id, job_id, through_sequence, summary, time.time()),
            )
        return compaction_id

    def get_replay_context(self, job_id: str) -> Dict[str, Any]:
        """Return the latest summary plus events that happened after it.

        Compaction is non-destructive: callers receive a smaller replay window,
        while ``get_events`` retains the complete evidence trail.
        """
        with self._lock, self._connect() as connection:
            compaction = connection.execute(
                """
                SELECT * FROM compactions
                WHERE job_id = ?
                ORDER BY through_sequence DESC, created_at DESC
                LIMIT 1
                """,
                (job_id,),
            ).fetchone()
            through_sequence = (
                int(compaction["through_sequence"]) if compaction is not None else 0
            )
            rows = connection.execute(
                """
                SELECT * FROM events
                WHERE job_id = ? AND sequence > ?
                ORDER BY sequence
                """,
                (job_id, through_sequence),
            ).fetchall()
        return {
            "summary": compaction["summary"] if compaction is not None else None,
            "through_sequence": through_sequence,
            "events": [
                {
                    "sequence": row["sequence"],
                    "event_type": row["event_type"],
                    "payload": json.loads(row["payload_json"]),
                    "created_at": row["created_at"],
                }
                for row in rows
            ],
        }

    @staticmethod
    def _job_dict(row: sqlite3.Row) -> Dict[str, Any]:
        result = dict(row)
        result["metadata"] = json.loads(result.pop("metadata_json"))
        return result

    @staticmethod
    def _memory_dict(row: sqlite3.Row) -> Dict[str, Any]:
        result = dict(row)
        result["value"] = json.loads(result.pop("value_json"))
        return result
