"""Reclaim old mission directories and fold back a peer's SQLite WAL.

Missions accumulate under ``<store_root>/<mission_id>/`` for as long as
nothing ever retires them; one deployment carried about 500 MB across 8 old
missions, none revisited after the fleet moved on. Nothing here runs itself:
``swarmdeck_peer``'s ``launch/peer.launch.py`` calls `garbage_collect_missions` and
`checkpoint_wal` once per peer launch, the "run at startup" point named in
the plan, before any node opens this peer's stores.
"""

from __future__ import annotations

from dataclasses import dataclass
import shutil
import sqlite3
import time
from pathlib import Path
from uuid import UUID

# Keeping only the mission in use would retire a mission the moment its
# robots go quiet, discarding a map an operator may still want; keeping
# everything defeats the point. Three keeps a small, bounded working set
# without a configuration operators must tune to avoid surprise deletions.
DEFAULT_KEEP_RECENT = 3


def _is_mission_dir(path: Path) -> bool:
    if not path.is_dir():
        return False
    try:
        return str(UUID(path.name)) == path.name
    except ValueError:
        return False


def _mission_recency(path: Path) -> float:
    """The newest mtime under `path`, or the directory's own if it is empty."""

    newest = path.stat().st_mtime
    for entry in path.rglob("*"):
        try:
            newest = max(newest, entry.stat().st_mtime)
        except OSError:
            continue
    return newest


@dataclass(frozen=True)
class MissionGcDecision:
    mission_id: str
    recency: float
    kept: bool
    reason: str


def plan_mission_gc(
    store_root: str | Path,
    current_mission_id: str,
    *,
    keep_recent: int = DEFAULT_KEEP_RECENT,
) -> tuple[MissionGcDecision, ...]:
    """Decide which missions under `store_root` to keep, without touching disk.

    Kept: `current_mission_id`, never a removal candidate whether or not it
    has a directory yet, and the `keep_recent` other missions with the newest
    activity anywhere under them. Everything else is a removal candidate.
    """

    if keep_recent < 0:
        raise ValueError("keep_recent must not be negative")
    root = Path(store_root)
    missions = (
        [p for p in root.iterdir() if _is_mission_dir(p)] if root.is_dir() else []
    )
    # Peers sharing a store (the simulation's four peers share /maps) run
    # this at the same moment, so a sibling's rmtree can remove a mission
    # while it is being read. A mission that cannot be read is left out of
    # this plan: neither kept nor removed here.
    recencies = []
    for path in missions:
        try:
            recencies.append((path.name, _mission_recency(path)))
        except OSError:
            continue
    by_recency = sorted(recencies, key=lambda item: item[1], reverse=True)
    decisions: list[MissionGcDecision] = []
    kept_others = 0
    seen_current = False
    for mission_id, recency in by_recency:
        if mission_id == current_mission_id:
            seen_current = True
            decisions.append(
                MissionGcDecision(mission_id, recency, True, "current mission")
            )
        elif kept_others < keep_recent:
            kept_others += 1
            decisions.append(
                MissionGcDecision(mission_id, recency, True, "recently active")
            )
        else:
            decisions.append(
                MissionGcDecision(
                    mission_id,
                    recency,
                    False,
                    f"older than the {keep_recent} most recently active missions",
                )
            )
    if not seen_current:
        decisions.append(
            MissionGcDecision(
                current_mission_id,
                time.time(),
                True,
                "current mission (not yet on disk)",
            )
        )
    return tuple(decisions)


def garbage_collect_missions(
    store_root: str | Path,
    current_mission_id: str,
    *,
    keep_recent: int = DEFAULT_KEEP_RECENT,
    dry_run: bool = False,
    log=print,
) -> tuple[MissionGcDecision, ...]:
    """Apply `plan_mission_gc`'s decisions: remove every mission not kept.

    Every decision is logged, kept or not; a directory is actually removed
    only when `dry_run` is False. Nothing removed here is the mission in use,
    so nothing removed here is ever read again.
    """

    decisions = plan_mission_gc(store_root, current_mission_id, keep_recent=keep_recent)
    root = Path(store_root)
    for decision in decisions:
        if decision.kept:
            log(f"mission-gc: keep {decision.mission_id} ({decision.reason})")
            continue
        if dry_run:
            log(f"mission-gc: would remove {decision.mission_id} ({decision.reason})")
        else:
            log(f"mission-gc: removing {decision.mission_id} ({decision.reason})")
            shutil.rmtree(root / decision.mission_id, ignore_errors=True)
    return decisions


def checkpoint_wal(db_path: str | Path) -> bool:
    """``PRAGMA wal_checkpoint(TRUNCATE)`` an existing SQLite database.

    False when there is nothing to checkpoint: a fresh peer or robot that has
    never opened its mapping store has no WAL file to fold back, and a
    checkpoint must never itself create the database.
    """

    path = Path(db_path)
    if not path.is_file():
        return False
    connection = sqlite3.connect(path)
    try:
        connection.execute("PRAGMA wal_checkpoint(TRUNCATE)")
        connection.commit()
    finally:
        connection.close()
    return True
