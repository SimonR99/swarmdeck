from __future__ import annotations

import os
import sqlite3
import time
import uuid

import pytest

from autonomy.mission_gc import (
    checkpoint_wal,
    garbage_collect_missions,
    plan_mission_gc,
)


def make_mission(root, age_s, robot="robot_0"):
    mission = str(uuid.uuid4())
    peer = root / mission / robot
    peer.mkdir(parents=True)
    marker = peer / "status.json"
    marker.write_text("{}")
    stamp = time.time() - age_s
    os.utime(marker, (stamp, stamp))
    return mission


def test_keeps_the_current_mission_and_the_n_most_recent_others(tmp_path):
    oldest = make_mission(tmp_path, age_s=300)
    older = make_mission(tmp_path, age_s=200)
    recent = make_mission(tmp_path, age_s=100)
    current = make_mission(tmp_path, age_s=0)

    decisions = {
        d.mission_id: d for d in plan_mission_gc(tmp_path, current, keep_recent=2)
    }
    assert decisions[current].kept
    assert decisions[recent].kept
    assert decisions[older].kept
    assert not decisions[oldest].kept


def test_the_current_mission_is_kept_even_without_a_directory_yet(tmp_path):
    make_mission(tmp_path, age_s=100)
    current = str(uuid.uuid4())

    decisions = plan_mission_gc(tmp_path, current, keep_recent=0)

    assert any(d.mission_id == current and d.kept for d in decisions)


def test_non_uuid_and_non_directory_entries_under_store_root_are_ignored(tmp_path):
    (tmp_path / "not-a-mission").mkdir()
    (tmp_path / "stray-file").write_text("x")
    current = str(uuid.uuid4())

    decisions = plan_mission_gc(tmp_path, current, keep_recent=0)

    assert [d.mission_id for d in decisions] == [current]


def test_dry_run_logs_without_removing_anything(tmp_path):
    oldest = make_mission(tmp_path, age_s=300)
    current = make_mission(tmp_path, age_s=0)
    logged = []

    garbage_collect_missions(
        tmp_path, current, keep_recent=0, dry_run=True, log=logged.append
    )

    assert (tmp_path / oldest).is_dir()
    assert any("would remove" in line and oldest in line for line in logged)


def test_removal_deletes_only_missions_not_kept(tmp_path):
    oldest = make_mission(tmp_path, age_s=300)
    kept = make_mission(tmp_path, age_s=100)
    current = make_mission(tmp_path, age_s=0)

    garbage_collect_missions(tmp_path, current, keep_recent=1, dry_run=False)

    assert not (tmp_path / oldest).exists()
    assert (tmp_path / kept).is_dir()
    assert (tmp_path / current).is_dir()


def test_negative_keep_recent_is_rejected(tmp_path):
    with pytest.raises(ValueError):
        plan_mission_gc(tmp_path, str(uuid.uuid4()), keep_recent=-1)


def test_checkpoint_wal_truncates_an_existing_database(tmp_path):
    db_path = tmp_path / "mapping.sqlite3"
    # sqlite auto-checkpoints and removes -wal/-shm when the last connection
    # closes, so the writer stays open while the WAL is inspected: exactly
    # the state a running peer's own long-lived connection leaves behind.
    writer = sqlite3.connect(db_path)
    try:
        writer.execute("PRAGMA journal_mode=WAL")
        writer.execute("CREATE TABLE t (v INTEGER)")
        writer.executemany("INSERT INTO t VALUES (?)", ((i,) for i in range(1000)))
        writer.commit()
        wal_path = db_path.with_name(db_path.name + "-wal")
        assert wal_path.is_file() and wal_path.stat().st_size > 0

        assert checkpoint_wal(db_path) is True

        assert wal_path.stat().st_size == 0
        assert writer.execute("SELECT COUNT(*) FROM t").fetchone() == (1000,)
    finally:
        writer.close()


def test_checkpoint_wal_is_a_noop_for_a_peer_that_never_opened_its_store(tmp_path):
    assert checkpoint_wal(tmp_path / "mapping.sqlite3") is False
    assert not (tmp_path / "mapping.sqlite3").exists()


def test_a_mission_removed_by_a_concurrent_gc_is_skipped(tmp_path, monkeypatch):
    # Four simulation peers share /maps and all collect at launch: a
    # sibling's rmtree can remove a mission between listing and reading it.
    import shutil

    from autonomy import mission_gc

    vanishing = make_mission(tmp_path, age_s=300)
    oldest = make_mission(tmp_path, age_s=200)
    current = make_mission(tmp_path, age_s=0)
    read = mission_gc._mission_recency

    def removed_by_a_sibling(path):
        if path.name == vanishing:
            shutil.rmtree(path)
        return read(path)

    monkeypatch.setattr(mission_gc, "_mission_recency", removed_by_a_sibling)

    decisions = garbage_collect_missions(
        tmp_path, current, keep_recent=0, dry_run=False, log=lambda line: None
    )

    assert {d.mission_id: d.kept for d in decisions} == {current: True, oldest: False}
    assert not (tmp_path / oldest).exists()
    assert (tmp_path / current).is_dir()
