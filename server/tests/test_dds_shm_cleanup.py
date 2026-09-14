"""Exercise boot cleanup against isolated filesystem/process fixtures."""

from pathlib import Path
import runpy

import pytest

REPO = Path(__file__).resolve().parents[2]
cleanup = runpy.run_path(str(REPO / "deploy/robots/systemd/swarmdeck-clean-dds-shm"))[
    "cleanup"
]


@pytest.fixture
def tree(tmp_path):
    shm, proc = tmp_path / "shm", tmp_path / "proc"
    shm.mkdir()
    proc.mkdir()
    return shm, proc, tmp_path / "done"


def process(proc, comm="python3", maps=""):
    pid = proc / "123"
    pid.mkdir()
    (pid / "comm").write_text(comm)
    (pid / "maps").write_text(maps)
    (pid / "fd").mkdir()
    return pid


def test_removes_empty_and_partially_initialized_dds_only_once(tree):
    shm, proc, marker = tree
    for name, size in [
        ("fastrtps_port19669", 52416),
        ("fastrtps_port19669_el", 0),
        ("fastrtps_0123456789abcdef", 100),
        ("sem.fastrtps_0123456789abcdef", 10),
    ]:
        (shm / name).write_bytes(bytes(size))
    (shm / "unrelated").write_text("keep")
    (shm / "fastrtps_port1").symlink_to(shm / "unrelated")
    (shm / "swarmdeck-quarantine").mkdir()
    cleanup(*tree)
    assert {p.name for p in shm.iterdir()} == {
        "unrelated",
        "fastrtps_port1",
        "swarmdeck-quarantine",
    }
    (shm / "fastrtps_port19669").write_text("live after startup")
    cleanup(*tree)
    assert (shm / "fastrtps_port19669").read_text() == "live after startup"


@pytest.mark.parametrize("kind", ["dockerd", "containerd-shim", "mapped", "open"])
def test_refuses_to_delete_anything_with_live_users(tree, kind):
    shm, proc, marker = tree
    port = shm / "fastrtps_port19669"
    port.write_bytes(bytes(52416))
    pid = process(proc, comm=kind, maps=str(port) if kind == "mapped" else "")
    if kind == "open":
        (pid / "fd" / "15").symlink_to(port)
    with pytest.raises(RuntimeError, match="refusing"):
        cleanup(*tree)
    assert port.exists()
    assert not marker.exists()


def test_unreadable_process_inspection_fails_closed(tree, monkeypatch):
    shm, proc, marker = tree
    port = shm / "fastrtps_port19669"
    port.touch()
    process(proc)
    read = Path.read_text

    def denied(path, *args, **kwargs):
        if path.name == "maps":
            raise PermissionError("denied")
        return read(path, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", denied)
    with pytest.raises(PermissionError):
        cleanup(*tree)
    assert port.exists()
    assert not marker.exists()
