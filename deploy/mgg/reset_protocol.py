"""Permissions for the reset-only bind mount shared by host and containers."""

from __future__ import annotations

import errno
import os
from pathlib import Path
import re
import tempfile


def prepare_directory(path: Path) -> None:
    """Publish a cross-UID writable directory without exposing an umask window.

    Atomic replacement of another UID's status files requires a writable,
    non-sticky directory. Only reset protocol directories may use this helper;
    ancestors (notably the user's sessions directory) retain their permissions.
    """
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(
            prefix=".reset-directory-", dir=path.parent
        ) as temporary:
            os.chmod(temporary, 0o777)
            try:
                os.rename(temporary, path)
            except OSError as exc:
                if exc.errno not in {errno.EEXIST, errno.ENOTEMPTY}:
                    raise
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    try:
        if os.fstat(descriptor).st_mode & 0o7777 != 0o777:
            os.fchmod(descriptor, 0o777)
    finally:
        os.close(descriptor)


def open_lock(path: Path) -> int:
    """Open a stable, readable lock inode, independent of the creator's umask."""
    if not path.exists():
        descriptor, temporary = tempfile.mkstemp(prefix=".reset-lock-", dir=path.parent)
        try:
            os.fchmod(descriptor, 0o644)
            try:
                os.link(temporary, path)
            except FileExistsError:
                pass
        finally:
            os.close(descriptor)
            os.unlink(temporary)
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    try:
        if os.fstat(descriptor).st_mode & 0o7777 != 0o644:
            os.fchmod(descriptor, 0o644)
    except BaseException:
        os.close(descriptor)
        raise
    return descriptor


def prepare_reset_directories(root: Path, robots: list[str]) -> None:
    """Prepare only the reset root and named robots, including old root-owned dirs.

    An unprivileged process cannot repair a restrictive directory owned by root;
    running this same preparation from the root MGG container repairs it without
    changing ownership or touching unrelated session files.
    """
    if any(re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", robot) is None for robot in robots):
        raise ValueError("robot IDs must be simple ROS namespaces")
    prepare_directory(root)
    os.close(open_lock(root / "protocol.lock"))
    if robots:
        prepare_directory(root / "robots")
    for robot in robots:
        directory = root / "robots" / robot
        prepare_directory(directory)
        prepare_directory(directory / "requests")
        os.close(open_lock(directory / "protocol.lock"))
