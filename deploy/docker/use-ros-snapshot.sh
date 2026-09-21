#!/usr/bin/env bash
# Pin ROS packages, including those inherited from a newer ros:* base image.
# Ubuntu security updates remain enabled; this is not a bit-for-bit OS lock.
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

case "${ROS_DISTRO:?ROS_DISTRO is required}" in
  jazzy) snapshot=2026-06-18; suite=noble ;;
  humble) snapshot=2026-07-02; suite=jammy ;;
  *) echo "No qualified ROS snapshot for ${ROS_DISTRO}" >&2; exit 1 ;;
esac

# SnapshotRepository's signing key, vendored so builds do not need a keyserver.
# https://wiki.ros.org/SnapshotRepository
key=/usr/share/keyrings/swarmdeck-ros-snapshot-keyring.gpg
gpg --batch --yes --dearmor --output "$key" /tmp/ros-snapshot-key.asc
fingerprint=$(gpg --batch --show-keys --with-colons "$key" | sed -n 's/^fpr:::::::::\([^:]*\):$/\1/p' | sed -n '1p')
test "$fingerprint" = 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA

# Official images have used both ros2.list and a ros2-apt-source-owned deb822
# symlink. Remove ROS entries in either representation, preserving Ubuntu and
# unrelated sources. Never leave a mutable ROS repository as a second candidate.
python3 - "$ROS_DISTRO" "$snapshot" "$suite" "$key" <<'PY'
from pathlib import Path
import re
import sys

distro, snapshot, suite, key = sys.argv[1:]
ros_source = re.compile(r'https?://(?:packages\.ros\.org/ros2(?:-testing)?|snapshots\.ros\.org/[^/]+)(?:/|\s)')
for path in [Path('/etc/apt/sources.list'), *Path('/etc/apt/sources.list.d').glob('*.list'), *Path('/etc/apt/sources.list.d').glob('*.sources')]:
    if not path.exists():
        continue
    original = path.read_text()
    separator = '\n\n' if path.suffix == '.sources' else '\n'
    entries = original.split(separator)
    retained = [entry for entry in entries if not ros_source.search(entry)]
    if len(retained) != len(entries):
        path.unlink()  # Do not rewrite the target of an apt-source package symlink.
        if any(entry.strip() for entry in retained):
            path.write_text(separator.join(retained).rstrip() + '\n')
Path('/etc/apt/sources.list.d/swarmdeck-ros-snapshot.list').write_text(
    f'deb [signed-by={key}] http://snapshots.ros.org/{distro}/{snapshot}/ubuntu {suite} main\n'
)
# A priority above 1000 allows a newer base image to downgrade to the snapshot.
Path('/etc/apt/preferences.d/swarmdeck-ros-snapshot').write_text(
    'Package: *\nPin: origin "snapshots.ros.org"\nPin-Priority: 1001\n'
)
PY

# HTTP is intentional: snapshots.ros.org does not serve a matching TLS
# certificate. Apt authenticates InRelease and every package with the key above;
# do not add trusted=yes, disable signature checks, or fall back to current apt.
apt-get update
apt-get -y --allow-downgrades --no-remove dist-upgrade
rm -f /tmp/ros-snapshot-key.asc
