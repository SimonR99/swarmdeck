#!/usr/bin/env bash
# Install for the NEXT boot; never clean live DDS objects or restart Docker.
set -euo pipefail
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
priv=()
if [[ $EUID -ne 0 ]]; then
    priv=(sudo)
fi
"${priv[@]}" install -m 0755 "$SRC/swarmdeck-clean-dds-shm" /usr/local/bin/swarmdeck-clean-dds-shm
"${priv[@]}" install -m 0644 "$SRC/swarmdeck-dds-shm-cleanup.service" /etc/systemd/system/
"${priv[@]}" install -d -m 0755 /etc/systemd/system/docker.service.d
"${priv[@]}" install -m 0644 "$SRC/docker.service.d/05-swarmdeck-dds-shm.conf" /etc/systemd/system/docker.service.d/
# Installing on a running robot must not schedule cleanup on a Docker restart.
# /run is volatile: this skip marker disappears on the next boot.
"${priv[@]}" touch /run/swarmdeck-dds-shm-cleaned
"${priv[@]}" systemctl daemon-reload
"${priv[@]}" systemctl enable swarmdeck-dds-shm-cleanup.service
echo 'DDS cleanup installed for next boot; no running services were restarted.'
