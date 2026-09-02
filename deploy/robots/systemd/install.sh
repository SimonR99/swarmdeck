#!/usr/bin/env bash
# Install the bounded time-sync gate on a robot. Safe to run on a robot that is
# in use: it only writes unit files, reloads the systemd config and enables units
# for the NEXT boot. It never restarts docker or any container.
set -euo pipefail
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

sudo install -m 0755 "$SRC/swarmdeck-wait-timesync"  /usr/local/bin/swarmdeck-wait-timesync
sudo install -m 0755 "$SRC/swarmdeck-late-timesync"  /usr/local/bin/swarmdeck-late-timesync
sudo install -m 0644 "$SRC/swarmdeck-timesync-wait.service"    /etc/systemd/system/
sudo install -m 0644 "$SRC/swarmdeck-late-timesync.service"    /etc/systemd/system/
sudo install -m 0644 "$SRC/swarmdeck-late-timesync.path"       /etc/systemd/system/
sudo install -d -m 0755 /etc/systemd/system/docker.service.d
sudo install -m 0644 "$SRC/docker.service.d/10-swarmdeck-timesync.conf" \
                     /etc/systemd/system/docker.service.d/
# Do not clobber local tuning, but say so loudly: an older copy will be missing
# newer keys and the defaults for those are then whatever the scripts fall back
# to, which may not be what you want.
if [ ! -e /etc/default/swarmdeck-timesync ]; then
    sudo install -m 0644 "$SRC/swarmdeck-timesync.default" /etc/default/swarmdeck-timesync
    echo "wrote /etc/default/swarmdeck-timesync"
else
    echo "NOTE: /etc/default/swarmdeck-timesync already exists and was left alone."
    missing=""
    for k in SWARMDECK_TIMESYNC_TIMEOUT SWARMDECK_LATE_TIMESYNC_THRESHOLD \
             SWARMDECK_LATE_TIMESYNC_RESTART SWARMDECK_LATE_TIMESYNC_CONTAINERS; do
        grep -q "^$k=" /etc/default/swarmdeck-timesync || missing="$missing $k"
    done
    if [ -n "$missing" ]; then
        echo "      keys missing from your copy:$missing"
        echo "      reference: $SRC/swarmdeck-timesync.default"
    fi
fi

sudo systemctl daemon-reload
sudo systemctl enable swarmdeck-timesync-wait.service swarmdeck-late-timesync.path

echo "installed. Docker ordering is now:"
systemctl show docker.service -p After --value | tr ' ' '\n' | grep -E "time|swarmdeck" | sed 's/^/  /'
echo "takes effect at next boot; nothing was restarted."
