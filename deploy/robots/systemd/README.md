# Bounded time-sync gate

## The problem

The Jetson boots with an untrusted clock and NTP corrects it later. Evidence
from Botman:

- `systemd-timesyncd` logs its own startup at `Dec 31 19:00:39`, which is Unix
  time 0 in local time.
- `journalctl --list-boots` reports a single boot spanning `2025-02-20` to
  `2026-08-31`, the signature of large clock steps within one boot.
- Container `StartedAt` values are recorded *before* the reported boot time,
  which is only possible if the clock moved underneath them.
- Measured: **44 s** from boot to `Initial synchronization to time server`.

Docker ships ordered `After=time-set.target`, which only means the clock has
been *set* from some source (RTC, fake-hwclock), not that it has been
*synchronized*. So sensor drivers start before the correction lands. Drivers
that start on opposite sides of the step disagree by the step size, which is
exactly FAST-LIVO2's

    [WARN] IMU and LiDAR not synced! delta time: 26.605148

and the same root cause as the earlier OAK-D stamp skew.

## Why not just enable systemd-time-wait-sync

Because it defaults to `TimeoutStartSec=infinity`, which would hang a robot in
the field that has no reachable time server. That is the normal case for us.

## What this does

`swarmdeck-timesync-wait.service` runs before `docker.service` and waits for
`/run/systemd/timesync/synchronized`, but:

- **skips the wait entirely when there is no default route** (field mode), so a
  disconnected robot boots at full speed;
- **gives up after `SWARMDECK_TIMESYNC_TIMEOUT` seconds** (default 60, chosen
  with margin over the measured 44 s) and continues anyway;
- **always exits 0**, so a robot that cannot reach NTP still boots cleanly with
  no failed units.

A clock that is wrong but free-running is fine for sensor fusion, because every
driver reads the same wrong clock and the stamps stay mutually consistent. The
damage comes from a *step* landing mid-session.

`swarmdeck-late-timesync.path` covers that remaining case: if the clock is
corrected after Docker started, it logs a loud warning naming the skew. The
automatic container restart is **opt-in** (`SWARMDECK_LATE_TIMESYNC_RESTART=1`)
because restarting drivers underneath a running mission can be worse than a
skewed clock.

## Install

    ./install.sh

Writes the units, reloads systemd and enables them for the next boot. It never
restarts Docker or any container, so it is safe to run on a robot in use.

Tune the timeout in `/etc/default/swarmdeck-timesync`.
