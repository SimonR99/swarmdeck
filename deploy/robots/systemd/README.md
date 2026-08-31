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

`swarmdeck-late-timesync.path` covers that remaining case, and sizes its
response to the damage.

The Orin has no RTC battery. `systemd-timesyncd` restores the last-known time
from `/var/lib/systemd/timesync/clock` at boot, so the clock comes up *behind*
by however long the machine was powered off, and the eventual correction is a
forward step of that same size. Six hours off the charger means a six hour step.

So the handler measures the step rather than guessing it, using a
realtime/monotonic pair recorded when the wait gave up (`/proc/uptime` is
unaffected by `settimeofday`, so monotonic-elapsed minus realtime-elapsed *is*
the step):

- **below `SWARMDECK_LATE_TIMESYNC_THRESHOLD`** (default 2 s): cosmetic, log and
  leave the stack alone.
- **at or above it**: every driver that started beforehand is stamping in the
  old frame, which invalidates the TF buffers (10 s by default) and nav2's
  message filters, so the session is already broken and restarting the sensor
  containers is strictly an improvement.

Set `SWARMDECK_LATE_TIMESYNC_RESTART=0` to force log-only regardless of size.

## Install

    ./install.sh

Writes the units, reloads systemd and enables them for the next boot. It never
restarts Docker or any container, so it is safe to run on a robot in use.

Tune the timeout in `/etc/default/swarmdeck-timesync`.
