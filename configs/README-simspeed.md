# Opt-in simulation rendering controls

Existing scenarios keep their current defaults: paced at 1× real time, lidar
at the profile's 10 Hz, and no parked-robot throttling. Do not run simulations
on the workstation. Reserve **tuf** through the controller before running any
simulation or the benchmark below.

For a workstation-oriented scenario **to benchmark on tuf**, copy the SubT
config and add:

```yaml
simulation:
  parked_lidar_rate: 2
```

Rebuild the ARGoS image: its camera-pool patch implements this setting. After
one simulated second with an exactly unchanged physical sensor anchor, lidar
rendering drops to 2 Hz. Any position or orientation change immediately restores
the normal 10 Hz schedule, even if the robot slides, turns, or is pushed without
a command. Small physical jitter conservatively keeps the full rate. All four
faces share a phase and observe the same anchor; a scan never combines faces
from different ticks. Cameras are not throttled.

`parked_lidar_rate` must be at least 1 Hz (the bridge's scan-age limit) and no greater than `fleet.lidar.rate`
and must divide the 100 Hz physics tick rate exactly (2 and 5 are supported;
3 is not). Omitting it disables the optimization. This is an opt-in reduction
in **parked** temporal coverage: moving objects near a parked robot are observed
less often. Do not assume safety/navigation acceptance from a throughput test.

For SubT, moving scans remain 10 Hz, 33 rings × 1024 azimuths, 100 m maximum
range, four 512×301 depth faces, and 3 cm range-noise standard deviation.
Parked scans retain the same density, range, and noise; only their cadence
changes. RGB and camera depth remain 320×240 at 5 Hz. No world mesh changes.

The generator also honors the existing `fleet.lidar.rate` field instead of
silently forcing 10 Hz. It must be finite, positive, no greater than 100 Hz,
and divide the physics tick rate exactly; 15 Hz is rejected rather than rounded.
A 5 Hz fleet is a **measurement probe**, not the chosen
moving-robot optimization. The ROS bridge still reports `LaserScan.scan_time`
as 0.1 s; qualifying a globally reduced spin rate requires bridge-owner review
of that metadata and Nav2/C-SLAM freshness assumptions. No bridge or adapter
contract is changed here.

## Unpaced GPU benchmark (tuf only)

```bash
# Run in this checkout ON tuf, after the controller grants an exclusive slot.
python3 argos/benchmark_simspeed.py --output /tmp/simspeed-results
```

The script refuses other hostnames and any existing `swarmdeck` containers.
It builds once, then alternates baseline / all-robot 5 Hz / parked-only 2 Hz
three times, reversing the two options in the middle round. Each fresh launch
warms up for 60 wall seconds before a 60-second `/clock` probe. The fleet is
parked; this is a renderer-throughput comparison, not exploration acceptance.
It stops its own stack after every window and on errors or interruption.

Benchmark configs set `simulation.realtime_factor: 0` to remove wall-clock
pacing. This is deliberately **not** the bridge's `realtime=false`: that flag
would switch to lockstep exchange and change the bottleneck. Normal scenarios
keep `realtime_factor: 1`; nonnegative finite factors are accepted.

Each result JSON records sim-seconds per wall-second, mean GPU utilization,
load average, available/free memory, and swap before/after the window. The
adjacent CSV contains one-second `nvidia-smi` samples (including ROS discovery
time); `.argos` files preserve the actual generated experiment. Config snapshots,
launch/stop logs, and revision/order metadata are retained. `--no-build` is for
an already rebuilt image, not an old image that ignores parked-lidar settings.

**Pending qualification:** native image rebuild/runtime probe, alternating tuf
measurements, and moving/parked cadence verification. RTX 4070 throughput and GPU
utilization indicate relative work saved; they do not prove Iris Xe RTF ≥ 0.8.
All-moving fleets receive no parked-throttling benefit. Distance culling is not
added: the pinned renderer already uses the lidar's far plane, and lowering it
would reduce the moving robot's range. Collision-only lidar rendering remains
deferred until measurements justify the additional renderer separation.
