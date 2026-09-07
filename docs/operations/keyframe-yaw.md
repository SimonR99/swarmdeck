# Rotated duplicates in the optimized map

Keyframe points travel in the robot's base frame at capture. The packet's
`t_odom_base` carries the adapter's map-frame pose for that same observation.
The optimizer reconstructs the cloud by applying a solved pose to those base
points. A scan paired with a later yaw can therefore draw a rotated copy of
otherwise correct geometry.

The September 2026 investigation found a reproducible capture-gate defect:

1. A scan during a fast turn updates the yaw-rate reference and is rejected.
2. The same timestamp arrives again, possibly with an updated TF pose.
3. The old gate returned `False` for the zero interval, allowing `consider()`
   to enqueue that observation. The motion/scan-novelty gate does not prevent
   this because the first copy was never accepted.

The active ARGoS adapter logged above-limit accepted-rate diagnostics and
unusable timestamps in `sessions/runs/turn-02`. Those counters alone do not
measure the true rotation of each cloud: the accepted-rate value was carried
from the previous valid interval. The regression reproduces the admission bug
through `consider()` and verifies the actual upload queue, rather than relying
on that counter as ground truth.

The gate now rejects duplicate, out-of-order, and sub-millisecond intervals
when yaw gating is enabled, retaining its reference for the next fresh scan.
Nonfinite timestamps are rejected before they can poison that reference.

Both simulator scan formats now use capture-time TF. The planar fallback
previously used the latest pose. Robot translation and wrapped yaw are
interpolated between odom-to-base samples, with a maximum interpolation span
of 0.5 seconds. Capture is skipped if the stamp is missing, outside retained
history, spans a longer TF outage, or no map-to-odom transform has arrived.
The slowly updated map-to-odom correction retains its nearest-value lookup.
The adapter reports skipped captures as `rejected_captures` in its gate log.
A scan that arrives before its required TF is skipped; capture resumes on
subsequent scans with available history.

The ROS 2 hardware adapter likewise skips keyframe capture when the requested
historical TF is unavailable, instead of substituting its latest odometry.

These changes prevent new bad captures from these paths. They do not repair
previously stored keyframes or prove that every rotated duplicate has the same
cause. Incorrect sensor timestamps, physical scan distortion, map-frame jumps,
and bad loop closures remain separate possibilities.

The service's default graph mode uses `odometry_as_pose=true`: occupancy and
cloud rendering place each onboard trajectory under a rigid alignment, rather
than applying every optimized keyframe pose. Thus an accepted loop closure
need not remove a bad individual captured yaw from the displayed map. Compare
rendering modes with a recorded run before changing
`SWARMDECK_SLAM_ODOMETRY_AS_POSE`; this fix does not change that default.
