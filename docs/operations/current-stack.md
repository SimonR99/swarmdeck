# Current stack

This page describes the interfaces and commands that are current on
`planning-refactor`. The architecture, invariants and open gates are in
[the plan](../plan.md); dated trials are in the
[acceptance log](acceptance-log.md). Run the commands below from the repository
root. They describe the checked-in launcher and Compose files; physical robot
profiles have additional sensor, ROS-domain and calibration requirements.

## Select a mode

| Mode | Start command | Map/planner authority |
| --- | --- | --- |
| Mock UI | `docker compose -f deploy/compose/docker-compose.yml --profile mock up --build -d` | Synthetic adapter; no ROS or simulator |
| ARGoS drift | `./scripts/sim-up --drift` | Peer Swarm-SLAM, native MOLA, MGG, and synthetic odometry |
| ARGoS estimator | `./scripts/sim-up --fast-livo2` | Peer Swarm-SLAM, native MOLA, MGG, and Fast-LIVO2 |
| ARGoS development fleet | `./scripts/sim-up --dev` | Three robots, DRI rendering, drift odometry, MOLA |
| Physical operator stack | `make up-deploy` | Hardware adapters, peer Swarm-SLAM and MOLA |

The drift run is a development workload. It exercises sensor, peer, MOLA, and
adapter integration but cannot reproduce estimator failures. The estimator run
adds Fast-LIVO2 explicitly. Physical MGG sidecars can use MOLA when their peer
capture provenance, calibration, and mission configuration are qualified; the
server remains the operator and replica endpoint.

Custom scenario files are supported with `--scenario path/to/config.yaml` when
they define literal scalar `fleet.robot_count` and `fleet.robot_prefix` values;
the launcher validates the prefix before creating peer services.

## Onboard MOLA/MGG trial

The launcher is preferred because it selects the matching peer count, starts
the reset supervisor, and persists the exact Compose service set for `--status`,
`--logs`, and `--down`:

```bash
./scripts/sim-up --scenario bistro --drift
./scripts/sim-up --status
./scripts/sim-up --down
```

The mapping worker is persistent and native. The launcher sets
`SWARMDECK_SLAM_BACKEND=cslam`, `SWARMDECK_MOLA_PLANNER_MAPS=true`, and
`SWARMDECK_PLANNER_MAP_PROVIDER=mola`. The folded base Compose file also sets
the simulation capture provider to `simulation`, creates a fresh mission/domain
epoch, and keeps a reset supervisor for lifecycle recovery. The indexed query
polls every 0.5 seconds. Advanced direct Compose usage is recorded in the
[decentralized autonomy record](../archive/decentralized-autonomy.md); it must
reproduce those mission, domain, capture-provider, and map-authority settings.
A sparse cold-start scan cannot certify the whole robot body volume, and
unknown or stale terrain remains subject to the planner's safety gates.

In MOLA mode, MGG reads the coherent native planner grid directly through
`MapInterface`; no OctoMap tree is constructed. Occupied-only collision queries
use measured surface heights, with full voxel bounds when measurements are
missing. Strict queries retain full voxel bounds and unknown occupancy. The
independent raw-cloud/depth mapper path is disabled.
Hardware MOLA use remains an overlay that requires qualified peer capture
provenance, calibration, and frame ownership.

The launcher also enables the simulation peer-body mask: it sets
`SWARMDECK_PEER_BODY_MASK=true` and `SWARMDECK_PEER_PLATFORMS` (each robot's
platform from the scenario's `fleet.robot_type` and `fleet.robot_types`) so
each peer bridge drops returns inside another robot's body at the capture
stamp. Peer poses come from `/robot_N/ground_truth` in the `world` frame
(`SWARMDECK_PEER_POSE_TOPIC_TEMPLATE`, `SWARMDECK_PEER_POSE_FRAME`), joined
within `SWARMDECK_PEER_MASK_POSE_TOLERANCE_S` (0.05 s) and inflated by
`SWARMDECK_PEER_MASK_MARGIN_M` (0.15 m). Hardware profiles leave the mask
unset. The bridge status reports `peer_body_mask_points_dropped`,
`peer_body_mask_points_dropped_by_peer` and
`peer_body_mask_peers_skipped_stale`.

Unchanged planner artifacts reuse their decoded grid without rereading or
hashing the file. The cache checks file identity and the current publication;
changed files undergo the full bounded read, hash, and validation again.

The Python reader keeps a compatible predecessor during `PublicationPending`
only until its original coherent-read deadline; repeatedly seeing a pending
successor cannot renew that age. Corrupt or incompatible products fail closed.
Indexed queries check durable epochs for both the owner and actual geometry
participants before and after execution. Rolling objective continuation waits
out a transient publication gap with the same token, deadline and authority
fences; an expired continuation is not resurrected.

The pinned MGG source is supplemented by
`deploy/patches/mgg-motion-reliability.patch`: exact unobserved goals inherit a
checked driving plane before footprint validation, sparse partial corridors
end on useful measured support, and continuation starts from the path endpoint
actually emitted. The Home backbone uses bounded, fully checked breadcrumb
connectors only under the explicitly qualified simulation-ground policy.
Known occupancy, steps, drops and geofences remain refusals.

ROS apt dependencies come from signed snapshots rather than historical Docker
cache layers: Jazzy 2026-06-18 and Humble 2026-07-02. The helper also downgrades
newer ROS packages inherited from a base image. HTTP transport is intentional
because the snapshot endpoint's TLS certificate does not match; apt still
verifies the vendored signing key, release signatures and package hashes.
Ubuntu security updates and mutable base/Python dependencies mean this is not
a bit-for-bit OS lock.

Use `make docker-ps`, `curl -fsS http://localhost:8080/api/config`, and the UI
status panels to check the services. For an onboard run, confirm the mission,
component, graph revision, geometry revision, navigation frame, and source
timestamp agree before interpreting a planner result. Rejected or stale
authority must not be repaired by relabeling a grid frame.

The normal ARGoS launcher composes peer Swarm-SLAM, MOLA, indexed query, MGG,
and the controller as one stack. MOLA consumes peer snapshots and does not
optimize poses or publish competing TF edges. MGG consumes an exact,
mission-pinned map authority. A missing or stale shared transform blocks the
dependent operation rather than assuming that two local frames coincide.

## Frames and ownership

Adapters capture points in a sensor frame and associate them with a pose at the
capture timestamp. The navigation frame is the robot's continuous odometry
frame. Peer Swarm-SLAM corrections are data, not TF edges. The same correction
identity and map revision flow into the MOLA product and indexed query. MGG
plans in that frame; Nav2 handles trajectory tracking and local obstacles, and
the adapter owns the final command boundary.

The server stores replicas, events, keyframes, and catalogue metadata for the
operator. It receives peer and MOLA revisions as replicas without becoming the
authority for an onboard planner. Browser overlays are therefore diagnostic
unless their component and frame metadata are valid.

Each peer frontend launch claims a durable robot map epoch within the same
fleet mission. Its run UUID is the keyframe `session_id`, distinct from the
replica envelope's mission `session_id`. The target-only Reset map operation
retires the old run across peers and server replicas, preserves unrelated
peers, and anchors a fresh Home at the reset location. Moving commands carry
mission, robot epoch and `map_run_id` fences; stop/cancel remain unconditional.
See [reset operations](simulation-reset.md) for UUID replay, the 60 s deadline,
automatic frontend restart and the unsupported-hardware boundary.

The UI keeps map transforms when a raster grows, rejects stale or malformed
patches, projects poses with their full SE(3) XYZ values in 3D, and samples
rendered routes to at most 1,024 points while preserving both endpoints. The
top-down 2D layer keeps the selected map-frame transform and uses XY for
display, so a path does not slide or acquire visually exaggerated Z jumps when
a new revision arrives.

The simulation terrain admission settings are 0.15 m for Bunker and Scout and
0.30 m for Spot. These are simulator parameters, not hardware guarantees.
Camera-colorized point clouds require synchronized images, camera/lidar
calibration, and capture-time poses; RGB-D depth is useful for correspondence
and occlusion handling but is not required for every colorized capture.
Gaussian splatting is an optional fixed-pose reconstruction workflow and is not
launched merely by selecting a UI layer. MGG reads MOLA's immutable native
planner grid directly, without constructing an OctoMap tree. Independent
raw-cloud mapping is disabled in MOLA mode; OctoMap belongs to the explicit
legacy cloud backend.

## Hardware boundary

The operator stack communicates with physical adapters over the configured
backend and robot profiles. Drivers, localization, TF, MGG, and Nav2 remain on
the robot. Start with [hardware bring-up](hardware-bringup.md), then verify a
short manual drive, fresh keyframes, a clear-space navigation goal, and
stop-all before enabling autonomous motion. The hardware profiles are not
interchangeable with ARGoS configs; robot IDs, frames, ROS domains, and sensor
calibration are platform-specific.

## Validation scope

Native MOLA insertion, correction/replacement, serialization, snapshot
coherence, and bounded indexed queries have focused tests. Simulation tests
cover the current ARGoS scenarios and selected MGG contracts. Server fan-out
serializes each JSON publication once, writes to clients concurrently with a
bounded timeout, and preserves publication order; it does not build an
unbounded per-client queue. A local synthetic benchmark with four clients and
a 2,000-point route measured a median reduction from about 17.9 ms to 4.9 ms
for fan-out encoding and delivery; this is a workload sample, not a universal
network guarantee.

This does not qualify every physical platform, long-duration resource budget,
terrain, or multi-host recovery path. Measured trials are in the
[acceptance log](acceptance-log.md); exact historical settings and the debug
chronology stay in [the archive](../archive/README.md).
