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
| Explicit legacy path | `./scripts/sim-up --legacy-cloud --drift` | Central cloud/OctoMap path for comparison |
| Physical operator stack | `make up-deploy` | `configs/hardware_fleet.yaml`, graph-mode central SLAM |

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
`SWARMDECK_MOLA_PLANNER_MAPS=true`, `SWARMDECK_PLANNER_MAP_PROVIDER=mola`, and
`SWARMDECK_MGG_MAP_BACKEND=mola_snapshot`; these are also the defaults in the
MOLA Compose overlays. It also sets the simulation capture provider to
`simulation`, creates a fresh mission/domain epoch, and keeps a reset
supervisor for lifecycle recovery. The indexed query polls every 0.5 seconds
and rejects snapshots older than 3 seconds. `--legacy-cloud` is the explicit
fallback. Advanced direct Compose usage is recorded in the archived
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

Use `make docker-ps`, `curl -fsS http://localhost:8090/health`, and the UI
status panels to check the services. For an onboard run, confirm the
mission, component, graph revision, geometry revision, navigation frame, and
source timestamp agree before interpreting a planner result. Rejected or stale
authority must not be repaired by relabeling a grid frame.

The normal ARGoS launcher composes the peer, mapping, and onboard-planning
services and selects MOLA as MGG's map authority. The explicit `--legacy-cloud`
launcher mode retains the former central cloud/OctoMap path for comparison and
recovery. MOLA consumes the peer snapshot and does not optimize poses or publish
competing TF edges. MGG consumes an exact, mission-pinned map authority. A
missing or stale shared transform blocks the dependent operation rather than
assuming that two local frames coincide.

## Frames and ownership

Adapters capture points in a sensor frame and associate them with a pose at the
capture timestamp. The local odometry and navigation frames remain robot-owned.
Peer Swarm-SLAM can establish a verified component frame and correction. The
same correction identity and map revision flow into the MOLA product and
indexed query. MGG plans in the configured robot navigation frame; Nav2 handles
local obstacle avoidance and the adapter owns the final command boundary.

The server stores replicas, events, keyframes, and catalogue metadata for the
operator. It receives peer and MOLA revisions as replicas without becoming the
authority for an onboard planner. Browser overlays are therefore diagnostic
unless their component and frame metadata are valid.

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
