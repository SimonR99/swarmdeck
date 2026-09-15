# Current stack operations

Use these commands from the repository root. They describe the checked-in
launcher and Compose files on `planning-refactor`; physical robot profiles
have additional sensor, ROS-domain, and calibration requirements.

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
fallback. Advanced direct Compose usage is documented in
[decentralized autonomy](decentralized-autonomy.md); it must reproduce those
mission, domain, capture-provider, and map-authority settings. A sparse
cold-start scan cannot certify the whole robot body volume, and unknown or
stale terrain remains subject to the planner's safety gates.

In MOLA mode, MGG reads a read-only OctoMap spatial index generated from the
coherent MOLA product. The independent raw-cloud/depth mapper path is disabled;
the OctoMap index is an internal query structure, not a second map authority.
Hardware MOLA use remains an overlay that requires qualified peer capture
provenance, calibration, and frame ownership.

Unchanged planner artifacts reuse their decoded grid without rereading or
hashing the file. The cache checks file identity and the current publication;
changed files undergo the full bounded read, hash, and validation again.

Use `make docker-ps`, `curl -fsS http://localhost:8090/health`, and the UI
status panels to check the services. For an onboard run, confirm the
mission, component, graph revision, geometry revision, navigation frame, and
source timestamp agree before interpreting a planner result. Rejected or stale
authority must not be repaired by relabeling a grid frame.

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
terrain, or multi-host recovery path. Exact historical test settings and debug
chronology are retained in [decentralized autonomy](decentralized-autonomy.md),
[navigation acceptance](navigation-live-map-acceptance.md), and related
acceptance pages.
