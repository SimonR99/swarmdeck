# Decentralized autonomy integration

The `planning-refactor` branch introduces an opt-in onboard mapping path and
full-path MGG execution. Existing deployments keep their current map and motion
configuration until their platform validation is complete. The detailed design
and acceptance gates are in the [integration plan](../architecture/decentralized-autonomy-plan.md).

## Components and ownership

```mermaid
flowchart LR
  Sensors[LiDAR / cameras / IMU] --> Odom[Selected odometry frontend]
  Sensors --> Capture[Capture-time normalization]
  Odom --> Capture
  Capture --> Peer[Onboard Swarm-SLAM participant]
  Peer <-->|descriptors / verification / graph| Other[Other robot participants]
  Peer --> Store[Revisioned local submaps]
  Store --> MOLA[MOLA map consumer]
  Store --> Terrain[Observed occupancy / terrain queries]
  Terrain -. planner map adapter .-> MGG[MGG graph and grid planning]
  MGG --> Controller[Nav2 FollowPath / local avoidance]
  Store --> Replica[Resumable server replica]
  Replica --> Browser[Component inspector]
  Store -. calibrated RGB-D and fixed poses .-> Gaussian[Optional reconstruction worker]
```

The Python `autonomy` package has no ROS or MOLA imports in its contracts and
replication code. The geometry mapper uses NumPy; native MOLA integration lives
in `swarmdeck_ros/src/swarmdeck_mapping`. Swarm-SLAM supplies corrected poses.
The MOLA consumer does not optimize them or publish competing TF edges.

The default MGG deployment retains its OctoMap for high-rate grid expansion and
gain calculations. Its final corridor check can use one bounded indexed-map
request before returning Explore, Navigate, or ReturnHome. Both that index and
MOLA consume the same corrected snapshot and immutable chunks; the query does
not deserialize the `.metricmap` artifact or make RPC calls per planner voxel.

## Build and test

Run from the repository root:

```bash
docker build -f deploy/docker/Dockerfile.cslam -t swarmdeck-cslam:planning .
docker build -f deploy/docker/Dockerfile.mapping -t swarmdeck-mapping:phase0 .
docker build -f deploy/docker/Dockerfile.mgg -t swarmdeck-mgg:planning .
docker build -f deploy/docker/Dockerfile.sim -t swarmdeck-sim:planning .
docker build -f deploy/docker/Dockerfile.argos -t swarmdeck-argos:planning .

PYTHONPATH=.:server server/.venv/bin/python -m pytest autonomy/tests \
  adapters/test/test_exploration.py adapters/test/test_peer_coordination.py \
  server/tests/test_replica_views.py -q
```

The recorded local MGG validation used the same recipe under the
`swarmdeck-mgg:indexed` tag. This first command enables `mgg_ros` tests and runs the
typed objective/indexed-query tests with at most two compiler jobs:

```bash
docker run --name swarmdeck-mgg-indexed-native --rm \
  swarmdeck-mgg:planning bash -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    cd /opt/mgg/ros2
    CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --packages-select mgg_ros \
      --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=ON
    source install/setup.bash
    colcon test --packages-select mgg_ros --executor sequential \
      --event-handlers console_direct+ --return-code-on-test-failure
    colcon test-result --verbose'
```

The inert DDS contract smoke uses an isolated ROS domain and an action server
that records and cancels paths without publishing velocity commands:

```bash
docker run --rm \
  -e ROS_DOMAIN_ID=219 -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -v "$PWD:/app/swarmdeck:ro" swarmdeck-mgg:planning bash -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /opt/mgg/ros2/install/setup.bash
    cd /app/swarmdeck
    PYTHONPATH=/app/swarmdeck:$PYTHONPATH timeout 120 /usr/bin/python3 \
      adapters/test/ros/mgg_contract_smoke.py'
```

The MOLA image runs native insertion, correction, snapshot immutability, and
replacement tests during its build. It also imports the actual Python snapshot
and chunk format and serializes a MOLA metric map. MOLA 2.9.0 is pinned because
that is the version available in the tested ROS Jazzy repository; the compiler
checks for the required keyframe pose-update API.

Add `deploy/compose/docker-compose.mapping.yml` to a peer compose invocation to
run the server-independent MOLA consumer. It watches the shared `peer_maps`
volume and publishes a checked per-component artifact index under each peer's
`mola/` directory. The worker does not join the ROS graph and owns no TF edges.
For the ARGoS stack, start both mapping consumers with:

```bash
docker compose -f deploy/compose/docker-compose.yml \
  -f deploy/compose/docker-compose.peers.yml \
  -f deploy/compose/docker-compose.mapping.yml \
  --profile argos up -d mapping mapping-query
```

`autonomy.indexed_mapping.IndexedMapView` separately builds an immutable sparse
voxel view for MGG. It caches decoded chunks by SHA-256, so a pose-only graph
revision rebuilds transformed indexes without rereading geometry. Replacement
and retraction publish a new index and cannot leave an old occupied voxel active.
The defaults bound input to 1,000,000 points, 2,000,000 total voxels, 4,000,000
ray steps, eight seconds of background build time, 4,096 samples, 1,000,000
queried body voxels, and 250 ms per service query. Exceeding any bound produces
`UNAVAILABLE`; a revision mismatch produces `STALE`. Unknown space remains
unknown. Five-degree angular ray selection retains only actual measured rays to
keep dense spinning-lidar free-space indexing bounded.

The ROS adapter is `deploy/autonomy/indexed_map_server.py` and serves
`/<robot>/mapping/query_batch`. Requests contain component-frame samples and an
exact `(component_id, epoch, graph_revision, geometry_revision, source_stamp)`
key. `source_stamp` is the newest active submap's original ROS capture time. A
canonical `SWARMDECK_MISSION_ID` is required, and discovery is restricted to
`/maps/<mission>/`; historical missions on a persistent volume are never
selected implicitly. The service separately tracks successful coherent reads
with a local monotonic clock. Re-reading an unchanged snapshot refreshes that
deadline without rebuilding the index or rereading chunks, so a stationary map
does not expire while its producer and filesystem remain healthy. MGG
transforms route samples with the authoritative `T_component_navigation`, calls
once for a densified corridor, and rejects stale, unavailable, occupied,
unknown, excessive step/drop, roughness, or insufficient measured-clearance
results. Clearance is finite only when an overhead return bounds it or measured
rays cover the complete requested body volume; occupied terrain points alone do
not certify it. A `NaN` clearance remains unknown. The deterministic VLP-16
test uses nine captures over four travelled metres: accumulated ground rays
certify a Bunker-sized 1.6 m forward corridor, an unseen interval stays
`UNKNOWN`, and a wall is `OCCUPIED`. A lone sparse scan generally cannot cover
every body voxel, so enabling the indexed gate from a cold start can block
exploration until another trusted controller has accumulated enough ray
coverage. The gate remains opt-in for that reason.

Terrain queries select the observed surface beneath the queried robot body,
rather than the lowest return in the whole column. Local regression fixtures
cover stacked floors, downward steps, missing ground, walls, and gentle ramps.
Both body cells and terrain-column probes count toward the query work budget;
oversized bodies and unrepresentable coordinates are rejected before allocation.
In a local 15-sample flat-surface microbenchmark, 280 measured queries after
20 warmups had median/p95 times of 0.481/0.512 ms versus 0.497/0.531 ms before
the change. This small CPU fixture is not a Jetson latency or terrain-accuracy
qualification.

Swarm-SLAM repository commits are pinned in `deploy/cslam/upstream.repos`.
The local patch adds mission and causal solution identities, checks successful
results, rejects older results, and carries the actual component anchor pose.
All participating peers must use the same patched message definitions.

## Isolated ARGoS workstation run

The test overlay uses UI port **15173**, API **18080**, SLAM diagnostics **18090**,
and a separate Compose project. Use a separate checkout and its `sessions/`
directory so another deployment's state is not reused. Bistro's runtime assets
are `bistro_exterior.glb`, `bistro_lamps.inc`, and the precomputed lighting KTX
files under the sibling `argos3-examples` checkout; the source FBX/texture archive
is unnecessary at runtime.

```bash
export SWARMDECK_MISSION_ID="$(python3 -c 'import uuid; print(uuid.uuid4())')"
export SWARMDECK_TEST_DOMAIN=173  # choose an unused domain for this mission
export SWARMDECK_CONFIG=/app/configs/4robot_bistro.yaml

docker compose -p planning \
  -f deploy/compose/docker-compose.yml \
  -f deploy/compose/docker-compose.gpu.yml \
  -f deploy/compose/docker-compose.mgg.yml \
  -f deploy/compose/docker-compose.planning-test.yml \
  -f deploy/compose/docker-compose.peers.yml \
  --profile argos up -d --no-build \
  server slam mediamtx duck_detector sim argos mgg ui peer0 peer1 peer2 peer3
```

The peers initially run **shadow mapping** alongside the existing navigation
map. They exchange SLAM information directly through ROS middleware and upload
immutable map chunks to the server independently. Stopping the test server must
not stop map capture or optimization; motion still follows the existing
stop-on-operator-loss policy. Do not interpret that stop as a mapping dependency.

The test overlay explicitly uses UDP for containers sharing a network namespace
but separate IPC namespaces. This is a test transport choice, not a requirement
to disable shared memory between colocated production ROS processes.

To enable leased target arbitration after peer frame validation, set
`SWARMDECK_PEER_COORDINATION=1` in the simulation adapter environment. It requires
fresh `/robot_N/map_authority` messages. Without them, exploration waits instead
of assuming an identity transform between robots. Reservations share
`/swarmdeck/intentions`; costs and deterministic robot-ID tie breaking resolve
conflicting targets within one established component. A partition can cause
redundant coverage after leases expire. Component-local exhaustion alone does
not establish fleet completion.

To switch the isolated simulation from shadow mapping to onboard navigation,
append `docker-compose.mapping.yml` and `docker-compose.onboard-planning.yml`
to that invocation, and also start `mapping` and `mapping-query`. This pins the adapter to `SWARMDECK_MISSION_ID` and sets
`SWARMDECK_MAPPING_AUTHORITY=onboard`, `SWARMDECK_PLANNING_BACKEND=mgg`, and
`SWARMDECK_PEER_COORDINATION=1` on the adapter. The adapter stops uploading
keyframes to the central optimizer and stops downloading its navigation grid.
ROS callbacks forward each robot's own SLAM grid to Nav2 independently of the
operator connection. A grid must already have the exact configured navigation
frame; the adapter never relabels an incompatible frame. Local maps and scan
telemetry can still be uploaded for the dashboard.

Authority readers reject older optimizer solution orders, older indexed map
revisions, partial snapshots, and downgrades to legacy metadata. Equal coherent
heartbeats renew the three-second monotonic freshness window; rejected messages
cannot renew it or invalidate a newer target reservation. A live navigation-frame
TF correction can legitimately change `T_component_navigation` without changing
the immutable map revision, so it remains accepted; material corrections still
invalidate the active reservation. `SWARMDECK_MISSION_ID`, when configured,
selects the exact mission. Otherwise a reader pins its first accepted mission;
changing missions requires restarting the adapter/reader, consistent with the
fresh fleet mission and DDS domain required after native frontend restarts.

The indexed final corridor gate remains separately opt-in with
`SWARMDECK_INDEXED_MAP_QUERY=1`: sparse cold-start LiDAR coverage can leave the
body volume unknown and prevent any exploratory movement. Until sensor coverage
is validated, MGG uses its existing OctoMap and Nav2 local avoidance. This is
an explicit deployment choice, not a fallback after an indexed query fails.

Adapters with MGG configured advertise `plan_objective`. The GUI then sends
Navigate and Return Home objectives directly to that robot, without requiring
a central A* route. Return Home resolves the initial keyframe through the fresh
onboard correction; the server does not substitute its cached home coordinates.
Nav2 FollowPath receives the complete MGG path. Removing the onboard overlay
and restarting the adapter restores the legacy central map path.

MGG retains a bounded sequence of travelled poses while initial terrain support
or a connecting edge is unavailable. It admits samples in travel order after
map updates, preserving observed corners between regularly spaced samples.
Interpolated samples still require mapped support and collision checks. A
blocked sample holds back later samples; unchanged map revisions do not repeat
that failed query. Nearby owned vertices can be reused only after checking the
travelled transition and the connection to the existing vertex.

| ROS parameter | Default | Bound |
| --- | --- | --- |
| `global_vertex_spacing` | 1 m | 0.01–100 m |
| `global_backbone_pending_max_samples` | 128 | 8–4096 samples |
| `global_backbone_pending_max_length_m` | 128 m | Vertex spacing through 10000 m |
| `global_backbone_drain_max_samples` | 32 | 1–512 admissions per callback |

The pending limits bound retained history, not the complete mission graph.
Exceeding a history limit or receiving an unrepresentable displacement latches
a diagnostic and blocks Return Home until the planner restarts under a fresh
mission. Navigate can still use its local graph. These checks preserve evidence
requirements; they do not infer free terrain from the robot having travelled
through an area. The drain count bounds work between callbacks, without
preempting an individual terrain query.

Explicit Navigate and Return Home routes use `mgg_core::BoundedGridPlanner`
through the transport-free `GridPlanner` interface. Direct terrain-checked
segments remain the fast path. A blocked segment triggers deterministic local
8-neighbor A* within a bounded rectangle; unknown space, unsupported ground,
body collisions, excessive steps/inclines, and geofence violations cannot form
a detour. The output retains the checked terrain polyline and exact final goal
heading, then passes through base-height conversion and the optional indexed
snapshot gate. Explore retains its existing selector and startup policy.

| ROS parameter | Default | Bound |
| --- | --- | --- |
| `grid_refinement_resolution_m` | 0.25 m | Map resolution through `max(2 m, map resolution)` |
| `grid_refinement_margin_m` | 1 m | 0–10 m around each segment |
| `grid_refinement_max_cells` | 4096 | 16–65536 cells per segment |
| `grid_refinement_max_expansions` | 2048 | 1–65536 expansions across the request |
| `grid_refinement_timeout_ms` | 50 ms | 1–5000 ms, checked between map calls |

The deadline is cooperative: an individual map callback is not preempted. The
core accepts a cancellation callback, but the ROS wrapper does not wire one and
the planning service has no cancel request. Stop invalidates the adapter's
pending result and stops execution while the bounded planner call finishes.
This is a local ground-plane search using the existing axis-aligned robot box and terrain support model, not a legged motion
planner or a full 3D search. Each XY cell keeps its first observed support height;
stacked-surface ambiguity can conservatively reject a feasible alternative.
All corridor vertices and the exact destination must remain valid. A blocked
vertex or a wall beyond the detour window returns `BLOCKED`; exclusion of the
failed edge from a new topological search and path speed limits remain future
work. Every projected segment receives a strict swept-box check at the actual
body center, including robots with zero center offset. These queries enumerate
all touched voxel keys and reject even one unknown voxel. Short swept AABB
envelopes cover motion between samples, including diagonal voxel crossings;
this adds conservative padding of at most one map resolution per axis.
Legacy box queries retain their existing partial-unknown policy (25% for
`getBoxStatus(..., true)`), and exploration continues through that interface.
Their voxel enumeration also now includes both faces. The underlying ground
predicate can conservatively reject some nonzero-offset footprints.

Map callbacks also bound their work: boxes exceeding 1,048,576 voxel keys and
strict sweeps exceeding an estimated 4,194,304 voxel visits return unknown.
These limits prevent oversized geometry from monopolizing a callback between
deadline checks; they are not hard real-time guarantees.

Every fleet Explore command carries a shared run UUID and participant list.
Robots exchange progress over `/swarmdeck/exploration_reports`. Completion means
all participants freshly report exhaustion of reachable frontiers, with no
assignments, in one verified component. Missing reports, disconnected components,
and map corrections prevent a fleet-complete report. Stop exploration and Stop
All invalidate exhausted status. This criterion does not certify coverage of
unobservable or unreachable physical space.

## Map persistence and replication

Each peer writes its own map under `/maps/<mission UUID>/<robot>/`:

- `geometry/`: SQLite metadata, calibrated captures, immutable XYZ-F32 chunks.
- `snapshot.json`: atomically replaced coherent map manifest.
- `status.json`: capture, optimization, and replica acknowledgement counters.
- `frontend-lifetime`: guard against reusing upstream keyframe IDs after restart.

The mapper keeps geometry in submap coordinates. Pose corrections update
transforms; geometry replacements and retractions remove the old active
contribution. Free-space queries require measured rays with known origins.
Absence of a point is never sufficient to declare traversability or clearance.

Optimizer diagnostics distinguish received results, accepted results, unchanged
accepted results, and applied corrections. `last_solution_order` records the
latest accepted causal identity. The legacy `solutions` counter counts only
pose-changing corrections; zero does not mean the optimizer is inactive.

The server exposes `/api/autonomy/replicas` and content-addressed
`/api/autonomy/chunks/<sha256>`. A manifest becomes visible only after every
referenced chunk exists. Reconnects negotiate missing hashes in one manifest request and transfer only
missing geometry. Pose-only updates reuse those hashes. The **Onboard map replicas**
control opens an inspector with explicit robot, session, and component selection.
It does not overlay disconnected components or issue navigation commands.

Choose **Open component in tactical map** to inspect that component with the
normal voxel, mesh, or point renderer. Immutable chunks are cached up to 64 MiB;
assembly is capped at 300,000 points before the existing graphics-quality
budgets are applied. Unchanged revisions skip geometry assembly. Pose-only
updates reuse chunk bytes and preserve the camera and ceiling; a different
component, frame, or epoch starts a new view. Transient fetch errors retain
the last coherent revision. A local browser fixture rendered 7,800 points and
preserved a 1.15 m ceiling across a pose revision and a quality change.

This component view is read-only. Robot poses, routes, costmaps, detections and
goals are hidden until their frame relationship can be verified. Component
Gaussian rendering is not enabled by this view yet. **Live map** or switching
to 2D leaves component inspection and restores the ordinary map source.

The server replica defaults to a 1 GiB geometry budget
(`SWARMDECK_REPLICA_MAX_BYTES`). When an upload reaches that limit, it first
reclaims up to 256 unreferenced chunks whose grace interval has elapsed. The
interval defaults to one hour (`SWARMDECK_REPLICA_RETENTION_S`) and starts again
when geometry loses a manifest reference or is uploaded again. Every published
robot/session manifest protects its referenced chunks, including shared geometry
and historical missions. A robot uploading over a very slow connection may need
to renegotiate missing hashes after the grace interval. Pose-only corrections
reuse geometry and do not make its chunks eligible for collection.

Inspect a bounded collection batch without deleting geometry:

```bash
python3 -m autonomy.replica_maintenance /data/replicas
# Add --collect to reclaim that eligible batch; no manifests are retired.
```

Publication, storage accounting, and collection use SQLite writer transactions
across server workers. Startup recovers interrupted file/metadata operations.
Collection never makes room by dropping a published map; storage exhaustion
remains explicit if protected geometry fills the budget. The onboard store
defaults to 512 MiB. Onboard archival and explicit retirement of old server
missions remain deployment policies to implement before long missions.

## ROS 2 hardware seam

`deploy/autonomy/peer.launch.py` runs one participant on one robot. Set
`SWARMDECK_PEER_NAMES` to the same ordered JSON robot list on every peer and
`SWARMDECK_PEER_INDEX` to that robot's index. Configure `SWARMDECK_CLOUD_TOPIC`,
`SWARMDECK_BASE_FRAME`, `SWARMDECK_ODOM_FRAME`,
`SWARMDECK_NAVIGATION_FRAME`, and `SWARMDECK_SENSOR_NAMESPACE` for its calibrated
ROS graph. Hardware with global TF topics should set `SWARMDECK_TF_TOPIC=/tf`
and `SWARMDECK_TF_STATIC_TOPIC=/tf_static`. Keep each robot's existing
`ROS_DOMAIN_ID` as its local sensor, adapter, MGG, and indexed-query domain. Set
one common, unused ROS domain across the participating robots as
`SWARMDECK_PEER_DOMAIN_ID`; if it is omitted, the bridge retains the existing
single-domain behavior. The dual-domain bridge consumes raw cloud and TF only
from the local domain, sends normalized cloud/odometry to Swarm-SLAM on the
peer domain, and returns map authority and keyframe metadata locally. It relays
only bounded `/swarmdeck/intentions` and `/swarmdeck/exploration_reports` JSON
between the domains. Robot-ID direction filters prevent relay loops, and the
JSON bytes are preserved exactly.
`deploy/compose/docker-compose.robot-peer.yml` profile packages this participant
with host networking and a persistent onboard map volume; the server URL is
optional. It does not change existing robot deployment defaults. Sensor normalization requires capture-time TF; it never substitutes
the latest pose for an older scan. The current TF input has no estimator
covariance or scan deskew interval, so these remain explicitly unknown at that
boundary. Selecting a full calibrated odometry/capture provider is required for
platform qualification.

The same `peer_mapping` profile provides optional `peer_mola_mapping` and
`peer_mapping_query` services on `onboard_peer_maps`. Start the three explicitly
after building the current mapping image:

```bash
docker compose -f deploy/compose/docker-compose.robot-peer.yml \
  --profile peer_mapping up -d \
  peer_mapping peer_mola_mapping peer_mapping_query
```

Hardware adapters read autonomy selection from their YAML. Qualifying a robot
for MGG planning and peer coordination therefore requires these explicit
additions alongside its existing `exploration.planner` calibration:

```yaml
actions:
  follow_path: /robot_namespace/follow_path  # real Nav2 FollowPath server
planning:
  backend: mgg
exploration:
  enabled: true
  peer_coordination: true
```

Set `SWARMDECK_MAPPING_AUTHORITY=onboard` in that adapter's environment. The
configured `follow_path` action must execute the complete MGG path; a robot with
only waypoint, trajectory, or direct velocity control needs a qualified path
adapter before enabling this configuration.

The query service uses host networking, host IPC, the local ROS domain,
and the exact mission UUID. The MOLA worker remains ROS-independent. Starting
these services does not enable MGG's client; set `SWARMDECK_INDEXED_MAP_QUERY=1`
only in a separately qualified MGG deployment after sufficient measured
free-space coverage exists.

`SWARMDECK_SERVER_URL` is optional. Peer discovery and graph exchange must remain
reachable without the dashboard or a server-hosted router. Test the actual
Humble/Jazzy and physical network combination before rollout. This branch does
not enable new motion defaults on hardware.

**Restart limitation:** upstream Swarm-SLAM uses robot-local integer keyframe IDs
without persisted estimator epochs. The launch exits if an authority process
dies, and a lifetime marker prevents a silent restart. Start a fresh fleet mission
and ROS domain after a frontend restart. Transparent single-peer restart and
restoration of its graph remain an upstream persistence task; the marker is a
fail-closed boundary, not a recovery implementation.

## Gaussian reconstruction

The [reconstruction jobs guide](reconstruction-jobs.md) describes optional fixed-
pose UMAMI batch jobs, immutable input manifests, cancellation, budgets, and stale
result rejection. The interface and fake-trainer tests run without private
sources or CUDA. Native UMAMI training, corrected Gaussian submap instancing,
and an onboard incremental trainer require their own validation. Gaussian
opacity is not occupancy and is not supplied to collision planning.

## Validation record

Earlier baseline validation on 2026-09-09, before grid refinement, included the
native MGG image, ROS planner suites, and a DDS fixture exercising full-path Explore, Navigate, cancellation,
and late-response fencing. The indexed fixture checks the exact component
transform and a 50 ms deadline against a delayed service response. The Python
autonomy, adapter, replication, reconstruction, command-routing, and backend
batch passed 332 tests from a clean export of the staged refactor, excluding
unrelated workspace edits.
Svelte type checks and the production build pass.
The browser rendered a local 7,800-point replica fixture. Replica preview code
loads on demand, limits rendering to 50,000 points and downloads to 64 MiB, and
preserves the camera and cached geometry during pose-only updates.

The local two-peer Swarm-SLAM fixture verifies a known 0.4 m translation and
6-degree yaw without a server. It requires both recipient estimates from the
same `(solution_clock, optimizer_robot_id, origin_robot_id)` and checks their
reported anchor. This exposed an upstream LiDAR transform-direction error:
TEASER/ICP estimates source points into destination coordinates, while GTSAM's
BetweenFactor requires the destination pose in source coordinates. The adapter
now inverts that relation before publishing either kind of LiDAR closure.
The corrected image passed the joint-solution fixture in about 25 seconds.
Bounded correspondence selection also prevents the prior multi-minute exact
clique search; the inlier acceptance threshold remains unchanged.

The native indexed-map ROS smoke checks actual generated request/response
serialization, FREE/OCCUPIED/UNKNOWN arrays, and stale key/timestamp rejection.
It caught and fixed a collision with rclpy's internal `_services` member that
unit tests of the index alone could not detect. The local MOLA native build
passed both C++ tests and serialized the Python fixture to a metric map.
The native dual-domain bridge smoke publishes conflicting `odom -> base_link`
transforms on the local sensor and shared peer domains. Normalized odometry uses
the local transform, raw TF/cloud never appears on the peer domain, normalized
capture reaches the peer domain, and authority/keyframe metadata returns only
to the local domain. Exact intention and exploration-report JSON crosses in
each permitted direction once; oversized payloads are rejected.

On the authorized amd64 workstation, ROS Jazzy Swarm-SLAM compiled with the
versioned message patch and its native Open3D/TEASER/message imports passed.
MOLA 2.9.0 compiled and passed native correction/replacement tests and the
Python-to-MOLA serialization smoke. Python tests cover stale solutions,
component changes, atomic replication, reconstruction cancellation, lease
expiry/reordering, and full-path execution arbitration.

The isolated four-robot Bistro ARGoS run produced keyframes and durable replicas
from all four robots (11, 14, 10, and 44 keyframes in the first bounded exploration
run). With only the test server stopped for 20 seconds, scan normalization
continued on every robot and robot 0 advanced its optimized map from revision 81
to 83 while its acknowledged revision stayed at 81. After restart, replication
advanced to revision 84 and the error cleared. This establishes server-independent
capture and solution processing; it does not establish inter-robot closure accuracy
or continued motion during an operator-link outage. Arm64 builds, physical robot trials,
transparent peer restart, and comparative four-robot coverage are separate
acceptance gates and must not be inferred from these unit/build results.

After reconnecting the workstation, a fresh mission
`88492d31-5c28-4de9-bbbd-8bfb1a74014d` ran Bistro with onboard authority, MGG
objectives, and peer coordination enabled. All four robots executed paths during
a 90-second Explore trial. Independently recorded ground-truth displacements
were approximately 12.7, 3.8, 15.1, and 25.2 m; the peers captured 26, 16, 32,
and 54 keyframes with no dropped keyframe pairs. Robot 1 encountered repeated
controller progress failures. All four reported stopped, inactive navigation,
and empty paths after Stop All; a subsequent short run also verified the
observer's command-write and shutdown handling. These measurements establish
execution, not exploration coverage or successful fleet coordination: the maps
remained in four disconnected components, and no inter-robot alignment was
validated. Singleton optimizer results were accepted without pose corrections.

A subsequent Return Home trial exposed an integration error: with the optional
indexed query disabled, MGG compared the request's onboard component ID with
its static local frame and rejected the request before path search. Authority
validation is now independent of that query option. An authority-bound request
must match the fresh component, graph, geometry, and source timestamp; the static
frame fallback applies only when no authority or snapshot metadata exists.
The original failed trial produced no route and no arrival: independent ground
truth stayed approximately 26.3 m from home. Stop All cleared navigation.
Five native objective-service tests cover this fix. A fresh mission
`b7f9edbb-4e97-4dcb-93bd-841baf074dd0` then executed a 25-second Explore trial
with all four robots and passed authority validation on Return Home. It exposed
a separate persistent-graph problem: Spot's first global vertex was inserted
after it had moved about 1.4 m, leaving home outside the graph; subsequent
parent connections also failed. Return Home produced no route, and independent
ground truth remained 6.32 m from home. This trial does not establish successful
return navigation.

The follow-up patch captures the initial home anchor from the first finite
odometry before motion. It keeps that landmark disconnected until mapped
support and free body clearance admit a terrain-following edge. Graph lookup
bounds both the current-pose and goal connections. Planner paths retain their
internal driving-height convention and are converted to robot base poses only
at response/publication boundaries. Explore retains the local graph's existing
startup policy for a root whose ground has not yet been observed; explicit
destinations still require mapped support. The updated patch was subsequently
retested in Bistro as recorded below. That native Jazzy validation passed 59 GoogleTest cases,
including actual legacy Explore → revision-pinned Explore calls with identical
base-height paths. Home service
fixtures exercise an observed detour, blocked walls and geofences, correct
nonzero body-center offsets, and rejection of one unknown body voxel. The
functional grid-service fixtures allow a one-second cooperative budget to
isolate behavior from host scheduling; they do not establish a 50 ms latency
result. Core tests separately exercise cancellation and deadline expiry.

The fixtures exposed incomplete voxel enumeration in free-box insertion and
collision queries, omitted path endpoints, and missed diagonal crossings in
sampled strict queries. Free-box and collision queries now enumerate discrete
keys; strict swept envelopes cover motion between samples. All eight deployment
patches replay from pinned MGG `902e868`; all 28 source files touched by the
patch stack match the retained native test source byte for byte. These checks establish native service behavior,
not end-to-end Bistro motion or successful arrival.

A fresh workstation trial on 2026-09-10 used commit `bff4fb4`, mission
`126bed69-9327-4d8f-a4b2-075fa7569d8e`, ROS domain 187, and newly built MGG and
mapping images. All four robots executed paths during a 30-second Explore
observation, all four replica revisions advanced, and Stop All cleared every
active path. Scout reported blocked, one Bunker reported local exhaustion after
less than a metre, and Spot lost a reservation. The peers remained in four
separate components; this does not pass coordinated coverage or closure gates.

Each subsequent Return Home request failed before producing a route:

| Robot | Native planner reason | Final independent XY distance from start |
| --- | --- | --- |
| `robot_0` (Bunker) | Current pose or goal outside the graph | 9.438 m |
| `robot_1` (Bunker) | Current pose or goal has no mapped terrain support | 0.773 m |
| `robot_2` (Scout) | Corridor has an unsupported endpoint | 6.888 m |
| `robot_3` (Spot) | Current pose or goal outside the graph | 6.861 m |

The passive recorder captured initial world poses before motion and more than
2,100 truth samples per robot without dropped samples. The distances above are
ARGoS ground truth, independently confirming failed return navigation. Persistent
graph growth stalled at six vertices for `robot_0` and one for `robot_3` despite
local exploration movement. Native fixture success therefore does not close the
Bistro acceptance gate. Preserve mapped-support and collision checks while
investigating persistent connectivity and sensor coverage.

The trajectory-history follow-up preserves poses recorded while terrain
admission is delayed, then checks them chronologically. Native Jazzy validation
passed 169 GoogleTest cases across 19 binaries, including 17 objective-service
cases. The delayed-support fixture retains an off-grid corner beyond the old
parent search radius; repeated travel reuses vertices, and lost history blocks
Home. The complete bent-path fixture took 78 ms including map construction,
which is not a per-callback latency benchmark. All nine deployment patches
replay from the pinned MGG revision and reproduce the tested source.

The next Bistro trial used commit `74c3484`, mission
`7bdcbb62-ddc3-438f-8bc5-0885fd374b1e`, and ROS domain 188. All four robots
executed paths during 30 seconds of Explore; Stop All cleared every active path.
Return Home then produced routes for the two robots previously outside the
graph, but neither completed arrival:

| Robot | Home result | Final independent XY distance from start |
| --- | --- | --- |
| `robot_0` | Executed a 37-point route; cancelled on an authority change | 8.380 m |
| `robot_1` | Corridor has an unsupported endpoint; no route | 8.650 m |
| `robot_2` | Current pose or goal outside the graph; no route | 9.076 m |
| `robot_3` | Executed a 38-point route; cancelled on an authority change | 7.512 m |

The passive recorder ran for 1118 seconds, including setup before physics.
The offline evidence checker classified all four Home trials as failed.
Trajectory retention improved connectivity, but arrival remains an open gate;
authority-change recovery and terrain coverage still require investigation.
For the two executing routes, cumulative navigation-transform shifts reached
21.458 mm (`robot_0`) and 25.423 mm (`robot_3`); cancellation followed within
84 ms. Mission, component, and optimizer correction revision stayed unchanged.
The live TF and inverse home transform moved, so the existing 20 mm validity
guard correctly rejected the routes. Routine map revision changes alone did
not cancel them. Recovery must replan against fresh authority while preserving
Stop and replacement-command precedence.

Staged launch also exposed the ARGoS experiment freshness rule: starting ARGoS
after the bridge has already generated `session.argos` makes its timestamp check
wait for another generation. For this trial, the generated file was verified to
postdate the current bridge start and contain four drift-odometry robots before
using `ARGOS_ACCEPT_STALE=true` for the ARGoS restart. Do not use that override
without checking that the experiment belongs to the current bridge session.

The operator reports that Bistro needs `max_mean_error` near 0.6 in the existing
registration pipeline. That parameter belongs to `swarmdeck_slam.verify`; native
Swarm-SLAM uses different admission controls. Investigating the relationship
and validating closure accuracy is deferred; this refactor does not relax either
pipeline's acceptance thresholds.

The trial also exposed repeated rebuilding of an unchanged map that exceeded
the indexed-map time budget. Failed sources now retry with exponential delays
from one to 60 seconds, retaining decoded chunks and returning unavailable.
A changed snapshot bypasses the delay, and healthy peers continue refreshing.
With the fleet stopped, a subsequent worker sample used about 1.5% of one CPU
instead of the earlier sustained full core. This is an idle observation, not a
worst-case build benchmark; budget-limited maps still return unavailable.

To repeat the bounded observation against the isolated API, use
`adapters/test/ros/onboard_exploration_observer.py --mission-id <UUID>` on the
workstation. It samples fleet and replica state and issues Stop All on exit.
Run `adapters/test/ros/simulation_evidence_recorder.py` in the simulation's ROS
domain before starting peers to retain first-keyframe timestamps and independent
ground truth. Its trajectory-cell overlap is a diagnostic heuristic, not a
sensor-coverage measurement.

For a bounded Return Home check, use
`python3 -m adapters.test.ros.onboard_return_home_observer` with an explicit
`--robot-id`, `--mission-id`, and `--authority-file`. Capture the authority as
plain JSON with `ros2 topic echo --once --field data --full-length`, removing
the trailing `---` separator. The fixture must be less than 30 seconds old.
Run the observer with the simulation image's Python and `websockets` dependency,
host networking, and a writable evidence directory; overriding the image's
entrypoint prevents it from starting another simulator. Reports are checkpointed
atomically and Stop All runs on exit. The observer checks full-path execution
against the corrected home projected into the server's display frame. Its
success evidence still requires review against independent ground truth and
stable map-authority transforms.

Review a completed Return Home report against the passive recorder with:

```bash
python3 -m adapters.test.ros.return_home_evidence \
  --observer-report robot_3-return-home.json \
  --truth-report truth.json --robot-id robot_3
```

The offline checker requires matching mission/home evidence, a real objective
route, initial truth before the home keyframe, departure before the command,
and advancing world-frame truth through arrival and the post-stop settling
window. Missing evidence is inconclusive; a server success flag alone cannot
pass. Exit codes are 0 for passed, 1 for failed, and 2 for inconclusive. Keep the
recorder running for at least one second beyond the observer's post-stop sample.

Additional local validation covers replica retention under concurrent publication,
indexed terrain support on stacked floors, authority reordering/freshness, and
frame-bound reconstruction delivery. The affected Python suite passed 291 tests
from a clean commit export; the subsequent Gaussian frame-ID checks passed in
the 45-test reconstruction suite. The native indexed-map DDS fixture passed
FREE/OCCUPIED/UNKNOWN queries and exact revision/source-stamp rejection.
UI replica/map tests, Svelte checking, and the production build passed. In an
isolated headless browser with a synthetic stored component, a six-second network
interruption retained the displayed map and its 1.15 m ceiling; reconnect loaded
the next pose revision without resetting the ceiling. Quality changes were
checked separately. These fixtures do not measure physical map accuracy or GPU
performance on deployed robots.

## Remaining acceptance work

The integration boundaries are implemented; the full rollout plan remains in
progress. In particular:

- Measure duplicate coverage and inter-robot alignment in the four-robot Bistro
  scenario after the deferred admission investigation. The onboard run above
  establishes four-robot execution but does not pass these accuracy gates.
- Exercise peers on separate hosts through partitions, rejoin, and optimizer
  loss. A frontend restart currently requires a fresh fleet mission and domain;
  transparent restart with preserved native graph state is not implemented.
- Validate each physical robot's calibrated capture, ARM image, and local
  controller. Moving-obstacle and blind-corner behavior needs controlled trials.
- Extend grid refinement to shared exploration objectives and feed failed
  corridors back into topological replanning. Current local detours require
  clear corridor vertices, and speed limits are not yet generated. The reusable
  topological stage handles Navigate and Home; Explore retains its selector.
- Establish a sensor coverage/bootstrap policy before enabling the strict indexed
  terrain gate. MGG still uses its local OctoMap for frontier construction and
  information gain; Inspect and Rendezvous remain unsupported objectives.
- Extend the passing native CUDA smoke to real captures and measure alignment,
  memory scaling, training time, and rendering cost. The three-view fixture
  completed nine optimizer iterations and converted 540 Gaussians; it does not
  establish reconstruction quality. Online incremental training remains later.
