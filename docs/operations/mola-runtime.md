# Native MOLA runtime

The `planning-refactor` pipeline gives MOLA persistent ownership of corrected
point geometry. Swarm-SLAM remains the pose-graph authority. MOLA neither runs a
second optimizer for these maps nor publishes another `map → odom` transform.

The framework and planner paths share the same C++ geometry runtime:

```mermaid
flowchart LR
  Store["Onboard coherent snapshot + immutable chunks"] --> Runtime["PersistentMolaRuntime"]
  Runtime --> Geometry["MOLA KeyframePointCloudMap"]
  Geometry --> Worker["Supervised worker: atomic product index"]
  Geometry --> Grid["Native occupied/free voxels + terrain samples"]
  Grid --> Worker
  Worker --> MGG["MGG planner reads native product"]
  Geometry --> Module["SwarmDeckMapSource: MOLA framework module"]
  Module --> Consumers["MOLA MapSource subscribers"]
```

The native planner product adds occupied/free voxels and terrain samples to the
point-geometry layer. MGG reads this immutable native grid directly for
graph/grid work, without an intermediate map-query process.
Unknown cells remain unknown. The independent raw-cloud/depth mapper is disabled. The robot
peer overlay remains hardware opt-in and keeps its planner-product defaults
disabled until capture provenance is qualified. The read-only fleet component
catalogue and aggregate display path are described in the [replica component
guide](replica-components.md).

## Worker deployment

Build `deploy/docker/Dockerfile.mapping` and use either the simulation
service. Both require the active `SWARMDECK_MISSION_ID`; old missions on the map
volume are not selected implicitly. Commands are in
[current stack operations](current-stack.md), with the historical direct Compose
invocations in the archived
[decentralized autonomy record](../archive/decentralized-autonomy.md).

Each peer launch (`deploy/autonomy/peer.launch.py`) first reclaims old mission
directories under the map store (`SWARMDECK_MAP_STORE`, default `/maps`). It
keeps the active mission and the `SWARMDECK_MISSION_GC_KEEP_RECENT` most
recently modified others (default 3) and deletes the rest. Set
`SWARMDECK_MISSION_GC_DRY_RUN=true` to only log what it would delete. The
launch then checkpoints the previous run's geometry SQLite WAL
(`PRAGMA wal_checkpoint(TRUNCATE)`), through the `geometry` link while it still
points at that run, before claiming the new map epoch moves the link to a new,
empty run directory. A first launch has no link and skips the checkpoint.

The default worker maintains one `swarmdeck-mola-import --serve` subprocess
per peer, started on the peer's first build and closed when the peer
disappears or its runtime fails, and builds up to `--parallel-peers` peers at
the same time (`SWARMDECK_MOLA_PARALLEL_PEERS`, default 4, minimum 1): one
build takes about 1.2 s per 60 keyframes and grows with the keyframe count
(benchbot, 2026-09-18), so building four robots in turn through one runtime
made each robot's product interval the sum of four builds, 2 to 6 s at 60
keyframes and heading for 20 s or more later in a mission.
It imports each component independently, then publishes a whole-peer,
self-described product: the component artifacts, then `mola/source.json` (the
exact `snapshot.json` bytes the generation was built from), then
`mola/index.json`, whose `source_sha256` and `source_snapshot_id` describe
`source.json`. A finished build is published even if the bridge replaced
`snapshot.json` during the build; the next poll builds the newer snapshot.
Readers (the indexed map server's provider and MGG's `MolaMap`) read
`index.json` and `source.json`, require the digest and snapshot identity to
agree, treat a disagreeing pair as mid-replacement and retry, and never read
`snapshot.json`. Unchanged component artifacts are reused after checking their
size and SHA-256. Append-only updates decode only new keyframes. Actual pose
corrections, removals and out-of-order observations rebuild from immutable source
chunks; metadata-only revisions reuse resident geometry. Published maps remain immutable.

The subprocess protocol has a versioned ready event, correlated requests,
explicit `replace` and `pose_only` modes, and checked revision/artifact responses.
Timeout, process death, and malformed output invalidate resident state. A later
request can rebuild from immutable chunks. Removing a component releases its
resident context.

| Limit | Default |
| --- | --- |
| Snapshot JSON | 64 MiB |
| JSONL request / response | 64 KiB each |
| Submaps / chunk references per component | 16,384 / 65,536 |
| Materialized metric points / planner surface extrema | 2,000,000 each |
| Runtime geometry and evidence units, including a replacement candidate | 8,000,000 |
| Resident component contexts (per peer runtime) | 256 |
| Standalone native serialized artifact | 1 GiB |
| Deployment native artifact / publication limit | 256 MiB |
| Deployment import deadline / poll / retry | 30 s / 1 s / 5 s |

These point limits account for maps owned by the runtime. Framework subscribers
can retain older immutable snapshots, so they must bound their own history.
The artifact byte limit is checked after serialization and before publication;
it is not a filesystem quota. The worker forwards its point, context and output
limits to the native process and checks the advertised effective limits. Worker
and native CLI resource flags can override their defaults.

`kMaxPointsPerMap` (`swarmdeck_mapping/point_budget.hpp`) defaults to
2,000,000. It bounds materialized metric points, planner surface extrema and a
single decoded source keyframe, not the sum of historical observations.
Metric geometry keeps a deterministic measured representative in each 5 cm
voxel. The planner separately consumes original endpoints and qualified sensor
rays, retaining exact minimum/maximum heights per occupied 20 cm voxel, timestamp
and clearing evidence, and observed-free voxels. It does not cast rays from the
compacted metric representatives.

Append-only updates retain this evidence and process new keyframes. Corrections,
removals and older observations rebuild it: all endpoints first, then a bounded
subset of original rays. Omitted rays establish no free space. The resident
budget includes metric points, occupied/free evidence records and surface
extrema; it is a record budget, not a byte-exact process memory limit.

The `SDMGRID1` binary layout now carries metadata schema
`swarmdeck.mola_planner_grid.v2`. `source_point_count` must equal the original
manifest chunk count; `point_count` counts materialized extrema and is bounded
independently. MGG validates both. Deploy the mapping worker/native importer and
MGG reader together; the old v1 reader deliberately rejects v2 products.

A component that outgrows the budget keeps its last product. The worker then
writes `<peer>/mola/worker.json` after every attempt (`version`,
`updated_at_ns`, `source_sha256` of the `snapshot.json` bytes it read, and
`error`, empty after a published build), the bridge reports that `error` in
`status.json` as `product_error` (null until a worker has reported) next to
`product_lag_revisions`, and the worker logs a failure when it first appears
or changes and once more when the peer publishes again, not on every retry.

## Visibility retirement of endpoints

The planner product forgets dynamic objects by visibility, not by age. A
stored endpoint is retired when at least `min_clearing_traversals` (default 3,
`PlannerGridLimits` in `swarmdeck_ros/src/swarmdeck_mapping`) qualified free
rays from captures observed later than every endpoint in its voxel pass
through that voxel, and no newer endpoint lands there. A ray passes through
the endpoints, rather than over them, only where it is sampled no higher than
the voxel's highest endpoint plus `clearing_height_tolerance_m` (0.05 m): a
road surface is a sheet near the bottom of its voxel, and rays from a lidar
above it that end far ahead cross the road's own ground voxels above that
sheet, proving nothing about it (see the known-issues row on the retired
lane). A retired endpoint leaves the occupied set and the terrain surface
samples, and its voxel becomes free. Rays observed before the endpoint prove
nothing and do not count; a single ray counts once per voxel; unqualified
captures neither carve nor retire. The `SDMGRID1` metadata carries
`retired_count`, and every reader requires
`surface_count + retired_count == point_count`; a grid without qualified rays
must report zero retirements. Neither `min_clearing_traversals` nor the
tolerance is recorded in the metadata.

## Planner product

Enable native planner products:

```bash
export SWARMDECK_MOLA_PLANNER_MAPS=true
```

The native builder reads corrected MOLA keyframe poses and exports a bounded
binary `SDMGRID1` product containing occupied voxels, observed-free voxels and
sorted surface-height samples. MGG reads the coherent product directly; the
worker owns publication and there is no separate query process.


The worker publishes each `.sdpg` planner product alongside its `.metricmap`
in one atomic index. Both outputs must succeed before native state commits.
The provider checks current snapshot/index identity, component revision, artifact
size and SHA-256, bounded record counts, and surface/voxel consistency before
publishing an immutable grid. It never reloads XYZ chunks. Unchanged components
can reuse an older artifact while the whole-peer snapshot changes; their nested
source identity records the generation that actually produced those bytes.
Native JSON digests are opaque identities, since C++ and Python JSON number
formatting differs. The worker's manifest digest and exact file hashes establish
the publication chain.

Free space requires explicit `RayEvidence`: `FIRST_RETURN`, `DESKEWED` or
`NOT_REQUIRED`, and `SINGLE_CAPTURE`. The store verifies the durable capture
record, source contract, clock domain, capture interval, capture-time transform,
calibration, estimator session, and the one associated sensor origin before
issuing this evidence. It is evidence about the original rays, not a property
that can be inferred from a provider name or a registered point cloud.

The simulation provider can qualify one-tick raw rays. The current
SuperOdometry and FAST-LIVO2 raw contracts attest first returns but not deskew;
FAST-LIVO2's registered cloud can attest deskew while losing the one-endpoint-
per-ray guarantee. Those hardware paths therefore remain occupied-only until
their capture contracts provide all three conditions. Old snapshots,
missing/partial evidence, and generic geometry replacements remain occupied-only.
Turning on MOLA does not make unqualified Bistro unknown cells traversable.

| Native planner limit | Default |
| --- | --- |
| Voxel resolution | 0.2 m |
| Points (the runtime's `max_points_per_map`) / combined occupied and free voxels | 2,000,000 / 2,000,000 |
| Selected ray steps / initial angular bins | 4,000,000 / 5° |
| Ray sample spacing | 0.75 × voxel resolution |
| Build deadline | 8 s |
| Artifact / metadata bytes | 256 MiB / 64 KiB |

Ray carving and export run only when requested. The builder keeps every occupied
endpoint and exact surface sample. It orders each qualified keyframe's measured
5-degree ray representatives by range, then admits them round-robin across
keyframes within the ray-step budget. This preserves near-field evidence from
old and new captures without turning cumulative, overlapping ray work into a
publication failure. Omitted rays remain unknown; they never become inferred
free space. Pose corrections reuse local point buffers but rebuild the planner
grid in its corrected frame. Geometry replacement and retraction remove previous
contributions. The deadlines and point/voxel limits bound work; they are not
measured worst-case latency promises.

Measured replay and correction timings are in the
[acceptance log](acceptance-log.md).

The point budget is still a hard publication limit. A map whose captures each
retain the configured maximum of 4,096 endpoints reaches 2,000,000 points at
489 captures (the 489th exceeds it), about eight minutes of driving at one
keyframe per second; turns and accepted keyframes can reach that count before
distance alone suggests it. The product spends 24 bytes per point (each is a
surface sample), so the budget is bounded by its readers rather than by the
build: on a 20-core workstation (RelWithDebInfo, 2026-09-19) the grid build
costs 2.3 ms per 4,096-point keyframe (0.71 s at 1,003,520 points, 1.50 s at
2,048,000, 4.70 s at 8,192,000, against the 8 s build deadline; benchbot's
four-core allowance is about twice slower), while the Python reader decodes
2,000,000 samples in 2.5 s and MGG loads the file within a 2 s deadline.
Long-duration qualification must measure this capacity boundary rather than
treating bounded ray selection as unbounded map growth.

## MOLA framework module

`swarmdeck_mola` builds `libmola_swarmdeck.so`, containing the registered
`swarmdeck_mola::SwarmDeckMapSource` module. It implements MOLA's `ExecutableBase`
and `MapSourceBase`, allowing launcher discovery and normal framework service
lookup rather than requiring a standalone import process.

Set `MOLA_MODULES_LIB_PATH` to the installed `swarmdeck_mola/lib` directory. The
mapping image sets this automatically. The installed
`share/swarmdeck_mola/config/external-map.yaml` provides a launcher configuration.
Its parameters are:

- `snapshot_file`: the coherent onboard snapshot.
- `component_id`: component to select from a whole-peer snapshot; omit only when
  the file contains exactly one manifest.
- `chunks_dir`: directory containing its immutable SHA-256-addressed chunks.
- `poll_s`: source polling interval, default 1 second.
- `max_points`: component point budget, default 2,000,000.
- `artifact_file`: optional atomic serialized output; omit for in-memory use.

The module publishes one named map layer with canonical manifest metadata,
source identity and availability. It suppresses unchanged revisions, checks the
source again before publication, and retracts the layer on invalid input or
shutdown. A valid source restores it. Repeated failures use bounded retry
backoff; a changed source can retry immediately. Late subscribers receive the
current layer rather than a history of old maps.

The module can read the same whole-peer file as the worker, with one module
instance per selected component. An unrelated component change does not reload
its geometry. A missing or ambiguous selection retracts the layer. The module
stays pinned when `component_id` is configured. An unselected module follows
changes to the sole component in a coherent snapshot, including component
reassignment after a merge. It releases the previous context after publication
and rejects ambiguous selection. The upstream graph/store remains responsible
for admitting the merge and its transforms.

## Validation

The Docker build executes native bridge, chunk-validation and persistent-runtime
tests, loads the actual module through the MOLA launcher factory, and runs a
Python-store fixture through both the JSONL and framework paths. For pose-only
checks, the chunk directory is removed temporarily: success therefore requires
resident geometry reuse. Tests also cover stale/conflicting revisions, immutable
held maps, geometry retraction, artifact failure, callback inspection, source
invalidation and recovery.

Worker tests exercise child death, timeout, malformed/oversized replies,
file-descriptor cleanup, cache recovery, artifact verification, mission
selection and atomic index publication. See
`autonomy/tests/test_mapping_worker.py` and the reusable scripts in
`tests/deployment/`.

The build also runs the actual `mola-cli` scheduler with the installed YAML
configuration and environment-variable expansion, verifies a newly created
artifact, and shuts the launcher down. The separate remote acceptance script
verifies a whole worker generation and its correction with chunk payloads
removed, while checking that exactly one native subprocess remains.

```bash
IMAGE=swarmdeck-mapping:mola-runtime-review \
  bash tests/deployment/mola_remote_acceptance.sh
```

Set `REMOTE` for a different workstation. To build first, set `SOURCE` to an
explicit clean source export on that workstation. All acceptance containers use
temporary map data and have networking disabled.

### Reproduce the correction benchmark

Run it with the repository on `PYTHONPATH` and the native importer's installed
path:

```bash
PYTHONPATH=. python3 tests/deployment/mola_benchmark.py \
  --binary /mapping_ws/install/swarmdeck_mapping/bin/swarmdeck-mola-import
```

Measured correction latencies, memory samples and per-robot publication timings
are in the [acceptance log](acceptance-log.md).

### Planner-product validation

The native planner and worker pass the captured-point acceptance fixture across
four publications. It checks measured free space before a wall, occupied
endpoints, unknown space beyond, missing/partial provenance, steps, drops,
stacked floors, translated/yaw-corrected rays, geometry replacement and
retraction. The image build runs the native worker checks automatically.

The native worker writes MOLA products under each peer root. Each MGG planner
reads exactly one peer root,
`${SWARMDECK_MAPS_ROOT}/${SWARMDECK_MISSION_ID}/${ROBOT_ID}`. The mission must
be a canonical UUID, the robot ID must match the launcher's simple identifier
grammar, and the maps root must be absolute. MGG reads the coherent native
product directly and waits for a complete publication when the worker is
between generations.

The launch sets `map.resolution=0.20` for this backend, overriding the legacy
Bistro cloud-map value of 0.15 m. The loader checks that the SDMGRID metadata
also says 0.20 m, so a product at another resolution is unavailable. In this
mode the input relay still publishes transformed odometry and keeps TF alive,
but `cloud_enabled=false` disables raw cloud and depth subscriptions and the
cloud publication timer. This prevents duplicate sensor-cloud ingestion while
MGG reads the immutable peer snapshot.

### Asynchronous publication contract

`MolaMap` queues the newest authority request on a worker thread. Planner
callbacks never decode the snapshot or grid. The worker reads
`mola/index.json`, `mola/source.json`, and the referenced `SDMGRID1` artifact
with stable-read checks, verifies the mission/component/revision/geometry/source
stamp identity and SHA-256 chain (`sha256(source.json)` must equal the index's
`source_sha256`; a disagreeing pair is mid-replacement and is retried), then
atomically publishes an immutable tree. It never reads the bridge's
`snapshot.json`.
A changed component, epoch or authority transform requires a matching product
publication. Until that product is decoded, MGG reports planning unavailable;
an older product never satisfies a newer identity.

The product is ternary. Occupied voxels and qualified observed-free voxels are
stored explicitly; unknown space is absent. Occupied wins if an observation
would overlap free space. A missing, stale, invalid, or not-yet-loaded product
reports `unknown`, and explicit strict box/path checks stop at unknown, so
unknown space cannot be treated as traversable. Free voxels are emitted only
when the source carries the required first-return, deskew/not-required and
single-capture provenance. Unqualified captures therefore produce an occupied-
only product.

The native product bounds work and input size. Malformed or over-limit metadata
is rejected, and a worker publication is accepted only after its artifact
identity and source snapshot are coherent.

MGG applies each platform's terrain limits while planning directly from the
native product. Unknown space or an unsupported step is never treated as free
terrain.

The native test target also builds `mola_map_probe`, which reports
`free`, `occupied`, or `unknown` for bounded sample points. Fixture generation
and the native probe/build/motion gates are separate checks; this guide does
not treat an unrun gate as passed.

With a test-enabled MGG image (`BUILD_TESTING=ON`, including its build tree), run
the cross-image fixture on the machine holding both images:

```bash
MAPPING_IMAGE=swarmdeck-mapping:navigation-map-review \
MGG_IMAGE=swarmdeck-mgg:mola-graph-review \
  bash tests/deployment/mola_mgg_acceptance.sh
```

The script prints image identities and checks initial, reused and corrected
publications. It uses temporary synthetic maps and containers with networking
disabled. It does not restart a deployment or establish motion acceptance.
