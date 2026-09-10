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
  Worker --> Provider["MolaDirectorySource"]
  Provider --> Query["Shared terrain query → MGG"]
  Geometry --> Module["SwarmDeckMapSource: MOLA framework module"]
  Module --> Consumers["MOLA MapSource subscribers"]
```

The optional native planner product adds occupied/free voxels and terrain samples
to the point-geometry layer. Both the MOLA provider and the existing indexed
provider use the same terrain-query implementation. MGG still uses its OctoMap
for exploration; replacing that map and qualifying real capture provenance remain
work in the [integration plan](../architecture/planning-refactor-remaining.md).

## Worker deployment

Build `deploy/docker/Dockerfile.mapping` and use either the simulation
`docker-compose.mapping.yml` overlay or the robot-local `peer_mola_mapping`
service. Both require the active `SWARMDECK_MISSION_ID`; old missions on the map
volume are not selected implicitly. Commands are in the
[integration guide](decentralized-autonomy.md).

The default worker maintains one `swarmdeck-mola-import --serve` subprocess.
It imports each component independently, then atomically publishes a whole-peer
`mola/index.json` only when the source still matches. Unchanged component
artifacts are reused after checking their size and SHA-256. Pose-only revisions
reuse resident keyframe geometry; replacement and retraction build a coherent
new map. Published map objects remain immutable.

The subprocess protocol has a versioned ready event, correlated requests,
explicit `replace` and `pose_only` modes, and checked revision/artifact responses.
Timeout, process death, malformed output, and source races invalidate resident
state. A later request can rebuild from immutable chunks. There is no silent
fallback to the legacy importer; `--mode oneshot` selects that compatibility
path explicitly. Removing a component releases its resident context.

| Limit | Default |
| --- | --- |
| Snapshot JSON | 4 MiB |
| JSONL request / response | 64 KiB each |
| Submaps / chunks per component | 4,096 / 16,384 |
| Points per component | 2,000,000 |
| Runtime points, including a replacement candidate | 8,000,000 |
| Resident component contexts | 256 |
| Standalone native serialized artifact | 1 GiB |
| Deployment native artifact / publication limit | 256 MiB |
| Deployment import deadline / poll / retry | 30 s / 1 s / 5 s |

These point limits account for maps owned by the runtime. Framework subscribers
can retain older immutable snapshots, so they must bound their own history.
The artifact byte limit is checked after serialization and before publication;
it is not a filesystem quota. The worker forwards its point, context and output
limits to the native process and checks the advertised effective limits. Worker
and native CLI resource flags can override their defaults.

## Planner map provider

Enable native planner products and select their query provider together:

```bash
export SWARMDECK_MOLA_PLANNER_MAPS=true
export SWARMDECK_PLANNER_MAP_PROVIDER=mola
```

Both the simulation mapping overlay and robot peer overlay pass these settings.
Recreate their mapping worker and query services after changing them. Defaults
remain `false` and `indexed`; selecting `mola` without a valid product returns
unavailable, with no fallback. The standalone options are worker `--planner-maps`
and query server `--map-provider mola`.

The native builder reads corrected MOLA keyframe poses and the same immutable
MRPT point buffers held by the map. It exports a bounded binary `SDMGRID1` file
containing occupied voxels, observed-free voxels, and sorted surface-height
samples. Unknown space is absent from both voxel sets; occupied wins when
observations overlap. The product preserves multiple surfaces per column.
`IndexedMapView` supplies the existing ground, slope, roughness, step, drop,
clearance, freshness and query-budget behavior.

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

Free space requires explicit `RayEvidence`: first-return endpoints, a deskewed
capture, and a single associated sensor origin. The store verifies the durable
capture record, timestamp and calibrated origin before issuing this evidence.
Old snapshots, missing/partial evidence, and generic geometry replacements
remain occupied-only. Existing Bistro SLAM captures lack this qualification;
turning on MOLA does not make their unknown cells traversable. Qualifying the
capture providers is the next dependency for motion use.

| Native planner limit | Default |
| --- | --- |
| Voxel resolution | 0.2 m |
| Points / combined occupied and free voxels | 1,000,000 / 2,000,000 |
| Ray steps / angular bins | 4,000,000 / 5° |
| Ray sample spacing | 0.75 × voxel resolution |
| Build deadline | 8 s |
| Artifact / metadata bytes | 256 MiB / 64 KiB |

Ray carving and export run only when requested. Pose corrections reuse local
point buffers but rebuild the planner grid in its corrected frame. Geometry
replacement and retraction remove previous contributions. The deadlines and
point/voxel limits bound work; they are not measured worst-case latency promises.

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

### Workstation measurements, September 10, 2026

On `benchbot.yannbouteiller.com`, a Release build using MOLA 2.9.0 processed
20 pose corrections of a deterministic 100,000-point cloud in one process.
Including serialized output, median correction latency was **30.0 ms**
(maximum 35.6 ms), compared with **163.8 ms** for five full one-shot imports.
Native RSS ranged from a first sample of 71.1 MiB to a final 65.0 MiB; the
maximum sampled RSS was 71.1 MiB. This fixture demonstrates reuse and startup
cost savings, not a worst-case latency or long-duration memory guarantee.

Reproduce the benchmark with the repository on `PYTHONPATH` and the native
importer's installed path:

```bash
PYTHONPATH=. python3 tests/deployment/mola_benchmark.py \
  --binary /mapping_ws/install/swarmdeck_mapping/bin/swarmdeck-mola-import
```

A read-only copy of the stopped Bistro fleet's four peer maps also imported
successfully through the persistent worker: 991,344 points in 0.83 seconds,
with four coherent artifact indexes. Individual maps held 214,959, 28,946,
71,213 and 676,226 points. This validates the real map-store format; it does
not resolve R1's limited exploration or qualify moving-robot terrain planning.

The tested image is `swarmdeck-mapping:mola-runtime-review`, image ID
`sha256:9868a7a0a4e74c85f25c22ae756c5123032013fece4f27a1c514ebfc5fb1d337`.
It is running as `planning-mapping-1` in the isolated workstation project, on
mission `92028a23-d1d1-45b8-8d0b-509815e35950`. All four live artifact indexes
match their source snapshot SHA-256 and one native process serves them. The
simulator, MGG, peer SLAM and production deployment were not restarted.

### Planner-product validation

The native planner and Python provider pass the captured-point acceptance fixture
in 0.24 seconds across four publications. It checks measured free space before a
wall, occupied endpoints, unknown space beyond, missing/partial provenance,
steps, drops, stacked floors, translated/yaw-corrected rays, geometry replacement
and retraction. The local autonomy suite passes 121 tests.

A fresh replay of the same copied Bistro maps, now requesting both native products,
completed in **2.38 seconds** including first provider loads, five warm refreshes
per component and query batches. All four unqualified maps retained zero free
voxels. Native RSS reached a maximum sampled **174.2 MiB**; this excludes Python
provider memory and is not a continuous peak measurement.

| Robot | Points | Worker publication | First provider load | Median warm refresh | Query, 8 samples |
| --- | ---: | ---: | ---: | ---: | ---: |
| `robot_0` | 214,959 | 295 ms | 233 ms | 8.4 ms | 1.1 ms |
| `robot_1` | 28,946 | 35 ms | 58 ms | 1.6 ms | 0.8 ms |
| `robot_2` | 71,213 | 88 ms | 97 ms | 3.2 ms | 1.1 ms |
| `robot_3` | 676,226 | 810 ms | 562 ms | 23.7 ms | 0.9 ms |

Warm refreshes reuse immutable decoded grids but still verify artifact hashes and
coherent source/index bytes. These measurements do not justify dropping those
checks. Queries read the published grid without filesystem access. The replay
uses stopped maps with unqualified rays, so it does not measure heavy free-space
carving or motion safety.

The image build runs the synthetic gate automatically. It also starts the actual
ROS query server and calls MGG's generated `QueryMapBatch` service, checking
FREE/OCCUPIED/UNKNOWN, stale revisions and unavailable results after index
corruption. This test uses loopback in ROS domain 197 with networking disabled. To replay another dataset,
first copy its map root to a disposable writable directory, then run:

```bash
PYTHONPATH=. timeout 90s python3 tests/deployment/mola_planner_acceptance.py \
  --binary /mapping_ws/install/swarmdeck_mapping/bin/swarmdeck-mola-import \
  --maps-root /tmp/copied-peer-maps \
  --mission-id <mission-uuid>
```

Replay writes MOLA products into that copy. It does not run ROS or send robot
commands. Omit the last two options for the synthetic acceptance fixture.

The planner image is `swarmdeck-mapping:mola-planner-review`, image ID
`sha256:fa4f63725a37514d372ce4877d1b3a87de022f3fa7335c2eaee4353a932de16f`.
It passed all image acceptance gates on the workstation. The isolated running
mapping worker and query service retain their previous images; the new planner
backend has not been enabled for robot motion.
