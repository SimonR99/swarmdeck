# Mapping, exploration, and display acceptance

This record covers the reviewed `planning-refactor` changes based on `297508f`,
committed together with this document. Sol and Luna implemented capture,
exploration, server, and UI tasks in parallel. Gemini 3.8 Flash, invoked through
`agy`, reviewed map identity and display semantics. Integration review corrected
an integer/string solution-order mismatch, manual-goal ownership races, legacy
tombstone handling, and replica publication validation before inclusion.

## Deployment

The September 10, 2026 trial used benchbot's RTX 3080 and ARGoS Bistro in the
isolated Compose project `planning-next`. Production and the older `planning`
project were untouched. The source export contains the reviewed changes and
excludes unrelated local ROS 1, route-tracking, and hardware deployment work.

- Mission: `c1af7f0a-8014-4558-b4c2-33ebfe86418a`; ROS domain: `201`.
- Test UI/API ports: `15174` / `18081`; SLAM: `18091`; media: `8192`.
- Compose overlays: base, GPU, MGG, planning-test, peers, mapping, onboard-planning.
- `SWARMDECK_CAPTURE_PROVIDER=simulation` admits proven instantaneous raw rays.
`SWARMDECK_MOLA_PLANNER_MAPS=true` enables native MOLA planner products, which
MGG reads directly. This run does **not** qualify MOLA as the full exploration
map replacement.
- Mapping image: `swarmdeck-mapping:parallel-review`; MGG runtime:
  `swarmdeck-mgg:diagnostics-10`; simulator and peer images use `:planning`.

Final mapping image ID:
`sha256:12801adebb1572870311f6ef76844c343adf3aa8622a8256a863141932b34c87`.
Live application and peer Python sources are mounted from the isolated reviewed
export.

## Executed checks

| Check | Result |
| --- | --- |
| Combined autonomy, exploration, objective, onboard-map, simulator and replica Python regression | 372 passed |
| Svelte/TypeScript check, replica loader/catalogue tests, production UI build | Passed; existing bundle-size warning remains |
| Native mapping/framework CTests | All five passed |
| Native importer, persistent JSONL, module loading, actual MOLA launcher | Passed |
| Actual isolated ROS sensor-to-peer capture join | Capture-time TF, original geometry/provenance and coordination relay smoke passed |
| UI HTTP contract against real ARGoS replicas | Passed without the fixture's skip condition |
| Chromium Layers/component rendering | Four components selectable; real geometry rendered, then capped at 60,000 points in Low power |
| Browser replica-fetch outage | Previous component remained visible; user ceiling remained 1.20 m |

The browser used local headless software rendering to check behavior; its frame
rate is not a low-end GPU performance benchmark.

## Live mapping measurements

All four robots produced qualified captures and nonempty native planner grids.
One live sample had the following counts; robots were moving, so these are
observations rather than a fixed replay benchmark.

| Robot | Qualified captures | Stored points | Observed-free voxels |
| --- | ---: | ---: | ---: |
| R0 | 2 | 8,192 | 45,120 |
| R1 | 2 | 8,192 | 43,985 |
| R2 | 4 | 16,384 | 59,755 |
| R3 | 3 | 12,288 | 65,550 |

The source retains at most 4,096 original endpoints per capture. It does not
replace them with voxel centroids before issuing free-space evidence. Missing
or mismatched provenance remains occupied-only. Hardware profiles have not
passed equivalent recorded-data qualification.

`fleet_replica_acceptance.py`, run on benchbot's loopback API during motion,
validated four separate components, 32 submaps, and 131,072 points in 179 ms.
Catalogue latency was 24 ms; component-view requests took 6–10 ms. Each unique
chunk was fetched and checked against its SHA-256, byte count, and point header.
The same test through the SSH tunnel took longer, as expected from network
latency. These are individual samples, not percentile or maximum guarantees.

After the trial, one Docker resource sample measured 88 MiB for the mapping
worker, 284 MiB for the query service, and approximately 602–625 MiB per complete
Swarm-SLAM peer container. Sampling included the native subprocess and other
container work; it does not establish worst-case resource bounds.

## Exploration outcome and remaining work

All four robots accepted Explore and became active within the first ten-second
sample. R0, R2, and R3 continued through successive paths. R1 became `blocked`
after its existing grid planner rejected every candidate: three consecutive
requests produced zero vertices and empty paths with collision/terrain rejection.
The fleet remained `incomplete`; it did not falsely report exploration complete.
Stop All left all four robots in stopped exploration, idle navigation, and estop.

The new controller-failure recovery is covered by deterministic tests for retry
limits, rejected services, deadlines, late paths, cancellation, and a manual goal
arriving during recovery. This live trial did not exercise every injected failure
case, nor establish successful Navigate/Home or dynamic-obstacle behavior.

No inter-robot closure occurred in this short trial. Real separate components
were displayed separately; compatible multi-publisher assembly, differing local
epochs, relay duplicates, retractions, and conflicts were exercised with coherent
contract fixtures. Multi-host closure accuracy and partition/rejoin remain gates.

Next work is the MGG exploration-map migration, blocked-route recovery and gain
computation costs, then shared Navigate/Home and multi-host acceptance. Metadata-
only accepted solver updates also merit profiling: causal publication advances
are required, but unchanged geometry should avoid unnecessary native grid work.
Colorized persistent submaps, streaming LOD, hardware capture qualification, and
real-capture Gaussian reconstruction remain separate work in the
[integration plan](../architecture/planning-refactor-remaining.md).
