# Fleet replica components

The replica component view assembles accepted per-robot map snapshots for the
ordinary Layers panel and the Replica Inspector. Automatic Live Map selections
can add fresh robot telemetry and send frame-qualified navigation objectives.
Explicit historical inspection remains read-only. Assembly never registers maps
or estimates transforms: those remain onboard mapping/Swarm-SLAM responsibilities.

## Compatibility and assembly

The server groups replica envelopes by `(session_id, component_id)`. A component
is ready only when its publishers agree on the frame and, for more than one
source, the same accepted Swarm-SLAM solution order. A single publisher is
available before a merge closes; separate components remain separate. The
server never joins sessions or guesses a transform between disconnected
components.

For a ready component, assembly validates snapshot and chunk identities,
manifest revisions, historical tombstones, and source ownership. A publisher's
current submap publication supersedes a relayed copy, including an explicit
retraction or move to another component. Conflicting active geometry is a
conflict rather than a best-effort merge. The resulting selected component keeps
its true `frame_id` and submap transforms. Its aggregate publication has a
deterministic `snapshot_id`; `revision` is `null` because there is no authoritative
fleet counter, while each source retains its own `revision`.

Catalogue `available` means that this metadata and geometry publication is
coherent. It does not mean that every point will be rendered: the browser keeps
the existing 300,000-point tactical budget and may sample immutable chunks.

## Incremental transport and raster storage

Robot-to-server replication bootstraps with a full v1 envelope, then sends v2
revision-based deltas: changed component metadata, inserted/replaced/deleted
submaps, and actual pose changes. Revision-only pose stamps do not resend every
submap. Immutable XYZ chunks remain content-addressed and transfer only when
missing. A missing or stale delta base returns `409` with `resync: true`; the
publisher recovers with a full envelope. The server checks the base again at
commit and exposes only a fully reconstructed, chunk-complete publication.

The server raster retains aggregate height histograms and swept-free cells.
Appends integrate only new submaps; corrections and removals rebuild the scope.
Historical raw-point count is no longer the raster admission limit. Heights
start in 1 mm bins and coarsen by powers of two only when a cell exceeds 512
populated bins, preserving counts and all height bands. Represented cells and
histogram bins remain bounded. A large cold rebuild can still be expensive.

Local 2D maps use `robot:<selected robot>` and contain only that robot's owned
submaps, even in a merged component. Missing local scopes remain pending; the UI
never substitutes a global map and clears the prior canvas when changing scope.
Browser catalogue responses and changed raster PNGs are still full products;
this is not a raster-tile delta protocol.

## HTTP interface

List components for all sessions:

```text
GET /api/autonomy/replicas/components
```

Limit the list to one mission with `?session_id=<mission UUID>`. The response is
versioned as `{version: 1, components: [...]}`. Each entry contains
`session_id`, `component_id`, `frame_id`, `robot_ids`, `source_count`,
`submap_count`, `point_count`, `available`, `status`, `detail`, numeric
`solution_order: [clock, optimizer] | null`, and source records with each
robot's revision and snapshot ID. `status` is `ready`, `syncing`, or `conflict`.
Unready entries remain visible so convergence and data conflicts are legible;
they are disabled in the selector.

Fetch one aggregate component view:

```text
GET /api/autonomy/replicas/components/view/<mission UUID>?component_id=<ID>
```

The response uses the existing replica view shape and adds `scope: "fleet"` and
`robot_id: "fleet"`. The selected component carries the true frame and
`graph_revision: null`; the view carries `revision: null`, deterministic
`snapshot_id`, and per-source revisions in `sources`. A `409` means the source
is still converging or conflicting. The UI keeps the last coherent display in
that case and does not draw a new overlay.

Views also expose `solution_order_known`. Current publications set it to true;
an older publication without a solution order remains available for read-only
inspection with the flag false, but cannot qualify live overlays or goals.

## Map behavior

The catalogue advertises `active_session_id` when the server has an explicit
`SWARMDECK_MISSION_ID`; the onboard-planning overlay supplies it. This identifies
the live mission without guessing from UUID order or independent robot revisions.
Without that configuration, the existing central-map default remains available.

The Layers panel's **Map source** selector also lists explicit catalogue entries
labeled with a short mission ID, component ID, and robot sources. Historical
missions remain available for deliberate inspection. Disconnected components
are never combined using assumed transforms.

Local filters submaps by the selected robot's ownership, including when that
robot's replica also carries relayed peer geometry. Global selects a verified
component shared by multiple robots. Before an
inter-robot closure establishes that frame, it reports waiting for alignment;
select Local to inspect a robot's accumulated map. It never substitutes the
selected robot's map under the Global label.

Fleet selections use a cache and frame key containing scope, mission, component,
and frame. Per-robot selections retain their graph epoch behavior. A verified
frame change resets the viewport; a failed or stale request leaves the previous
coherent geometry visible. Automatic live views add robot icons, goals and plans
from separately qualified telemetry. Explicit catalogue/history views remain
read-only. Legacy world-frame costmaps and histories cannot be overlaid on a
component without their own qualified transform.

### Live telemetry and goals

```text
GET /api/autonomy/replicas/components/live/<mission UUID>?component_id=<ID>
POST /api/autonomy/replicas/components/live/<mission UUID>/goal
```

GET returns mission/component/frame identity, `solution_order`, and bounded
per-robot navigation poses, goals, paths, and `T_component_navigation`. The adapter reuses its current
mapping-authority subscription and serializes state before the server's legacy
world conversion. Pose is planar (`z=0` unless supplied by the adapter); the
transform is full SE(3). Freshness combines the adapter's authority age and the
server's monotonic receipt age, with a three-second budget. ROS timestamps are
never compared with wall time. Missing or stale authority suppresses the overlay
without discarding cached map geometry. Ordinary geometry revision increments
do not hide robot icons.

POST accepts `{robot_id, component_id, solution_order, goal: {x, y, z, yaw}}`
in the displayed component frame. The server requires the displayed solution
order, fresh matching authority, and onboard
Navigate capability, applies the inverse rigid transform once, and dispatches
an objective containing the frame token, navigation-frame projection, and
immutable `component_goal`. The robot rejects a token that changed before
initial admission, then re-resolves the accepted anchor against current
authority during execution and correction recovery. Historical missions, stale membership, and
unavailable command links are rejected. A successful HTTP response acknowledges
dispatch; the onboard planner/controller still determine feasibility and arrival.
Navigate/Home require a complete path to the requested XY before controller
submission. Partial responses are rejected. Fleet telemetry exposes
`objective_continuation` with phase `planning` or `following_final`, plus the
`mgg_native`/`mola_indexed` evidence source. Planning also covers replacement of a
full route after a material map correction; reaching a short proxy no longer
starts another planner request. Long display paths retain both endpoints while
sampling to at most 200 points. Controller paths remain complete.

The aggregate route is deliberately three segments deep so it cannot be
captured by the legacy `/api/autonomy/replicas/{robot_id}/{session_id}` route.
The implementation does not reread or rebuild XYZ chunks in the provider; it
uses committed peer metadata and the browser's existing chunk cache. Catalogue
assembly is cached by each publisher's revision; it does not reload point data.
Reads are bounded to 128 sources and 64 MiB of metadata; select a mission when
the all-mission catalogue exceeds that budget.

## Qualification boundaries

The SLAM panel uses live peer diagnostics when onboard peers are active. It
shows keyframe counts, verified reports and current shared-component membership
reports are grouped by mission and unordered robot pair, taking the larger
count from the two endpoints to avoid counting their mirrored reports twice.
They are diagnostic outcomes, not a count of unique optimized graph edges.
Only an accepted optimizer result can establish a shared component.

Robot markers retain their last qualified placement for up to three seconds
during transient telemetry gaps. Mission/component/frame changes clear that
placement immediately; stale telemetry cannot authorize goal input.

This view proves coherent publication and frame identity. It does not qualify
free-space carving, planner safety, MGG exploration behavior, Home, hardware
calibration, or Gaussian reconstruction. Those gates remain in the
[plan](../plan.md)
and [MOLA runtime guide](mola-runtime.md).

## Acceptance command

```bash
python3 tests/deployment/fleet_replica_acceptance.py \
  --base-url http://localhost:18081 --session-id <mission-uuid> --deadline 60
```

This read-only observer waits for four committed sources, then checks aggregate
views and immutable point chunks. Separate components are valid; it does not
create closures or send motion commands. Use `--expected-robots` for another
fleet size. The [acceptance log](acceptance-log.md) contains the
executed trials and current limitations.
