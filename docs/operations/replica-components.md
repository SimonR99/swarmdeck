# Fleet replica components

The replica component view is a read-only inspection path over accepted
per-robot map snapshots. It lets the ordinary map Layers panel select a coherent
fleet component while keeping the existing per-robot Replica Inspector. It does
not register a map, estimate a transform, publish a goal, or enable planning.

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

## Map behavior

The Layers panel's **Map source** selector lists Live map and explicit catalogue
entries labeled with a short mission ID, component ID, and robot sources. It
loads all sessions by default; selecting a mission is always explicit. A current
per-robot inspector selection is shown separately, and Live map clears tactical
selection.

Fleet selections use a cache and frame key containing scope, mission, component,
and frame. Per-robot selections retain their graph epoch behavior. A verified
frame change resets the viewport; a failed or stale request leaves the previous
coherent geometry visible. Tactical fleet views are read-only: navigation,
network, sensor, plan, and costmap overlays remain disabled while selected.

The aggregate route is deliberately three segments deep so it cannot be
captured by the legacy `/api/autonomy/replicas/{robot_id}/{session_id}` route.
The implementation does not reread or rebuild XYZ chunks in the provider; it
uses committed peer metadata and the browser's existing chunk cache. Catalogue
assembly is cached by each publisher's revision; it does not reload point data.
Reads are bounded to 128 sources and 32 MiB of metadata; select a mission when
the all-mission catalogue exceeds that budget.

## Qualification boundaries

This view proves coherent publication and frame identity. It does not qualify
free-space carving, planner safety, MGG exploration behavior, Home, hardware
calibration, or Gaussian reconstruction. Those gates remain in the
[planning-refactor integration plan](../architecture/planning-refactor-remaining.md)
and [MOLA runtime guide](mola-runtime.md).

## Acceptance command

```bash
python3 tests/deployment/fleet_replica_acceptance.py \
  --base-url http://localhost:18081 --session-id <mission-uuid> --deadline 60
```

This read-only observer waits for four committed sources, then checks aggregate
views and immutable point chunks. Separate components are valid; it does not
create closures or send motion commands. Use `--expected-robots` for another
fleet size. The [benchbot record](planning-parallel-acceptance.md) contains the
executed trial and current limitations.
