# SwarmDeck MOLA map bridge

This package is the native map-product boundary. Swarm-SLAM remains the pose
authority. `MolaSubmapBridge` inserts immutable local XYZ clouds into MOLA's
`mola::KeyframePointCloudMap`, applies external graph corrections through
`setKeyframePose()`, and advertises the result through `mola::MapSourceBase` for
MOLA visualization and ROS 2 bridge consumers.

Geometry replacement and retraction use `replaceGeometrySnapshot()`. It builds a
fresh keyframe map from one coherent `autonomy.mapping` snapshot, so an old cloud
cannot remain as an additive obstacle. Rigid corrections use
`applyPoseSolution()` and do not copy or reinsert point data.

## Dependency contract

The bridge requires MOLA 2.9.0 or newer, currently targeted at the public API in
`mola_kernel` and `mola_metric_maps` 2.9.0, plus MRPT 2.15 or newer and
`mp2p_icp_map` (a transitive `mola_metric_maps` dependency). The key API is not
available in older MOLA releases: `KeyframePointCloudMap` must provide
`lastInsertedKeyFrameID()` and `setKeyframePose()`.

Build it in the same colcon workspace as the pinned MOLA source checkout:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to swarmdeck_mapping \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

The package itself does not import ROS and can be linked into a MOLA executable
or a thin ROS node. MOLA and this linked bridge are GPL-3.0; keep that license
boundary in mind when assembling runtime images. The Python contracts and store
remain independently usable without MOLA, MRPT, or ROS installed.

The phase-0 Docker build has compiled this bridge against the ROS Jazzy MOLA
2.9.0 packages. Its CTest executable performed insertion, copy-on-write pose
correction, exact replay, invalid-batch rollback, retraction, and stale/conflict
rejection. The Python fixture then crossed the native boundary with five points
and produced a nonempty serialized metric map. Arm64 remains a deployment gate.

`swarmdeck-mola-import SNAPSHOT_JSON CHUNKS_DIR OUTPUT.metricmap` validates one
component from the canonical Python snapshot, checks every chunk digest, length,
point count, and finite XYZ value, constructs the MOLA keyframe map, observes the
`MapSourceBase` publication, and serializes an MRPT metric-map artifact. Inputs
are bounded to 4 MiB manifests and 8 MiB per geometry chunk.

`deploy/autonomy/mola_worker.py` is the continuous onboard consumer. It watches
`/maps/<mission>/<robot>/snapshot.json`, invokes the importer once per component,
and, after all generation-specific products finish, publishes
`/maps/<mission>/<robot>/mola/source.json` (the exact snapshot bytes it built
from) followed by `/maps/<mission>/<robot>/mola/index.json`, whose
`source_sha256` names those bytes. Readers verify that pair and never read
`snapshot.json`, which may already be ahead of the product. In addition to the
MOLA metric map, an
optional `planner_output_path` produces a deterministic `SDMGRID1` sparse grid
from the resident keyframe map. The grid carries corrected occupied endpoints,
observed-free voxels, exact surface-height samples, and the complete source and
graph identities. Product construction is bounded by point, voxel, ray-step,
elapsed-time, metadata and artifact-byte limits.

Unknown is explicit: a voxel absent from both occupied and free sets is unknown,
and occupied always wins. Sensor origins alone do not prove free space. Rays are
carved only when the manifest declares `first_return`, `deskewed`,
`single_capture` evidence with exactly one origin. Missing, legacy, ambiguous or
otherwise unqualified evidence produces occupied endpoints and surface samples
without any free-space claims.

Endpoints are retired by visibility, never by age. A dynamic body captured in
front of the sensor (a peer robot at a grouped start, for example) leaves an
occupied voxel and terrain surface samples that obstacle expiry alone cannot
disprove. The builder drops both once at least `min_clearing_traversals`
qualified free rays, each observed strictly later than every endpoint in that
voxel, have passed through it; one ray counts once per voxel however many steps
it spends there. An endpoint observed after those rays re-confirms the voxel and
keeps everything in it, including the older endpoints. A retired voxel joins the
free set because the same rays carved it; a voxel no ray ever crossed stays
exactly as it was. `min_clearing_traversals` defaults to 3 and lives in
`PlannerGridLimits` (`include/swarmdeck_mapping/planner_map.hpp`).

`SDMGRID1` therefore carries `retired_count` beside `surface_count`, and
`surface_count + retired_count == point_count` always holds, where `point_count`
remains the manifest's stored point count that every reader cross-checks against
the geometry chunks.

Run it directly in the mapping image:

```bash
swarmdeck-mola-worker --maps-root /maps --mission-id "$SWARMDECK_MISSION_ID" \
  --planner-maps --timeout 30 --poll 1
```

The persistent runtime reuses resident local point buffers for pose-only
corrections. Planner export reads those exact immutable buffers and the corrected
poses from `KeyframePointCloudMap`; it does not reread chunks or run another
optimizer. Geometry replacement and retraction build a complete candidate grid,
so old walls and their ray evidence disappear together. Requested metric-map and
planner artifacts must both succeed in the pre-commit hook before the new native
snapshot becomes current.
