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
and atomically publishes `/maps/<mission>/<robot>/mola/index.json` only after all
generation-specific metric maps finish. The index contains the source snapshot
ID/hash and, for each component, its epoch, graph revision, geometry revision,
relative artifact path, byte size, and SHA-256. It coalesces unchanged inputs,
uses a bounded subprocess timeout, retracts an index if the input races, and
keeps the prior index on failure. It never publishes TF or runs optimization.

Run it directly in the mapping image:

```bash
swarmdeck-mola-worker --maps-root /maps --timeout 120 --poll 1
```

The current worker rebuilds a serialized MOLA map per changed component. A
future long-lived MOLA module can call `applyPoseSolution()` directly to avoid
serialization on pose-only corrections; this is an optimization, not a missing
consumer boundary.
