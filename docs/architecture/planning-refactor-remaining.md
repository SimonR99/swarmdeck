# Remaining integration work

This work continues on `planning-refactor`, with MOLA integration first. The
existing robot deployments remain available while each replacement passes its
acceptance checks. The [full design](decentralized-autonomy-plan.md) defines the
architecture; the [implementation record](../operations/decentralized-autonomy.md)
contains measured results and known failures.

## Order of implementation

| Priority | Deliverable | Acceptance before enabling |
| --- | --- | --- |
| 1 | Persistent native MOLA map ownership and a loadable MOLA framework module. Reuse keyframe geometry for pose corrections; replace/retract geometry coherently. | Native insertion/correction/replacement tests, bounded process failure recovery, real MOLA module loading and map callbacks, same-process multi-revision replay. |
| 2 | MOLA map products behind the planner's map-provider interface, with observation provenance and explicit free/occupied/unknown terrain semantics. | Corrected geometry and sensor origins agree; old walls disappear; floor, curb, step, drop and stacked-surface tests pass; measured query/build budgets. MGG's OctoMap remains explicit until this replacement is qualified. |
| 3 | Selectable calibrated odometry/capture providers for simulation, SuperOdometry and FAST-LIVO2; MOLA-native odometry can use the same boundary. | Capture-time transforms, estimator resets, clock domains and covariance provenance verified against recorded data. Only one provider publishes local odometry; Swarm-SLAM remains the corrected-pose authority. |
| 4 | Finish shared graph → grid → local-control planning for Explore, Navigate and Home. Add blocked-corridor topological replanning and speed limits. | Four-robot startup and Home success against independent simulation truth; cancellation and map corrections stop/replan correctly; moving-obstacle and blind-corner trials. |
| 5 | Qualify peer SLAM and exploration coordination on Bistro and separate hosts. | Accurate inter-robot closures, reduced duplicate coverage, no starvation or false completion, partition/rejoin and optimizer-loss tests. Frontend restart must have an explicit persistence/epoch solution. |
| 6 | Qualify per-robot gateways, ARM builds and physical ROS 2 deployment. | No unintended sensor/DDS traffic on fleet links; bounded peer traffic; server-independent collaboration, calibrated capture and low-speed controller trials for each platform. |
| Parallel, after map identities stabilize | Extend fixed-pose Gaussian batch reconstruction to real captures, then evaluate incremental training. | Measured alignment, held-out image quality, memory/training/rendering budgets, correction replacement and cancellation. Gaussian opacity is never used as collision occupancy. |

## Current work split

- **Sol — native MOLA:** resident MRPT point buffers, corrected planner-grid
  construction, bounded binary export and native regression tests.
- **Sol — planner provider:** strict product loading, immutable grid publication,
  shared terrain queries and provider selection.
- **Luna — acceptance:** captured-point fixtures across the native/Python
  boundary, image integration and copied-map replay.
- **Lead review/integration:** provenance contract review, atomic worker product
  publication, framework component lifecycle, deployment wiring and independent
  workstation tests. Agent changes are reviewed before inclusion.

## Validation and rollout

Use local tests for contracts and failure handling. Build native packages and run
isolated fixtures on `sroy@benchbot.yannbouteiller.com`, using separate container
names and ROS domains. Preserve the production checkout and running robot maps.
Use a fresh mission for motion trials that restart native SLAM frontends.

For each milestone, record the source revision, image identity, executed tests,
measured limits and remaining failures. Enable the new path first in the isolated
planning deployment; hardware rollout follows its platform-specific gates.

## First milestone implemented

The persistent runtime, supervised worker, and loadable MOLA map-source module
are implemented. The module can select a component directly from a whole-peer
snapshot and publishes the manifest's coordinate frame. Native and framework
tests exercise correction without source chunks, coherent geometry replacement,
retraction, callback access, source invalidation and recovery. The local
autonomy suite passes 100 tests. See the
[runtime guide](../operations/mola-runtime.md) for measured replay results and
the distinction between point geometry and the planner's required map products.
The complete image also passes actual MOLA launcher scheduling and worker
acceptance with unavailable chunk payloads. The persistent worker is enabled in
the isolated planning deployment and its four live indexes are coherent.

## Second milestone implemented; motion qualification remains

Native MOLA now produces occupied/free voxels and surface samples for a selectable
planner provider. The provider reuses the existing terrain query implementation;
metric maps and planner grids publish as one coherent worker generation. The
framework's automatic selection follows a changed sole component and rejects
ambiguous input. Corrected origins, disappearing walls, floor/step/drop/stacked
surfaces and artifact failures are covered across the native/Python boundary.

The deployment defaults remain the existing indexed provider. Bistro captures
currently lack explicit first-return and deskew provenance, so both providers
conservatively preserve unknown space instead of inferring free cells from a
sensor origin alone. Priority 3 capture-provider qualification is therefore the
next dependency for enabling MOLA in motion planning. MGG's exploration OctoMap
and graph/grid/controller qualification also remain outstanding. See the
[runtime guide](../operations/mola-runtime.md) for the provider settings and
measured validation results.

Existing Bistro startup, terrain/Home and inter-robot closure failures remain
acceptance work, not reasons to weaken map admission guards.
