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

- **Native MOLA and planner provider:** resident MRPT point buffers, corrected
  planner-grid construction, bounded `SDMGRID1` export, strict product loading,
  immutable publication, and shared terrain queries are implemented and covered
  by local tests.
- **Capture qualification:** new ARGoS captures pass the original-ray contract
  and produce native free-space grids. SuperOdometry and FAST-LIVO2 still need
  recorded hardware qualification. Legacy keyframe clouds remain occupied-only.
- **Fleet display:** the read-only component catalogue and aggregate view are
  available from the map Layers panel; the [replica component guide](../operations/replica-components.md)
  documents its compatibility and failure rules. Per-robot inspection remains
  available.
- **Integration and acceptance:** worker publication, framework lifecycle,
  deployment wiring, remote replay, MGG service checks, and measured budgets are
  reviewed and recorded by the integration owner. No remote result is implied by
  the local implementation status.

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

The deployment defaults remain the existing indexed provider. Legacy Bistro
captures lack the qualified original-ray contract and remain occupied-only.
The third milestone below qualifies new instantaneous simulation captures;
hardware capture qualification, MGG's exploration OctoMap replacement, and
graph/grid/controller qualification remain outstanding. See the [runtime guide](../operations/mola-runtime.md)
for the provider settings and admission rules.

## Capture provenance gate

Free-space carving is admitted only when the stored endpoints retain their
original-ray meaning. A qualified capture identifies one physical capture,
uses first returns, has an instantaneous or deskewed frame, and associates all
endpoints with one calibrated sensor origin. In the versioned contract this is
`RayEvidence` with `FIRST_RETURN`, `DESKEWED` or `NOT_REQUIRED`, and
`SINGLE_CAPTURE`.

The capture provider also verifies the concrete source contract, clock domain,
capture interval, capture-time transform, estimator session, and calibration.
Simulation can qualify a one-tick ray capture (`NOT_REQUIRED`). The current
SuperOdometry and FAST-LIVO2 raw boundaries attest first returns but do not yet
attest deskew; a FAST-LIVO2 registered cloud may be deskewed but does not attest
one endpoint per ray. Those paths therefore remain occupied-only until their
capture contracts are completed. Missing, partial, stale, or generic geometry
evidence never creates free space. This conservative rule is required on
hardware, where a plausible sensor origin is not proof that intermediate cells
were observed.

## Remaining qualification

The remaining work is to replace MGG's exploration OctoMap with the qualified
planner product, complete shared graph-to-grid-to-controller behavior for
Explore, Navigate, and Home, and extend four-robot startup validation to correction,
replanning, and moving-obstacle trials. Physical ROS 2 gateways, ARM builds,
calibration, bounded peer traffic, and low-speed controller trials remain
hardware gates. Gaussian reconstruction remains a separate fixed-pose batch
path until real-capture alignment, correction replacement, resource budgets,
and cancellation are measured; Gaussian opacity is never collision occupancy.

Existing Bistro startup, terrain/Home and inter-robot closure failures remain
acceptance work, not reasons to weaken map admission guards.

## Third milestone: captures, recovery, and fleet display

The original ARGoS endpoint/provenance join, bounded controller-failure recovery,
and fleet component Layers selector are implemented. Review added manual-goal
ownership checks, strict replica identity/owner validation, and a tombstone fix
for retraction followed by map correction. Sol and Luna implemented separate
areas; Gemini 3.8 Flash reviewed display semantics through `agy`. Changes were
reviewed and tested together before committing.

The [benchbot acceptance record](../operations/planning-parallel-acceptance.md)
records four-robot native free-space production, HTTP/browser checks, and the
bounded Explore/Stop All trial. R1 still reaches a planner-generated blocked
state. No live inter-robot closure or full MOLA-driven exploration qualification
is claimed. The next priority is MGG's map-provider migration and blocked-route
recovery, then Navigate/Home and coordinated multi-host acceptance.
