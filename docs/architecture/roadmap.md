# Development objectives

Priorities based on the repository review in September 2026. These are proposed
engineering outcomes, not claims about deployed hardware. Current behavior is
in the [architecture overview](overview.md); [requirements](requirements.md)
define the product contract.

The main objective remains a reliable dashboard for a heterogeneous fleet:
operators can understand robot state, trust map alignment, issue commands, and
recover from interruptions. ARGoS is the default validation environment; Gazebo
and alternative estimators remain explicit comparison paths.

## 1. Make the current workflow dependable

- Keep ARGoS build/start/stop commands consistent across renderers and scenarios.
- Exercise reconnects, slow map consumers, adapter loss, camera loss, cancelled
  goals, resets, and component restarts. Preserve capability checks and stop gates.
- Extract server state/lifecycle responsibilities incrementally; keep one API
  worker until state ownership supports multiple processes.
- Consolidate duplicate Cortex chat implementations after auditing direct API
  consumers. Keep the assistant optional.

Done when: CI covers the relevant contracts, and two recorded four-robot ARGoS
soak runs of at least 15 minutes complete with no unexplained command replay,
silent state loss, or orphaned simulation processes. Record machine, config,
source versions, dropped-frame counters, and unresolved failures with each run.

## 2. Measure mapping quality and sensor timing

- Fix the external-estimator capture-time contract: the ARGoS observation bridge
  carries sensor stamps, but the estimator input protocol still only carries an
  exchange tick. See [simulation performance](../operations/simulation-performance.md).
- Compare Fast-LIVO2 against the synthetic-drift development baseline with the
  same scene, seed, and motion. Measure trajectory/map error and false merges.
- Use existing keyframe capture, replay, fault injection, and scoring tools in
  `slam/tools/` before introducing another evaluation framework.
- Collect time-aligned hardware datasets with surveyed or motion-capture ground
  truth; retain configuration, calibration, and source revisions.

Done when: a versioned dataset and reproducible command report ATE/RPE, map error,
merge failures, and data loss against ground truth. State acceptance thresholds
and compare every estimator change against that baseline.

## 3. Make complete sessions recordable and replayable

- Extend current manifests, JSONL events, keyframe captures, and Cortex history
  into a coherent session format for telemetry, maps, detections, and commands.
- Snapshot configurations, calibration references, and software versions.
- Implement completeness validation and GUI replay without connected robots.
  MCAP remains the target interchange format in the requirements.

Done when: a stopped session validates and reproduces the operator view offline;
truncated recordings and dropped streams are reported instead of silently accepted.
Offline keyframe replay alone does not meet this objective.

## 4. Validate heterogeneous hardware contracts

- Verify capture timestamps, gravity/frame conventions, extrinsics, sensor
  coverage, and disconnect/stop behavior for each deployment profile.
- Evaluate estimator replacements against measured failures. A shared wire/frame
  contract is required; one identical odometry implementation on every robot is
  not. Preserve working native/ROS 1 integration where appropriate.
- Treat fleet documentation as profile configuration, and record deployment
  verification separately. Do not infer current robot state from an old incident.

Done when: every supported profile has a reproducible bring-up and validation
record, with any unverified sensor transform or navigation capability identified.

## 5. Enforce access and assistant boundaries before broader deployment

- Add authentication and authorization around fleet-control and assistant APIs.
- Separate coding-worker access from robot credentials and motion authority;
  do not treat the tool-free shadow planner as isolation for the active provider.
- Evaluate optional provider/planner changes against the existing Cortex contracts
  and saved cases before granting additional authority.

Done when: unauthorized control attempts are rejected, deployment privileges are
explicit, and tests demonstrate that a coding worker cannot acquire fleet authority.
Keep existing deployments on a trusted network or behind an authenticating proxy
until this boundary exists.

## Deferred choices

New estimators, agent frameworks, a native numerical rewrite, larger fleet sizes,
and cloud/high-availability deployment need a measured problem and acceptance
criteria first. Historical proposals and benchmark results are useful evidence;
they are not automatic implementation commitments.
