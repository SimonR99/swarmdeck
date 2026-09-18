# SwarmDeck plan: peer Swarm-SLAM, native MOLA, MGG planning

This is the single plan for the `planning-refactor` work: the architecture, the
invariants, what is done, what is open, and how to validate it. Dated trials and
their numbers are in the [acceptance log](operations/acceptance-log.md).
Superseded design, tracker and chronology documents are kept unchanged in
[the archive](archive/README.md).

## Architecture

Every robot runs the whole onboard chain. Robots collaborate directly when
connected. The server receives replicated results for supervision; it is not
required to compute their maps or routes.

```mermaid
flowchart TB
  Sensors["ARGoS or physical sensors"] --> Capture["Adapter capture<br/>source frame + capture-time pose"]
  Capture --> Peer["Peer Swarm-SLAM<br/>verified component correction"]
  Peer --> Chunks["Revisioned geometry<br/>and immutable snapshots"]
  Chunks --> MOLA["Native MOLA<br/>persistent products"]
  MOLA --> Query["Bounded indexed query"]
  Query --> MGG["MGG graph + grid"]
  MGG --> Nav2["Nav2 local controller"]
  Nav2 --> Adapter["Robot adapter<br/>command/action output"]
  Chunks --> Replica["Server replica"]
  Replica --> Server["FastAPI server"]
  Server <--> UI["Svelte UI"]
  Server <--> SLAM["Central SLAM diagnostics"]
```

The planning hierarchy is graph planner, then grid planner, then local
trajectory planner and controller. Exploration and coordination influence
planning objectives; collision, terrain feasibility and actuator limits remain
hard constraints.

### Ownership

| Concern | Authority | Source |
| --- | --- | --- |
| Continuous local motion estimate | One selected odometry frontend per robot and session | `adapters/`, estimator containers |
| Collaborative keyframe poses and component membership | Peer Swarm-SLAM solution adapter | `deploy/cslam/`, `deploy/autonomy/` |
| Occupancy, surface geometry, map revisions | Native MOLA mapper | `swarmdeck_ros/src/swarmdeck_mapping/` |
| Snapshot contracts, coordination, indexed map queries | Peer mapping layer | `autonomy/`, `deploy/autonomy/` |
| Robot feasibility | Shared traversal model, used by both planners and the controller | `deploy/mgg/` |
| Exploration routes and navigation goals | MGG graph and grid planning | `deploy/mgg/` |
| Actuator commands and cancellation | Onboard command arbiter in the adapter | `adapters/` |
| Gaussian appearance and checkpoints | Optional reconstruction worker, bound to a graph revision | `scripts/reconstruction/` |
| Fleet state, sessions, replicas, operator commands | ROS-free server | `server/` |
| Operator display | Cached replica in the browser | `ui/` |
| Server-facing pose graph and occupancy diagnostics | Central SLAM service on `:8090` | `slam/` |

Adapters capture points in a sensor frame and associate them with the pose at
the capture timestamp. Local odometry and navigation frames stay robot-owned.
Peer Swarm-SLAM can establish a verified component frame and correction. The
same correction identity and map revision flow into the MOLA product and the
indexed query. MGG plans in the configured robot navigation frame, Nav2 handles
local obstacle avoidance, and the adapter owns the final command boundary.

MGG is built from the `swarmdeck` branch of
[MGGPlanner](https://github.com/MISTLab/MGGPlanner), currently pinned at commit
`54ca865a41685094d689e5216f2d50be8c0c089e` (the 59 ported commits over
upstream `902e868` plus the authority tilt tolerance, the loader coherence retry, visibility retirement and validated-prefix navigation), selected by `MGG_REV` in `deploy/docker/Dockerfile.mgg`. Planner
changes are made in that repository and the pin is advanced.
`deploy/docker/build-mgg-msgs.sh` builds only `mgg_msgs` from the same pin, so
service type hashes match across images.

## Invariants

These rules are non-negotiable. A failing trial is not a reason to weaken one.

1. One provider publishes local odometry per robot and session. Multiple
   estimators consuming the same IMU and LiDAR are not independent measurements.
2. Peer Swarm-SLAM is the only corrected-pose authority. MOLA neither optimizes
   those poses nor publishes a competing `map -> odom` transform.
3. No identity transform connects unverified components. A disconnected robot
   has its own map root; a missing or stale shared transform blocks the
   dependent operation instead of assuming two local frames coincide.
4. Occupied points alone cannot certify free space. Free space requires explicit
   `RayEvidence`: `FIRST_RETURN`, `DESKEWED` or `NOT_REQUIRED`, plus
   `SINGLE_CAPTURE`. Missing, partial, stale or generic geometry evidence never
   creates free space.
5. Unknown stays unknown. Unknown ground and missing returns are not
   traversable, omitted rays never become inferred free space, and obstacle
   expiry alone cannot prove an area clear.
6. Gaussian opacity is not occupancy probability, and absence of splats is not
   observed free space. Gaussian output never reaches collision planning.
7. Stop, a replacement goal and link loss win over every late path or service
   response. Physical or local stop always overrides planning.
8. Never claim Stop All was delivered to an unreachable robot. Show pending or
   unconfirmed status and use mission generations to reject stale commands.
9. Hardware adapters never advertise or implement the simulation-only `reset`
   capability.
10. A distant goal is never silently replaced with a nearby proxy. A response
    claiming a complete route must end within 1 mm of the requested
    navigation-frame XY; supported terrain may change endpoint height.
11. A route binds to the geometry it was checked against. Revision increments
    alone are not physical frame changes; material corrections revalidate or
    replan the same destination.
12. A local empty frontier set is not fleet completion. Distinguish `blocked`,
    `waiting_for_map`, `locally_exhausted` and `complete`.
13. The server is a replica. It does not solve a robot's graph or issue map
    corrections in decentralized mode, and robots must not depend on pulling a
    server grid back to continue planning.
14. The backend and the browser stay ROS-free. Planner and middleware details
    stay inside adapters.
15. Selecting a backend that has no valid product returns unavailable. There is
    no silent fallback to a coarser map, to raw clouds, or to a relabelled frame.
16. A coarse voxel index cannot enforce a fine step limit. The exact surface
    query is the safety gate; 20 cm voxel centres alone cannot enforce a 10 cm
    platform step limit.
17. Simulation terrain step settings are 0.15 m for Bunker and Scout and 0.30 m
    for Spot. These are simulator parameters, not hardware guarantees.
18. Simulation ground truth is used only for scoring, never as an inter-robot
    alignment source or a planner input. The one exception is the
    simulation-only peer-body mask, which reads every robot's ground-truth pose
    to delete returns that landed on a neighbour at the capture stamp; it feeds
    no pose estimate and is off on hardware. Trials that use it must say so,
    because hardware robots remain visible to each other.
19. Frontier reservations are arbitrated only between robots that share a
    frame: one verified map component, or, where a deployment has surveyed
    every robot's start pose, that deployment frame
    (`SWARMDECK_COORDINATION_FRAME=deployment`, on in simulation). The
    deployment frame carries reservation targets only, whose radius absorbs
    metres of drift; it never aligns maps, moves a pose or reaches a planner's
    geometry. Without either, robots explore independently. The same frame
    carries each robot's reported position to its peers' planners as a
    transient keep-out disc; it expires with the reports and is never stored
    in a map.

## Status

| Priority | Deliverable | State | Evidence |
| --- | --- | --- | --- |
| 1 | Persistent native MOLA map ownership and a loadable MOLA framework module | Done | 2026-09-10 workstation correction benchmark; 2026-09-15 Bistro replay |
| 2 | MOLA map products behind the planner map-provider interface, with explicit free, occupied and unknown terrain semantics | Sim only | 2026-09-10 four-robot native free-space production; the native grid is the simulation default and hardware use is unqualified |
| 3 | Selectable calibrated odometry and capture providers for simulation, SuperOdometry and FAST-LIVO2 | Partial | 2026-09-15 Fast-LIVO2 mission on domain 219; SuperOdometry and FAST-LIVO2 capture contracts remain occupied-only |
| 4 | Shared graph, grid and local-control planning for Explore, Navigate and Home, with blocked-corridor replanning and speed limits | Partial | 2026-09-18 `e7affc8` with MGG head `3486391`: three consecutive grouped-start Fleet Explore trials passed for all four robots (15 to 40 m each) with server-sequenced departures, peer bodies as transient discs, a 0.9 s planner and the route-progress watchdog; speed limits are not implemented |
| 5 | Qualify peer SLAM and exploration coordination on Bistro and on separate hosts | Partial | 2026-09-16 live merges are geometrically wrong in Z (about 0.3 m between platforms) and break terrain gates; merges are disabled again on benchbot; no multi-host, partition or optimizer-loss trial |
| 6 | Qualify per-robot gateways, ARM builds and physical ROS 2 deployment | Not started | no ARM build, packet capture or hardware motion trial is recorded |
| Parallel | Fixed-pose Gaussian batch reconstruction, then incremental training | Partial | native CUDA smoke completed nine optimizer iterations and converted 540 Gaussians; no real-capture alignment measurement |

## Open work

Each item names its acceptance gate.

- [ ] **Bistro road crossing.** Gate: R0 reaches the 20 m forward road
      destination, or a measured barrier explains every rejected detour. The
      failure reproduces with synthetic drift and with Fast-LIVO2 odometry, and
      is not deadline exhaustion.
- [ ] **Fleet motion acceptance.** Gate: four-robot startup, Navigate and Home
      succeed against independent simulation truth; cancellation and map
      corrections stop or replan correctly.
- [ ] **Blocked-corridor topological replanning and speed limits.** Gate: a
      failed edge is excluded from a new topological search, and refined paths
      carry speed limits. Implemented in MGG at `494ce66` (branch head `c4eff1b`): Explore now runs
      through the shared topological and grid stages, and a bounded, expiring
      blocked-corridor registry (64 entries, 10 s, 8 map revisions) steers the
      topological search for all three objectives; native tests grew from 319
      to 340 entries. Not yet pinned in `Dockerfile.mgg` because it changes the
      live exploration path (refined corridor polylines instead of the
      shortcut walk, and Explore can now be refused on terrain). Advance
      `MGG_REV` after one fresh Bistro exploration trial. Speed limits remain
      unimplemented; `queryIndexedMap` refuses height refinement and prefix
      truncation whenever speed limits are present, which must be resolved
      first.
- [x] **Cold-start Navigate beyond the first ground ring.** Validated-prefix
      sections with a continuation (MGG `swarmdeck`), adapter no-progress bound
      of three sections, scene-change keyframes for parked robots, paced
      retries for a momentarily unavailable planning or indexed map, and an
      index that keeps serving while the next product is pending. Measured
      2026-09-17 from a fresh grouped start: 16 of 18 goals at 3 m and five
      headings per robot arrived within 0.32 m; both refusals named a goal on a
      0.30 m and a 0.67 m rise.
- [x] **Peers as live obstacles for the global route.** Done 2026-09-17: each
      robot reports its position in the surveyed deployment frame and its
      peers' planners hold a 0.7 m transient disc around it (MGG `004c517`);
      with server-sequenced departures and the route watchdog, three grouped
      starts in a row passed on 2026-09-18. Earlier notes follow.
      Original gate text: Phantom bodies of
      robots that have left are now retired by visibility (three later
      qualified rays through the voxel); present neighbours remain an open
      gate. Gate: at a grouped
      Bistro start, exploration routes avoid neighbouring robots and no robot
      ends in Nav2 `Failed to make progress` against a peer. The capture-time
      peer-body mask keeps robots out of the persistent map, so MGG's route
      no longer sees them while Nav2's live costmap still does; feed each
      peer's current position and body radius to MGG as a temporary exclusion
      (simulation adapter first, then the intentions channel for hardware).
      Measured 2026-09-17: this is what breaks Fleet Explore from the grouped
      start. First paths run through parked neighbours, the local controller
      improvises outside the validated corridor, and robots wedge beside a
      0.19 m kerb at (-9.7, 3.3) and near (-11.6, 7.6) in the simulation frame.
      Limiting reverse to 0.15 m/s made one grouped start pass and did not
      cure the next; Explore from dispersed poses passes (25 to 33 m each).
      Also find out why a route MGG admits beside that kerb is not drivable.
      Keyframe bursts during recovery motion are explained (2026-09-17):
      robot_2's 121 keyframes in three minutes on 2026-09-16 predate the
      scene-change rule and come from the distance rule alone, which
      compares against the last keyframe's position only, has no rotation
      term, and so pays one keyframe per swing whenever recovery rocks the
      robot by more than 0.25 m. With the scene-change rule live, robot_3 in
      mission `a72b1c3d` added a keyframe every 5.0 to 5.8 s for 160 s while
      dithering inside 0.7 m by 0.4 m with 5 to 39 degree yaw swings (27 of
      51 steps under 0.2 m): the azimuth signature was binned in the sensor
      frame, and on a stored keyframe a pure 5 degree yaw moved four of 72
      sectors by more than 0.5 m. The patch now bins in the odometry frame
      (yaw-aligned); verify on the next rebuilt cslam image that a rocking
      robot's keyframe count tracks its distance again.
- [ ] **Moving obstacles and blind corners.** Gate: controlled trials where the
      local controller stops or avoids, then resumes or requests a valid
      replacement corridor.
- [ ] **Inter-robot closure accuracy and duplicate coverage.** Gate: measured
      inter-robot candidate and accepted-closure counts, a consistent shared
      component on peers and the server, and reduced duplicate coverage versus
      independent MGG. Offline replay (2026-09-16) links all six robot pairs
      with zero false merges after the peer admission change (similarity 0.70,
      18 inliers, ICP overlap gate 0.40). Live on 2026-09-16 the fleet merged
      within seconds and XY and yaw matched ground truth within 6 mm and 0.01
      degrees, but vertical placement between platforms with different lidar
      heights was off by about 0.3 m and pairwise errors grew to 0.45 to
      0.83 m in XY after 20 m of drift-odometry travel; the merged floors then
      failed the 0.15 m step gate everywhere and Navigate and Explore stopped
      working. The deployed override runs the strict no-merge cslam image
      again. Next: planar or ground-plane-corrected inter-robot constraints
      for ground robots, then re-measure Z before re-enabling merges.
- [ ] **Separate hosts, partitions and optimizer loss.** Gate: peers collaborate
      with the server stopped; partition and rejoin do not duplicate commands or
      falsely declare completion. A frontend restart currently requires a fresh
      fleet mission and domain; transparent restart is not implemented.
- [ ] **Hardware capture provenance.** Gate: SuperOdometry and FAST-LIVO2
      capture contracts attest first returns, deskew and one endpoint per ray,
      verified against recorded data. Until then those paths stay occupied-only.
- [ ] **Physical robot qualification.** Gate: per-platform sensor and TF
      validation, ARM images, calibration, bounded peer traffic, and low-speed
      controller trials, followed by the strict indexed terrain gate.
- [ ] **Per-robot network gateways.** Gate: packet captures show no DDS
      discovery on fleet links; required collaboration survives server and peer
      loss within measured traffic budgets.
- [ ] **Long-duration map capacity.** Gate: measure when a long mission reaches
      the indexed-map input bounds (1,000,000 points, 2,000,000 voxels,
      4,000,000 ray steps, eight seconds of build time) and what the planner does
      past them; exceeding a bound currently returns `UNAVAILABLE`.
- [ ] **Gaussian reconstruction from real captures.** Gate: measured alignment,
      held-out image quality, memory, training and rendering budgets, correction
      replacement and cancellation.
- [ ] **Reproducible Jazzy images.** Gate: every image builds on a clean host
      without Docker cache. On 2026-09-16 the ROS apt repository served only
      MOLA 3.2.0 (the mapping image pins 2.9.0) and a fresh apt layer gave the
      simulation image nav2 1.3.13 and September slam_toolbox and tf2 builds,
      under which the navigation lifecycle bring-up never answers. The mapping,
      sim and robot-ros2 Dockerfiles now carry an `ARG MGG_REV=902e868` cache
      anchor above their apt layers so those layers keep coming from cache;
      this only works on hosts that already hold the cache. Either mirror the
      exact June packages or qualify MOLA 3.2 and nav2 1.3.13.
- [ ] **Frontend restart persistence.** Gate: an explicit persistence or epoch
      solution for a restarted Swarm-SLAM frontend.

## Validation

```bash
# Normal simulation run: peer Swarm-SLAM, native MOLA, MGG, drift odometry
./scripts/sim-up --scenario bistro --drift
./scripts/sim-up --status
./scripts/sim-up --down

# Fleet exploration startup against a running four-robot simulation
server/.venv/bin/python tests/deployment/exploration_acceptance.py \
  --simulation --base-url http://localhost:8080 --duration 180

# Python, UI and SLAM suites
make test
make test-ui
make test-slam

# MGG native PCI contract smoke, without robot access
docker run --rm --network none -e ROS_DOMAIN_ID=173 \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -v "$PWD:/workspace:ro" -w /workspace \
  --entrypoint bash swarmdeck-mgg:local -lc '
    source /opt/ros/jazzy/setup.bash
    source /opt/mgg/ros2/install/setup.bash
    export PYTHONPATH=/workspace:"$PYTHONPATH"
    python3 adapters/test/ros/mgg_contract_smoke.py'
```

Operational detail is in [current stack operations](operations/current-stack.md),
[MGG exploration](operations/mgg-exploration.md) and the
[MOLA runtime guide](operations/mola-runtime.md). Record the source revision,
image identity, executed tests, measured limits and remaining failures for every
trial in the [acceptance log](operations/acceptance-log.md). A waiting status or
a visible map is not acceptance of autonomous navigation.
