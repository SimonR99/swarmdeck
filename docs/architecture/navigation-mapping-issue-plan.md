# Navigation and live mapping implementation plan

This plan consolidates the issues observed in the Bistro deployment on port
15173 and the remaining `planning-refactor` integration work. It supersedes
isolated fixes that make an inspection view look live or make a local planner
appear to support arbitrary navigation goals.

## September 13 fleet regression plan

The operator still reports failures after the narrow 12 m acceptance trials.
Those trials are retained as evidence for their specific cases, not as proof
of general navigation, exploration or shared mapping. This pass addresses the
following lanes in parallel, with root review before deployment.

| Lane and owner | Investigation and implementation | Acceptance gate |
| --- | --- | --- |
| Navigation — Sol | Trace a click through component, map and odometry frames; retain the selected physical goal and full route. Remove distance-dependent rejection caused by grid construction, evidence policy or request budgets. Unknown terrain remains a possible route; known impassable terrain remains excluded. Share the route pipeline with Home. | 5/12/25/50/100 m planning cases, including unknown destinations and known detours; no displaced endpoint or distance-only failure. A fixed-world route stays fixed during motion and SLAM gauge updates. Live near/far arrivals and Home, plus blocked-route and three-physical-failure cases. |
| Peer mapping — Sol | Trace descriptors, candidate verification, peer optimization, component merging and server replication. Fix broken transport/frame/identity contracts before adjusting admission thresholds. | Measured inter-robot candidate and accepted-closure counts; consistent shared component on peers and server after genuine verification. Global UI renders its accumulated submaps. Unverified components are never fabricated into a shared frame. |
| Tactical rendering — Sol | Separate actual surface triangles from voxel blocks. Trace RGB availability and color selection; trace Gaussian artifacts and capability gates. Stabilize robot overlays across asynchronous geometry/live updates. | Distinct point/voxel/mesh geometry, working RGB on qualified colored data, actual Gaussian artifact rendering where available and explicit absence otherwise. Stable robot markers, ceiling and source through Local/Global and 2D/3D transitions. |
| Exploration — root | Trace fleet Explore command, PCI state, candidate/path rejection, peer reservations and FollowPath ownership. Surface the actual waiting/blocked cause and repair no-motion states. | All eligible robots start or report a specific actionable reason; accepted paths execute before replacement; Stop exploration and Stop All halt all motion. No false completion from a transient empty graph. |
| Reset — root with independent audit | Route reset to the active simulator and coordinate pose, odometry, controller, SLAM, mapping and server epochs. Fence stale callbacks and uploads. | One UI reset restores the deployment and a fresh coherent map; all robots acknowledge or report specific failed stages. Repeated reset works; old paths/maps do not return and exploration stays stopped. |

Execution order:

1. Record deployed images and reproduce failures using bounded service/status
   probes and synthetic/static fixtures. Preserve unrelated local changes.
2. Implement the independent lanes with tests for the broken contracts; share
   frame and reset contracts across owners before changing their consumers.
3. Review each diff, run affected tests and native/UI builds, and combine only
   reviewed changes into a reproducible benchbot export.
4. Test the actual GUI command paths and ARGoS motion on benchbot. Measure
   planning latency, route endpoint stability, peer closure/component counts,
   renderer behavior and reset completion. Record failures as failures.
5. Request a fresh independent quality pass over the combined work, resolve its
   findings, then leave one server and one simulation on port 15173.

Runtime limits must distinguish incomplete search or unavailable inputs from
proof that no path exists. A distant goal must not be silently replaced with a
nearby proxy. For ground robots, an arbitrary spatial click is interpreted as
a terrain destination; known walls, excessive steps and unsupported elevated
targets cannot become traversable by extrapolating an imaginary ramp.

### Findings from this pass

- **Navigation:** native Bunker fixtures with only a small observed starting
  patch return exact 25/50/100 m destinations through provisional unknown
  ground within the existing 500 ms request budget. Increasing that budget or
  treating unavailable footprint queries as clear ground is not justified by
  these cases. A downward terrain ray must retain signed clearance: taking an
  absolute value could mirror an obstacle above a sample into ground below it.
- **Exploration:** the native graph inserted a correctly projected root, then
  seeded its lattice from unprojected base odometry. With the Bunker's actual
  body dimensions this overlaps the floor and rejects otherwise usable samples.
  The regression fixtures exercise actual graph construction, not an adapter
  mock. Waiting/blocked telemetry also carries the planner's reason.
- **Peer mapping:** the current bench mission already has a verified R0/R1
  component, while R2/R3 remain separate. The inactive centralized service's
  zero counters did not describe this peer graph. The UI now consumes bounded
  peer verification reports and component membership. No admission threshold
  was changed and no unverified alignment was fabricated.
- **Visualization:** peer RGB was lost both when choosing raw capture geometry
  and when the browser checked colored chunks against XYZ-only lengths. The
  bridge now projects the chosen points at the image capture pose. Optional
  RGBA survives the replica pipeline; native MOLA uses its geometry prefix.
  Mesh mode connects measured neighboring cells with surface triangles.
  Gaussian rendering accepts qualified component artifacts, but selecting the
  mode does not create a trained model.
- **Reset:** the legacy adapter used Gazebo reset operations in ARGoS and could
  call ARGoS's physical `set_pose` service as if it were an odometry reset. The
  onboard path now requires a host supervisor to restart the whole deployment
  with a new mission and shared DDS domain, then wait for navigation readiness.
  A missing supervisor is reported immediately.

These are local implementation and fixture results. They do not establish
full Bistro navigation, all-robot exploration, Home, camera quality, or a
successful live reset. The previously running mission was not restarted during
this pass and should not be used for motion acceptance after its legacy reset
attempt. Start a fresh supervised mission for the next live trial. Native CUDA
training and real RGB-D reconstruction remain separate acceptance work.

### Local acceptance record

- 270 focused Python tests passed across capture/color, mapping, fixed-frame
  display, objective execution, exploration, reset, runtime, peer telemetry,
  replica observation and camera launch configuration.
- Fleet, replica and 3D UI suites passed; Svelte checking reported no errors or
  warnings and the production UI build succeeded.
- All four native MOLA mapping test programs passed. An actual Python colored
  snapshot imported into the native mapper as one submap with five points.
- The ROS cSLAM diagnostic smoke passed the exact image/scan TF query and
  explicit camera axis conversion, using a local container without sensor data.
- Native MGG service fixtures return full 25/50/100 m goals in about 160 ms
  total on the local test host. Bunker and Spot exploration fixtures return
  usable paths on partially observed floor in about 60 ms total. These are
  small synthetic scenes; the measurements do not establish Bistro latency.
- A clean build of the exact final MGG patch sequence passed all 15 core and
  three ROS test programs: 144 core tests and 49 ROS tests, including 37
  objective-service cases, with zero failures in their result XML. The final patches are
  `mgg-ground-projection-signed-clearance.patch`,
  `mgg-distant-objective-regression.patch`, and
  `mgg-exploration-driving-height.patch`, applied after terrain caching.

Next deployment gate: review the final source bundle, rebuild MGG, simulation,
cSLAM and native mapping in the existing `planning-next` workspace, load the
new UI build, and start the reset supervisor with the existing deployment
settings. Then use one fresh mission for all live cases below; retain only the
existing UI on 15173 and its one backend/simulation stack.

1. Place near, far and diagonal road goals, including unknown destinations;
   measure endpoint position and controller action replacements during travel.
   Exercise a known detour and an impassable curb, then Home for each platform.
2. Start fleet Explore; require motion or a specific waiting/blocked reason
   for every eligible robot. Stop exploration and Stop All must win over late
   planner responses. Observe actual physical blockage before counting retries.
3. Verify shared-component membership and peer reports in Global, ownership
   filtering in Local, and stable filled robot markers through map updates.
   Inspect points, voxels and triangle mesh with newly qualified RGB captures.
4. Reset through the GUI twice, including a lost POST response and a reconnect.
   Require a new mission/domain, all four navigation services ready, stopped
   exploration, fresh maps, and no prior route or keyframe identity reuse.
5. Publish a genuinely trained, correctly qualified Gaussian model before
   claiming reconstruction acceptance; an explicitly labeled proxy is not that
   acceptance result.

### Benchbot rebuild and reset acceptance

The reviewed `f03e68f6ee1c` bundle is now deployed in the existing
`planning-next` workspace on benchbot. Six images (MGG, MOLA mapping, cSLAM,
simulation adapter, server and UI) were rebuilt from the installed dependency
images under the `fleet-regression-f03e68f6ee1c` tag. The native MGG/MOLA checks,
colored native map import, peer ROS smoke and 11 server smoke tests passed.
The UI served on port 15173 matches the reviewed build, and all four robots
report online and navigation-ready.

A real request to the dashboard reset endpoint completed with a new mission
and shared DDS domain. All four robots returned idle and navigation-ready;
repeating the completed request's ID returned its original result without a
second reset. This trial caught a missing `COMPOSE_PROFILES=argos` in the host
supervisor service, which was corrected before the passing run. The supervisor
is enabled as a user service with lingering, so it survives SSH disconnects.
Only one server/UI and one simulation stack remain running.

This qualifies deployment and reset lifecycle, not live navigation arrivals,
Home, exploration coverage, inter-robot closures in the new mission, rendered
camera quality or trained Gaussian reconstruction. Those live acceptance cases
remain to be run. Build logs, source backups, image identities and reset results
are retained in the workspace's `.deploy/fleet-regression-f03e68f6ee1c` directory.

## Issue register

| Issue | Evidence and working diagnosis | Required outcome |
| --- | --- | --- |
| NAV-1: distant and straight-line goals fail | MGG Navigate used a ±6 m local exploration graph and substituted nearby progress proxies. Runtime also reports missing ground support and graph-tolerance failures. | A reachable distant goal in the accumulated known map receives a complete checked route to the selected destination, without proxy continuation. |
| NAV-2: paths disappear before arrival | PCI proximity replanning was corrected. Objective authority binding still includes correction-revision identity, and runtime reports cancellation after map-authority changes. | Controller results own arrival. Metadata-only revisions do not cancel valid motion; material changes revalidate/replan without losing the original goal. |
| NAV-3: impassable steps and obstacles are proposed | Accepted routes sometimes fail the controller progress check; exploration sometimes finds many free 3D samples but almost no ground-connected vertices. Exact terrain rejection counts are incomplete. | Graph edges, refined grid paths, and local control agree on supported ground, swept body clearance, step limits, slopes, and unknown space. Failed corridors are not retried indefinitely. |
| HOME-1: Return home reports Nav Failed | Home uses recorded authority landmarks and a separate bounded recovery path; its exact current failure needs tracing. | Preserve the initial physical home across corrections and follow the same global/grid/local route pipeline back to it. Report the actual reason when unreachable. |
| MAP-1: Live 3D has no cloud | Legacy cloud endpoint is empty in onboard mode; current robot replicas contain roughly 90–119k points each. Manual component inspection renders them. | Live uses accepted current-mission submaps automatically, with bounded cached geometry delivery. |
| MAP-2: Global shows selected local robot | Automatic source selection used the selected robot even when Global was active. There are currently no verified multi-robot components in the observed mission. | Local follows the selected robot's local map; Global shows accumulated geometry in verified shared frames. Unaligned components are explicitly distinguished, never silently presented as an aligned fleet map. |
| MAP-3: robot icons and navigation disappear in 3D | Replica mode was an inspection renderer that disables live overlays and input. | Live component maps carry frame-qualified robot poses, goals, paths, and navigation input. Historical inspection remains read-only. |
| RECON-1: no Gaussian model | The reconstruction endpoint returns 404 for this run. | Clearly distinguish absent reconstruction from absent geometry; later publish and render frame-qualified fixed-pose reconstruction artifacts. |

## Contracts to establish first

1. **A complete route to the operator's destination.** Navigate/Home must return
   a checked route ending at the selected goal before motion begins. Nearby
   progress proxies and partial-route continuation are not an acceptable
   substitute. Search the accumulated map and persistent topology; distinguish
   blocked terrain from unknown terrain or an exhausted search
   budget. Unknown terrain may be traversed provisionally under the configured
   policy. Stop, a replacement goal, and link loss win over all late
   path/service responses. Material corrections may replan the same destination.
2. **Stable frame identity, versioned evidence.** Mission, component, navigation
   frame, accepted transform, and map snapshot are explicit. Revision increments
   alone are not physical frame changes. A route binds to the geometry it was
   checked against; new evidence triggers appropriate corridor validation, not
   unconditional cancellation of every goal.
3. **Graph → grid → local controller.** Persistent topology over observed,
   traversable terrain chooses the long-range corridor. A bounded grid planner
   refines it and the local controller follows it while checking current obstacles.
   Exploration, Navigate, and Home share these stages; exploration adds utility
   and peer assignment, rather than a separate movement pipeline.
4. **Live map frames are not inspection frames by assumption.** Replicated geometry
   stays immutable and cached. Live poses and accepted navigation-to-component
   transforms arrive separately with freshness and identity. Display and goal
   conversion use that same transform. No pose is guessed from the last keyframe,
   a configured starting position, or an unrelated display frame.

## Parallel implementation lanes

### A — MGG native planning and terrain (Sol)

- Trace objective projection, graph binding, corridor refinement and controller
  rejection against actual current map products.
- Remove the local exploration horizon from explicit Navigate/Home planning.
  Use accumulated-map grid search and persistent topology to obtain a complete
  route; never dispatch a nearby progress proxy when the selected goal is beyond
  the local graph. Keep search memory, expansion count and time bounded.
- Build/reuse persistent traversable connectivity over the accumulated qualified
  MOLA map instead of expanding the expensive local graph to whole-map scale.
- Make exploration and objective route refinement use the same terrain decisions.
  Test platform step caps (Bunker/Scout 0.10 m, Spot 0.30 m), walls, drops, slopes,
  thin obstacles, and swept-body clearance after smoothing/resampling.
- Record concise rejection categories for low graph acceptance, not per-sample
  debug logs. Feed repeated controller failures back to corridor selection.

### B — Objective execution and Return home (Sol)

- Fix metadata-only authority cancellation while preserving mission/frame and
  material-transform guards.
- Reject partial or incorrectly terminated Navigate/Home routes before controller
  submission. Keep the original destination through map-correction recovery;
  follow a complete accepted route with one controller action.
- Diagnose Home end to end, preserve its initial landmark through corrections,
  and reuse the same route and recovery machinery as Navigate.
- Bound retries by failures and measurable progress. A new path revision or a
  near-zero segment must not replenish the retry budget. Preserve Stop All,
  manual ownership, deadlines, and late-response handling.

### C — Live tactical map (Luna, reviewed by Sol/root)

- Finish Global/Local source semantics and an explicit unaligned-global state.
- Distinguish live rendering from historical inspection. Restore filled robot
  icons, selection, goals, paths and applicable overlays in the component frame.
- Use the server's live frame contract for both drawing and goal conversion.
  Keep the previous coherent geometry across ordinary updates, including ceiling
  and camera settings; reset the view only for a deliberate source/frame change.
- Verify Live Map, explicit component choice, robot changes, Global/Local and
  2D/3D transitions in a real browser. Absent Gaussian reconstruction remains
  clearly unavailable; point geometry is not misrepresented as a trained model.

### D — Server/frame integration and acceptance (root)

- Publish/cache bounded live navigation-frame metadata independently of geometry.
  Reuse shared adapter/session and server registry boundaries so simulation and
  ROS 2 robots use the same contract.
- Expose accepted live component membership, transforms, raw navigation poses and
  paths to the tactical client; validate frame-qualified goal dispatch without
  feeding component coordinates through an unrelated legacy world transform.
- Review every lane before deployment. Keep unrelated local changes untouched.
- Run native tests and ARGoS on benchbot. Only this lane controls service updates
  and motion trials. Keep one server and one simulation on port 15173 afterward.

## Acceptance matrix

The current contract requires complete Navigate/Home routes, fresh component-goal
projection, metadata-only revision tolerance, and bounded recovery following
material map corrections. Live replicas have
separate qualified pose/path telemetry and a component-frame goal endpoint;
Local filters geometry by submap ownership, while Global requires a verified
shared component. These changes do not yet establish persistent long-range
connectivity or successful navigation through every Bistro terrain case.

`PlanObjective.partial` remains in the ROS wire definition for compatibility,
but the adapter rejects it for Navigate/Home before controller submission. A
claimed-complete response must end within 1 mm of the requested navigation-frame
XY; supported terrain may change endpoint height. Long display paths are sampled
across their whole length, retaining their destination while bounding telemetry
to 200 points; the controller receives the full path.
`PlanObjective.indexed_map_validated` distinguishes native MGG evidence from an
actual indexed MOLA corridor check; echoed snapshot fields alone are not proof
of validation. All ROS participants must rebuild the same service definition.
The adapter applies exact MOLA snapshot fencing when that evidence was used and
always retains mission/component and material-transform guards.
During execution, `planning.execution_authority_tolerance_m` (default `0.25`,
bounded to `0.01`–`5.0` m) limits the maximum accumulated SE(3) displacement of
any accepted route pose or its goal. Frame rotation independently retains
`authority_rotation_tolerance_rad` (default `0.02` rad), including for a short
route whose points barely move. Planning and dispatch also retain the stricter
`authority_translation_tolerance_m` admission fence. The position deadband
prevents small optimizer translations from repeatedly canceling motion; it is
not a clearance certificate. Nav2's live local costmap remains the collision
authority during execution, and exact MOLA snapshot checks remain mandatory at
initial planning and every replacement dispatch.
Active Home anchors use the same position budget and the separate
`planning.execution_goal_yaw_tolerance_rad` (default π, bounded to 0–π). The
current Nav2 point-goal profiles do not require final yaw, while deployments
that do can configure a smaller explicit heading budget.

Navigate uses persistent topology where available and completes the route with
bounded search on the accumulated map. A rejected graph corridor can retry direct
map search within the same remaining refinement deadline. Native ROS parameters
`objective_grid_margin_m`, `objective_grid_max_cells`,
`objective_grid_max_expansions` and `objective_grid_timeout_ms` default to
4 m, 32,768 cells, 16,384 expansions and 500 ms respectively. Their upper bounds
are 25 m, 262,144 cells/expansions and 5,000 ms. These limits bound search work;
they do not convert an incomplete search into a successful shorter route. The
deadline is cooperative between map queries. Final MOLA validation and map
generation checks remain separate required stages when that backend is selected.

| Scenario | Passing evidence |
| --- | --- |
| Nearby clear goal | Full path executes to controller tolerance; no premature cancellation. |
| Distant known clear goal | Complete route reaches beyond the local exploration horizon; one controller action reaches the exact final goal without proxy continuation. |
| Blocked straight line with known detour | Planner routes around the obstacle through known traversable space. |
| Step below/above each platform limit | Traversable step succeeds; impassable step is rejected or avoided before repeated physical impacts. |
| Map update during motion | Metadata-only changes preserve motion; material changes safely revalidate/replan while retaining the objective. |
| Repeated failure | At most three failed movement attempts for the recovery episode; repeated route revisions cannot evade the limit. |
| Return home | Each robot returns near its independently recorded initial physical position after travel and map corrections. |
| Dynamic obstacle | Local controller stops/avoids it, then resumes or requests a valid replacement corridor. |
| Global/Local 3D | Correct source and frame, visible correctly placed robots and paths; no assumed inter-robot alignment. |
| UI transitions | Live source, explicit choice, ceiling, and camera remain consistent across updates and 2D/3D changes. |
| Operator stop/replacement | No old RPC, queued segment, or map update restarts cancelled motion. |

## Remaining integration after the current defects

1. Qualify MOLA terrain products as MGG's graph-construction source, not merely a
   final corridor validator. Measure incremental graph/grid rebuild costs and
   correction/retraction behavior.
2. Complete persistent long-range connectivity, blocked-edge recovery and common
   speed/terrain constraints for Explore/Navigate/Home. Local proxy continuation
   alone does not solve obstacle detours or guarantee global reachability.
3. Qualify inter-robot Swarm-SLAM closures, component merging, exploration
   assignment and completion; test partitions and optimizer loss. The Bistro
   admission-threshold concern remains a separate measured SLAM investigation.
4. Validate fixed-pose Gaussian reconstruction from qualified RGB-D captures,
   using lidar odometry/optimized poses rather than a redundant visual tracker;
   measure alignment, correction replacement, training and rendering budgets.
5. Qualify SuperOdometry/FAST-LIVO2 captures, ARM images and ROS 2 hardware with
   the same frame/planner contracts. Simulator success does not establish real
   collision geometry, calibration or controller readiness.

Implementation records must include the exact source/image, actual tests,
remaining failures and deployment state. A waiting status or a visible map alone
is not acceptance of autonomous navigation.

### Bistro regression: waiting exploration and missing camera color

The September 13 live trial failed the autonomy acceptance gate despite passing
the earlier unit fixtures. R2 selected lattice routes that became empty during
the terrain check after smoothing; R1 lost connectivity at the physical start;
R3 rejected Home when neither endpoint had a measured floor hit. Explicit
navigation also exhausted the 500 ms grid deadline after thousands of terrain
queries. Review and exercise the physical-start, original-lattice fallback,
provisional Home, and goal-biased grid-search changes against these failures.
They must retain exact objectives, platform step checks, obstacle sweeps and
the existing work limits. Completion requires live travel and Home checks after
exploration has accumulated a map, not just a successful standing-start plan.

All 88 R0 keyframes inspected through encoding metadata were geometry-only.
Live checks also exposed a deterministic timestamp bug: Swarm-SLAM regenerates
its keyframe cloud with an empty header. Color capture now uses the paired
odometry header, which retains the same acquisition timestamp used for the raw
capture and calibration joins. Separately, the peer bridge kept only the latest
RGB and depth messages despite cross-topic delivery skew. Capture now selects
timestamp-qualified pairs from eight frames per stream, bounded additionally by
64 MiB per stream and 16 MiB per message. Synchronization, calibration, temporal
TF and depth-visibility checks remain required. Deployment acceptance must show
increasing `colored_captures` on every peer and replicated
`application/vnd.swarmdeck.xyzrgba-f32-u8.v1` chunks; a connected camera topic or
an enabled UI control alone does not establish correct color capture.

The first rollout reached a five-metre R3 goal within 17.5 cm and produced
multiple successful exploration routes on R0 and R3. It also exposed two further
failures. Home needs direct refinement even when the home floor is mapped but a
historical route waypoint is unsupported. Nav2 can report its typed
`FAILED_TO_MAKE_PROGRESS` result with an empty message; both ROS 2 adapters now
decode that constant so the existing three-attempt recovery applies. They do not
infer physical blockage from arbitrary numeric errors. Static Bistro collision
geometry places R2's eight-metre endpoint across an approximately 18 cm curb,
above the Scout's 10 cm step limit. The later five-metre test initially appeared
to be clear road, but checking its exact endpoint and complete footprint also
found a 17–18 cm discontinuity. The centre alone was on the lower road; the
footprint straddled raised pavement to its west. That rejection is valid.

The third rollout corrects footprint terrain sampling outside the robot's
circumscribed circle. Known steps inside that conservative footprint remain a
veto. The simulation now uses Nav2's `SimpleProgressChecker` with 20 cm
translation and a 20-second simulation-time allowance. This permits a bounded
initial skid-steer turn while preventing repeated yaw changes from indefinitely
masking a translation stall. The separate 25 cm arrival tolerance is unchanged.
Temporary loss of an authority heartbeat cancels controller motion and waits
under the existing recovery deadline before planning again to the retained
destination. It does not consume a planning attempt while no input is available;
mission, component, frame and Home identity changes still prevent resumption.

The timestamp correction is now verified live: every peer's successful color
capture counter increases, all four replicas contain RGBA chunks, and the
production UI bundle supports that encoding. This confirms data availability,
not visual calibration accuracy. In a fresh two-minute exploration trial, all
four robots completed routes and remained actively exploring. Maximum observed
pose displacement from their starts was 21.6–36.5 m; this is displacement, not
travelled path length or coverage. Stop All ended the trial. R3 then completed
Return home after travelling approximately 32 m back.

R2's Home test exposed a separate budget mismatch: Home's direct grid search
still used the 50 ms local budget and 1 m detour margin, whereas Navigate used
the 500 ms objective budget and 4 m margin. Both explicit objectives must use
the existing objective limits. A failed historical-corridor fallback must also
retain the primary search failure instead of hiding it behind a rejected
breadcrumb. With that correction deployed, R2 reached the statically qualified
12 m road goal within 19.7 cm, and R3 reached a five-metre goal within 22 cm
and completed Home. The deliberately curb-straddling R2 goal remained rejected
with its measured step reason.

R2 still rejected Home before motion with a 14.3 cm known-rise diagnostic after
a successful 12 m outbound trip. The failure is reproduced by the footprint
query's spatial bounds. The Scout's extended body has a 45.69 cm circumscribed
radius. The query expands ray centres to 56.30 cm by adding half a 15 cm voxel
diagonal; the containing voxels then expand the effective reach again to
69.43 cm. At the recorded Home phase, that double padding selects a pavement
cell outside the physical footprint. Full-material static evaluation of the
exact selected cells spans 14.54 cm, matching the live diagnostic, while even a
56.3 cm circle centered at Home spans only 5.28 cm. The native fix
enumerates map-aligned cells whose bounds intersect the physical
circumscribed circle, applying voxel uncertainty once. It does not raise the
Scout's 10 cm step limit. All core, map-backend and ROS service native test targets
pass. In a fresh live trial R2 completed the 12 m road trip and Home, with
11.86 m outbound and 11.60 m return displacement. The curb-straddling goal
remained rejected with its 14.3 cm measured-rise diagnostic.

That trial also exposed a separate controller limit: R3 reached its five-metre
outbound goal, then approached Home but stayed active for 110 seconds without
finishing. A retry rejected occupied space at its current position. Static
geometry near the approximate stopping area contains a wall close to the
conservative planning footprint; this does not establish an exact registered
collision cause. The old rotation-aware progress checker could mask a stall,
so the translation watchdog above bounds stationary rotation. After that change,
R2 completed the 12 m goal within 17.5 cm and R3 its five-metre goal within
21 cm; both then completed Home. R2 remained out on the road during R3's test
to leave clearance around the parked fleet. Stop All was verified after every
case. This qualifies those routes, not arbitrary passage through the congested
starting area.

The following fleet trial exposed an exploration-specific authority race. R1
had already started a replacement path when a transient authority lapse
cancelled it. Recovery reused the earlier, expired planning deadline and
immediately declared exploration blocked. A new authority interruption needs a
fresh bounded wait, retained physical retry count, and atomically captured
ownership of the cancelled goal. Stop or a newer command must win over resuming
that path. The first trial after this correction kept R0, R1 and R2 progressing,
but exposed a separate R3 failure: after moving 6.7 m, it lost authority briefly
and received its original full path again. Nav2's bounded closest-pose search
could no longer find the robot along that path and rejected the transformed
path as empty. A cancelled executing path must therefore be used only to settle
the pending reservation, then replaced by a fresh MGG plan from the current
pose. It must never be replayed into a new FollowPath action. Fleet acceptance
remains pending for this follow-up.

Source deployment now uses the `planning-refactor` Git branch on benchbot:
commit and push locally, then pull with `--ff-only` in the simulation workspace
before rebuilding. Runtime deployment settings remain local. The initial Git
rollout, `0e1704e`, produced colored chunks for all four robots and retained one
server/simulation stack with the UI on port 15173. This verifies RGB data
availability, not camera calibration or complete fleet exploration.

Zero-stamp guards passed 29 focused bridge tests and the ROS input smoke. They
remain useful protocol hardening because TF2 interprets zero as “latest,” but
the pinned simulator increments its clock before its first populated render;
the guard did not resolve the fresh-mission failure and is not its cause. A
Navigate/Home comparison made from different robot frames was likewise not
diagnostic. Separately, when R2 later stopped at world approximately
(-14.47, 3.92), the 56.3 cm padded query overlapped a real pavement rise:
the full-material static span was 21.14 cm. Recovery from that conservative
current-footprint rejection is a distinct case and must not be addressed by
weakening terrain validation.

## MOLA graph integration follow-up

The opt-in `mola_snapshot` backend now supplies MGG graph construction from the
native planner product through `MapInterface`. A bounded background loader
validates and publishes immutable component-frame indexes. Planning transactions
hold one generation, and a route is discarded if that generation changes during
an external corridor query. MOLA mode requires the exact-surface query service:
20 cm voxel centers alone cannot enforce a 10 cm platform step limit.

The startup connector preserves the physical odometry and Home anchors. It can
bridge a bounded missing-floor region only when the full swept body has observed
clearance and measured terrain remains within the platform limits. Historical
Home clearance must be observed; there is no unknown-footprint exemption.

A Bistro trial exposed another mismatch: graph construction checked a lattice
sample's body before projecting it to driving height. The projected waypoint
could lie in unknown space and be rejected immediately by route refinement.
Fresh Navigate graphs now check that projected endpoint before selection.
Explore retains its legacy candidate policy; complete graph/grid/controller
policy unification and persistent Home connectivity are still acceptance work.

The next physical-body clearance change must preserve observed-free evidence
above each support surface. Neither skipping the bottom occupied voxel nor
retaining only exact hit heights establishes clearance inside that voxel. The
reviewed experiments were withheld for this reason: occupied-only captures can
otherwise certify an unobserved body volume, and a curb can share the floor
voxel. A replacement product/query needs qualified per-column ray or sub-voxel
free-volume provenance, a small bounded contact-noise allowance, and a bounded
query-cell budget. Include occupied-only captures, same-cell curbs, overhead
obstacles, tilted frames and voxel-boundary cases before planner integration.

Live map goal dispatch also needs a publication fence: geometry, live overlays,
and a new click must refer to a coherent accepted map correction. A correction
arriving between geometry and telemetry updates must reject stale clicks, rather
than silently reinterpret their coordinates. This fence is independent of the
adapter's tolerance for metadata-only updates during an already active goal.
