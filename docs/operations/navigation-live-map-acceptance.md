# Navigation and live-map deployment acceptance

The running deployment is `road-home-b511457`, using one benchbot stack on UI
port 15173. All 228 native tests pass, and R2 completed a Navigate/Home round
trip through the side corridor. R0's raised-road recovery, R3's latest Home
timeout and the source of R2's earlier endpoint terrain rejection remain open.
See the [latest arrival results](#road-search-candidate-arrival-results) for measured
errors and exact image evidence. Earlier runs below are retained for comparison;
their mission IDs, image tags and limits describe those runs only.

Endpoint errors in the historical arrival logs through `b511457` describe the
last observed nonempty path in XY, not the maximum error throughout a trial or
full XYZ/yaw identity. The current harness retains maximum qualified XY errors
and rejects malformed or non-finite endpoint evidence.

## September 13 navigation follow-up

The September 13 run used benchbot project `planning-next` on UI port 15173,
with one server and one ARGoS simulation. Mission
`bacca582-d826-4581-a825-e992956f8a98` uses ROS domain 201. Native MOLA mapping
is active, while MGG uses `cloud_octomap`; indexed corridor validation remains
disabled. This run qualifies the simulated cloud planner, not the MOLA planner
backend or hardware navigation.

| Trial | Full route | Result | Time | Remaining distance |
| --- | --- | --- | --- | --- |
| R0 road, 6 m | 21 poses | Arrived | 11.15 s | 0.218 m |
| R0 Return Home | 19 poses | Arrived | 11.45 s | Not recorded |
| R0 road, approximately 12 m | 41 poses | Arrived | 27.33 s | 0.240 m |
| R1 road, approximately 12 m | 42 poses | Arrived | 21.56 s | 0.210 m |

Distances are measured in the navigation component frame; simulated odometry
drift means these are not ground-truth positioning errors. Both long trials
used one FollowPath action, with no controller cancellation. R0 had no route
invalidation; R1 remained in `following_final` throughout execution. Test goals were cleared
afterwards. The R0 trials preceded a request-local cache optimization; R1 ran
with the final image. Acceptance artifacts are retained on benchbot as
`/tmp/swarmdeck-unknown-r0-six-meter-arrival.json`,
`/tmp/swarmdeck-unknown-r0-return-home.json`,
`/tmp/swarmdeck-unknown-r0-long-road-arrival.json`, and
`/tmp/swarmdeck-unknown-r1-long-road-arrival.json`.

Static Bistro geometry explains one misleading straight-line rejection: the
point 12 m south of R0's deployment, world `(-12, -8)`, sits on a roughly
12–16 cm curb, beyond its 10 cm step capability. The successful long route
instead reaches road at `(-10, -8)`. The current generic ground-support failure
message can describe incompatible known terrain as well as missing evidence.

Images used for this run:

- MGG: `swarmdeck-mgg:terrain-cache-final-review`,
  `sha256:55b5c6170824e166028a4863e71452fe079bc16ec5fc91d2fb37234f16773f2f`.
- Adapter: `swarmdeck-sim:unknown-final-review`,
  `sha256:37a1feb67e26e1cffd5441b7dbe7b6c97084fe3a73bb47194c490116e1f583e1`.
- Mapping/query: `swarmdeck-mapping:navigation-map-review`,
  `sha256:906419185e30cb2e9d49e17a414858aeadef2b43c5c29d08198199f358407891`.

The final native image passed 247 tests with no failures or skips. Focused
objective, exploration and launch Python suites passed 137 tests. The live ROS
route-validation smoke passed across the adapter and MGG images, including
rejection of a mismatched mission. All 20 Nav2 lifecycle nodes became active.
An independent final agent review was completed and its findings addressed.
Fleet-wide exploration completion, broader return-home coverage and hardware
qualification remain outstanding; the conservative footprint envelope also
needs further ramp testing. Earlier failed trials remain below for comparison.

### Unknown destinations and curb avoidance

The simulated cloud pipeline selects
`objective_ground_evidence_policy=provisional_unknown`. Navigate and Return
Home retain the requested XY destination and may cross unobserved ground at
the current physical driving height. A raised endpoint cannot create an
inferred ramp through unknown space: observed surfaces are projected to their
measured height and checked against the platform's step limit. Known body
obstacles, footprint terrain and geofences still veto the route. The policy
does not add inferred free cells or claim indexed-map validation. Hardware
and MOLA retain strict evidence requirements.

Repeated footprint projections share an exact-coordinate cache within one
locked planning request. The cache is discarded before the next request, so
new map hazards cannot reuse old results; strict evidence modes bypass it.
The objective deadline remains 500 ms. R1's long road query completed in
24.7 ms after this change and a planner restart. This is an observed latency,
not a controlled speedup measurement: the restart also reloaded the map.

Exploration previously returned its graph path before explicit-objective
terrain refinement. Its candidate and neighbour edges now receive the same
known footprint, step and swept-body veto before selection, with a final
check after shortcutting and interpolation.

`ValidateObjectiveRoute` checks the next 3 m of an accepted Navigate/Home path
against the latest native map. The adapter polls at most once per second with
one request in flight. Only `INVALID` cancels the matching controller action
and requests a full route to the retained destination. `VALID`, unavailable
queries, timeouts and old action results do not trigger replanning. Requests
bind mission, component and planning frame; ambiguous route progress is
reported unavailable. Native work is bounded by path size, relevant vertices,
query limits and a cooperative 100 ms deadline. This complements Nav2's local
obstacle response; unavailable validation is not a certificate of safe ground.

Navigate/Home and exploration allow three physical no-progress failures total
(the initial execution plus two replacement attempts). The classifier uses
the controller's explicit `Failed to make progress` result. Planner rejection,
transport failures and `NO_VALID_CONTROL` do not consume this movement budget.
Each recovery has its own bounded planning window; time spent executing a
route does not exhaust the next recovery window. Fleet retains the terminal
controller or planner reason.

The live, non-motion integration check is
`adapters/test/ros/mgg_route_validation_smoke.py`. Run it in the sim container
with the ROS and MGG message overlays sourced; alternatively copy it into the
MGG container and source that planner's install. It requests a complete route,
checks the exact destination, validates its lookahead through DDS, and verifies
that a different mission is rejected. Native fixtures separately cover newly
known curbs, walls, geofences and invalid query bounds.

## Earlier deployment history

The reviewed `planning-refactor` changes were uploaded and deployed on benchbot
on 2026-09-10 (2026-09-11 UTC). Navigation acceptance **did not pass** that run.
All benchbot services were subsequently stopped at the operator's request on
2026-09-11; the image identities and results below describe the recorded run.

## Deployment

- Host: `sroy@benchbot.yannbouteiller.com`.
- Workspace: `/home/sroy/workspaces/swarmdeck-exploration-review`.
- Compose project: `planning-next`; UI 15173, API 18080, SLAM 18090, media 8190.
- Fresh mission: `6f6afc5c-9a34-4eb4-8243-731629872d25`, ROS domain 201.
- The run used one server and one ARGoS simulation, with four connected robots.
- Native MOLA mapping is enabled; indexed corridor validation remains disabled
  for this run (`SWARMDECK_INDEXED_MAP_QUERY=0`). Odometry uses simulated drift.

The planner image is `swarmdeck-mgg:partial-route-review`, image ID
`sha256:b8586a1f1c0e117182331a4071f33ceb035e6d0e1ee6dd517ad3d257a8b26fbc`.
The native patch SHA-256 is
`1776e664d277b10447160e38159a0942379b61c84ca9a54d0b716b608248b5fe`.
The adapter image is `swarmdeck-sim:navigation-map-review`, image ID
`sha256:32fe94d2f4efcc4bcdaf66c70b526f9d1f41e85749a667b20c91599c978d1ceb`.
The mapper/query image is `swarmdeck-mapping:navigation-map-review`, image ID
`sha256:906419185e30cb2e9d49e17a414858aeadef2b43c5c29d08198199f358407891`.

All three installed `PlanObjective.srv` definitions have matching SHA-256
`908deae6d3915e500eb4c51a384ea9f5c49da64f2e276a2122b09d599ec05525`.
Server/adapter source and the built UI are mounted from the reviewed export.
Unrelated local changes were excluded from the upload.

The planning-test overlay explicitly separates the optional FAST-LIVO2 service
from the ARGoS profile, because this overlay uses synthetic odometry. This fixes
an initial `up` failure caused by attempting to pull the absent estimator image.
The normal `bash compose.sh up -d --no-build` command now succeeds.

## Validation

- Native MGG core/ROS suites: 185 passed, no failures or skipped tests.
- Focused Python integration suites: 229 passed before the final telemetry fix;
  the affected objective/session/live-map suites then passed all 71 tests.
- UI replica, acceptance and 3D suites: 11 test files passed; Svelte checks and
  production build passed.
- Real browser: automatic Local live source displayed 4,096 points, 3,245 voxels
  and the selected robot marker. Points, mesh and voxel modes remained available
  after 2D/3D switching. The ceiling retained its selected 1.18 m value.
- Global showed an explicit waiting-for-inter-robot-alignment state; it did not
  relabel the selected local map as a unified map. Historical components did not
  replace the active mission. Gaussian rendering remained unavailable because
  this mission has no reconstruction artifact.

## Failed motion acceptance and next investigation

R0 received component-frame goals 3 m and 12 m ahead, followed by Return Home.
All three were rejected before movement with:

```text
current pose rejected: no mapped ground support at (0.01, 0.00, -0.00)
```

The initial goal remained visible during planning; the rejection produced an
explicit failed state. The trials do not qualify route continuation, terrain
avoidance or Return Home.

A subsequent read-only inspection identified a near-field support blind spot.
R0's MGG odometry was `(0.029, 0, -0.002)` with physical body height 0.40 m,
projection offset 0.475 m and map resolution 0.15 m. Ground projection searches
the current column and lateral offsets of only 0.30 m. The Bunker lidar is
approximately 0.72 m above the floor; its lowest 15-degree ray first intersects
flat floor about 2.69 m away. The forward camera also does not observe the
underbody columns. The initial graph anchor consequently remains awaiting
mapped support. Broad floor coverage does not supply evidence beneath the
current footprint.

The next correction should investigate the existing hanging local-root
convention and require a map-refined, collision/unknown/step-checked connection
to supported terrain before admitting an objective. Home-anchor promotion
requires equivalent evidence. Enlarging the projection search to snap the robot
several metres away, or inventing floor occupancy, would not establish a valid
connection. No such change was included in this deployment.
Stop All was sent after each trial and all four robots ended stopped.

The broader implementation and remaining acceptance cases are tracked in
[the navigation and mapping plan](../architecture/navigation-mapping-issue-plan.md).


## MOLA graph-provider milestone follow-up

The next reviewed planner image is `swarmdeck-mgg:mola-graph-review`, identity
`sha256:0f7cc98d6a88ff3e047f752fd8ce660aca6b625f5a2e3b92634374ab7f5309be`.
It includes the partial-route patch above, followed by:

| Patch | SHA-256 |
| --- | --- |
| Startup support | `09f8eb5ae64db57b83bd5226d6325497ad926ff71c5046ec0c739f9c7e3c1661` |
| MOLA map provider | `51999a573e544ee8390f1794f5c34573ff7b7231b030a26f62b8ebcfc2d0b659` |
| Projected Navigate endpoints | `690a22a076dcad7331f15736d0aec344921ac469ac83e3298ff9890381221e29` |

The native core/ROS suites pass 216 tests with no failures or skips; the applied
source was compared with the reviewed patch sequence. The actual MOLA worker
and MGG probe also pass initial, reused-artifact and corrected-publication
checks using `tests/deployment/mola_mgg_acceptance.sh`. The fixture exercises
free, occupied and unknown cells. Small fixture loads took 7–11 ms; these are
not worst-case map budgets.

MOLA graph construction is opt-in. Its final corridor check must use the exact
terrain query service even when the legacy cloud-mode query flag is disabled.
The native graph's 20 cm voxel representation alone cannot establish a 10 cm
step limit. The new startup connector preserves the physical anchor and permits
only bounded missing-floor support with observed body clearance. Navigate now
checks the body at its final ground-projected endpoint.

### Live graph and motion evidence

In mission `e5e13bd9-4097-4161-9151-4c19aa25c569`, the initial MOLA graph request
was unavailable while index and snapshot publication disagreed. Subsequent
read-only requests loaded maps for all four robots:

| Robot | Graph result | Build time |
| --- | --- | --- |
| R0 | No vertices; 871 candidate edges reported as occupied | 19 ms |
| R1 | No vertices; 908 reported occupied and 32 steep edges | 25 ms |
| R2 | No vertices; 321 reported occupied and 140 steep edges | 17 ms |
| R3 | 780 vertices, 7,953 edges, 26-pose selected path | 2,056 ms |

The current collision diagnostic combines unknown clearance, occupied volume
and some support failures, so these labels do not establish actual obstacles.
Split aggregate rejection counters are needed before changing admissibility.

R0's unchanged solver results still advanced its graph revision roughly every
10 seconds. The snapshot preceded its matching MOLA index by 1.2–1.6 seconds;
completed pairs had matching identities and source hashes. This explains the
transient fail-closed availability gap. A bounded retry within the existing
load deadline and a worker-qualified ready-publication key remain follow-ups.
Identical authority heartbeats reuse the tree but still revalidate the 16 MB
artifact in this trial; lightweight reference checks could reduce that work
without accepting a deleted, replaced or stale artifact.

R3 spent 1,758 ms computing gain. This establishes live MOLA graph ingestion,
not four-robot navigation acceptance. The other graph rejections and the cost
of gain evaluation remain investigation targets.

Earlier motion trials in that mission used the startup-support image before
the projected-endpoint follow-up. Cold 3 m and 12 m Navigate requests reached
refinement but failed on unknown body clearance; Home lacked mapped support.
A 120-second Explore trial moved all four robots, with sampled net displacement
of approximately 28.55, 2.88, 18.45 and 31.35 m. R1 became blocked. Warm Navigate
moved R0 about 1.4 m before exhausting three recovery attempts; the far goal
and Home still failed on graph connectivity. Stop All left every robot stopped.
Exploration motion is not evidence of correct completion or obstacle detours.

### Interface and review findings

Browser checks verified Local geometry growing from 4,096 to 60,000 points,
visible selected robot, and points/mesh/voxel modes across 2D/3D transitions.
Global explicitly waited for verified inter-robot alignment. Gaussian mode was
unavailable because this mission contained no reconstruction artifact.

The ceiling range now uses a fixed 5 cm step origin; fractional map bounds no
longer move the browser's slider quantization. Its selected 1.20 m cutoff
survived live updates and view switches. Camera HLS playback reached 320×240
with advancing frames. The initial Connecting state in the tunnel was the
12-second WebRTC probe before the working HLS fallback.

The final quality pass identified a correction race between displayed geometry,
live robot telemetry and clicked goals. These now carry the accepted
`solution_order`; the server and adapter reject a newly dispatched click after
a correction, while ordinary publications in the same frame remain usable.
Historical maps without a known correction order remain read-only.


### Final combined-image acceptance

A fresh mission, `1492f645-b702-4a11-a94a-ba4a47b76677`, ran the final planner
image above with `cloud_octomap` to qualify the projected-endpoint follow-up,
plus the reviewed server/adapter frame fences and rebuilt UI. All four robots
published qualified live telemetry. The 3 m and 12 m goals both reached grid
refinement, then failed with `no observed traversable grid detour for route
segment 0->1`. R0 did not materially move. Home again failed with `no mapped
ground support at (0.01, 0.00, -0.00)`. These results do not qualify nearby,
long-range or Home navigation; the earlier endpoint check fixes one mismatch
but does not establish a traversable swept corridor.

The final combined Python deployment/adapter/server regression run passed
158 tests. Fleet/replica/browser-contract UI tests passed; Svelte checks found
zero errors or warnings and the production build succeeded. The independent
quality pass was reviewed before deployment. Actual browser acceptance passed
Local geometry, selected robot marker, points/mesh/voxels, 2D/3D round-trip,
1.20 m ceiling persistence and Global's explicit alignment wait. Stop All was
sent after every motion trial and all four robots ended stopped.

Final deployment state: the simulation, interface, peer and native test
containers were stopped. `docker ps` returned no running containers on benchbot;
ports 15173, 18080, 18090 and 8190 were closed. Images, exported source and map
volumes were retained for the next acceptance cycle.

## Waypoint execution and clearance follow-up (2026-09-13)

The next baseline run used mission `3e8581c2-bd5c-4f6a-85be-6273f7f53ec8`
on the same 15173 deployment. During an operator Navigate trial, R2's controller
logged 16 client-requested cancellations before its final arrival, with no
intermediate successful segment completions. The captured interval also contains
one R1 cancellation. This is premature cancellation, not evidence that MGG limits
each path to one metre. Code inspection identified the active objective's 2 cm
map-correction threshold as a source of repeated cancellation.

The adapter now separates initial route validation from execution monitoring.
Initial/RPC/dispatch checks retain their strict tolerances. During execution,
route-point displacement is measured from the originally accepted transform
against a configurable 25 cm budget; small successive corrections cannot reset
that baseline. The independent 0.02 rad frame-rotation guard, stale-authority
rejection and mission/component checks remain. Cancellation diagnostics report
both displacement and rotation. This does not establish obstacle clearance:
Nav2's live local costmap remains responsible during execution.

Explore also ignores unsolicited replacement paths and planner terminal states
while its current controller action is active. Operator Stop, authority loss,
coordination revocation and failed-action recovery retain their existing paths.
Peer reservation rejection now records its cause and winning peer using the same
arbitration decision, rather than a second potentially inconsistent lookup.

Two separate simulation defects were found. R0's lifecycle manager could wait
indefinitely after a configure transition took effect but its service response
was lost. Simulation now uses one bounded startup owner that queries actual
states before deciding the next transition. An isolated ROS service smoke test
passed the delayed-response case without duplicate transitions. This is startup
recovery, not ongoing lifecycle supervision. Proximity scans now use canonical
robot mount geometry; the old Bunker and Scout heights could discard real low
obstacles. Ground filtering is capped at 15 cm for Spot so it retains Scout
chassis returns, and at 10 cm for Bunker/Scout. Full step traversal still requires
terrain and clearance evidence. MGG's simulated gain FOV matches the selected
3D lidar; unsupported planar profiles fail explicitly at launch.

The native rejection-diagnostics image passed its core/ROS tests and has identity
`sha256:47b3dd493b76c6d8e6b6ae673c6038851153f8a612030394d211ef27d2b19af0`
(`swarmdeck-mgg:diagnostics-review`). The combined local Python regression run
passed 281 tests. A proposed physical-body change was rejected during review:
exempting a whole floor voxel can hide a curb sharing that voxel. Exact surface
provenance is required before integrating an alternative ground-body query.

### Reviewed deployment and waypoint trial

The final fixes were uploaded through benchbot's configured Cloudflare SSH
proxy. Direct TCP SSH to the hostname is not its working access path. A fresh
mission, `08e34c3d-c4e7-4d70-bafd-7b0f2c4a0539`, uses:

| Image | Identity |
| --- | --- |
| `swarmdeck-sim:clearance-review` | `sha256:5889b05816118aca0454e40481aa07b9d245323e1136f01b213fd94d56ca79fd` |
| `swarmdeck-mgg:calibration-review` | `sha256:b86651942286aeab3b1aafb505234ffe3ddfcf39bd181d192d983669d802109b` |

The latter adds the matching canonical robot specification to the tested native
diagnostics image. Its first launch exposed an older baked-in specification;
rebuilding that layer corrected the launch failure before motion acceptance.
The normal repository Dockerfile already copies the canonical simulation source.
All three participant `PlanObjective` contracts match, and the running objective,
exploration and coordinator source hashes match the reviewed local files.

All 20 Nav2 lifecycle nodes were confirmed active with `GetState` after staggered
startup. A 90-second Explore warm-up produced successful arrivals 0.202–0.234 m
from their endpoints. Three active Explore cancellations were explicitly caused
by material-correction invalidation of peer reservations, rather than unsolicited
path replacement. R1, R2 and R0 also encountered controller progress failures;
R2 exhausted recovery. Those failures remain unresolved.

R0 then received a Navigate goal 7.779 m away on its just-traversed road. It
completed one partial segment and then the final segment in 25.95 seconds,
finishing 0.213 m from the component-frame goal. Controller logs show two
successful segment completions and **zero client-requested cancellations**.
Between the recorded before/after authorities, translation changed by 0.105 m,
rotation was unchanged, and solution order advanced from `[36, 0]` to `[38, 0]`.
This trial exercises correction retention beyond the former 2 cm threshold.
It supports the waypoint execution fix, but does not qualify Home connectivity,
all obstacle detours, step avoidance or fleet exploration completion.

Stop All was sent after each trial. One server and one four-robot ARGoS simulation
remain online under the `planning-next` project, with the interface on **15173**.

## Complete routes to selected destinations (2026-09-13)

The earlier local-proxy implementation is superseded. Navigate now uses
persistent global topology and bounded accumulated-map grid completion, without
rebuilding the local exploration graph or calculating exploration gain. Missing
graph connections can fall through to map search; a blocked persistent prefix
can retry direct search using the remaining refinement deadline. No partial
Navigate path is dispatched. The adapter also rejects partial Home responses
and claimed-full endpoints more than 1 mm from the requested XY. Supported
terrain may change endpoint height. Home retains its existing persistent graph
policy rather than receiving Navigate's broader search budget.

The native patch is `mgg-complete-distant-route.patch`, SHA-256
`0786da214865d51b55619d89d380c71b830bcf894b6f2f2c1d95a7886eefa06d`.
Image `swarmdeck-mgg:complete-route-review` has identity
`sha256:a2d5245250d973771792424df784baf5a699eb9290198a2cd1eeeb475a67a9c0`.
All six native packages built, and 230 core/map/ROS/PCI tests passed, including a
30 m exact service response, a long obstacle detour, an unknown gap, platform
step limits and existing snapshot/ownership cases. The combined local
adapter/server/coordination/deployment run passed 161 tests; a final telemetry
compatibility adjustment passed all 28 affected server tests. Independent review
preceded rollout.

The adapter submits one complete controller path and has no partial-segment
continuation state. Unchanged and translation-only map transforms no longer
require scanning every route point during execution monitoring. Full display
paths retain both endpoints within the existing 200-point telemetry bound;
explicit empty paths clear an old goal's cached route during replanning.

One qualification limit remains in the existing ground projection: an endpoint
seed below a raised surface can be rejected. The Spot step fixture supplies the
correct raised endpoint height and retains below-cap traversal coverage. This
change does not relax the collision model or infer unknown terrain as free.

### Live complete-route acceptance

Mission `3635493c-e251-46cf-aa45-e3a1c3abe9f0` runs the complete-route image above
and the existing reviewed simulation image `5889b058…`. Runtime source hashes
matched the reviewed adapter, display helper and server registry files. All
20 Nav2 lifecycle nodes reported active. After a 90-second Explore warm-up,
R0 was sent a single component-frame goal at a previously traversed road point
12.168 m away.

The displayed route contained 50 poses and reached the selected destination;
telemetry reported only `following_final` before terminal success. R0 finished
0.159 m from the component-frame goal. Controller logs, restricted to R0 after
the request, contain exactly **one goal received, one goal reached, zero
cancellations and zero partial-segment continuations**. Controller execution
took 22.21 s; the API observer recorded success at 26.45 s including observation
overhead. This qualifies one full-route Bistro Navigate case beyond the former
local graph horizon; it does not establish that every terrain/unknown-space
failure or Home route is resolved.

Stop All ended the trial. One `planning-next` deployment remains online on
benchbot, with the interface on **15173** and four stopped robots. No second
server or simulation was started.


### Forward-goal regression investigation

The successful 12 m trial above followed previously traversed ground. It did
not qualify forward navigation into the visible road. A subsequent stationary
R0 probe reproduced immediate rejection: 1 m and 3 m requests succeeded; 6 m
failed body-known clearance and 12 m failed ground support. Raising the goal
seed by 0.3 m did not change either failure. R2 independently passed 1 m and
3 m, then rejected 6 m for missing support and 12 m for unknown body volume.
These failures precede A* and cannot be fixed by raising its search budget.

The simulation camera publishes valid finite depth up to its 40 m far plane,
including clear-to-range background readings. The relay discarded all depths
at or beyond 20 m, although OctoMap can truncate those observed rays at its
20 m integration range without marking their endpoints occupied. One live R0
frame lost 545 of 4,800 sampled rays this way. Sampling every fourth pixel also
leaves gaps in forward support and clearance evidence. The original native
30 m fixture pre-cleared a free prism; it cannot qualify this sensor pipeline.

The relay correction preserves valid simulation rays, keeps invalid hardware
depth unknown, and explicitly binds the simulation map range below the camera
far plane. Sensor-faithful tests and ordinary forward controller trials are
required before claiming general navigation acceptance. The raised planning
body envelope and sparse long-range ground support remain separate terrain
model limitations; this relay change does not resolve those by declaring
unknown cells free.

The qualified simulation relay now samples depth at stride 2 and adds bounded
surface samples only inside full-resolution, four-pixel depth quads. Accepted
quads require finite observations, a surface normal within 5 degrees of level,
no edge/diagonal longer than 3 m, and at most 8 cm vertical variation. The latter
admits the measured 6 cm road undulation without admitting the Scout's 10 cm
step limit. Invalid pixels and the 40 m background sentinel never create floor
support. Hardware retains its existing stride and depth admission policy.

With the same R2 pose and camera observations, 6 m and 12 m road goals changed
from support/clearance failures to complete 21- and 41-pose paths in roughly
1–2 ms. A controller trial then exposed an independent frame problem: local
SLAM's map-to-odometry correction canceled a valid route after 3.6 m. A repeated
12 m trial arrived within 0.146 m in 20 s, but still replanned as the map gauge
moved. The transform recording showed 1.11 m map-to-odometry translation with
no graph correction (`solution_order=[0,-1]`, `correction_revision=0`). Separately
sampled TF and authority messages can disagree transiently; consumers must not
infer a correction from their unsynchronized product.

The onboard simulation overlay therefore selects `{robot}/odom` through
`SWARMDECK_PLANNING_FRAME_TEMPLATE` for cloud accumulation, MGG, peer route
reservations, objective guards and FollowPath. cSLAM publishes the component-to-
planning transform atomically alongside the existing UI map transform. Display
copies of the complete route and goal are converted back to the current map
frame; the controller's poses remain unchanged. Genuine component corrections,
stale authority, changed mission/component identity and obstacle checks remain
active. Hardware keeps its configured map frame unless this option is explicitly
set and its continuous odometry/TF chain has been qualified. Changing the frame
requires restarting MGG so old accumulated voxels cannot retain the former frame.

### Ground navigation evidence in the cloud simulation

Cold-start probing also found an inconsistent evidence requirement: the
physical starting anchor could be admitted without dense air observations,
while every outgoing sweep required all those air voxels to be observed free.
In one R2 request, 339 projected positions were supported but all 338 attempted
connections failed at the same unknown near-start body volume. The clearance
box extends above the physical chassis, so clearing the robot's own volume
cannot resolve this sensor-coverage gap.

Bistro's simulation cloud backend explicitly selects
`objective_body_evidence_policy=observed_ground`. Regular route samples still
need mapped ground and must satisfy step, slope and geofence limits. Any
occupied voxel in the continuously swept clearance box rejects the route;
unknown air alone does not. A bounded connection from the exact physical start
to the first observed support handles the existing near-field ground blind
spot. It cannot replace the requested destination with a nearby waypoint.
This policy does not write inferred free voxels into the map.

`strict_volume` remains the native default. The launch selects the ground policy
only for simulated depth with `cloud_octomap`; hardware and `mola_snapshot`
use strict volume checks. This is a ground-navigation evidence contract, not
proof that every part of the 3D body volume has been observed. An unseen
obstacle or overhang remains a limitation until sensors observe it; Nav2
provides runtime obstacle response. Indexed-map evidence reporting is governed
by the separate exact-snapshot query, not by this policy setting.

### Terrain precision and physical stalls

Complete-route trials separated a later physical blockage from the earlier
immediate planning failures. During a fresh R2 trial, both Nav2 and the ARGoS
driver commanded 0.6 m/s for roughly ten seconds while the robot remained
stationary. The action then failed its progress check. R1 traveled about 10 m
of a 12 m route before a similar stop. These trials retained the complete
destination, but did not qualify arrival or obstacle avoidance.

Two terrain checks were missing: ground projection normally returned its first
center support hit without comparing the wheel footprint, and the legacy
step/inclination conjunction could accept a 15 cm curb sampled over 30 cm as a
26.6-degree incline. The simulation objective policy now checks known terrain
across a yaw-independent footprint envelope, including the bounded initial
connector, and enforces the per-sample step cap independently. Lateral columns
without an observed hit remain neutral; ordinary route support is still
required. Invalid or oversized queries fail before ray traversal. This policy
does not certify unseen pits or obstacles.

The footprint envelope conservatively encloses every robot orientation. Its
center-relative height limit can also reject continuous ramps that the physical
robot could climb; the configured 30-degree inclination is therefore not a
guarantee of usable ramp slope in this Bistro policy.

A full 5 cm occupancy map was tested and removed after it caused ten-second
request timeouts and exceeded 10 GB across the four planners. Occupancy remains
at the existing coarse resolution. A separate, opt-in surface-height record
retains the maximum measured endpoint Z per occupied voxel, allowing terrain
checks to use observed heights instead of 15 cm voxel centers. Generic collision
queries and disabled backends retain their existing behavior. Height records
survive isolated miss observations while their voxel remains occupied, then
clear with occupancy, explicit free-box updates, or map reset. Range-clipped
background endpoints never create height records. Maximum height is deliberately
conservative and can retain upward measurement error until the voxel clears;
the option is enabled only for the simulated cloud backend.

Controller errors now pass through adapter telemetry and objective decoration
to `nav_failure_reason`, with a 512-character display bound. Planner rejection
reasons take precedence, and generation changes prevent old action results from
overwriting a replacement goal's status.


## Local regression and rolling Home checks (2026-09-14)

The local checks used `ec84488` plus the authority-heartbeat and Home-handoff
corrections and ROS contract smoke described below. The native MGG workspace was rebuilt from the
committed patch set, including the matching `RefineObjectiveRoute` message ABI.

| Check | Result |
| --- | --- |
| Native MGG build and tests | Six packages built; 21 CTest targets, containing 242 GTest cases, passed |
| Server, map, replica, navigation and adapter regressions | 255 passed |
| UI regression scripts | 14 test files passed |
| Svelte/TypeScript and production bundle | Zero check errors/warnings; build passed with the existing chunk-size advisory |
| Native MOLA bridge and framework module | Five CTests and persistent JSONL smoke passed |
| Final objective/authority/peer-launch regressions | 92 passed after both corrections |
| Rolling Home ROS service contract | PlanObjective and two RefineObjectiveRoute handoffs passed with generated messages |
| Authority heartbeat with frozen simulated time | Actual rclpy executor smoke passed |
| Black | Repository formatting check passed |

The rolling Home smoke is
`adapters/test/ros/mgg_rolling_home_smoke.py`. Run it with the rebuilt MGG ROS
message overlay sourced, the repository on `PYTHONPATH`, and an isolated DDS
domain. It uses real rclpy services and generated messages with an inert path
sink. It verifies native-evidence provenance, a shared component, an immutable
global Home route, two local-section handoffs, and final cleanup. It does not
substitute for a native-planner or physical-motion trial.

The initial four-robot Bistro run used software Vulkan because no usable GPU
was available. A bounded sample measured a real-time factor of 0.036; odometry
arrived, but no authority heartbeat arrived during the 15-second probe. This
exposed mixed clocks: the bridge scheduled authority publication every simulated
second, while consumers expired it after three wall-clock seconds. At that
speed, the nominal publication interval was about 28 wall-clock seconds.

The snapshot/authority timer now uses steady wall time. Sensor freshness and
consumer expiry remain on monotonic wall time with the existing three-second
limit; paused or disconnected sensors therefore still lose authority. TF and
capture timestamps retain their original clock semantics. Snapshot files and
envelopes remain revision-gated, so unchanged geometry is not rebuilt every
wall-clock tick. The actual ROS smoke freezes `/clock` and verifies that the
steady timer fires while ROS time remains zero.

The reduced live trial retained the Bistro collision world, Bunker footprint,
full VLP16 lidar, MGG cloud backend and Nav2 controller, with one robot, drift
odometry and a 160×120 camera. Its measured real-time factor was 0.368; all 13
authority messages received during a 15-second probe passed validation. A native
three-metre route request took 20.1 ms, and its route validation took 12.2 ms.
Camera reduction changes depth sampling, so this trial cannot qualify thin
depth-only obstacles, production camera fidelity, or multi-robot coordination.

R0 reached the first 12-metre destination and reported success after 128.6 wall
seconds, with 11.905 m of reported displacement. A post-arrival component query
returned HTTP 404, so that first harness did not pass its final map-query check.
Stop All did pass. Return Home then followed the first approximately eight-metre
graph section before failing the final-chunk endpoint check by 0.018 m.

The native route retained its original goal, while the adapter replaced its
comparison target with the latest slightly corrected Home landmark. Corrections
inside the existing execution deadband intentionally preserve a route; they must
also preserve its bound endpoint. Final refinement now checks the retained Home
goal. Material corrections and identity changes still require replanning, and
the strict one-millimetre returned-endpoint check is unchanged. All 77 objective
planning tests passed, including the observed 18 mm correction case.

Headless Chromium connected to the actual local backend, loaded live 3D geometry
and both rendering workers, and switched successfully to a correctly sized 2D
canvas without JavaScript exceptions. These are activation and loading checks,
not a pixel-alignment or graphics-performance qualification. Video was disabled
for the navigation trial; its unavailable-stream response was expected. Optional
agent endpoints also returned 502 without preventing map/navigation UI loading.

The intermittent live-component 404 is separate from the fixed heartbeat issue.
Authority can advance to a new optimizer solution before the replica worker
publishes the matching component. Live overlays require exact solution-order
agreement and deliberately reject that interval. A later bounded metadata read
recovered with HTTP 200 and 0.654-second telemetry freshness. Read-only acceptance
probes should retry briefly; the frame-consistency check must remain strict.

### Fresh-mission round-trip result

The repeat trial with both fixes passed: Navigate reported success after 86.3
wall-clock seconds, with 11.988 m of reported displacement. Return Home followed
the initial local section of its retained global graph route, refined the next
section, entered `following_final`, and reported success after 100.3 seconds.
Its reported displacement was 11.702 m. Stop All was verified after both actions,
and both bounded command harnesses exited successfully. These are action and
odometry observations, not an independent simulation-ground-truth error bound.

At the end of that trial the local deployment had one server, one simulation
and one UI at port 15173, with the single robot stopped. Four-robot Bistro motion, independent
truth-based arrival measurements, inter-robot closures, dynamic obstacles and
full-resolution rendering still require their separate acceptance trials.

## Distant-goal budget investigation on benchbot (2026-09-14)

Subsequent arbitrary-goal trials exposed a separate failure: the native grid
search used its entire 500 ms cooperative deadline and returned `BLOCKED`.
That result establishes budget exhaustion, not the absence of a route. Local
failure evidence contained only 46–106 expansions despite hundreds of terrain
queries. Exact-goal terrain rejections are a different failure and must retain
their own diagnostic and safety checks.

The directed traversal cache now reuses completed terrain checks within one
request and map snapshot. Its key includes both full endpoint states in order;
reverse edges remain distinct. It preserves the checked polyline, never caches
an interrupted query, and has an explicit entry limit. The native core CTest
targets passed, including the repeated-edge regression that fails without the
cache.

Navigate now refines a useful connected graph corridor first, extending its
checked route to the exact operator destination. Missing topology permits a
direct grid search. A rejected graph corridor may fall back to direct search
only within the same remaining deadline. Home retains its persistent graph and
rolling local refinement. The native ROS CTest targets passed, including a
helpful graph prefix with an unknown exact suffix and a blocked graph with a
valid direct alternative.

The adapter treats this specific native deadline result as temporary, retaining
the original goal and authority binding during bounded recovery. Mission or
component changes invalidate that binding; a delayed response cannot silently
adopt a new authority. Recovery is limited to the existing three attempts and
15-second window. Exhaustion reports the planning-budget failure rather than
claiming the goal is unreachable. Actual terrain rejection remains terminal.

Benchbot runs the four-robot GPU Bistro scenario with full camera resolution.
The source is pulled from `planning-refactor` into the existing review checkout;
the unrelated modified `main` checkout is preserved. A read-only R0 request
20 metres forward and 2 metres right exhausted the 500 ms budget. With a
2000 ms explicit-objective budget, the same relative request returned all
104 route poses in 857.5 ms and passed live validation in 15.7 ms. These are
odometry-relative test offsets, not general declarations that every point at
those offsets is clear road. The fleet launch now uses this bounded 2000 ms
budget for Navigate/Home; exploration and hardware defaults are unchanged.
The combined adapter, live-mapping and launch regressions passed (125 tests).

Other probes illustrate the limits of this evidence: a 30-metre R0 route passed
with the original 500 ms budget, and an R1 12-metre route passed with both
profiles. Some endpoints were rejected for excessive known height within the
footprint; one repeat met a transient stale-authority check. Increasing a
deadline does not resolve an occupied endpoint or authorize stale map data.

The first motion harness aborted on a live-view HTTP timeout and verified
Stop All; the repeat failed during observation setup before dispatching a new
goal. Neither run qualifies as an arrival trial. Profiling isolated the slow
request: the unfiltered catalogue contained 104 robot publications across
100 components, mostly historical missions. Reading their JSON took 212 ms,
normalizing them took 3811 ms, and assembling the index took 307 ms. The active
mission alone contained four publications and took 11 ms to read, 16 ms to
normalize, and 3 ms to index. Version lookup remained below 1 ms in both cases.
The native planner is separate from this server catalogue path.

The server now shares normalization of unchanged publications by exact
mission, robot and revision, and serializes concurrent catalogue builds for
one store. Changed or invalid publications still undergo validation. Cached
sources are limited to exact revisions referenced by the last two coherent
catalogues and at most 128 owners. The harness discovers the active mission
once and scopes later catalogue reads to it; a mission change invalidates the
trial. All 58 replica/component/live-view and deployment regressions passed.

### Road detours, endpoint height and physical Home trial

Static Bistro pavement checks found a diagonal curb approximately 17 cm high
across R0's original four-metre detour window. R0's ten-centimetre step limit
correctly rejects crossing it, but the opening around the curb lies outside
that window. A failed bounded search therefore does not establish that the
destination is unreachable. Simulation now permits explicit Navigate/Home
search margins up to eight metres, choosing a single window that fits the
existing cell budget. Exploration limits are unchanged. The native regression
requires failure with the four-metre window and an exact-destination route
around the same wall with the wider window; a smaller cell budget prevents
unbounded widening.

The complete native objective suite passed all 52 cases with the final patch
sequence, including exact final heading and the cell-budget negative control.

A separate endpoint-height fix allows a distant ground goal to resolve to
observed terrain above the starting height. This is a bounded surface lookup,
not permission to exceed the robot's step or footprint limits. A shallow-ramp
regression fails without the fix and passes with it. Same-position requests
retain the physical driving surface, including stale requested heights, so
they cannot bypass traversal by projecting onto a ceiling. All 51 objective
cases passed before the additional search-window regression.

After the catalogue cache update, the unfiltered live catalogue took 615 ms
in a bounded repeat measurement, compared with roughly five seconds before
the update. The active-mission live view took seven milliseconds. These are
individual observations, not latency percentiles or a cold-start guarantee.

R0 then followed a 12-metre relative Navigate request for approximately
10.9 metres of reported displacement. New observations rejected its endpoint
for a 53 cm height difference and exhausted bounded recovery. This trial
demonstrates continued motion and obstacle discovery, not successful arrival.

From that location, Return Home succeeded in 139.8 seconds. The retained graph
contained up to 18 poses and execution progressed through `following_local`
and `following_final`. Independent simulator truth measured 14.458 metres
from spawn before the action and 0.248 metres at completion. Stop All then
verified all four robots idle. The final approach included substantial
manoeuvring; this is a successful arrival trial, not a controller-efficiency
qualification. The deployment used the synthetic drift odometry test profile;
ground truth supplied only scalar acceptance measurements and was never sent
to the planner or controller.

Qualified live telemetry now optionally includes the Home keyframe and its
navigation transform within the same mission/component authority snapshot.
The acceptance harness uses that field, rejects malformed provenance and
does not substitute an unqualified fleet pose. Missing Home data remains
backward compatible. The combined navigation, Home telemetry, replica and
launch regression suite passed all 188 tests.

A final provenance check additionally binds Home keyframe identities to the
enclosing robot and mission. The Home continuation regression now waits for
the completed public phase transition rather than a mock call made before
the asynchronous state handoff. The expanded combined suite passed 193 tests,
and Black accepted all 348 tracked Python files.

### Deployed wider-window checks and remaining road case

The fresh four-robot benchbot mission uses
`swarmdeck-mgg:nav-window-8384392`. Exact native routes passed for R0 at
20 metres forward/two metres right and 30 metres forward/two metres left,
and for R2 and R3 at 12 metres forward. Planning took 966, 270, 545 and
210 milliseconds respectively; every returned route retained its requested
XY and passed live validation. These probes did not command motion.

The 20-metre R0 motion trial subsequently travelled about 10.8 metres of
reported displacement before recovery exhausted three attempts near the
raised road section. Its local and global endpoints stayed fixed relative
to the qualified destination. The final search used 1394 milliseconds and
reported no detour within its window, with a known 11.5 cm rise exceeding
the Bunker's 10 cm limit. This remains a failed arrival trial.

A follow-up static check used the Bunker's extended 1.073 by 0.828 metre
planning footprint, comparing each footprint sample against the centre's
ground height and checking height changes on neighbouring edges. For the
nominal spawn and requested road destination, known-pavement connectivity
remained disconnected with 8, 12 and 24 metre margins. This does not rule
out a side passage outside that sampled pavement or through currently
unknown terrain, and it does not justify increasing the robot's step limit.
The earlier four-metre-window diagnosis explains one bounded-search limit;
it does not fully explain the remaining Bistro road case.

The qualified Home harness also exposed an intermediate-controller-success
handling error: it stopped a rolling Home trial before observing the final
segment. That trial is inconclusive. The harness now waits for the final
Home phase and requires a fresh qualified success sample before evaluating
arrival. Later route probes must record their current starting state; they
cannot be compared directly with the earlier stationary cold-start probes.

The UI-only deployment `swarmdeck-ui:merged-count-97214af` fixes the Global
control's membership source. At verification, the legacy map service reported
zero merged members while the active replica catalogue contained a verified
three-robot component and one separate robot. The control now follows the
same component-selection rules as Global, uses existing catalogue polling,
and excludes historical or unverified components. All 33 named replica UI
tests passed, together with Svelte checks and the production build. The
server and simulation were not restarted for this UI change.

### Unknown terrain after a gradual climb

A separate native regression reproduced a false step when a gently rising
observed road ended at an unknown region. Every unknown sample was reset to
the robot's starting height, discarding the height reached on the ramp.
Navigate now carries the adjacent checked driving height into unknown terrain.
Known surfaces still use measured heights, and footprint, step, occupancy and
geofence checks still apply. Exact goal XY and heading are preserved.

Grid states distinguish different inherited heights at the same XY cell, with
stable parent indices and the existing cell, expansion and time limits. Known
surface states reuse their measured-height identity; lookup keys remain bounded
by twice the cell limit. Only complete Navigate goals can reconnect at a
provisional inherited height. Home and partial-route endpoint rules are retained.
Live route validation preserves those provisional heights while checking new
observations for hazards.

The reviewed patch passed 15 core test targets and all 53 native ROS objective
cases, including the ramp-to-unknown route, distinct-height approaches, exact
endpoints, known curbs, ceiling guards and Home. This establishes the synthetic
bug and its fix; it does not yet establish that this was the cause of the
remaining Bistro road failure. The route smoke script accepts an optional
`--planner-namespace` for querying an isolated native candidate without changing
the robot's live inputs or commanding motion.

The corresponding Python navigation, Home, replica and deployment checks
passed all 194 tests. Black accepted all 348 tracked Python files after testing.

The complete Docker patch-chain check exposed an incomplete baseline in the
initial patch packaging. The corrected patch includes the sparse state store
needed for multiple heights per XY, instead of allocating every cell in the
bounding rectangle. Nearby start connectors retain only the nearest candidates
that fit the remaining cell budget, check coordinate arithmetic before integer
conversion/addition, and share the cooperative deadline. The production body
evidence policy gate is preserved. Exact-image native testing is required after
this correction; the earlier isolated test results do not qualify that image.

### Exact-image validation and fresh mission

`swarmdeck-mgg:nav-window-a17d2a9` built successfully from the corrected patch
sequence. In an isolated container, all 15 core and three ROS test targets
passed: 219 cases, including 35 grid-refinement and 53 objective-service cases.
The container was removed after testing. The authoritative XML and build log
are under the bench review checkout's `.deploy/nav-timeout/`, with prefix
`test-parent-native-20260915T044103Z`.

The subsequent fresh mission uses ROS domain 216 and retains the single UI
on port 15173. R0's 30-metre left-offset route and the 12-metre R2/R3 routes
passed native planning and live validation. R0's 20-metre right-offset goal
still exhausted three two-second planning attempts before meaningful motion;
the final attempt made 6,464 projections. This route is not qualified.

R3 reached its 12-metre Navigate goal in 23.4 seconds, with a qualified arrival
error of 0.219 metres and zero local/global endpoint error relative to the
requested component-frame goal. Its subsequent Home trial used rolling graph
continuation but failed about 3.55 metres from home. An occupied intermediate
corridor pose was rejected before any traversal or grid expansion, so the
local planner did not attempt a detour. Stop All verified the fleet idle after
both trials.

The mesh investigation distinguishes an abrupt 16–17 cm direct curb from an
eastern passage with successive 6.8 cm and 7.5 cm rises. The former exceeds
the Bunker's 10 cm capability. A static full-route search can use the latter
when the footprint accepts known, connected supports with individually
traversable transitions; the old centre-relative height test seals it. This
is geometry evidence, not yet a successful native or physical crossing.

### Connected terrain and intermediate Home obstacles

The follow-up footprint rule retains the existing centre-height fast path.
When measured terrain spans more than one step, every outlying hit must have
a four-neighbour chain of known supports back to centre-compatible terrain.
Adjacent transitions obey the existing step/inclination limits. Missing hits
cannot connect those supports; unavailable or non-finite queries still fail.
The fallback completes the bounded footprint scan and reuses its ray results,
with at most 4096 samples and four neighbour checks per hit; it does not resample
terrain during the connectivity search.

Home can now bypass an obstructed intermediate graph hint during initial
planning or rolling refinement. The local grid keeps the exact section endpoint,
and the global route, final Home pose and continuation token remain unchanged.
Only physical waypoint-precheck failures qualify; malformed geometry, blocked
section endpoints, search failures and exhausted deadlines do not trigger an
extra search. The fallback uses the original request's remaining time budget.

Known ground projections also memoize repeated exact incoming-height keys within
one immutable planning snapshot. The additional memo is capped at the cell
limit and safely falls back to projection when full. A synthetic ramp fixture
identified 441 redundant exact-key calls among 1075 projections; this predicts
less map work, rather than establishing a live speedup. Distinct inherited
heights remain separate states.

The complete 46-patch Docker sequence applies to pinned MGG revision `902e868`.
The combined Python suite passed 195 tests, and Black accepted all 348 tracked
Python files after testing. Exact-image native and physical arrival checks are
the remaining gates for this candidate.

### Reviewed image and arrival results

The final image `swarmdeck-mgg:road-home-29afed0` passed all 227 native cases
across 18 test targets, including 37 grid-refinement and 59 objective-service
cases, with no failures or disabled tests. The first image exposed three
service-fixture assumptions: one positive detour lacked observed lateral ground,
and two nominally zero-margin negative tests configured only the generic planner
instead of Home's objective envelope. Correcting those fixtures preserved their
safety assertions and changed no planner runtime source.

The independent quality pass also found a merged-count ownership edge case:
an unavailable explicitly selected component must not display a different
component's membership. That fix passed the 38-case replica UI suite, Svelte
checks and the production build. Obsolete tool diagnostics were removed from
an older patch preamble without changing the resulting MGG source.

Both reviewed images are deployed in mission
`59456af8-66aa-4eae-b5cb-4c294bfeee8a`, ROS domain 217, on the single `planning-next`
stack and UI port 15173. After startup authority became ready:

| Trial | Result | Time | Qualified arrival error |
| --- | --- | --- | --- |
| R3 Navigate, 12 m | Arrived | 22.8 s | 0.245 m |
| R3 rolling graph Home | Arrived | 24.0 s | 0.162 m |
| R2 Navigate, 12 m | Endpoint terrain rejection | 1.0 s | No motion |

R3's last observed local/global endpoints had zero XY error relative to the
requested component goal in both trials. Home used local and final phases;
the passive simulator observer measured a 0.168 m physical return error. Stop All
verified the fleet idle after each trial. These results qualify this R3 round
trip, not fleet-wide navigation.

The R0 20 m/right-offset probe still exhausted its two-second planning deadline
after 4455 projections and 1939 expansions. The 30 m/left-offset R0 probe and
12 m R2/R3 probes passed, including exact XY and live lookahead validation. R2's
later commanded trial rejected a known 0.397 m footprint rise at its endpoint.
Rechecking that coordinate `(11.99, 0.15, -0.17)` at map revision 3565 still
rejected a 0.394 m rise. The later successful 12 m target was about 0.8 m away:
each invocation derives its destination from the current odometry and heading.
API and native-smoke goal transforms agreed within 4.2 mm vertically and exactly
in XY/yaw at a shared snapshot. This does not establish a collision mesh defect
at the rejected endpoint. Neither the R0 road crossing nor this R2 commanded
destination is qualified.

Native XML, probe results and arrival logs are under benchbot's review checkout
`.deploy/nav-timeout/`, prefixed `road-home-29afed0`. The isolated native test
container was removed after the run.

### Road-search tuning candidate

Planner-only shadows on the same live R0 mapping topics tested these bounded
Navigate/Home settings without commanding motion:

| Search lattice / deadline | Observed planning results |
| --- | --- |
| 0.25 m / 2 s | Deadline exhausted |
| 0.25 m / 4 s | Deadline exhausted |
| 0.5 m / 2 s | One route accepted; one deadline exhausted |
| 0.5 m / 4 s | Three routes accepted and live lookahead validation passed |

These were changing maps and relative destinations, not a controlled same-goal
benchmark. New probe logs record exact goal XYZ, frame and planning/validation
map revisions; arrival logs record both component and navigation goals. A fresh
motion trial remains the acceptance gate.

The simulation candidate uses 0.5 m search nodes and a four-second objective
deadline. This changes Navigate and Home refinement only; Explore's graph
search, measured terrain, body sweeps, step limits, Nav2 resolution and hardware
defaults retain their existing settings. A coarser lattice can miss a narrow
valid passage, and the existing overall recovery deadline still bounds retries.

The final review also found a separate planner/validator inconsistency. Planning
can accept its odometry-proven current pose despite a footprint ground-support
veto, allowing escape, while validation previously rejected that same first
sample. The candidate mirrors the planner's position predicate and waives only
that first sample's footprint `kNoGround` result. Occupied body volume, geofence,
unknown queries, edge projection, sweeps and all later samples remain strict.
A native regression plans and validates at one map revision, then verifies that
a newly observed footprint hazard farther along the route still invalidates it.
This does not explain the separately observed R2 projected-edge rejection.

### Road-search candidate arrival results

The exact `b511457` image passed all 228 native cases across 18 targets,
including the immediate Plan/Validate regression. The launch and acceptance
suite passed 26 Python tests, followed by Black on all 348 tracked Python files.
Independent review found no static production blocker; the stationary branch lacks
its own direct regression, while the moving-start and later-hazard branches
are covered.

The active mission is `17e628f4-1df1-4e6c-9537-ff150a711593`, ROS domain 218,
with MGG `swarmdeck-mgg:road-home-b511457` and the previously tested UI image
`swarmdeck-ui:road-home-29afed0`. The MGG image ID is
`sha256:6d25ce4de50e8db52cedde5b404e39d5180ae8845b4ad30ae9256f08070f8093`.

| Trial | Result | Time | Qualified arrival error |
| --- | --- | --- | --- |
| R0 Navigate, 20 m / 2 m right | Recovery failed after 10.56 m displacement | 31.3 s | 9.78 m remaining |
| R2 Navigate, 12 m / 4 m left | Arrived | 22.3 s | 0.218 m |
| R2 rolling graph Home | Arrived | 22.7 s | 0.236 m |
| R3 Navigate, 12 m | Arrived | 42.1 s | 0.211 m |
| R3 rolling graph Home | Timed out after 3.10 m displacement | 301.0 s | 9.24 m remaining |

The final observed Navigate endpoints and global Home endpoints had zero XY
error relative to their qualified component-frame destination. R3's unfinished
local Home section correctly ended before the final Home destination. Stop All
verified the fleet idle after each trial. R2 Home used local and final phases;
R3 remained in the local phase. Rounded local pose updates are recorded for
diagnosis and are not counts of replanning events. The final quality pass found
that this harness version overwrote earlier endpoint errors; the corrected
harness retains their maximum and keeps invalid evidence sticky.

The passive observer measured R2 within about 0.18 m of its initial position
after Home. It subsequently moved while parked when R3 traversed its position:
R2's physical displacement rose to about 10.45 m during R3's Navigate trial.
This is consistent with contact between robots and needs separate inspection;
an idle control state does not establish physical immobility. R3's Home trial
then travelled substantial distance while making little net progress, and
recorded a live swept-occupancy rejection followed by controller replacement.
The trial timed out and Stop All cancelled it; this run does not qualify R3 Home.

R0's runtime settings were confirmed as 0.5 m resolution, four-second deadline,
8 m margin, 16,384 expansion limit and 32,768 cell limit. Its first route was
cancelled by live validation (`remaining route footprint intersects known
terrain`). The final recovery rejected the bounded search after 1,622 expansions
and 2,008 ms, with a first reported 11.5 cm footprint rise. This was not deadline
exhaustion, and the first rejected footprint does not establish which boundary
cut every possible route. The crossing remains unqualified.

The R2 side corridor was selected from the static collision mesh before the
trial. For the nominal 12 m / 4 m left destination near world `(-10, -8)`, the
Scout footprint spans about 1.84 cm in height; the sampled straight approach
spans at most 4.46 cm. Direct and smaller-offset approaches encounter the
diagonal curb's roughly 17–19 cm discontinuity.

The old rejected R2 endpoint is a separate unresolved mapping question. Its
approximate world coordinate lies on flat static pavement, but a later rigid
transform from synthetic drift odometry cannot precisely register that earlier
query. Planner inputs combine unclassified lidar and reconstructed-depth hits;
occupied columns retain their maximum measured height until free observations
clear them. A retained non-ground return can therefore cause a footprint veto
without proving a pavement step. No captured evidence identifies that return's
source. The captured odometry had yaw only, and R2 uses its own sensed cloud,
so these checks do not establish pitch/roll accumulation or pollution from a
merged peer map. Keep the known-hazard veto; attributing the old return requires
capture-time registration and observation provenance that were not recorded.

Build, native XML, arrival logs and passive distance observations are preserved
under `.deploy/nav-timeout/road-home-b511457*` on benchbot.
