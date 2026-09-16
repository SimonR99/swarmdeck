# MGG autonomous exploration

A single **Explore** button stays at the bottom of the **Fleet** tab, below
its scrollable robot list. It starts all connected, enabled robots that support
exploration, regardless of selection. The button reads **Stop exploration**
while any fleet robot is exploring; clicking it stops exploration across the
fleet. Each exploring robot's state badge reads **EXPLORING**, including while
its navigation stack executes a goal. Stop All also stops exploration. Manual
navigation, teleoperation, reset, and operator-link loss end the affected robot's
session; it never resumes automatically.

The button is disabled unless the adapter advertises `explore`. Enable this only
with a configured MGG planner and a working navigation stack. No planner service
is called merely by launching the dashboard or containers.

## Interface checked

This integration starts from [MGGPlanner's ROS 2 branch](https://github.com/MISTLab/MGGPlanner/tree/ros2),
commit `902e868b1d7ec70be8ccfd0351b0d0a94e07c2ca`, and applies SwarmDeck's pinned
patch series. The upstream commit alone does not provide the external path
execution, replan, lifecycle, coordination-exclusion, or map-query contract
described here. Its top-level README still contains ROS 1 instructions; the
relevant packages are under `ros2/src`.

Each robot gets its own namespace, normally `/<robot_id>/mgg`:

| Endpoint | ROS 2 type | Purpose |
| --- | --- | --- |
| `pci_trigger` | `std_srvs/srv/Trigger` | Start PCI's autonomous planning cycle |
| `pci_replan` | `std_srvs/srv/Trigger` | Request the next path after controller success, failure, or a rejected reservation |
| `pci_stop` | `std_srvs/srv/Trigger` | Stop planning; PCI also publishes an empty path |
| `status` | `std_msgs/msg/String` | Timestamped JSON lifecycle state: starting, exploring, waiting, complete, blocked, stopped |
| `command_path` | `nav_msgs/msg/Path` | PCI's current exploration path; transient-local QoS |
| `mggplanner` | `mgg_msgs/srv/PlannerSrv` | Internal PCI-to-planner request |
| `plan_objective` | `mgg_msgs/srv/PlanObjective` | Plan a graph route for Navigate or Return Home and refine its first local section |
| `refine_objective_route` | `mgg_msgs/srv/RefineObjectiveRoute` | Refine the next section of the retained graph route after local arrival |
| `validate_objective_route` | `mgg_msgs/srv/ValidateObjectiveRoute` | Check the remaining local route against current terrain and obstacles |
| `map_odometry` | `nav_msgs/msg/Odometry` | Robot pose in the planner's map frame |
| `mapping_cloud` | `sensor_msgs/msg/PointCloud2` | Live sensor clouds, retaining each sensor’s frame/stamp |

SwarmDeck launches PCI with `external_path_execution=true`. The adapter sends
the complete ordered `command_path`, including every MGG waypoint, to Nav2
`FollowPath` or the configured hardware trajectory action. That controller owns
path tracking, collision handling, arrival, and movement-failure detection. PCI
does not use its odometry proximity or stall watchdog to replace a path while
the controller is still executing it.

After the controller reports success, the adapter releases the peer reservation
and calls `pci_replan` for the next path. A controller failure also releases the
reservation, but enters bounded recovery. The default permits two replacement
paths, which means at most three failed movement attempts including the original
path. Replacement planning has a 15-second recovery window; a controller action
already executing can finish, but a subsequent failure cannot extend that window.
Publishing or accepting a replacement path does not reset
this budget; only an actual controller success clears it. A rejected replan,
an unavailable replan service, expiry of the recovery deadline, or the third
controller failure changes the robot to **EXPLORE BLOCKED**. The operator can
press Explore again to begin a new session after correcting the cause.

An empty native plan, or a path ending within PCI's minimum progress distance
(`reach_distance`, 0.3 m by default), is not arrival evidence and does not declare exploration
complete in external-execution mode. PCI remains in `waiting` and retries after
1, 2, 4, 8, then at most 10 seconds between attempts. A later usable path
continues the same session. Fleet completion remains relative to the planner's
mapped, reachable space, current component, and configured bounds; it does not
certify full global or physical-scene coverage. A failed start or 30-second
start timeout stops local navigation and requests PCI stop.

Peer coordination reserves the path endpoint only after transforming it with a
fresh, accepted map authority. Causal optimizer and map-revision metadata can
advance without cancelling an otherwise unchanged active reservation. Reordered,
stale, partial, or wrong-mission authority cannot refresh it. A component change,
a materially changed component-to-navigation transform, an expired authority,
or a conflicting lower-cost lease still invalidates it. Reservations use bounded
receipt-relative leases and are renewed while the full path executes.

Stops disable path intake first, cancel navigation, and issue zero velocity,
even if PCI's stop service is unavailable. Old latched paths and late service
responses cannot re-enable the session. A new start waits for earlier start/stop
requests to finish (or expire after 30 seconds if the planner restarted).
No independent path follower should also consume
`command_path`: that would bypass SwarmDeck's stop gate.

## ARGoS simulation

`scripts/sim-up` starts the peer Swarm-SLAM, MOLA, indexed query, MGG and ARGoS
services together. MGG remains idle until you press Explore. For Bistro:

```bash
./scripts/sim-up --scenario bistro --drift
```

The default planner backend is `mola_snapshot`, with the `mola` indexed query
provider. MGG builds a read-only spatial index from each robot's coherent MOLA
product; it does not accumulate a second map from raw clouds or depth images.
Capture-time sensor transforms and qualified rays in the peer map supply the
geometry and observed free space. The relay still supplies odometry and camera
extrinsics. See [current stack operations](current-stack.md) for lifecycle and
port options.

The MOLA grid resolution is 0.20 m. Ground checks use measured surface heights;
the terrain-step limits are 0.15 m for Bunker and Scout and 0.30 m for Spot, with
a 30-degree slope limit. Robot dimensions, sensor offsets and controller step
limits come from the simulation platform table, so the adapter accepts the same
platform steps as the planner. The supplied planning bounds are ±60 m horizontally.
These settings apply to simulation; hardware needs its own qualified profile.

In MOLA mode, MGG reads the immutable native planner grid directly. It does not
reconstruct an OctoMap tree. Occupied-only body checks retain measured surface
heights so a coarse floor voxel cannot protrude upward into the robot's body
solely because of its cell boundary. Missing height evidence remains conservative,
and strict observed-volume checks retain full voxel bounds. The separate legacy
cloud backend still uses OctoMap.

Sparse LiDAR rays do not observe the whole body volume or the floor underneath
a stationary robot. The simulation's explicit `observed_ground` body policy
preserves unknown occupancy and vetoes known obstacles. Its separately enabled
`provisional_unknown` ground policy permits a bounded connector from the
physical starting pose to measured floor. One return beneath the robot does not
prove that the floor ahead is observed, so the local graph's root retains this
connector even when its own ground height is known. Its reach remains bounded
independently of the ordinary graph-edge limit.

The final indexed query also checks paths already accepted by native terrain
projection. Its surface fit needs several nearby columns, so sparse LiDAR rings
can leave gaps between measured supports. Under the simulation's provisional
policy, each such gap must close on measured ground within the same connector
limit, including the distance to that closing sample. A height change across a
gap cannot exceed the platform step limit. Occupied cells, known insufficient
clearance, excessive roughness, steps and drops remain vetoes throughout. A
route cannot finish in a gap or use missing ground without reaching measured
support. Hardware retains strict observed-volume and terrain requirements.

The lattice candidate filter uses the same body-evidence policy as edge
validation. Requiring mostly ray-cleared body volume at that earlier stage can
discard usable ground before terrain projection, especially for the taller Spot
model. Allowing unknown body cells does not supply ground: each candidate still
needs measured terrain, and known obstacles still reject it.

Qualified MOLA exploration starts at the robot's measured pose. A nearby floor
return must not move that starting pose vertically; the first edge connects it
to measured terrain while retaining the step and collision checks. The extra
body check sweeps a circular footprint that encloses the robot at every yaw,
avoiding the unused corners of its enclosing square. It checks occupied
voxel boundaries and has a fixed work budget. Hardware's strict body policy is
unchanged.

For MOLA exploration in this qualified simulation policy, a route height can be
refined once from the indexed ground fit. Native projection may use a nearby
return while the fit estimates ground under the footprint centre. Only the
smallest height correction needed to enter the existing tolerance band is
applied. It must remain within the platform step limit and preserve the physical
start, XY route and heights already within tolerance. The adjusted route must
pass the native swept-body check again, followed by a second indexed terrain
and body query against the same map revision. Both queries share one timeout;
routes already within tolerance use only one. This exploration query samples
terrain at horizontal intervals, keeping sample positions consistent across
height refinement; the native body sweep still checks the full 3D segment.
Navigate, Home and hardware keep their existing height-refinement policy.

While a robot follows a qualified MOLA route, the remaining-route validator
preserves those accepted heights and samples between them. It checks the latest
map for footprint terrain, occupied space, steps and geofence violations within
a bounded lookahead. Resampling keeps the geometry that planning approved;
reprojecting it onto a nearby floor return could falsely reject that route.

If a candidate's far end fails these checks, qualified simulation exploration
can use its already-validated prefix to collect more ground data. The prefix
must end on measured support at least `partial_route_min_progress_m` (1 m by
default) from the physical start, before the first rejected sample. Every
included sample retains the same terrain and body checks; adjusted heights
still need the second query. This fallback applies only to exploration and
does not shorten manually requested destinations or Return Home routes.

MOLA mode sizes the graph's maximum edge and initial connector from the selected
LiDAR's first ground-return ring, with one map-cell margin and grid rounding.
For the Bistro VLP16 this gives 3 m for Bunker, 2 m for Scout and 4 m for Spot;
the maximum supported configuration is 5 m. These are reach limits, not unchecked
forward-motion commands. A path still needs a measured endpoint and the same
terrain and collision validation.

Unchanged optimizer poses advance the replica publication and causal solution
order without replacing the map snapshot. Captures and actual pose corrections
advance the mapping graph revision. This avoids rebuilding identical MOLA
products and repeatedly interrupting an idle robot's planner.

Each planning cycle admits a fresh mapping snapshot before graph construction.
An expensive graph build must not expire its own snapshot simply by holding
the callback lock. The query still uses the exact captured revision and source
stamp, checks the live authority and transform after each response, and rejects
provider staleness or a changed MOLA generation before publishing the route.

The native MOLA read lease also retains that fresh, immutable map throughout
the planning call. It permits publication during the indexed query while
preserving the caller's map view; actual corrections still invalidate the
result. Other callers cannot acquire an expired snapshot, and completing a
lease does not extend its freshness. This prevents longer graph searches from
invalidating themselves while their lock delays a heartbeat refresh.

To test actual fleet startup against a running four-robot simulation:

```bash
server/.venv/bin/python tests/deployment/exploration_acceptance.py \
  --simulation --base-url http://localhost:8080 --duration 180
```

The test starts from a verified idle fleet, presses Explore through the GUI
protocol, and requires path observations, an executing state and at least five
metres of displacement in each robot's stable navigation frame. A brief startup
movement followed by a stall is insufficient. It supports separate local
components and always sends Stop All at the end. Also inspect completed routes
and subsequent progress: passing this startup test does not establish full scene
coverage or exploration completion.

On 2026-09-16, revision `c31425e` passed a 180-second Bistro trial on Benchbot
with drift odometry and MOLA. Maximum displacement from the initial pose is
measured in each robot's stable navigation frame, not cumulative travel:

| Robot | Maximum displacement | Completed routes | Controller progress failures |
| --- | ---: | ---: | ---: |
| R0 | 40.0 m | 7 | 0 |
| R1 | 12.8 m | 4 | 2 |
| R2 | 42.3 m | 8 | 0 |
| R3 | 31.3 m | 5 | 0 |

R1 recovered from both movement failures and continued exploring. All four
retained stable mission/component/frame identities, with 156 observations each
and no observation errors. Stop All cleared every active route. The exact
Release image passed all 21 native test executables (288 GoogleTests). Three
local timing-sensitive failures passed on a filtered rerun and did not recur
in the full Benchbot suite; planner deadlines were not increased. This validates
startup and repeated route execution, not full Bistro coverage or hardware use.

The independent cloud/OctoMap path remains available through
`./scripts/sim-up --legacy-cloud --drift`. For a direct legacy Compose invocation
with NVIDIA rendering, add the MGG overlay to the same files and environment
used for the simulation:

```bash
SWARMDECK_CONFIG=/app/configs/4robot_bistro.yaml \
SWARMDECK_ODOMETRY=drift \
docker compose -f deploy/compose/docker-compose.yml \
  -f deploy/compose/docker-compose.gpu.yml \
  -f deploy/compose/docker-compose.mgg.yml \
  --profile argos up --build -d
```

Keep your existing odometry/rendering variables when adding the overlay to an
existing deployment. Recreating `sim` restarts its ROS stack and local maps; it
loads the adapter's exploration support. The overlay enables
`SWARMDECK_MGG_ENABLED=1`, builds a pinned MGG image, and starts one planner/PCI per
configured robot. MGG stays idle until Explore is pressed.

The sidecar shares the simulator's network namespace. The peer launcher selects
a fresh mission and ROS domain; the direct legacy overlay uses domain 42. The
isolated test overlays use UDP between separate container IPC namespaces; this
does not require disabling shared memory for colocated production ROS processes.
Both paths use the existing ARGoS sensor bridge. In legacy cloud mode, the input
relay resolves `map_frame <- base_link` at each odometry stamp and caps LiDAR
publication at 2 Hz. It also projects the depth camera at
2 Hz, sampling every fourth pixel, to observe ground inside the elevated
LiDAR’s blind region. Camera extrinsics come from the same platform table
as the simulated sensor. No synthetic floor is inserted. Missing historical TF
suppresses the observation. Odometry waits in a bounded ten-message queue
for up to one simulated second for its matching TF, avoiding callback-order
races without substituting a newer pose.
Planner and PCI use simulation time. Legacy cloud mode uses 0.15 m OctoMap
voxels and the same platform-specific collision and terrain limits.

PCI's forward bootstrap is disabled (`bootstrap_distance=0`);
exploration waits for actual mapped geometry. Each planner maintains its own
graph. Graph exchange is deliberately not connected across local
map frames: enable it only with verified inter-robot transforms, not assumed
identity transforms.

## ROS 2 hardware

Hardware must run the SwarmDeck-patched MGG image. A workspace built directly
from the pinned upstream commit is incompatible with the adapter lifecycle.
Build the image from the repository root, or use the equivalent image built by
the robot-local deployment profiles:

```bash
docker build -f deploy/docker/Dockerfile.mgg -t swarmdeck-mgg:local .
```

The Dockerfile checks out the pinned upstream revision, applies the complete
patch series in order, and rebuilds `mgg_ros` and `mgg_pci`. For development
outside the image, reproduce that exact patched source and build; do not launch
an unpatched upstream workspace. Exploration uses standard Trigger and Path
messages in the adapter. The opt-in `planning.backend: mgg` objective interface
also requires matching generated `mgg_msgs` in the adapter, including
`PlanObjective` and `RefineObjectiveRoute`. Rebuild the simulation, mapping and
ROS 2 adapter images together with MGG when that interface changes.

The checked-in Spot and Aslan vendor-stack profiles enable exploration without
enabling that objective interface. Before enabling MGG Navigate/Home on either,
build and source a matching `mgg_msgs` overlay in its adapter runtime; the
vendor image alone does not supply this contract.

The supplied image launches `deploy/mgg/robot.launch.py`. Configure it with the
real topic and frame names; the equivalent direct launch inside a patched and
sourced workspace is:

```bash
ros2 launch deploy/mgg/robot.launch.py \
  robot:=spot_0 map_frame:=map base_frame:=body \
  odom:=/lio_sam/mapping/odometry cloud:=/ouster/points \
  params:=/absolute/path/to/spot-site-mgg.yaml use_sim_time:=false
```

The parameter file must target `/**/mggplanner_node` and describe the robot's
collision size, sensor extrinsics, ground clearance, and site's allowed planning
bounds. The upstream foot-bot demo parameters are not hardware calibration.
The map frame must match the adapter's `map_frame`; the controller rejects paths
in another frame. `odom.child_frame_id` must resolve into that frame through TF.
Use `base_frame:=...` to select the chassis when the odometry child names a
sensor frame. Use `tf:=... tf_static:=...` if the robot has nonstandard TF topics. Use an actual
sensor cloud with correct extrinsics, not an accumulated global cloud whose
origin no longer describes individual LiDAR rays.

Add to the ROS 2 adapter YAML and restart the adapter:

```yaml
exploration:
  enabled: true
  namespace: /spot_0/mgg
  frame: map
```

ROS 1 adapters do not advertise exploration. Hardware deployment must validate
its navigation action, TF, planner parameters, and actual stopping behavior;
unit tests and a container build do not establish physical-robot performance.

### Robot-local deployment

The ROS 2 deployment profiles for **Botman, Aslan, Spot, and Asimov** include
an `mgg` service. Each robot builds and runs its own pinned MGG image; the
operator computer does not run their planners. `make deploy ROBOT=botman`
(and `aslan`, `spot`, or `asimov`) includes this service in the normal deployment.
It maps while idle and starts navigation only when Fleet → Explore is pressed.
Scout/TARS's ROS 1 deployment is unchanged.

The sidecar joins the robot's existing ROS domain over host networking, using
standard ROS messages/services between its Jazzy runtime and the Humble adapter.
It consumes the raw Ouster cloud on the Bunkers/Spot and the Mid-360 cloud on
Asimov. Registered or accumulated SLAM clouds must not replace these inputs:
the sensor-frame origin is required for correct free-space raycasting.

`hardware.launch.py` reads the same adapter YAML as the robot adapter. It uses
`base_frame` explicitly when resolving timestamped poses: SuperOdometry and
LIO-SAM odometry child-frame names do not necessarily describe the robot body.
Aligned depth and CameraInfo use the hardware's existing optical-frame TF;
16-bit millimetre and 32-bit metre depth are supported. Both sensor streams
are limited to 2 Hz; depth is sampled every fourth pixel. No synthetic camera
transform or floor is added on hardware.

The shared planner defaults are in `deploy/mgg/config/hardware.yaml` (500
vertices, a 20-degree slope limit, no shared graphs). Each adapter YAML supplies
its nominal body height, existing footprint and local planning bounds under
`exploration.planner`. These bounds default to ±60 m in the robot's map frame;
set them to the actual site and check the body envelope and sensor calibration
before field use. Spot still requires its usual claim, power and standing
state; Explore does not perform those body actions.

Validation covers launch configuration, ROS sensor projection, frame selection,
and adapter start/stop behavior. A Humble adapter also passed start, path delivery and stop checks against
native Jazzy PCI in isolated Docker containers. Physical driving and the
robot’s actual sensor/TF connectivity still need an on-robot check; these
tests do not establish that a particular hardware deployment is ready to move.

## Navigate, return home, and terrain

Navigate and Return Home retain a complete graph route to the requested
destination. The grid planner refines only the next local section (8 m by
default), and the indexed terrain query validates that section before the
controller follows it. Reaching a local endpoint advances the retained route;
it does not complete the operator's command. The displayed global route and
requested destination remain fixed while local sections advance. Arrival is
reported only after the final section reaches the requested endpoint.

For a Navigate destination outside existing topology, the graph may include a
tentative connection toward that destination. It supplies a global direction,
not free-space or ground evidence. The robot must pass the same local terrain,
body, and obstacle checks as each section becomes relevant. Unknown distant
terrain therefore need not invalidate the entire request before motion starts.
Local grid repair stays within the current section; it does not replace a
long graph route with a full-distance grid search.

Continuation retains the exact goal in the stable planning frame. Transforming
the UI goal happens before planning; the final path is compared in that same
frame. Route tokens, mission/component authority, and current map checks fence
continuations, and cancellation invalidates outstanding work.

The ARGoS Jolt stand-ins are upright rigid bodies. SwarmDeck adds collision-checked
step assistance for 0.15 m steps on Bunker/Scout and 0.30 m steps on Spot. It
sweeps the complete body upward, forward, and down to supported static ground;
ceilings, taller walls, other dynamic robots, and unsupported climbs are refused.
This approximates step traversal, not wheel suspension or articulated leg dynamics.
The low proximity obstacle band remains conservative so Spot can see Scout;
automatic navigation may avoid some physically traversable low obstacles.

MGG uses the same platform step limits when testing projected edges. Ground
probes can cross unobserved air to find known occupied floor, without inserting
synthetic ground. Missing support retains the existing graph height instead of
turning the `-1` sentinel into an artificial upward jump. The root uses the same
collision-box offset as projected vertices even inside the sensor blind spot;
MOLA's sensor-derived connector limits are described above. The
simulation fleet gives each explicit Navigate/Home grid refinement a 2000 ms
deadline. Refinement is local even when the destination is much farther away.
The terrain service reports geometric steps and surface roughness separately;
MGG applies the platform step limit and its own roughness limit to those metrics.
This remains a deadline, not a promised route. Exploration
keeps its shorter graph-search budget, and hardware retains its configured MGG
default unless a site profile overrides it. The pinned patch series includes
`deploy/patches/mgg-traversal-lifecycle.patch` and
`deploy/patches/mgg-external-path-execution.patch`; rebuild the MGG image when
any MGG patch changes. External execution leaves movement failure and its
bounded replacement budget to the robot controller boundary. Empty or
near-endpoint planner results remain waiting work and use the bounded retry
schedule described above. A stopped session cannot publish a late planner
result.

**Return home** is per robot. The simulation adapter records its first complete,
finite `map_frame -> odom -> base_link` pose, rather than the startup `(0,0)`
fallback. The server keeps this robot-local position and converts through the
current fleet alignment when issuing the normal navigation command. Home is
not a teleport or automatic emergency behavior. The button waits for valid home
telemetry and navigation capability; physical adapters that do not report
`home_pose` leave it disabled. Restarting the simulation adapter records a new
home; keep the server and simulation in the same run when resetting map frames.

Regression checks include adapter/backend tests, `deploy/patches/argos/test_steps.cpp`
against Jolt, and `adapters/test/ros/mgg_contract_smoke.py` against native PCI
in an isolated ROS domain with an inert navigation sink.

Run the native PCI test without robot access:

```bash
docker run --rm --network none -e ROS_DOMAIN_ID=173 \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -v "$PWD:/workspace:ro" -w /workspace \
  --entrypoint bash swarmdeck-mgg:local -lc '
    source /opt/ros/jazzy/setup.bash
    source /opt/mgg/ros2/install/setup.bash
    export PYTHONPATH=/workspace:"$PYTHONPATH"
    python3 adapters/test/ros/mgg_contract_smoke.py'
```

The Bistro regression run with drift odometry confirmed goals and motion on all
four robots, including Spot, and a successful Spot return to within the existing
navigation position tolerance. Completion and repeated-stall outcomes are tested
with native PCI and controlled planner responses; a full Bistro coverage run is
not part of that short regression.
