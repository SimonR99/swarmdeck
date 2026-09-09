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
with a configured MGG planner and a working navigation stack. `make up-sim`
leaves MGG idle by default (`EXPLORE=0`). Direct Compose invocations should set
`EXPLORE_SECONDS=0` explicitly to disable startup exploration.

## Interface checked

This integration targets [MGGPlanner's ROS 2 branch](https://github.com/MISTLab/MGGPlanner/tree/ros2),
commit `902e868b1d7ec70be8ccfd0351b0d0a94e07c2ca`. Its top-level README still
contains ROS 1 instructions; the relevant packages are under `ros2/src`.

Each robot gets its own namespace, normally `/<robot_id>/mgg`:

| Endpoint | ROS 2 type | Purpose |
| --- | --- | --- |
| `pci_trigger` | `std_srvs/srv/Trigger` | Start PCI's autonomous planning cycle |
| `pci_stop` | `std_srvs/srv/Trigger` | Stop planning; PCI also publishes an empty path |
| `status` | `std_msgs/msg/String` | Timestamped JSON lifecycle state: starting, exploring, complete, blocked, stopped |
| `command_path` | `nav_msgs/msg/Path` | PCI's current exploration path; transient-local QoS |
| `mggplanner` | `mgg_msgs/srv/PlannerSrv` | Internal PCI-to-planner request |
| `map_odometry` | `nav_msgs/msg/Odometry` | Robot pose in the planner's map frame |
| `mapping_cloud` | `sensor_msgs/msg/PointCloud2` | Live sensor clouds, retaining each sensor’s frame/stamp |

The adapter passes each path's **final goal** to its existing navigation backend
(Nav2 or the configured hardware trajectory action). The navigation backend
plans and collision-checks the route; this is not exact MGG waypoint following.
PCI observes map-frame odometry and requests the next plan on arrival or lack of
progress. Empty paths stop navigation immediately. The accompanying lifecycle status
distinguishes exhausted reachable planning from an unusable map or blocked
route. The fleet card reports **EXPLORED** or **EXPLORE BLOCKED**. Completion
is relative to the planner’s mapped, reachable space and configured bounds; it
does not certify that every part of the physical scene has been observed. A failed start or 30-second start timeout
stops local navigation and requests PCI stop.

Stops disable path intake first, cancel navigation, and issue zero velocity,
even if PCI's stop service is unavailable. Old latched paths and late service
responses cannot re-enable the session. A new start waits for earlier start/stop
requests to finish (or expire after 30 seconds if the planner restarted).
No independent path follower should also consume
`command_path`: that would bypass SwarmDeck's stop gate.

## ARGoS simulation

`scripts/sim-up` (and the `make up-sim` targets) starts MGG automatically.
For example, `./scripts/sim-up --dri --drift` starts the default scene with
exploration available; MGG remains idle until you press Explore.

For direct Compose invocations, add the MGG overlay to the same files and
environment used for the simulation. For Bistro with NVIDIA rendering and synthetic drift, from the
repository root:

```bash
SWARMDECK_CONFIG=/app/configs/4robot_bistro.yaml \
SWARMDECK_ODOMETRY=drift EXPLORE_SECONDS=0 \
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

The sidecar shares the simulator's network namespace and ROS domain 42.
It uses Fast DDS UDP transport to avoid stale shared-memory ports after sidecar
restarts; the simulator keeps its default transport for internal communication.
It uses the existing ARGoS sensor bridge, not MGG's separate demo simulator.
An input relay resolves `map_frame <- base_link` at each odometry stamp and caps
LiDAR publication at 2 Hz. In ARGoS it also projects the depth camera at
2 Hz, sampling every fourth pixel, to observe ground inside the elevated
LiDAR’s blind region. Camera extrinsics come from the same platform table
as the simulated sensor. No synthetic floor is inserted. Missing historical TF
suppresses the observation. Odometry waits in a bounded ten-message queue
for up to one simulated second for its matching TF, avoiding callback-order
races without substituting a newer pose.
Planner and PCI use simulation time. The simulation uses 0.15 m OctoMap voxels
and a platform-specific collision-box clearance;
the simulation slope limit is 30 degrees. Robot dimensions and sensor offsets come
from SwarmDeck's platform table. The supplied simulation planning bounds are
±60 m horizontally; adjust `fleet.launch.py` for another site. Hardware requires
its own site and robot parameter file.

The upstream PCI's forward bootstrap is disabled (`bootstrap_distance=0`);
exploration waits for actual mapped geometry. Each planner maintains its own
OctoMap and graph. Graph exchange is deliberately not connected across local
map frames: enable it only with verified inter-robot transforms, not assumed
identity transforms.

## ROS 2 hardware

Build MGG's ROS 2 packages on the robot's ROS distribution (the supplied Docker
image uses Jazzy), then source that workspace. Only the planner process needs
`mgg_msgs`; the SwarmDeck adapter uses standard Trigger and Path messages.

```bash
git clone --branch ros2 https://github.com/MISTLab/MGGPlanner.git /path/to/MGGPlanner
git -C /path/to/MGGPlanner checkout 902e868b1d7ec70be8ccfd0351b0d0a94e07c2ca
cd /path/to/MGGPlanner/ros2
# Install dependencies for your ROS distribution, then:
colcon build --packages-up-to mgg_ros mgg_pci \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

Launch from the SwarmDeck repository, using the real topic and frame names:

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

## Simulation steps and return home

The ARGoS Jolt stand-ins are upright rigid bodies. SwarmDeck adds collision-checked
step assistance for 0.10 m steps on Bunker/Scout and 0.30 m steps on Spot. It
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
Spot’s edge cap reaches 2.5 m so its graph can reach camera-observed floor. The pinned upstream patch is
`deploy/patches/mgg-traversal-lifecycle.patch`; rebuild the MGG image when it changes.
PCI stops after three consecutive empty/failed plans or three stalled legs.
A leg also has a time limit of six times `stuck_timeout_sec` so circling cannot
keep it active forever. A stopped session cannot publish a late planner result.

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
