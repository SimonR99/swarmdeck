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

This integration targets [MGGPlanner's ROS 2 branch](https://github.com/MISTLab/MGGPlanner/tree/ros2),
commit `902e868b1d7ec70be8ccfd0351b0d0a94e07c2ca`. Its top-level README still
contains ROS 1 instructions; the relevant packages are under `ros2/src`.

Each robot gets its own namespace, normally `/<robot_id>/mgg`:

| Endpoint | ROS 2 type | Purpose |
| --- | --- | --- |
| `pci_trigger` | `std_srvs/srv/Trigger` | Start PCI's autonomous planning cycle |
| `pci_stop` | `std_srvs/srv/Trigger` | Stop planning; PCI also publishes an empty path |
| `command_path` | `nav_msgs/msg/Path` | PCI's current exploration path; transient-local QoS |
| `mggplanner` | `mgg_msgs/srv/PlannerSrv` | Internal PCI-to-planner request |
| `map_odometry` | `nav_msgs/msg/Odometry` | Robot pose in the planner's map frame |
| `mapping_cloud` | `sensor_msgs/msg/PointCloud2` | Live LiDAR cloud, retaining its sensor frame/stamp |

The adapter passes each path's **final goal** to its existing navigation backend
(Nav2 or the configured hardware trajectory action). The navigation backend
plans and collision-checks the route; this is not exact MGG waypoint following.
PCI observes map-frame odometry and requests the next plan on arrival or lack of
progress. Empty paths end exploration. A failed start or 30-second start timeout
stops local navigation and requests PCI stop.

Stops disable path intake first, cancel navigation, and issue zero velocity,
even if PCI's stop service is unavailable. Old latched paths and late service
responses cannot re-enable the session. A new start waits for earlier start/stop
requests to finish. No independent path follower should also consume
`command_path`: that would bypass SwarmDeck's stop gate.

## ARGoS simulation

Add the MGG overlay to the same Compose files and environment used for the
simulation. For Bistro with NVIDIA rendering and synthetic drift, from the
repository root:

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

The sidecar shares the simulator's network/IPC namespace and ROS domain 42.
It uses the existing ARGoS sensor bridge, not MGG's separate demo simulator.
An input relay resolves `map_frame <- base_link` at each odometry stamp and caps
cloud publication at 2 Hz. Missing historical TF suppresses the observation.
Planner and PCI use simulation time. Robot dimensions and sensor offsets come
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
  robot:=spot_0 map_frame:=map odom:=/spot/odom cloud:=/ouster/points \
  params:=/absolute/path/to/spot-site-mgg.yaml use_sim_time:=false
```

The parameter file must target `/**/mggplanner_node` and describe the robot's
collision size, sensor extrinsics, ground clearance, and site's allowed planning
bounds. The upstream foot-bot demo parameters are not hardware calibration.
The map frame must match the adapter's `map_frame`; the controller rejects paths
in another frame. `odom.child_frame_id` must resolve into that frame through TF.
Use `tf:=... tf_static:=...` if the robot has nonstandard TF topics. Use an actual
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
