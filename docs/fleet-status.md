# Fleet status

A snapshot of the physical robots this stack is meant to eventually run on, taken while
preparing them for `adapters/adapter_ros2/` per `docs/hardware-bringup.md`. Re-verify
before trusting anything here — this is a point-in-time audit (2026-08-06), not a
maintained spec, and two of these four machines are shared with other people's work.

| Robot | SSH alias | Platform | Reachable | `rclpy` (native) | `rospy` (native) | SwarmDeck cloned | Adapter config |
|---|---|---|---|---|---|---|---|
| tars | `scout` | AgileX **Scout Mini** (Jetson AGX, R35) | yes | yes (Foxy) | yes (Noetic) | yes, `/ssd/swarmdeck` | **live**: SLAM, map, camera, teleop and `navigate_to` all verified working |
| botman | `botman` | AgileX Bunker (Jetson AGX Orin, R36) | yes | no (Docker only) | no | yes, `/ssd/swarmdeck` | **live**: SuperOdometry pose, 2D raytraced map and 3D cloud verified; e-stop interface available |
| aslan | `aslan` | AgileX Bunker (Jetson AGX Orin, R36) | yes | no (docker-only, containers busy) | no | yes, `/ssd/swarmdeck` | not started |
| spot | `spot` | Boston Dynamics Spot + Orin payload | **no** | unknown | unknown | not attempted | not started |

`rclpy` works natively on tars (correcting this doc's own earlier claim — see the tars
section). It's still the wrong tool for tars specifically, since tars's actual autonomy
stack is ROS 1; `adapters/adapter_ros1/` (new this pass) is what actually reaches it
without a bridge. botman and aslan still have no native ROS of either generation —
`rclpy`/`rospy` exist only inside their Docker images.

---

## scout (`ssh scout`, hostname `tars`, user `rover`)

**Hardware:** AgileX **Scout Mini** ground rover (`is_scout_mini` is hardcoded `true` in
`rover_launch/scout_base.launch`; `scout_mini.urdf.xacro` confirms a 0.62 x 0.585 m base),
Jetson AGX Xavier (R35.1, Ubuntu 20.04 focal).
`/ssd` is a 916 GB NVMe at 79% used; root is 83% used (4.6 GB free — worth watching).
A `ttyUSB0` device is present (likely the Scout base's serial link, matches `ugv_sdk` in
`catkin_ws`).

**Network:** `eth0` on `192.168.1.24/24` (this is how `ssh scout` reaches it — no mDNS
alias needed). A second tagged interface `eth0.2` carries `192.168.2.2/24`; the two
bunkers each carry the identical `192.168.2.2/24` address on their own tagged interface.
If that VLAN is ever bridged across robots rather than point-to-point, all three would
collide on the same IP — worth confirming with whoever set it up before relying on it.

**Current use:** idle. No other users logged in, no containers running.

**ROS state:** both `/opt/ros/noetic` (ROS 1) and `/opt/ros/foxy` (ROS 2) are natively
installed and **both actually work** — correcting an earlier pass of this audit, which
ran `ssh scout 'source /opt/ros/foxy/setup.bash'` as an inline multi-line string and got
`no such file or directory: /home/rover/setup.sh`. That error is an artifact of how
`BASH_SOURCE` resolves under that specific invocation style, not a broken install:
`source /opt/ros/foxy/setup.bash` from an actual script file imports `rclpy` cleanly, and
the interactive-shell path (`~/.bashrc` → `~/.ros1_bashrc` → `/opt/ros/noetic/setup.bash`
→ `/ssd/catkin_ws/devel/setup.bash`) imports `rospy`/`geometry_msgs`/`nav_msgs`/`tf2_ros`
cleanly too. `~/.ros2_bashrc` exists but is a stub that doesn't actually source Foxy
(its one `source /opt/ros/noetic/setup.bash` line is commented out and nothing references
`/opt/ros/foxy` at all) — so ROS 2 is reachable, just not wired into daily use here.
`catkin_ws/src` has only `scout_ros` and `ugv_sdk` — the base driver, ROS 1. Bash history
shows a ROS 1 SLAM/exploration workflow
(`aslam_ws`, `tare_ws` — TARE planner + LIO-SAM, launched via `roslaunch rover_launch
lvisam.launch` / `tars_tare.launch`) that was **deliberately deleted**
(`sudo rm -r aslam_ws tare_ws`) on the most recent session — read as "no longer the
active setup," not as accidental loss.

Docker has ROS 2 images pulled (`ros:humble-desktop-full`, `ros:foxy`, `ros:galactic`,
plus project images `swarmslam`, `superodom-ros2`, `ros2_drivers`) but no ROS 2 container
is currently running, and none of them are wired to `/ssd/swarmdeck` yet.

**There is a real, currently-designed-for-this-robot ROS 1 stack**, not just a bare
driver: `/ssd/mist_ws` (MIST lab's shared workspace, same one cloned on the bunkers).
`rover_launch/launch/rover_agx.launch` is what actually runs on tars:

- **Sensors:** VectorNav IMU (`/vectornav/IMU`) + Ouster lidar (`/os_cloud_node/points`,
  64-channel, 1024 horizontal), started via `vectornav.launch` / `ouster_new.launch`.
- **SLAM:** LVI-SAM (`lvi-sam/launch/include/module_sam_agx.launch`), configured for this
  robot in `lvi-sam/config/params_ouster_scout_vectornav.yaml` — lidar↔IMU extrinsics
  (`extrinsicTrans: [0.12, 0.01, -0.224]`), loop closure on, PCD export to `/ssd/LOAM/`.
  It publishes `/lvi_sam/lidar/mapping/odometry`, not a 2D occupancy grid.
- **State estimate:** one `robot_localization` EKF (`ekf_agx.yaml`) fuses that odometry
  and publishes TF **`odom_lidar -> base_link_filtered`** — there is **no `map` frame
  published anywhere in this pipeline**. `odom_lidar` (loop-closure-corrected via LVI-SAM)
  is the closest thing to a fixed/global frame this robot has.
- **Driving/exploration:** `gbplanner` (graph-based exploration) + `local_planner` +
  RosBuzz (`rosbuzz.launch`, swarm coordination scripts) — not `move_base` or Nav2, so
  there is no `navigate_to_pose`-style action server.
- **Adapter-to-swarm glue:** `control/adapters/scripts/rover_scout_adapter.py` already
  bridges some of this for RosBuzz — it republishes battery as `/robot_battery`
  (`sensor_msgs/BatteryState`, derived from `/scout_status`) and forwards
  `/odometry/filtered` to `local_position/pose`. It's a useful reference for real topic
  names, not something SwarmDeck depends on.
- **Base driver:** `scout_base_node` (from the top-level `/ssd/scout_ros`, not
  `catkin_ws`) subscribes `cmd_vel` with a leading slash in `scout_messenger.cpp`, so it
  resolves to the global `/cmd_vel` regardless of namespace.

The `aslam_ws`/`tare_ws` workspaces mentioned in the bash history (TARE planner + a
different LIO-SAM setup) were a separate, now-deleted experiment — `mist_ws` is the
current one.

**Adapter config written, twice — once per ROS generation:**

- `adapters/adapter_ros2/config/scout_mini.yaml` (`rclpy`), with `map_frame: odom_lidar`,
  `base_frame: base_link_filtered`, `topics.odom: /odometry/filtered`, `topics.cmd_vel:
  /cmd_vel`, `topics.battery: /robot_battery`, `topics.map`/`camera` left empty (nothing
  publishes them today). This one still needs a `ros1_bridge` to see any of those
  topics, since they're all ROS 1 — see the gap below.
- `adapters/adapter_ros1/config/scout_mini.yaml` (`rospy`) — same topic/frame values,
  and **this one needs no bridge**: `adapter_ros1.py` talks `rospy` directly, and tars's
  stack already is `rospy`. `navigate_to_pose` points at `move_base`, the ROS 1
  convention, though nothing serves it yet (see below).

Both configs' values are sourced from the mist_ws files listed above, not from a live
topic — `rostopic echo`/`hz` against the real graph is the next verification step, per
`docs/hardware-bringup.md`.

**`adapter_ros1.py` (`adapters/adapter_ros1/`) is a new file this pass** — `rospy`/
`actionlib` port of `adapter_ros2.py`, same protocol logic, same capability/deadman
rules (see the file's own docstring for exactly what differs and why). Import-checked
against tars's **real** environment (`source /opt/ros/noetic/setup.bash && source
/ssd/catkin_ws/devel/setup.bash`, then `import adapter_ros1`) — succeeds cleanly,
including `geometry_msgs`/`nav_msgs`/`tf2_ros`/`actionlib`. It has not been run
(`rospy.init_node` + a live `roscore`) — that starts touching the real robot's ROS graph
and deserves an explicit go-ahead, not a silent SSH pass. 21 unit tests (12 existing +
9 new, mirroring `test_adapter_ros2.py`'s ROS-stubbed structure) pass locally: `env -u
PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH server/.venv/bin/python -m pytest
adapters/test -q`.

**Gap to actually running the ROS 2 adapter:** this stack is ROS 1, and `adapter_ros2.py`
needs `rclpy` — the config tells a bridge what topics to carry, but importing `rclpy`
doesn't make it see ROS 1 topics. That needs an actual `ros1_bridge` (untried) or running
the adapter inside a ROS 2 docker image bridged to this graph. **`adapter_ros1.py`
sidesteps this entirely** — it's the more direct path for tars specifically, precisely
because tars's own stack is ROS 1.

**tars is live**, as of this pass (2026-08-06, still running): `adapters/adapter_ros1/launch/scout_mini_slam.launch`
brings up sensors + LVI-SAM + the EKF + `scout_base_node` — `rover_agx.launch` with
`local_planner_agx.launch`, the gbplanner include and `rosbuzz.launch` removed, so nothing
publishes `cmd_vel` on its own (confirmed by node list: no gbplanner, no local_planner, no
RosBuzz node running). `/odometry/filtered` confirmed publishing `frame_id: odom_lidar`,
`child_frame_id: base_link_filtered`, matching `scout_mini.yaml` exactly.
The adapter now runs in the `swarmdeck-adapter` Docker container with host networking and
is connected — `tars_0` appears in `GET /api/fleet` with a live pose and
`capabilities: [navigate, map, camera, battery, estop]` (`battery` reads `null`: no publisher, since
`rover_scout_adapter.py` — the node that republishes it — was deliberately left out of
the trimmed launch too). ROS sensor/SLAM launches remain detached host processes; the
adapter and media publisher use Docker's `unless-stopped` restart policy:

```
rover  10049  roslaunch .../scout_mini_slam.launch
rover  11627  roslaunch rover_launch rs_d400.launch
docker        swarmdeck-adapter
docker        swarmdeck-media
```

To stop only SwarmDeck's robot-side integration, use `docker stop swarmdeck-adapter
swarmdeck-media`; do not stop the sensor/SLAM launches with it.

**Navigation wiring exists** (2026-08-06): `local_planner_agx.launch` (`localPlanner` +
`pathFollower` + `loamInterface`) — reactive local obstacle avoidance (`checkObstacle:
true`, `maxSpeed: 0.4 m/s`), **not** a global planner (no `gbplanner`, `adjacentRange: 5
m` only). It takes goals on `/move_base_simple/goal` (`geometry_msgs/PoseStamped`, the
standard RViz "2D Nav Goal" topic — confirmed from source, not actionlib), so
`adapter_ros1.py` has a topic-based `navigate_to` path (`topics.nav_goal`/
`topics.nav_stop`) taking priority over the actionlib one when both are configured.

**Two real bugs found and fixed by actually testing teleop and navigate_to live** (later
same day, after `docs/fleet-status.md` had already called this "live" — it wasn't):

1. **Teleop didn't work whenever `local_planner` was running.** `pathFollower` runs its
   control loop continuously and publishes to `cmd_vel` at ~27 Hz **regardless of whether
   a goal is active** — confirmed live, it was streaming zero-velocity Twists with nothing
   to do. `topics.nav_stop` makes its *commanded value* zero but doesn't stop it
   *publishing*, so a single teleop Twist racing that stream got overwritten within tens
   of milliseconds. Fix: `adapters/adapter_ros1/launch/local_planner_muxed.launch` remaps
   `pathFollower`'s output to `/cmd_vel_nav` (mist_ws's own launch file anticipated this
   with a commented-out, never-finished `cmd_vel_mux` remap); `adapter_ros1.py` now
   relays `/cmd_vel_nav` to the real `/cmd_vel` **only** while `nav_status == "active"`
   (`topics.nav_cmd_vel`, `HardwareBridge._on_nav_cmd_vel`) — it is the sole publisher of
   the real topic, so nothing ever races teleop again. Verified: 6 consecutive teleop
   commands arrived on `/cmd_vel` with zero interleaved noise, where before the same test
   showed 6 messages lost inside 541 (mostly `pathFollower`'s zero-spam).
2. **`navigate_to` accepts a goal, computes a real path, and never actually drives.**
   `/path` confirmed publishing at ~10 Hz with genuine multi-point local trajectories the
   whole time — this is not a wiring problem. `pathFollower`'s linear speed is gated by
   `joySpeed`, which with `autonomyMode: false` (the default) is **only** ever set from a
   joystick's throttle axis or a `/speed` publisher, neither of which exists here, so it
   stays permanently zero forever. Tried `autonomyMode: true` next: that removes the
   joystick gate, but also switches the goal source from `goalX/goalY` (`/move_base_simple/goal`,
   what we send) to `RelativeSetPointX/Y` (`/setpoint_position/local`, which nothing
   publishes) — confirmed live, the robot immediately reports `goalReached` for a goal it
   never looked at. **Neither mode alone works with an absolute-goal adapter and no
   joystick.** Reverted to `autonomyMode: false` (correct goal source, zero speed) rather
   than leave it in the mode that silently ignores the goal — see "Open, needs a decision"
   below.

Both fixes verified against the real robot, supervised, with fresh camera checks before
each drive test (once with a person visible nearby — held off until confirmed clear).
Every test used a small (0.5-1.5 m) goal computed from the robot's *live* pose at send
time, not a fixed point, given the pose drift below.

**Resolved** (later same day): neither option above was it. `speedHandler`'s `/speed`
guard was correctly flagged as suspicious — confirmed it does require `autonomyMode:
true`, so it was dropped. The actual fix came from reading one function further:
`joystickHandler` sets `joySpeed = |axes[1]|` **completely unconditionally** — no
`autonomyMode` check at all, unlike `/speed` and the `autonomyMode` branch. That's the
one speed input this stack has that doesn't fight the goal source. `adapter_ros1.py`
gained `topics.nav_joy`: publishes a fake `sensor_msgs/Joy` (`axes[1] =
nav_joy_throttle`, default 0.5) every state tick while `nav_status == "active"`, zero
otherwise (`HardwareBridge._pump_nav_joy`) — republished continuously rather than once,
since this build's joystick-staleness timeout exists in source but is commented out, so
a one-shot publish would otherwise latch forever. `autonomyMode` stays `false`, so the
goal source is still `goalX/Y` via `/move_base_simple/goal`, exactly what `topics.nav_goal`
already sends — zero changes needed there.

**Verified live, supervised, with a fresh camera check immediately before the drive**
(operator physically beside the robot): sent a real 1.5 m goal computed from the robot's
live pose. `/cmd_vel` showed genuine smoothly-ramping `linear.x` (the `smoothVelocityGain`
curve, 0 → 0.28 m/s, still accelerating when the goal was reached), pose moved from
`(-1.955, 6.663)` to `(-1.199, 5.264)`, `nav_status` transitioned `active` → `succeeded`
at the correct distance, and the robot stopped cleanly. `navigate_to` is now genuinely
functional on tars, not just wired — the only caveat carried forward is the pose-drift
warning below, since goal coordinates are expressed in that same drifting frame, and the
5 m `adjacentRange` local-only horizon (no `gbplanner`).

**One more real bug, found by the operator during normal use**: a goal placed *behind*
the robot drove it straight ahead anyway, as if the goal were in front. Root cause: the
fix above only got `pathFollower`'s *speed* right — `localPlanner.cpp` separately
computes `joyDir = atan2(axes[2], axes[1])`, the candidate-path *direction*, straight
from the joystick axes with no `autonomyMode` gate and no reference to `goalX/Y` at all.
Publishing a constant `axes=[0, throttle, 0]` always signalled "goal straight ahead," so
the first live nav test having driven correctly was coincidence — the goal happened to
already be roughly ahead. Fixed: `_pump_nav_joy` now computes the real bearing from the
robot's current pose to the goal every tick and encodes it into both axes so `atan2`
recovers it exactly; `axes[1]` goes negative for a rear goal (safe — `pathFollower` takes
`|axes[1]|` for speed, sign only ever changes direction). Verified live, supervised,
camera-checked: a goal 1.0 m directly behind drove the robot there in reverse
(`twoWayDrive: true` — it backs up rather than turning around), confirmed via
consistently negative `linear.x` and pose converging on the correct point.

**Map is now live too**: LVI-SAM has no 2D grid of its own, so `adapter_ros1.py` forwards
`/lvi_sam/lidar/mapping/cloud_registered` (`topics.map_cloud`) and the backend raytraces
it into one server-side (`mapsvc/scan_grid.py`) — see the commit history for the design
reasoning. Confirmed: `GET /api/map` decodes to real free/occupied cells, not blank.
`rtabmap-ros`/`octomap-server` were considered and explicitly NOT installed — the
scan-grid approach needed no new packages on the robot at all.

**Camera added** (also 2026-08-06, same pass): a RealSense D435i was connected and
`rover_launch/launch/rs_d400.launch` (already installed — `ros-noetic-realsense2-camera`
via apt, no new install) launched separately, alongside the SLAM stack, not part of the
trimmed launch file. Confirmed `/d400_arm/color/image_raw` at ~30 Hz.
`scout_mini.yaml`'s `topics.camera_compressed` now points at
`/d400_arm/color/image_raw/compressed`; `capabilities` now includes `camera`. The D435i
also has its own depth cloud (`/d400_arm/depth/color/points`), unused so far.

**Production video and detection validated** (2026-08-06): `swarmdeck-media` subscribes
to that compressed ROS topic inside Docker, produces baseline-profile H.264 at 640×480,
15 fps and 1.2 Mbps, and pushes RTSP/TCP to central MediaMTX. Chrome established a real
WHEP peer connection (`readyState=4`, live unmuted track), and the dashboard rendered the
stream plus normalized boxes. The classical detector found both physical rubber ducks in
the initial frame (scores 0.892 and 0.883); during the final live check it continued
updating the boxes at the configured 5 Hz. Software encoding used about 53 MiB and 29–42%
of one Xavier CPU core. End-to-end latency is not yet timestamp-instrumented, so this does
not claim the <300 ms requirement even though the live path is operational.

**⚠ Pose is drifting significantly while the robot sits still.** A few minutes after
connecting, `tars_0`'s reported pose moved from near-origin to `(x=-1.82, y=6.91,
yaw=-59°)` and kept changing — confirmed against both the EKF's TF and LVI-SAM's own
`/lvi_sam/lidar/mapping/odometry` directly, so this is LVI-SAM's own estimate drifting,
not an adapter bug. No robot motion happened (nothing publishes `cmd_vel` in this
launch) and no loop-closure/error/reset lines appear in the SLAM log — it's just walking.
**Do not trust `tars_0`'s displayed position in the GUI right now.** Not investigated
further this pass — the lidar↔IMU extrinsics in `params_ouster_scout_vectornav.yaml`
were flagged as unverified from the start (see the file itself), and this is exactly the
kind of error a wrong extrinsic produces. Worth checking before doing anything
motion-related (teleop or otherwise) based on the displayed pose — including the first
real `navigate_to` goal, whose target is expressed in this same drifting frame.

**SwarmDeck terrain prepared:**
- Cloned to `/ssd/swarmdeck` (public repo, plain `git clone`, no auth needed).
- `websockets`, `pyyaml`, `numpy` importable at the system Python level (`pip3 install
  --user websockets`; yaml/numpy were already present). `python3 -m venv` is unavailable
  (`python3.8-venv` isn't installed and `sudo` needs a password), so this is a `--user`
  site-packages install, not an isolated venv.
- `python3 adapter_ros2.py --help` now fails on exactly one line: `import rclpy`.

---

## botman (`ssh botman`, hostname `botman`, user `botman`)

**Hardware:** AgileX Bunker, Jetson AGX Orin Dev Kit (R36.4.4, Ubuntu 22.04 jammy). `/ssd`
916 GB NVMe at 58% used; root 58% used — comfortable headroom.

**Network:** `eno1` carries **two** addresses, `192.168.1.49/24` and `192.168.1.154/24`
(the ssh alias resolves `botman.local` via mDNS to one of these). Same tagged
`192.168.2.2/24` sub-interface as the other two robots — see the VLAN note under scout.

**Current use: idle as of 2026-08-06 19:05 EDT.** No users were logged in and no
containers were running when the SwarmDeck configuration audit began. An earlier check
the same day found four SSH sessions and these two containers, so always re-check before
starting anything:
- `dinonav_ros2_2` (`dinonav_ros2:dev`, up ~1 h) — `/ssd/VNMs/DinoNav` mounted, GPU
  passthrough, X11 forwarded.
- `bunker_super_odom` (`bunker_super_odom:dev`, up ~2 h) — `/ssd/mist_ws` mounted, same
  GPU/X11 setup; two of the four sessions are `docker exec -it` shells attached to it.

That earlier state was a SuperOdometry-based SLAM and DinoNav navigation run. The
SwarmDeck audit used read-only SSH commands only: nothing in `/ssd/mist_ws`, Docker, or
the ROS graph was changed, and future work must still check `who`/`docker ps` first.

**ROS state:** no native `/opt/ros` at all — this robot runs ROS 2 exclusively through
Docker. Images relevant to navigation/SLAM: `bunker:dev`, `bunker_super_odom:dev`,
`bunker_steering:dev`, `dinonav_ros2:dev`, `liosam-humble-jammy`, `phntm/bridge:humble`
(a remote-bridge/teleop client — `phntm_bridge_client` also exists under `/ssd`). The
standard workflow is `cd /ssd/mist_ws && ./docker_setup.sh` (also present under
`/ssd/VNMs/DinoNav` and `/ssd/steering_vector/SafeGNM`). `mist_ws` is a 173-entry ROS 2
workspace (`control`, `driver`, plus `build`/`install`) — the MIST lab's shared multi-robot
codebase, not SwarmDeck-specific.

**SwarmDeck configuration launched and verified on 2026-08-06.** Botman uses
SuperOdometry in SLAM mode (`localization_mode: false`) with Ouster lidar/IMU. It
publishes `/laser_odometry` and `/registered_scan`; the source-declared
`/state_estimation` and `map -> os_lidar` TF had no live samples. The Bunker base consumes
`/cmd_vel`. There is no OccupancyGrid and no TF connection to the driver's independent
`odom -> base_link` tree. SwarmDeck therefore uses `/laser_odometry` directly for the
map-frame sensor pose and raytraces the registered scan server-side. See
`docs/botman.md` and `adapters/adapter_ros2/config/bunker.yaml` for the source-by-source
audit.

The adapter has its own generic Humble image and does not install anything into MIST's
image. `docker-compose.robot-botman.yml` mounts `/ssd/mist_ws` read-only and calls its
existing `rover_launch bunker_gnm.launch.py` interface. Host IPC is shared so Fast DDS
point clouds cross the container boundary. The live backend showed Botman's updating
pose, local 2D map, and merged 3D cloud; only `map` and `estop` are advertised because
the robot currently has no `/dev/video0`.

**SwarmDeck terrain prepared:**
- Cloned to `/ssd/swarmdeck`.
- `websockets` installed `--user` (`pip3 install --user websockets`); `pyyaml`/`numpy`
  were already present at the system Python level.
- `python3 adapter_ros2.py --help` fails at `import rclpy`, as expected — no native ROS 2
  exists outside the containers above.
- No files in that checkout or in `/ssd/mist_ws` were changed during the Botman audit;
  all new configuration is in the operator's SwarmDeck worktree.

---

## aslan (`ssh aslan`, hostname `aslan`, user `aslan`)

**Hardware:** AgileX Bunker, Jetson AGX Orin Dev Kit (R36.4.7, Ubuntu 22.04 jammy). `/ssd`
916 GB NVMe at 23% used; root 19% used — most headroom of the three.

**Network:** `eno1` on `192.168.1.139/24`; same tagged `192.168.2.2/24` sub-interface as
the other two (VLAN note under scout applies here too).

**Current use: active — do not disrupt.** Two SSH sessions from `192.168.1.143` logged in
as the shared `aslan` user. One container running: `bunker` (`bunker:dev`, up ~1 h,
`ROS_DISTRO=humble` confirmed via `docker exec`), `/ssd/mist_ws_ros2` mounted, GPU
passthrough, `--network=host`, `--privileged`. Treat this the same as botman's containers
— nothing was touched.

**ROS state:** no native `/opt/ros`; ROS 2 Humble lives in the `bunker:dev` container
(base image `dustynv/ros:humble-desktop-l4t-r36.4.0`). Bash history shows a real fight to
install ROS 2 natively via apt (repeated `ros-archive-keyring` / `ros2.list` attempts, all
abandoned) before settling on the Docker route via `mist_ws_ros2/docker_setup.sh` — same
pattern as botman, and worth knowing so nobody repeats that apt detour on a future robot.
Two ROS 2 workspaces exist: `mist_ws_ros2` (cloned from `git.mistlab.ca`, run via
`docker_setup.sh`) and `mggplanner_ws` (`exploration`, `mapping`, `misc` — a multi-robot
exploration/mapping planner, built with a `bunker.repos` vcs import). Neither is
SwarmDeck-specific.

**Gap to a working adapter:** same shape as botman — `rclpy` only exists inside the
`bunker` container. Additionally, **this machine had no `pip` at all** (not even the
`ensurepip` module — `python3-pip` was never apt-installed), which the venv/deps step
below had to work around.

**SwarmDeck terrain prepared:**
- Cloned to `/ssd/swarmdeck`.
- Bootstrapped `pip` from scratch (`curl https://bootstrap.pypa.io/get-pip.py | python3 -
  --user`, since neither `pip3` nor the `ensurepip` module existed), then installed
  `websockets --user`. `pyyaml`/`numpy` were already present.
- `python3 adapter_ros2.py --help` fails at `import rclpy`, as expected.

---

## spot (`ssh spot` → `orin.local`, user `indro`)

**Unreachable — DNS resolution for `orin.local` fails from this network.** Nothing was
inspected; this section only records what's configured, for when it's back:

- SSH alias points at `orin.local` (mDNS), user `indro` — likely the payload compute
  (an Orin) rather than Spot's own internal computer, matching the "InDro" naming.
- No terrain was prepared. Once reachable, repeat the same three steps used on the other
  robots: `git clone https://github.com/SimonR99/swarmdeck.git /ssd/swarmdeck`, then get
  `websockets`/`pyyaml`/`numpy` importable, then check whether `rclpy` is already usable
  natively (payload computers on Spot are more often a clean ROS 2 install than the
  Jetsons above, but that's a guess, not a finding).
- Spot needs its own adapter config (no `spot.yaml` exists yet under
  `adapters/adapter_ros2/config/` — `generic.yaml` is the right starting point per
  `docs/hardware-bringup.md` step 1, plus mapping the SwarmDeck protocol's
  `navigate_to`/`stop` onto whatever Spot's own SDK or ROS driver exposes for the
  deadman-critical `cmd_vel`/estop path).

---

## What "prepared" means here, precisely

On each of the three reachable robots:

1. `/ssd/swarmdeck` — a plain clone of this repo (public, HTTPS, no credentials needed).
2. `websockets`, `pyyaml`, `numpy` importable from the system `python3` (installed
   `--user` where missing — no `venv` was used, since `python3-venv` isn't installed on
   any of the three and none of them offered passwordless `sudo`).
3. Confirmed, on each: `python3 adapters/adapter_ros2/adapter_ros2.py --help` now fails
   at exactly `import rclpy` and nothing earlier — i.e. every other dependency the
   adapter needs is already satisfied. On scout, `adapters/adapter_ros1/adapter_ros1.py`
   additionally *does* import cleanly against the real environment (see the scout
   section) — it's the one with a real path forward there.

**Deliberately not done**, and why:
- **`adapter_ros1.py` has not been run**, only import-checked. Calling `rospy.init_node`
  against a live `roscore` starts touching tars's real ROS graph (even before sending any
  `drive` command, TF/odometry subscriptions begin flowing), which deserves an explicit
  go-ahead rather than happening as a side effect of "prepare the terrain."
- **Botman is running from SwarmDeck.** Its dedicated adapter image and project-owned
  config are live; starting the base driver changes the robot to commanded mode, so future
  restarts still require an operator and physical e-stop. Aslan still needs the same
  file-by-file audit and dedicated adapter setup.
- **No `adapters/adapter_ros2/config/<robot>.yaml` exists for Aslan.** Botman's driver and
  SLAM sources were read file-by-file once it became idle; Aslan's were not. Botman's
  values remain a documented source-derived hypothesis until the first supervised live
  `ros2 topic hz`/TF check.
- **No sudo/system-package changes, no touching `bunker`/`bunker_super_odom`/
  `dinonav_ros2_2` containers, no killing sessions.** botman and aslan had other people
  actively working on them during this pass.

## Next step, in bring-up order

Continue with scout — it's idle, and `adapter_ros1.py` has no remaining environment
blocker there: `source /opt/ros/noetic/setup.bash && source
/ssd/catkin_ws/devel/setup.bash` gets it everything it needs. What's left is real
verification, not setup: start a `roscore` (or use tars's own, if `rover_agx.launch` is
running), run `adapter_ros1.py --robot-id tars_0 --config config/scout_mini.yaml --host
<operator-host>` against it, and check `docs/hardware-bringup.md`'s step-1 table (does
`tars_0` appear in the GUI? is the pose live or stuck at origin?). `rostopic echo
/odometry/filtered` / `/robot_battery` and `rostopic hz /cmd_vel` are the fallback if it
doesn't. `navigate_to_pose` has no real target yet either way — that needs a translation
layer onto gbplanner/RosBuzz, not a config change.

For Botman restarts, re-check `who` and `docker ps`, put an operator at the e-stop, then
use the self-contained command in `docs/botman.md`; validate `/laser_odometry`,
`/registered_scan`, the 2D height band, merged 3D cloud, and deadman in that order.
For Aslan, coordinate with whoever is using its `bunker` container and repeat Botman's
read-only source audit before writing a configuration.
