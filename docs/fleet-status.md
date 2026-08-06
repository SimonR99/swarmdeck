# Fleet status

A snapshot of the physical robots this stack is meant to eventually run on, taken while
preparing them for `adapters/adapter_ros2/` per `docs/hardware-bringup.md`. Re-verify
before trusting anything here — this is a point-in-time audit (2026-08-06), not a
maintained spec, and two of these four machines are shared with other people's work.

| Robot | SSH alias | Platform | Reachable | `rclpy` (native) | `rospy` (native) | SwarmDeck cloned | Adapter config |
|---|---|---|---|---|---|---|---|
| tars | `scout` | AgileX **Scout Mini** (Jetson AGX, R35) | yes | yes (Foxy) | yes (Noetic) | yes, `/ssd/swarmdeck` | both `adapter_ros2` + `adapter_ros1` `scout_mini.yaml` written; `adapter_ros1` import-verified live |
| botman | `botman` | AgileX Bunker (Jetson AGX Orin, R36) | yes | no (docker-only, containers busy) | no | yes, `/ssd/swarmdeck` | not started |
| aslan | `aslan` | AgileX Bunker (Jetson AGX Orin, R36) | yes | no (docker-only, containers busy) | no | yes, `/ssd/swarmdeck` | not started |
| spot | `spot` | Boston Dynamics Spot + Orin payload | **no** | unknown | unknown | not attempted | not started |

`rclpy` works natively on tars (correcting this doc's own earlier claim — see the tars
section). It's still the wrong tool for tars specifically, since tars's actual autonomy
stack is ROS 1; `adapters/adapter_ros1/` (new this pass) is what actually reaches it
without a bridge. botman and aslan still have no native ROS of either generation —
`rclpy`/`rospy` only exist inside their in-use Docker containers.

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
because tars's own stack is ROS 1. Regardless of which adapter, `navigate_to_pose` has no
real target on this robot today — tars drives via gbplanner/RosBuzz, not `move_base` or
Nav2, so a goal will correctly report `nav_status: failed` rather than hang, but nothing
will actually navigate until a translation layer exists onto that stack.

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

**Current use: active — do not disrupt.** Four SSH sessions from `192.168.1.109` are
logged in as the shared `botman` user, and two containers were running at the time of
this audit:
- `dinonav_ros2_2` (`dinonav_ros2:dev`, up ~1 h) — `/ssd/VNMs/DinoNav` mounted, GPU
  passthrough, X11 forwarded.
- `bunker_super_odom` (`bunker_super_odom:dev`, up ~2 h) — `/ssd/mist_ws` mounted, same
  GPU/X11 setup; two of the four sessions are `docker exec -it` shells attached to it.

This looks like someone actively running SuperOdometry-based SLAM and DinoNav navigation
on the real robot. Nothing was stopped, restarted, or exec'd into on this container during
terrain prep, and any future work here should check `who`/`docker ps` again first.

**ROS state:** no native `/opt/ros` at all — this robot runs ROS 2 exclusively through
Docker. Images relevant to navigation/SLAM: `bunker:dev`, `bunker_super_odom:dev`,
`bunker_steering:dev`, `dinonav_ros2:dev`, `liosam-humble-jammy`, `phntm/bridge:humble`
(a remote-bridge/teleop client — `phntm_bridge_client` also exists under `/ssd`). The
standard workflow is `cd /ssd/mist_ws && ./docker_setup.sh` (also present under
`/ssd/VNMs/DinoNav` and `/ssd/steering_vector/SafeGNM`). `mist_ws` is a 173-entry ROS 2
workspace (`control`, `driver`, plus `build`/`install`) — the MIST lab's shared multi-robot
codebase, not SwarmDeck-specific.

**Gap to a working adapter:** `rclpy` only exists inside these project-specific
containers, none of which currently have `websockets`/`pyyaml` installed, and installing
into a live, in-use `--rm` container wouldn't persist anyway. The adapter needs its own
image (or an exec into `bunker_super_odom` once it's safe to touch, with deps installed
each session) — not attempted here, both because it wasn't necessary for "get the code
onto the robot" and because the machine is mid-use.

**SwarmDeck terrain prepared:**
- Cloned to `/ssd/swarmdeck`.
- `websockets` installed `--user` (`pip3 install --user websockets`); `pyyaml`/`numpy`
  were already present at the system Python level.
- `python3 adapter_ros2.py --help` fails at `import rclpy`, as expected — no native ROS 2
  exists outside the containers above.

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
- **botman/aslan still have no `rclpy`/`rospy` access set up.** Both need a dedicated
  container (their existing `bunker*` images are shared, in-use, and `--rm`, so anything
  installed into them disappears on exit and risks another person's live session). This
  is real bring-up work, matches `docs/hardware-bringup.md` step 0-1, and deserves a robot
  in hand and nobody mid-SLAM-run on it.
- **No `adapters/adapter_ros2/config/<robot>.yaml` for the bunkers.** Unlike scout, their
  driver stacks weren't read file-by-file this pass — `generic.yaml` is the documented
  starting point, but filling it in properly means reading real topic names off the
  `bunker`/`bunker_super_odom` containers, and neither was safe to interrupt for that here.
  `scout_mini.yaml` (written this pass, see the scout section above) was derived purely by
  reading `mist_ws`'s launch/config files, not by querying a running graph — treat its
  topic and frame names as a documented hypothesis, not a verified fact, until someone runs
  `rostopic echo`/`hz` against them.
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

For the bunkers, coordinate with whoever is running
`dinonav_ros2_2`/`bunker_super_odom`/`bunker` before building or attaching anything to
those containers, then read their driver stacks the same way this pass did for scout
before writing `botman.yaml`/`aslan.yaml`.
