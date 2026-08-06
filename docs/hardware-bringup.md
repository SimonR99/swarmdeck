# Hardware bring-up

A checklist for putting SwarmDeck on real robots, ordered so that **every step is
verifiable before the next one depends on it**. Skipping ahead is how a fleet ends up with
four robots and no idea which layer is broken.

**Nothing here has been run against physical hardware.** The code exists, is unit-tested
where it can be without a robot, and the sim-only assumptions are now parameters rather
than constants — but every timeout, QoS choice and topic name is a hypothesis until a robot
proves it. Treat this as a test plan, not a deployment runbook.

## 0. Before touching a robot

Confirm the stack works end to end with no ROS at all:

```bash
make docker-up-mock          # synthetic fleet, no Gazebo
```

Four robots, a merged map and camera panels should appear at <http://localhost:5173>. If
this fails, the problem is the backend or UI, and no amount of robot debugging will help.

## 1. One robot, state only

Get `robot_state` flowing before anything else. Start with the generic config and disable
everything optional:

```bash
python3 adapters/adapter_ros2/adapter_ros2.py \
  --robot-id robot_0 \
  --config adapters/adapter_ros2/config/generic.yaml \
  --host <operator-host>
```

**Check, in this order:**

| Symptom | Almost certainly |
|---|---|
| Robot never appears in the GUI | websocket cannot reach the backend; check `--host` and firewall |
| Appears but pose is stuck at origin | no `map -> base_link` in TF. The adapter logs this once and falls back to raw odometry |
| Pose drifts steadily and never corrects | you are on the odometry fallback — localisation or SLAM is not publishing TF |
| Robot flickers online/offline | `robot_id` is not stable across reconnects (protocol rule 5) |

The adapter names the two frames it looks up (`map_frame`, `base_frame`). If the robot's
stack uses different ones — `odom_combined`, `base_footprint` — change them in the config
rather than adding a transform to paper over it.

## 2. Add the map

Set `topics.map` and confirm a grid reaches the GUI. The single most likely failure is
**QoS**: a map is normally latched (`TRANSIENT_LOCAL`), and a `VOLATILE` subscriber
receives nothing at all — silently. The adapter subscribes latched for this reason. If the
robot publishes `VOLATILE` instead, it will work anyway; the reverse would not.

Verify the merged map against something you can measure. `merge_mode: static` with
configured start poses is the honest first step, because it needs no registration to be
correct.

## 3. Add teleop, carefully

Set `topics.cmd_vel`. **Test the deadman before testing the driving**: hold a direction in
the GUI, then pull the network cable or kill the backend. The robot must stop within
`drive_timeout_s` (0.45 s default).

A robot that keeps executing the last velocity it heard after losing its operator is the
failure that hurts someone. This is unit-tested in `test_adapter_ros2.py`, but test it on
the actual robot, with a hand on the e-stop.

## 4. Add navigation

Set `actions.navigate_to_pose`. The adapter maps the protocol's `navigate_to` onto Nav2's
action; a robot using `move_base` or a vendor SDK needs that method replaced, and nothing
else. Goals arrive in the robot's **own** navigation-map frame — the backend converts from
the merged frame using the same transform as map merging.

## 5. Add the camera last

Set `topics.camera_compressed` in preference to `camera`. A raw stream at full rate is the
most expensive thing an adapter can subscribe to over a robot's network, and the preview is
throttled to 5 Hz regardless.

## 6. Only now, a second robot

Everything above is per-robot and independent. Fleet problems start here:

```bash
docker compose -f docker-compose.yml -f docker-compose.zenoh.yml up -d
```

On each robot:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
# session config: { mode: "client", connect: { endpoints: ["tcp/<host>:7447"] } }
```

Default DDS discovery is multicast, which most Wi-Fi networks drop or flood, and it scales
as O(n²) in participants. Zenoh's router model replaces it. **This is the least-tested
piece in the whole repo** — the simulation overlays share a Linux IPC namespace instead,
which cannot work across machines.

## Settings that MUST change from their simulation defaults

| Setting | Sim default | Hardware |
|---|---|---|
| `deskew` (`slam_rtabmap.launch.py`) | `false` | **`true`** — Gazebo has no per-point timestamps; real drivers do, and this is worth 0.8 m at 10 m while turning |
| `lidar_x` / `lidar_z` | `-0.07` / `0.402` | From that unit's URDF or a calibration. A wrong extrinsic tilts every scan in a way SLAM cannot recover from |
| `covariance_relay.py` | runs on the 3D path | Should not run. It invents covariance because Gazebo publishes zeros; real drivers publish their own |
| `fuse_covariance` | `false` | Re-tune `process_noise_covariance` against real sensors first — it was tuned against Gazebo's all-zero covariance and real covariance made it 10x worse in sim |
| `explore.py` | drives the fleet | Do not run. It publishes `cmd_vel` directly and fights Nav2; it is a scenario driver, not autonomy |

## What is still missing regardless

- **No video pipeline.** MediaMTX/WHEP is unbuilt; the 5 Hz JPEG preview is a development
  fallback, not the <300 ms target.
- **Collaborative SLAM produces a partial map** at verification thresholds strict enough to
  be accurate. The grid-registration path is the accurate one.
- **Nothing detects wheel slip in the estimator.** `explore.py` detects a wedged robot
  behaviourally, but odometry still integrates phantom motion.

See `docs/hardware-readiness.md` for the full audit and `docs/KNOWN_ISSUES.md` for the
measurements behind each of these.
