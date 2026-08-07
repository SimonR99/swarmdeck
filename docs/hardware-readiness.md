# Hardware readiness

**Short answer: a hardware path now runs on Scout, but navigation and estimator issues
remain robot-specific work.**

Everything below is a factual audit, not an estimate of effort. It exists so that "can we
put this on a robot?" has an answer that does not require reading the whole tree.

## The adapter is live on Scout

`adapters/adapter_ros2/` is the hardware adapter. Unlike `adapter_sim` it hardcodes
nothing: topic names, frames, rates and the drive deadman all come from a per-robot-type
YAML (`config/generic.yaml`, `config/duckiebot.yaml`), because a real robot's driver names
its own topics and its URDF names its own frames. Capabilities are derived from what is
actually configured, so a robot with no camera advertises none rather than advertising one
that times out.

The pose is a **tf2 lookup** between the configured `map_frame` and `base_frame`, not a
composition of transforms recognised by name. `adapter_sim` can compose a chain because we
built that TF tree; a real one has links we do not know about, and hardcoding a path
through them is exactly how this project once shipped a 0.47 m pose error nobody noticed.

The ROS 1 adapter and Dockerized media publisher have now run against the physical Scout
Mini (`tars_0`). Its RealSense compressed camera topic feeds both the detector and a
640×480, 15 fps H.264 RTSP stream; MediaMTX delivers that stream to the dashboard over
WHEP/WebRTC. See `docs/fleet-status.md` for the robot-specific state and unresolved pose
drift. Unit tests still cover the protocol behavior independently of hardware.

## Things that are correct for hardware already

Worth stating, because they are the parts that are usually wrong in a sim-first project:

- **The backend imports no ROS.** Enforced by a test (`test_backend_imports_no_ros`). A real
  fleet talks to it over the same websocket the mock fleet uses.
- **Ground truth never enters estimation or control.** Verified: `ground_truth` appears only
  in the Gazebo bridge and the robot model, never in `adapter_sim`, `explore.py` or any SLAM
  configuration. Scoring only, exactly as documented.
- **The protocol is versioned** and additive — a protocol-1 adapter still works against a
  protocol-2 backend.
- **SLAM, Nav2 and the map merge consume ordinary ROS topics.** Nothing in
  `swarmdeck_slam` or `swarmdeck_nav` depends on Gazebo; they would run against real
  sensor drivers unchanged.

## Things that are set for simulation and MUST change

| What | Where | Why it is wrong on hardware |
|---|---|---|
| `deskewing: False` | `slam_rtabmap.launch.py` | Off only because Gazebo emits no per-point timestamps. Real drivers do (Ouster `t`, Velodyne `time`, Livox `offset_time`) and de-skewing is worth 0.8 m at 10 m while turning. **Turn on.** |
| `pointcloud/deskew: false` | `swarmdeck_dlio/config/dlio.yaml` | Same reason, same fix. DLIO without de-skew is DLIO without its main idea. |
| Lidar extrinsics `-0.07, 0.402` | `slam.launch.py`, `slam_rtabmap.launch.py` | Hardcoded from the simulated model. Real robots need these from a URDF or a calibration, per unit. |
| `covariance_relay.py` | `swarmdeck_slam/nodes/` | A shim that invents covariance because Gazebo publishes zeros. Real drivers publish their own; the relay should not run. |
| EKF tuned for zero covariance | `config/ekf.yaml` | `process_noise_covariance` was tuned against Gazebo's all-zero covariance. Real covariance made it 10x worse in sim (KNOWN_ISSUES). **Re-tune against real sensors before trusting it.** |
| Shared IPC/network namespaces | `docker-compose.cslam.yml`, `docker-compose.dlio.yml` | Works because every "robot" is one container on one machine. Real robots are separate machines: this needs `rmw_zenoh_cpp` with a router, or a DDS discovery server. **Untested.** |
| `explore.py` | `swarmdeck_sim/scenario/` | A scenario driver that publishes `cmd_vel` directly and fights Nav2. Useful for bootstrapping maps in sim; it is not an autonomy stack. |

## Things that are unfinished regardless of platform

- **Video latency is not instrumented.** The WHEP path is working on Scout, but the <300 ms
  target still needs a capture/display timestamp measurement (KNOWN_ISSUES #1).
- **YOLOE duck detection has only narrow field validation.** It is a real neural model and
  worked on Botman, but every camera/range/lighting combination still needs measurement
  and the Ultralytics license must fit the deployment (KNOWN_ISSUES #3).
- **Collaborative SLAM produces a partial map.** With verification strict enough to be
  accurate, only some robots merge (KNOWN_ISSUES #5). The grid-registration path (`auto`)
  is the accurate one and is what the default stack uses.
- **Nothing detects a robot stuck with its wheels spinning** in the estimator. `explore.py`
  now detects it behaviourally and backs out, but odometry still integrates the phantom
  motion (KNOWN_ISSUES).

## Suggested order for a hardware bring-up

1. **Write the hardware adapter** against `adapters/protocol/README.md`, using `adapter_sim`
   as the reference. Validate it against the existing backend with one robot before adding
   more — the fleet-size problems are all downstream of a working single robot.
2. **Fix the extrinsics** properly: publish `base_link -> lidar` and `base_link -> imu` from
   the robot's URDF rather than the hardcoded static transforms in the launch files.
3. **Turn de-skewing on** and re-measure `icp_odometry` against DLIO. On hardware that
   comparison may well invert, and it is cheap to run once the adapter exists.
4. **Re-tune the EKF** against real covariance, or leave `fuse_imu:=false` and let lidar
   odometry own `odom -> base_link` as the 3D path already does.
5. **Replace the shared-namespace networking** with Zenoh before running more than one
   robot. This is the piece with the least evidence behind it today.
