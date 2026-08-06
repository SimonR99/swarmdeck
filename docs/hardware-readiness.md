# Hardware readiness

**Short answer: no. This is a simulation stack with no hardware path yet.**

Everything below is a factual audit, not an estimate of effort. It exists so that "can we
put this on a robot?" has an answer that does not require reading the whole tree.

## The blocker: there is no hardware adapter

`adapters/` contains `adapter_mock` (synthetic) and `adapter_sim` (Gazebo). That is all.
The README previously claimed adapters for `ros2`, `ros1` and `spot`; they have never
existed. The adapter is the **only** component that speaks both a robot's native interface
and the SwarmDeck protocol, so without one there is nothing to connect a real robot to.

What a hardware adapter has to do is fully specified — `adapters/protocol/README.md` is the
contract of record, and `adapter_sim` is a complete working reference at ~700 lines. The
work is real but bounded, and none of it requires changing the backend.

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

- **No video pipeline.** `swarmdeck_media` is empty and MediaMTX is not installed; the 5 Hz
  JPEG preview is a development fallback, not the <300 ms WHEP target (KNOWN_ISSUES #1).
- **The duck detector is a classical colour/shape baseline**, not a trained model, blocked
  on training-data licensing (KNOWN_ISSUES #3).
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
