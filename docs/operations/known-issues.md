# Known issues

| Area | Limitation, trap, and mitigation |
|---|---|
| Python environment & GTSAM segfault | `slam/` is strictly pinned to Python 3.12 and `numpy<2`. GTSAM 4.2.2 segfaults with bare SIGSEGV under NumPy 2.x (Python 3.13). Never combine `server/.venv` (3.13) and `slam/.venv` (3.12). |
| GTSAM tangent vector order | Information matrices and tangent vectors in `slam/` are 6x6 **rotation first** ($\omega_x, \omega_y, \omega_z, v_x, v_y, v_z$). Ordering mistakes fail silently without errors; guarded by tests in `slam/tests/`. |
| Transform naming direction | Every transform `T_a_b` maps points from frame `b` to frame `a` ($p_a = T_{a\_b} \cdot p_b$). |
| PCM clique threshold | `min_pcm_clique_size = 2`: trajectory merging requires at least two corroborating inter-robot loop closures. A single drive-by overlap will deliberately remain unmerged until re-visited. |
| Motion skew at turn rates | In Gazebo, spinning lidar clouds have no per-point timestamps and distort during fast yaw (> 8 deg/s), causing GICP to lock onto warped structure. `keyframe_producer.py` drops frames above `DEFAULT_MAX_YAW_RATE = 0.14 rad/s`. |
| Simulation reset boundary | The `reset` capability is strictly simulation-only (`adapter_sim`, `mock_adapter`). Hardware adapters (`adapter_ros1`, `adapter_ros2`) must never advertise or implement `reset`. |
| Auto map registration overhead | In `merge_mode: auto`, grid correlation costs ~2.44 s per ingest if reference robot is missing. Use `merge_mode: graph` (default in `configs/4robot.yaml` and `hardware_fleet.yaml`) for trajectory-based merging. |
| Collaborative merge (legacy cslam) | RTAB-Map grids and Swarm-SLAM trajectories disagree by 11–16 m. Treat `merge_mode: cslam` as a legacy diagnostic; use `merge_mode: graph` for production. |
| 3D simulation | `GRID_3D=true` preserves cloud height but roughly halves simulation speed. Leave it off unless the 3D structure is needed. |
| DLIO in Gazebo | Simulated clouds lack per-point timestamps, so DLIO cannot demonstrate de-skewing. Do not extrapolate its Gazebo result to hardware. |
| Odometry covariance | The simulation EKF is tuned for its current measurement model. Enabling `FUSE_COVARIANCE=true` requires retuning process noise. |
| Gazebo lidar ring parity | Height-filtered planar mapping requires 1 or an odd number of vertical rings (e.g., 33); even counts leave no beam at elevation 0 and distant walls truncate. |
| Detection prompts | YOLOE results depend on prompt and hardware domain. Validate catalog changes with `tests/perception/test_catalog_recall.py` inside the detector container. |
| Hardware frames | Camera/lidar extrinsics must be measured. Botman intentionally refuses deployment without its six OAK mount values. |
| ROS domains | Keep each hardware stack on its documented `ROS_DOMAIN_ID` unless the entire graph is reconfigured together. |
| Remote access | SwarmDeck has no authentication. Put an authenticating proxy in front of any hardware-facing deployment. |
