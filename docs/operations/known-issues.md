# Known issues

| Area | Limitation and response |
|---|---|
| Auto map registration | Requires overlapping known space and may refuse repetitive or disjoint maps. Inspect `GET /api/map/status`; use `static` mode with surveyed transforms when available. |
| Collaborative merge | RTAB-Map grids and Swarm-SLAM trajectories currently disagree by metres. Treat `cslam` as experimental; do not use its transforms for physical navigation. |
| 3D simulation | `GRID_3D=true` preserves cloud height but roughly halves simulation speed. Leave it off unless the 3D structure is needed. |
| DLIO in Gazebo | Simulated clouds lack per-point timestamps, so DLIO cannot demonstrate de-skewing. Do not extrapolate its Gazebo result to hardware. |
| Odometry covariance | The simulation EKF is tuned for its current measurement model. Enabling `FUSE_COVARIANCE=true` requires retuning process noise. |
| Gazebo lidar | Height-filtered planar mapping requires one or an odd number of vertical rings; even counts have no horizontal return. Low horizontal sample counts make distant walls discontinuous; use the configured lidar profiles. |
| Detection prompts | YOLOE results depend on prompt and hardware domain. Validate catalog changes with `tests/perception/test_catalog_recall.py` and use operator review. |
| Hardware frames | Camera/lidar extrinsics must be measured. Botman intentionally refuses deployment without its six OAK mount values. |
| ROS domains | Keep each hardware stack on its documented `ROS_DOMAIN_ID` unless the entire graph is reconfigured together. |
| Remote access | SwarmDeck has no authentication. Put an authenticating proxy in front of any hardware-facing deployment. |
