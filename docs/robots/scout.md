# Scout Mini (TARS)

Scout is an AgileX Scout Mini on native ROS 1 Noetic with LVI-SAM, Ouster,
VectorNav, RealSense D435, local planning, and the ROS 1 adapter.

Prerequisites on the robot:

- SSH alias `scout`; robot IP `192.168.1.230`; checkout `/ssd/swarmdeck`.
- Catkin workspace `/ssd/catkin_ws` and MIST workspace `/ssd/mist_ws`.
- Ouster hostname and camera topic from `deploy/robots/scout.env`.

```bash
make deploy ROBOT=scout
```

Scout is the exception to the generic Compose start: deployment builds/resets
the robot-side inputs, then `scripts/scout-up` cleanly stops Scout's known native
ROS launchers, recreates the operator-side detector/adapter/media containers, and
starts the native graph again. It verifies the ROS master, lidar, odometry,
camera, planner, adapter, media subscriptions, and backend registration. Use
`scripts/scout-up` directly only after deployment.

The default deployment refreshes only the known Scout launch files and leaves
`roscore` and unrelated ROS processes alone. To preserve an already-running
native graph, use:

```bash
make deploy ROBOT=scout DEPLOY_ARGS=--no-native-reset
```

The repository currently has no unified Scout shutdown helper. Compose services
can be stopped with the generated `.deploy/scout.env`; native ROS launches must
also be stopped using the robot's process supervisor or launch-session procedure.

Scout's ROS 1 profile uses LVI-SAM's accumulated
`/lvi_sam/lidar/mapping/map_global` as the SwarmDeck map source. The adapter
projects the 3D cloud to 2D using returns from 15 cm above the floor through
1.0 m (0.150–1.000 m), and forwards the accumulated cloud to
the 3D viewer. The projection marks returned cells occupied; unobserved cells
remain unknown rather than being treated as safe free space.
