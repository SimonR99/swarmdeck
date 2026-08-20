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
containers, then `scripts/scout-up` starts the native ROS graph and verifies the
ROS master, lidar, odometry, camera, planner, adapter, and media subscriptions.
Use `scripts/scout-up` directly only after deployment.

The repository currently has no unified Scout shutdown helper. Compose services
can be stopped with the generated `.deploy/scout.env`; native ROS launches must
also be stopped using the robot's process supervisor or launch-session procedure.
