# Physical fleet

| Robot ID | Platform | Compute / ROS | Localization | Sensors | Deploy target |
|---|---|---|---|---|---|
| `tars_0` | AgileX Scout Mini | Jetson AGX Xavier, ROS 1 Noetic | LVI-SAM / filtered odometry | Ouster OS1, VectorNav, RealSense D435 | `scout` (`192.168.1.230`) |
| `botman_0` | AgileX Bunker | Jetson AGX Orin, ROS 2 Humble | SuperOdometry | Ouster OS1-128, OAK-D Pro RGB-D | `botman@192.168.1.49` |
| `aslan_0` | AgileX Bunker | Jetson AGX Orin, ROS 2 Humble | SuperOdometry | Ouster OS0-64, VectorNav, OAK-D | `aslan@192.168.1.139` |
| `spot_0` | Boston Dynamics Spot | Jetson AGX Orin, ROS 2 Humble | LIO-SAM | Ouster, VectorNav, RealSense D435 | `spot` (`192.168.1.192`) |

The operator/server address is `BACKEND_HOST` in `deploy/fleet.env`. Hardware
profiles keep their documented ROS domains separate. Video uses RTSP ingest on
8554 and WHEP/WebRTC on 8889; a Zenoh router may use TCP 7447.

See [hardware bring-up](../operations/hardware-bringup.md) for the common
procedure and the per-robot pages for prerequisites and safety exceptions.
