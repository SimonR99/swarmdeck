#!/usr/bin/env bash
# Bring up the simulated stack for manual testing / debugging.
#   ./run_stack.sh <n_robots>
# Stop with ./stop_stack.sh
set -o pipefail

N="${1:-2}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SIM="$REPO/swarmdeck_ros/src/swarmdeck_sim"
WORLD="$SIM/worlds/indoor.sdf"
LOGS="${SWARMDECK_LOGS:-/tmp/swarmdeck-logs}"
mkdir -p "$LOGS"

set +u; source /opt/ros/jazzy/setup.bash 2>/dev/null; set -u
export QT_QPA_PLATFORM=offscreen

bg() { setsid nohup "$@" </dev/null >>"$LOGS/$(basename "$1").log" 2>&1 & disown; }

echo "[1/6] world"
python3 "$SIM/scenario/generate_world.py" --seed 20260801 -o "$WORLD" >/dev/null

echo "[2/6] gazebo"
setsid nohup gz sim -s -r --headless-rendering -v 1 "$WORLD" </dev/null >"$LOGS/gz.log" 2>&1 & disown
sleep 15

echo "[3/6] spawn $N robots"
python3 "$SIM/scenario/spawn_fleet.py" --config "$REPO/configs/${N}robot.yaml" 2>&1 | sed 's/^/      /'
sleep 8

echo "[4/6] clock bridge"
setsid nohup ros2 run ros_gz_bridge parameter_bridge \
  /world/swarmdeck_indoor/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  --ros-args -r /world/swarmdeck_indoor/clock:=/clock \
  </dev/null >"$LOGS/clock.log" 2>&1 & disown
sleep 4

echo "[5/6] per-robot bridges"
for i in $(seq 0 $((N-1))); do
  ns="robot_$i"
  setsid nohup ros2 run ros_gz_bridge parameter_bridge \
    "/$ns/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" \
    "/$ns/proximity_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" \
    "/$ns/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry" \
    "/$ns/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry" \
    "/$ns/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist" \
    --ros-args -p use_sim_time:=true \
    </dev/null >"$LOGS/bridge_$ns.log" 2>&1 & disown
  setsid nohup ros2 run ros_gz_bridge parameter_bridge \
    "/$ns/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V" \
    --ros-args -r "/$ns/tf:=/tf" -p use_sim_time:=true \
    </dev/null >"$LOGS/tf_$ns.log" 2>&1 & disown
  setsid nohup ros2 run tf2_ros static_transform_publisher \
    --x -0.07 --z 0.402 --frame-id "$ns/base_link" --child-frame-id "$ns/base_link/lidar" \
    --ros-args -p use_sim_time:=true \
    </dev/null >"$LOGS/tfs_$ns.log" 2>&1 & disown
  setsid nohup ros2 run tf2_ros static_transform_publisher \
    --x 0.24 --z 0.05 --frame-id "$ns/base_link" --child-frame-id "$ns/base_link/proximity_lidar" \
    --ros-args -p use_sim_time:=true \
    </dev/null >"$LOGS/tfs_proximity_$ns.log" 2>&1 & disown
done
sleep 8

echo "[6/6] slam + lifecycle"
for i in $(seq 0 $((N-1))); do
  ns="robot_$i"
  setsid nohup ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
    -p use_sim_time:=true -p odom_frame:="$ns/odom" -p base_frame:="$ns/base_link" \
    -p map_frame:="$ns/map_frame" -p resolution:=0.05 -p max_laser_range:=16.0 \
    -p transform_timeout:=1.0 -p minimum_travel_distance:=0.2 -p map_update_interval:=1.0 \
    -r __node:="slam_$ns" -r scan:="/$ns/scan" -r /map:="/$ns/map" \
    </dev/null >"$LOGS/slam_$ns.log" 2>&1 & disown
done
sleep 10
for i in $(seq 0 $((N-1))); do
  ros2 lifecycle set "/slam_robot_$i" configure >/dev/null 2>&1
  sleep 1
  ros2 lifecycle set "/slam_robot_$i" activate >/dev/null 2>&1
  echo "      slam_robot_$i: $(ros2 lifecycle get /slam_robot_$i 2>/dev/null)"
done

echo "stack up. logs in $LOGS"
