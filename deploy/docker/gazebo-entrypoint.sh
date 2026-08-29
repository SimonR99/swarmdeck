#!/usr/bin/env bash
# Start Gazebo session (world, fleet, bridges, SLAM, Nav2) then adapter_sim.
set -eo pipefail

# ROS setup scripts reference optional unbound vars; disable nounset while sourcing.
set +u
source /opt/ros/jazzy/setup.bash
source /app/swarmdeck_ros/install/setup.bash
set -u

export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

BACKEND_HOST="${BACKEND_HOST:-server}"
BACKEND_PORT="${BACKEND_PORT:-8080}"
CONFIG="${SWARMDECK_CONFIG:-/app/configs/4robot.yaml}"
HEADLESS="${HEADLESS:-true}"
# Must outlast session.launch.py's staggered per-robot bringup, or the adapter
# starts advertising robots whose SLAM is not active yet.
ADAPTER_DELAY="${ADAPTER_DELAY:-60}"
SLAM_BACKEND="${SLAM_BACKEND:-toolbox}"
FUSE_IMU="${FUSE_IMU:-true}"
FUSE_COVARIANCE="${FUSE_COVARIANCE:-false}"
GRID_3D="${GRID_3D:-false}"
# Seconds of reactive exploration after startup, to bootstrap the maps before the
# operator takes over. 0 leaves the fleet stationary until a goal arrives.
EXPLORE_SECONDS="${EXPLORE_SECONDS:-0}"
# Simulation exposes raw sensor_msgs/Image topics. The shared media publisher
# JPEG-encodes those frames, then sends one low-latency H.264/RTSP stream per
# robot to MediaMTX, exactly like the hardware media services do.
VIDEO_BITRATE_KBPS="${VIDEO_BITRATE_KBPS:-700}"
VIDEO_FPS="${VIDEO_FPS:-10}"
VIDEO_WIDTH="${VIDEO_WIDTH:-640}"
VIDEO_HEIGHT="${VIDEO_HEIGHT:-480}"
MEDIA_ROBOT_COUNT="${SWARMDECK_ROBOT_COUNT:-4}"

if ! [[ "${MEDIA_ROBOT_COUNT}" =~ ^[1-5]$ ]]; then
  echo "[gazebo] invalid SWARMDECK_ROBOT_COUNT=${MEDIA_ROBOT_COUNT}; using 4 for media" >&2
  MEDIA_ROBOT_COUNT=4
fi

mkdir -p /app/sessions

echo "[gazebo] launching session config=${CONFIG} headless=${HEADLESS}" \
     "slam_backend=${SLAM_BACKEND} fuse_imu=${FUSE_IMU}" \
     "explore_seconds=${EXPLORE_SECONDS}"
ros2 launch swarmdeck_bringup session.launch.py \
  "config:=${CONFIG}" \
  "headless:=${HEADLESS}" \
  "slam_backend:=${SLAM_BACKEND}" \
  "fuse_imu:=${FUSE_IMU}" \
  "fuse_covariance:=${FUSE_COVARIANCE}" \
  "grid_3d:=${GRID_3D}" \
  "explore_seconds:=${EXPLORE_SECONDS}" &
LAUNCH_PID=$!

cleanup() {
  echo "[gazebo] shutting down"
  for pid in "${MEDIA_PIDS[@]:-}" "${ADAPTER_PID:-}" "${LAUNCH_PID}"; do
    if [ -n "${pid}" ]; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  wait || true
}
trap cleanup EXIT INT TERM

echo "[gazebo] waiting ${ADAPTER_DELAY}s for sim/SLAM/Nav2 before adapter_sim"
sleep "${ADAPTER_DELAY}"

if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  echo "[gazebo] session.launch exited early" >&2
  wait "${LAUNCH_PID}" || true
  exit 1
fi

echo "[gazebo] starting adapter_sim -> ${BACKEND_HOST}:${BACKEND_PORT}"
ADAPTER_ARGS=(--host "${BACKEND_HOST}" --port "${BACKEND_PORT}")
if [ -n "${SWARMDECK_ROBOT_COUNT:-}" ]; then
  ADAPTER_ARGS+=(--robots "${SWARMDECK_ROBOT_COUNT}")
fi

MEDIA_PIDS=()
echo "[gazebo] starting ${MEDIA_ROBOT_COUNT} camera RTSP publisher(s) -> mediamtx:8554"
for ((i = 0; i < MEDIA_ROBOT_COUNT; i++)); do
  ROBOT_ID="robot_${i}"
  # The Gazebo bridge publishes raw Image only. Keep the compressed topic
  # distinct so ros2_rtsp subscribes to the raw fallback without a ROS type
  # collision on the actual Image topic.
  python3 /app/adapters/media/ros2_rtsp.py \
    --robot-id "${ROBOT_ID}" \
    --topic "/${ROBOT_ID}/camera/image/compressed" \
    --raw-topic "/${ROBOT_ID}/camera/image" \
    --rtsp-url "rtsp://mediamtx:8554/${ROBOT_ID}" \
    --bitrate-kbps "${VIDEO_BITRATE_KBPS}" \
    --fps "${VIDEO_FPS}" \
    --width "${VIDEO_WIDTH}" \
    --height "${VIDEO_HEIGHT}" &
  MEDIA_PIDS+=("$!")
done

python3 /app/adapters/adapter_sim/adapter_sim.py "${ADAPTER_ARGS[@]}" &
ADAPTER_PID=$!

# Exit if the session, adapter, or a media publisher dies. The publisher has
# its own RTSP reconnect loop, so a dead process indicates a real configuration
# or dependency failure that should restart the simulation container.
while kill -0 "${LAUNCH_PID}" 2>/dev/null && kill -0 "${ADAPTER_PID}" 2>/dev/null; do
  for pid in "${MEDIA_PIDS[@]}"; do
    if ! kill -0 "${pid}" 2>/dev/null; then
      echo "[gazebo] camera media publisher ${pid} ended" >&2
      wait "${pid}" || true
      exit 1
    fi
  done
  sleep 2
done

if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  echo "[gazebo] session.launch ended" >&2
  wait "${LAUNCH_PID}" || true
  exit 1
fi
echo "[gazebo] adapter_sim ended" >&2
wait "${ADAPTER_PID}" || true
exit 1
