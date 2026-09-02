#!/usr/bin/env bash
# Start the ROS side of the simulated fleet, then adapter_sim.
#
# The simulator itself is NOT started here: `launch_argos:=false` leaves it to
# the `argos` service, which is the container with Vulkan and no ROS. What this
# does start is the world and experiment generation, the bridge that ARGoS
# dials, and the per-robot SLAM and Nav2 stacks.
set -eo pipefail

# ROS setup scripts reference optional unbound vars; disable nounset while sourcing.
set +u
source /opt/ros/jazzy/setup.bash
source /app/swarmdeck_ros/install/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

BACKEND_HOST="${BACKEND_HOST:-server}"
BACKEND_PORT="${BACKEND_PORT:-8080}"
CONFIG="${SWARMDECK_CONFIG:-/app/configs/4robot.yaml}"
RUNTIME_DIR="${RUNTIME_DIR:-/run/swarmdeck}"
SIM_BACKEND="${SWARMDECK_SIM_BACKEND:-argos}"
# Must outlast session.launch.py's staggered per-robot bringup, or the adapter
# starts advertising robots whose SLAM is not active yet.
ADAPTER_DELAY="${ADAPTER_DELAY:-60}"
SLAM_BACKEND="${SLAM_BACKEND:-toolbox}"
GRID_3D="${GRID_3D:-false}"
TARGETS="${SWARMDECK_TARGETS:-10}"
# external = Ultra-Fusion. drift = ARGoS's synthetic model, ~4x faster and
# correspondingly less faithful; see docs/architecture/simulation.md. With
# drift the generated experiment declares no <external_estimator>, so the
# ultrafusion service is simply not needed.
ODOMETRY="${SWARMDECK_ODOMETRY:-external}"
# Seconds of reactive exploration after startup, to bootstrap the maps before
# the operator takes over. 0 leaves the fleet stationary until a goal arrives
# or the operator presses Explore. Read by adapter_sim, which owns the process;
# exported below so it reaches that process rather than the launch file.
EXPLORE_SECONDS="${EXPLORE_SECONDS:-0}"
export EXPLORE_SECONDS

mkdir -p /app/sessions "${RUNTIME_DIR}"

# Clear the previous run's generated experiment BEFORE anything regenerates it.
#
# The runtime directory is a named volume and outlives the containers, and the
# argos service waits for session.argos to exist and then reads it. Left in
# place, a stale file is one it can read and act on before this launch has
# rewritten it: switching to `odometry:=drift` left ARGoS waiting on the
# estimator socket named in the PREVIOUS run's experiment, which nothing was
# ever going to bind. Any change of robot count, world seed or fleet config has
# the same hazard.
rm -f "${RUNTIME_DIR}/session.argos" "${RUNTIME_DIR}/indoor.gltf" \
      "${RUNTIME_DIR}/indoor.bin" "${RUNTIME_DIR}/indoor_collision.gltf" \
      "${RUNTIME_DIR}/indoor_collision.bin"

echo "[sim] launching session config=${CONFIG} backend=${SIM_BACKEND}" \
     "slam_backend=${SLAM_BACKEND} explore_seconds=${EXPLORE_SECONDS}"
ros2 launch swarmdeck_bringup session.launch.py \
  "config:=${CONFIG}" \
  "sim_backend:=${SIM_BACKEND}" \
  "runtime_dir:=${RUNTIME_DIR}" \
  "launch_argos:=false" \
  "targets:=${TARGETS}" \
  "odometry:=${ODOMETRY}" \
  "headless:=true" \
  "slam_backend:=${SLAM_BACKEND}" \
  "grid_3d:=${GRID_3D}" \
  &
LAUNCH_PID=$!

cleanup() {
  echo "[sim] shutting down"
  kill "${ADAPTER_PID:-}" "${LAUNCH_PID}" "${MEDIA_PIDS[@]:-}" 2>/dev/null || true
  wait || true
}
trap cleanup EXIT INT TERM

echo "[sim] waiting ${ADAPTER_DELAY}s for the bridge, SLAM and Nav2 before adapter_sim"
sleep "${ADAPTER_DELAY}"

if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  echo "[sim] session.launch exited early" >&2
  wait "${LAUNCH_PID}" || true
  exit 1
fi

echo "[sim] starting adapter_sim -> ${BACKEND_HOST}:${BACKEND_PORT}"
ADAPTER_ARGS=(--host "${BACKEND_HOST}" --port "${BACKEND_PORT}")
if [ -n "${SWARMDECK_ROBOT_COUNT:-}" ]; then
  ADAPTER_ARGS+=(--robots "${SWARMDECK_ROBOT_COUNT}")
fi
# The adapter resets a simulation it did not start, and the two backends reset
# differently. It cannot probe for the difference cheaply, so it is told.
export SWARMDECK_SIM_BACKEND="${SIM_BACKEND}"
python3 /app/adapters/adapter_sim/adapter_sim.py "${ADAPTER_ARGS[@]}" &
ADAPTER_PID=$!

# Video, the same way a real robot delivers it: H.264 to MediaMTX over RTSP,
# then WHEP to the browser. adapter_sim deliberately sends no images over the
# adapter connection (it runs the detector on them and nothing else), so
# without these the dashboard's camera panel has no source for a simulated
# robot and reports it unavailable.
#
# --raw-topic, not --topic: the ARGoS bridge publishes sensor_msgs/Image, and
# nothing in simulation produces a compressed one.
MEDIA_PIDS=()
if [ "${SWARMDECK_VIDEO:-true}" = "true" ]; then
  COUNT="${SWARMDECK_ROBOT_COUNT:-$(python3 - "$CONFIG" <<'PY'
import sys, yaml
print((yaml.safe_load(open(sys.argv[1])).get("fleet") or {}).get("robot_count", 4))
PY
)}"
  PREFIX="$(python3 - "$CONFIG" <<'PY'
import sys, yaml
print((yaml.safe_load(open(sys.argv[1])).get("fleet") or {}).get("robot_prefix", "robot_"))
PY
)"
  for i in $(seq 0 $((COUNT - 1))); do
    RID="${PREFIX}${i}"
    echo "[sim] video ${RID} -> rtsp://${MEDIA_HOST:-mediamtx}:${MEDIA_RTSP_PORT:-8554}/${RID}"
    python3 /app/adapters/media/ros2_rtsp.py \
      --robot-id "${RID}" \
      --topic "/${RID}/camera/image/compressed" \
      --raw-topic "/${RID}/camera/image" \
      --rtsp-url "rtsp://${MEDIA_HOST:-mediamtx}:${MEDIA_RTSP_PORT:-8554}/${RID}" \
      --bitrate-kbps "${VIDEO_BITRATE_KBPS:-700}" \
      --fps "${VIDEO_FPS:-10}" \
      --width "${VIDEO_WIDTH:-640}" \
      --height "${VIDEO_HEIGHT:-480}" &
    MEDIA_PIDS+=($!)
  done
fi

# Exit if either child dies.
while kill -0 "${LAUNCH_PID}" 2>/dev/null && kill -0 "${ADAPTER_PID}" 2>/dev/null; do
  sleep 2
done

if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  echo "[sim] session.launch ended" >&2
  wait "${LAUNCH_PID}" || true
  exit 1
fi
echo "[sim] adapter_sim ended" >&2
wait "${ADAPTER_PID}" || true
exit 1
