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
CONFIG="${SWARMDECK_CONFIG:-/app/study/4robot.yaml}"
HEADLESS="${HEADLESS:-true}"
ADAPTER_DELAY="${ADAPTER_DELAY:-45}"

mkdir -p /app/sessions

echo "[gazebo] launching session config=${CONFIG} headless=${HEADLESS}"
ros2 launch swarmdeck_bringup session.launch.py \
  "config:=${CONFIG}" \
  "headless:=${HEADLESS}" &
LAUNCH_PID=$!

cleanup() {
  echo "[gazebo] shutting down"
  kill "${ADAPTER_PID:-}" "${LAUNCH_PID}" 2>/dev/null || true
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
python3 /app/adapters/adapter_sim/adapter_sim.py \
  --host "${BACKEND_HOST}" \
  --port "${BACKEND_PORT}" &
ADAPTER_PID=$!

# Exit if either child dies.
while kill -0 "${LAUNCH_PID}" 2>/dev/null && kill -0 "${ADAPTER_PID}" 2>/dev/null; do
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
