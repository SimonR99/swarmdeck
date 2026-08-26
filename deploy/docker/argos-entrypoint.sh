#!/usr/bin/env bash
# Wait for the ROS side to publish an experiment, then run the simulator.
set -eo pipefail
set -u

RUNTIME_DIR="${RUNTIME_DIR:-/run/swarmdeck}"
EXPERIMENT="${EXPERIMENT:-$RUNTIME_DIR/session.argos}"

# The `sim` service generates the world and the experiment (it is the one with
# the session config) and BINDS the bridge socket; ARGoS dials it. So this
# container starts last and waits, rather than racing and failing.
echo "[argos] waiting for ${EXPERIMENT}"
for _ in $(seq "${WAIT_SECONDS:-300}"); do
  [ -f "${EXPERIMENT}" ] && break
  sleep 1
done
if [ ! -f "${EXPERIMENT}" ]; then
  echo "[argos] no experiment at ${EXPERIMENT}; is the sim service up?" >&2
  exit 1
fi

# Headless Vulkan. On a machine with a usable GPU the device ICD is present in
# /usr/share/vulkan/icd.d and is chosen; VK_DRIVER_FILES is set only to force
# the software rasterizer, which is what CI and GPU-less workstations get.
# Measured on lavapipe with four robots, a 17-ring lidar and 320x240 RGB-D
# cameras: 0.93x real time. Gazebo's CPU raytracing managed 0.58x.
if [ "${SWARMDECK_SOFTWARE_RENDER:-false}" = "true" ]; then
  export VK_DRIVER_FILES=/usr/share/vulkan/icd.d/lvp_icd.json
  echo "[argos] forcing the software Vulkan rasterizer"
fi

# The installed plugins plus SwarmDeck's own module, which lives beside them.
export ARGOS_PLUGIN_PATH="${ARGOS_PLUGIN_PATH:-/usr/local/lib/argos3}"

echo "[argos] vulkan devices:"
vulkaninfo --summary 2>/dev/null | sed -n '/Devices:/,/^$/p' | head -20 \
  || echo "[argos]   (vulkaninfo unavailable)"

# Run from the runtime directory: the experiment names the world and the props
# by absolute path, but a relative one in a hand-edited file then still
# resolves against the place the generated assets actually are.
cd "${RUNTIME_DIR}"
echo "[argos] argos3 -c ${EXPERIMENT}"
exec argos3 -c "${EXPERIMENT}"
