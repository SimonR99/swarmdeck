#!/usr/bin/env bash
# Runs Fast-LIVO2 against a live SwarmDeck ARGoS simulation.
#
# Unlike Ultra-Fusion (which only ships x86_64 binary .debs), Fast-LIVO2 compiles
# from source and deploys natively on both ARM64 (Jetson / Apple Silicon / ARM servers)
# and x86_64 architectures.
#
# Sensor extrinsics and camera intrinsics are extracted dynamically per-robot from
# the generated ARGoS experiment file so that heterogeneous fleet configurations
# (Bunker, Scout Mini, Spot) remain strictly calibrated.

# Source ROS 2 setup
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi

if [ -f "/fast_livo_ws/install/setup.bash" ]; then
    source /fast_livo_ws/install/setup.bash
fi

set -u

export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-/tools/fastdds_large.xml}

# Check kernel socket buffer limits for high-rate point clouds
RMEM_MAX=$(cat /proc/sys/net/core/rmem_max 2>/dev/null || echo 0)
RMEM_NEEDED=7000000
if [ "$RMEM_MAX" -lt "$RMEM_NEEDED" ]; then
    echo "############################################################" >&2
    echo "WARNING: net.core.rmem_max is ${RMEM_MAX}, below ${RMEM_NEEDED}." >&2
    echo "High-rate multi-robot LiDAR streams may drop scans under default buffers." >&2
    echo "To raise on host: sudo sysctl -w net.core.rmem_max=8388608" >&2
    echo "############################################################" >&2
else
    echo "=== net.core.rmem_max ${RMEM_MAX}, sufficient for 7 MB DDS buffers ==="
fi

RUNTIME_DIR=${RUNTIME_DIR:-/run/swarmdeck}
EXPERIMENT=${EXPERIMENT:-$RUNTIME_DIR/session.argos}
MOUNTS=/tools/argos_mounts.py
MAKE_PROFILE=/tools/make_profile.py
SOCK=${SOCK:-/sock/argos_uf.sock}
RESULTS=${RESULTS:-/results}

echo "=== Fast-LIVO2: Waiting for $EXPERIMENT ==="
for _ in $(seq 300); do
    [ -f "$EXPERIMENT" ] && break
    sleep 1
done
if [ ! -f "$EXPERIMENT" ]; then
    echo "no experiment at $EXPERIMENT after 300s; is the sim service up?" >&2
    exit 1
fi

read -r -a ROBOTS <<< "$(python3 "$MOUNTS" "$EXPERIMENT" --robots)"
if [ ${#ROBOTS[@]} -eq 0 ]; then
    echo "no robots found in $EXPERIMENT" >&2
    exit 1
fi
echo "=== Fast-LIVO2 Fleet: ${ROBOTS[*]} ==="

GRAVITY=${GRAVITY:-9.81}
ARGOS_IMU_NOISE=${ARGOS_IMU_NOISE:-"0.002 0.02 0.0002 0.002"}
IMU_HZ=${IMU_HZ:-100}
SCAN_LINE=${SCAN_LINE:-17}
VOXEL=${VOXEL:-0.5}

mkdir -p "$RESULTS" /tmp/fast_livo_profiles

PIDS=()

echo "=== Generating Fast-LIVO2 configuration profiles per robot ==="
for r in "${ROBOTS[@]}"; do
    LIDAR_IN_BODY="$(python3 "$MOUNTS" "$EXPERIMENT" --lidar "$r")"
    CAMERA_IN_BODY="$(python3 "$MOUNTS" "$EXPERIMENT" --camera "$r")"
    LIDAR_ELEV="$(python3 "$MOUNTS" "$EXPERIMENT" --lidar-elev "$r")"
    CAMERA_RESOLUTION="$(python3 "$MOUNTS" "$EXPERIMENT" --camera-resolution "$r" | tr " " ",")"
    CAMERA_FOV="$(python3 "$MOUNTS" "$EXPERIMENT" --camera-fov "$r")"
    echo "  $r: lidar ($LIDAR_IN_BODY) camera ($CAMERA_IN_BODY) elev ($LIDAR_ELEV)"

    python3 "$MAKE_PROFILE" \
        --out "/tmp/fast_livo_profiles/$r" \
        --lidar-in-body $LIDAR_IN_BODY \
        --camera-in-body $CAMERA_IN_BODY \
        --lidar-elev $LIDAR_ELEV \
        --camera-resolution "$CAMERA_RESOLUTION" \
        --camera-fov "$CAMERA_FOV" \
        --gravity "$GRAVITY" \
        --imu-hz "$IMU_HZ" \
        --argos-imu-noise $ARGOS_IMU_NOISE \
        --img-en "$IMG_EN" \
        --scan-line "$SCAN_LINE" \
        --voxel "$VOXEL" || exit 1
done

echo "=== Starting one Fast-LIVO2 node per robot ==="
for r in "${ROBOTS[@]}"; do
    mkdir -p "$RESULTS/$r"
    CONFIG_YAML="/tmp/fast_livo_profiles/$r/fast_livo_argos.yaml"

    # Fast-LIVO2 ROS 2 executable with namespace & topic remappings
    # The ament build installs the upstream executable name, fastlivo_mapping,
    # under lib/fast_livo. The two names probed here before
    # (fast_livo2_node / fast_livo_node) never existed, so this always fell
    # through to relay mode and no estimator ever ran.
    FLV_BIN=/fast_livo_ws/install/lib/fast_livo/fastlivo_mapping
    if [ ! -x "$FLV_BIN" ]; then
        FLV_BIN="$(command -v fastlivo_mapping || true)"
    fi
    if [ -z "$FLV_BIN" ]; then
        echo "fastlivo_mapping not found; did the image build?" >&2
        exit 1
    fi
    # FAST-LIVO2 declares every publisher with an ABSOLUTE name
    # (advertise<...>("/aft_mapped_to_init") and friends), so -r __ns has no
    # effect on any of them: all four robots would publish onto the same
    # topics, and the link, which listens on /{robot}/aft_mapped_to_init, would
    # see nothing. This is the same trap run_uf.sh documents for Ultra-Fusion.
    # Explicit -r rules do work.
    OUT_REMAPS=()
    for t in aft_mapped_to_init path cloud_registered cloud_effected Laser_map \
             cloud_visual_sub_map_before dyn_obj dyn_obj_removed dyn_obj_dbg_hist \
             planner_normal voxels; do
        OUT_REMAPS+=(-r "/$t:=/$r/$t")
    done
    OUT_REMAPS+=(-r "/LIVO2/imu_propagate:=/$r/LIVO2/imu_propagate")

    "$FLV_BIN" \
        --ros-args -r __ns:=/$r -p use_sim_time:=true \
        --params-file "$CONFIG_YAML" "${OUT_REMAPS[@]}" \
        > "$RESULTS/$r/fast_livo.log" 2>&1 &
    PIDS+=($!)
    echo "  $r -> pid ${PIDS[-1]}, log $RESULTS/$r/fast_livo.log"
done

sleep 2

echo "=== Starting Fast-LIVO2 Lockstep Socket Link ==="
python3 /tools/fast_livo_link.py --socket "$SOCK" 2>&1 | tee "$RESULTS/fast_livo_link.log"

echo "=== Stopping Fast-LIVO2 Estimators ==="
for pid in "${PIDS[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
sleep 2
for pid in "${PIDS[@]}"; do kill -KILL "$pid" 2>/dev/null || true; done

echo "=== Estimator Session Completed ==="
