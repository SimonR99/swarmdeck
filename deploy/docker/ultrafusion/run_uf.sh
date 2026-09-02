#!/usr/bin/env bash
# Runs Ultra-Fusion against a live SwarmDeck ARGoS simulation.
#
# VENDORED from argos3-examples/tools/ultra_fusion/run_uf.sh. It is here rather
# than referenced because it has to be tuned to the fleet it serves, and
# SwarmDeck's fleet is heterogeneous in exactly the dimension this script
# configures. Keep the two in sync by hand; the upstream is the place where the
# estimator's behaviour was actually characterised, and its README is worth
# reading before changing anything here.
#
# WHAT SWARMDECK CHANGED
#
# The robot list and every sensor extrinsic are read out of the generated
# experiment file (argos_mounts.py) instead of coming from environment
# variables. Upstream takes one LIDAR_IN_BODY for the whole fleet, which is
# correct there and wrong here: a Scout Mini carries its lidar at 0.4525 m, a
# Bunker at 0.720 m and a Spot at 0.970 m, in the same run. Ultra-Fusion
# corrects for the mount, so a single shared value is a fixed bias on three
# quarters of the fleet that the estimator cannot see and cannot recover from.
#
# Reading the experiment also removes the other half of that trap: the file
# this script reads is the file ARGoS is running, so the two cannot disagree.

# ROS's setup scripts read unset variables (AMENT_TRACE_SETUP_FILES and
# friends), so they must be sourced BEFORE -u is turned on or the
# container dies before it does anything.
source /opt/ros/humble/setup.bash
set -u

# One VLP-16 revolution is ~630 KB, larger than Fast DDS's default UDP
# socket buffer, and the excess is dropped silently -- see
# fastdds_large.xml. Must be exported before ANY node starts.
export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-/tools/fastdds_large.xml}

# The socket buffer this estimator needs is a HOST setting; see the note on
# the ultrafusion service in docker-compose.yml for why it cannot be set here.
# Checked rather than assumed, because being short does not fail: Fast DDS
# drops most of the point clouds, the publisher reports success, and the
# estimator looks like it diverged. Upstream measured 87% loss presenting as
# 47% drift.
RMEM_MAX=$(cat /proc/sys/net/core/rmem_max 2>/dev/null || echo 0)
RMEM_NEEDED=7000000
if [ "$RMEM_MAX" -lt "$RMEM_NEEDED" ]; then
    echo "############################################################" >&2
    echo "WARNING: net.core.rmem_max is ${RMEM_MAX}, below ${RMEM_NEEDED}." >&2
    echo "One lidar revolution is ~630 KB and fastdds_large.xml asks for 7 MB" >&2
    echo "buffers. The kernel will silently hand out smaller ones, Fast DDS" >&2
    echo "will drop most scans, and this estimator will appear to diverge." >&2
    echo "On the HOST:  sudo sysctl -w net.core.rmem_max=8388608" >&2
    echo "############################################################" >&2
else
    echo "=== net.core.rmem_max ${RMEM_MAX}, enough for 7 MB DDS buffers ==="
fi

RUNTIME_DIR=${RUNTIME_DIR:-/run/swarmdeck}
EXPERIMENT=${EXPERIMENT:-$RUNTIME_DIR/session.argos}
MOUNTS=/tools/argos_mounts.py

# The ROS container generates the experiment; this one only reads it. Wait for
# it rather than racing: a missing file here would fall back to a default fleet
# whose ids do not exist in the arena, and every robot would then sit still
# with no odometry while the logs said nothing at all.
echo "=== Waiting for $EXPERIMENT ==="
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
    echo "no robots in $EXPERIMENT" >&2
    exit 1
fi
echo "=== Fleet: ${ROBOTS[*]} ==="

SOCK=${SOCK:-/sock/argos_uf.sock}
MODE=${MODE:-lwio}            # lio | lwio | lvwio | vio | viwo
MAPPER=${MAPPER:-0}           # 1 enables the optional map-PCD export
RESULTS=${RESULTS:-/results}

# Sensor mounts and camera intrinsics come from the experiment file, per
# robot; see the header. Nothing below is a default to be overridden.
# 0 = raw sensor_msgs/Image, 1 = CompressedImage. uf_link sends colour
# PNG-compressed and depth raw, and a mismatch here is silent.
IMG0_TYPE=${IMG0_TYPE:-1}
IMG1_TYPE=${IMG1_TYPE:-0}
# Must match <gravity g="...">
GRAVITY=${GRAVITY:-9.81}
# The four <imu> attributes from the .argos file, in ARGoS's per-sample
# units; make_profile.py converts them to Ultra-Fusion's per-sqrt(Hz)
# densities. Order: gyro, accel, gyro_bias_walk, accel_bias_walk.
ARGOS_IMU_NOISE=${ARGOS_IMU_NOISE:-"0.002 0.02 0.0002 0.002"}
# These are the rates the ARGoS side actually produces, and they are not
# cosmetic: make_profile.py turns the IMU noise figures into per-sqrt(Hz)
# densities using them, so an error here scales the noise model by its square
# root. Keep them equal to TICKS_PER_SECOND, LIDAR_HZ and CAMERA_HZ in
# swarmdeck_sim/scenario/make_argos_session.py.
IMU_HZ=${IMU_HZ:-100}
WHEEL_HZ=${WHEEL_HZ:-100}
IMAGE_HZ=${IMAGE_HZ:-5}

mkdir -p "$RESULTS"

#
# Ultra-Fusion declares every topic with an ABSOLUTE name, so a
# namespace argument would not move any of them (this is the same trap
# Ground-Fusion++ has). Explicit "-r /a:=/b" rules do work, verified on
# the 0.2.2 binary. The lists below were captured from
# `ros2 topic list -t` against a running uf_node, so that N robots do
# not publish on top of each other.
#
INPUT_TOPICS=(
    # The IMU topic is MODE DEPENDENT: the lidar profiles read
    # /livox/mid360/imu, the vio/viwo ones /camera/imu. Both are
    # remapped, since remapping a topic a node never subscribes to
    # costs nothing and getting it wrong starves the estimator of
    # inertial data with no error message.
    "/livox/mid360/imu:imu"
    "/camera/imu:imu"
    "/livox/mid360/lidar:points"
    "/odom:odom"
    "/camera/color/image_raw/compressed:color/image_raw/compressed"
    "/camera/aligned_depth_to_color/image_raw:aligned_depth_to_color/image_raw"
)
# The one that matters: /odom_lidar is the fused estimate uf_link reads
# back and hands to ARGoS. The rest are per-subsystem odometries, paths
# and debug clouds, remapped only so they stay separable per robot.
OUTPUT_TOPICS=(
    odom_lidar odom_lidar_by_imu_pre odom_camera odom_wheel
    laser_odometry map_matching_odometry camera_extrinsic lidar_extrinsic
    result_path result_lidar_path result_camera_path result_wheel_path
    wheel_path imu_path
    curr_cloud all_image_cloud colored_lidar_cloud voxel_map_cloud
    match_cloud ref_cloud reg_match_cloud v_cloud sphere_cloud
    vio_dense_map feature_reproject_cloud lidar_reproject_cloud
    cloud_image_reproject intensity_factor_cloud p2plane_factor_cloud
    ceres_feature_source_cloud ceres_feature_target_cloud
    synced_imu synced_gnss
)

PIDS=()

echo "=== Generating one profile per robot (mode=$MODE, mapper=$MAPPER) ==="
for r in "${ROBOTS[@]}"; do
    MAP_ARGS=()
    if [ "$MAPPER" = "1" ]; then
        MAP_ARGS=(--map-pcd --map-dir "$RESULTS/$r/map"
                  --map-service "/$r/ultrafusion/generate_map_pcd")
    fi
    LIDAR_IN_BODY="$(python3 "$MOUNTS" "$EXPERIMENT" --lidar "$r")"
    CAMERA_IN_BODY="$(python3 "$MOUNTS" "$EXPERIMENT" --camera "$r")"
    LIDAR_ELEV="$(python3 "$MOUNTS" "$EXPERIMENT" --lidar-elev "$r")"
    CAMERA_RESOLUTION="$(python3 "$MOUNTS" "$EXPERIMENT" --camera-resolution "$r" | tr " " ",")"
    CAMERA_FOV="$(python3 "$MOUNTS" "$EXPERIMENT" --camera-fov "$r")"
    echo "  $r: lidar ($LIDAR_IN_BODY) camera ($CAMERA_IN_BODY) elev ($LIDAR_ELEV)"
    python3 /tools/make_profile.py \
        --mode "$MODE" --out "/tmp/uf_profiles/$r" \
        --lidar-in-body $LIDAR_IN_BODY \
        --camera-in-body $CAMERA_IN_BODY \
        --lidar-elev $LIDAR_ELEV \
        --camera-resolution "$CAMERA_RESOLUTION" --camera-fov "$CAMERA_FOV" \
        --img0-type "$IMG0_TYPE" --img1-type "$IMG1_TYPE" \
        --gravity "$GRAVITY" \
        --imu-hz "$IMU_HZ" --wheel-hz "$WHEEL_HZ" --image-hz "$IMAGE_HZ" \
        --argos-imu-noise $ARGOS_IMU_NOISE \
        --planar-wheel \
        "${MAP_ARGS[@]}" || exit 1
done

echo "=== Starting one uf_node per robot ==="
for r in "${ROBOTS[@]}"; do
    REMAPS=()
    for pair in "${INPUT_TOPICS[@]}"; do
        REMAPS+=(-r "${pair%%:*}:=/$r/${pair#*:}")
    done
    for t in "${OUTPUT_TOPICS[@]}"; do
        REMAPS+=(-r "/$t:=/$r/$t")
    done
    mkdir -p "$RESULTS/$r"
    uf_node "/tmp/uf_profiles/$r/uf_argos.yaml" \
        --ros-args -p use_sim_time:=true "${REMAPS[@]}" \
        > "$RESULTS/$r/uf_node.log" 2>&1 &
    PIDS+=($!)
    echo "  $r -> pid ${PIDS[-1]}, log $RESULTS/$r/uf_node.log"
done

echo "=== Waiting for the estimators to come up ==="
sleep 10
for i in "${!PIDS[@]}"; do
    if ! kill -0 "${PIDS[$i]}" 2>/dev/null; then
        echo "uf_node for ${ROBOTS[$i]} died on startup:"
        tail -30 "$RESULTS/${ROBOTS[$i]}/uf_node.log"
        exit 1
    fi
done

echo "=== Starting the link; ARGoS may connect now ==="
python3 /tools/uf_link.py --socket "$SOCK" 2>&1 | tee "$RESULTS/uf_link.log"

if [ "$MAPPER" = "1" ]; then
    echo "=== Generating the maps (this is the optional mapper) ==="
    for r in "${ROBOTS[@]}"; do
        # Must be called BEFORE uf_node stops: the service is what
        # merges the saved keyframes into map.pcd
        timeout 120 ros2 service call "/$r/ultrafusion/generate_map_pcd" \
            std_srvs/srv/Trigger '{}' || echo "  $r: map generation failed"
    done
fi

echo "=== Stopping the estimators ==="
for pid in "${PIDS[@]}"; do kill -INT "$pid" 2>/dev/null; done
sleep 5
for pid in "${PIDS[@]}"; do kill -KILL "$pid" 2>/dev/null; done

echo "=== Results ==="
find "$RESULTS" -maxdepth 3 -type f | sort | head -40
