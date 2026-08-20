#!/usr/bin/env bash
# Phase 0 exit criterion: headless sim runs in CI and terminates cleanly.
#
# Orphaned `gz sim` processes hold DDS ports and silently poison the next run,
# so every path through this script reaps by PID.
set -o pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SIM="$REPO/swarmdeck_ros/src/swarmdeck_sim"
WORLD="$SIM/worlds/indoor.sdf"
LOG="$(mktemp -d)/gz.log"
GZ_PID=""

cleanup() {
  [[ -n "$GZ_PID" ]] && kill -9 "$GZ_PID" 2>/dev/null
  pkill -9 -f "gz sim.*indoor.sdf" 2>/dev/null
  sleep 1
}
trap cleanup EXIT

fail() { echo "FAIL: $*" >&2; exit 1; }

set +u; source /opt/ros/jazzy/setup.bash 2>/dev/null || fail "ROS 2 Jazzy not found"; set -u
export QT_QPA_PLATFORM=offscreen

echo "== 1. deterministic world generation =="
python3 "$SIM/scenario/generate_world.py" --seed 20260801 -o "$WORLD" >/dev/null || fail "generate"
python3 "$SIM/scenario/generate_world.py" --seed 20260801 -o "$WORLD.check" >/dev/null
cmp -s "$WORLD" "$WORLD.check" || fail "world not deterministic (NFR-5)"
rm -f "$WORLD.check"
gz sdf -k "$WORLD" 2>&1 | grep -q Valid || fail "world SDF invalid"
echo "   ok: byte-identical across runs, SDF valid"

echo "== 2. headless simulation starts =="
gz sim -s -r --headless-rendering -v 1 "$WORLD" >"$LOG" 2>&1 &
GZ_PID=$!
sleep 15
kill -0 "$GZ_PID" 2>/dev/null || fail "gz sim died; see $LOG"
echo "   ok: pid $GZ_PID alive"

echo "== 3. fleet spawns =="
python3 "$SIM/scenario/spawn_fleet.py" --config "$REPO/configs/4robot.yaml" || fail "spawn"
sleep 10

echo "== 4. sensors publish =="
TOPICS="$(timeout 10 gz topic -l 2>/dev/null)"
# camera/depth_image and camera/camera_info are the other half of the RGBD
# sensor. Without them a detection is a box on a video frame and nothing
# on the map, which is a silent loss: video and detection both keep working.
for t in scan/points proximity_scan odom imu ground_truth \
         camera/image camera/depth_image camera/camera_info; do
  echo "$TOPICS" | grep -q "/robot_0/$t" || fail "missing topic /robot_0/$t"
done
echo "   ok: lidar, odom, imu, ground truth, colour + depth camera"

echo "== 5. lidar produces data =="
W=$(timeout 12 gz topic -e -n 1 -t /robot_0/scan/points 2>/dev/null | grep -m1 '^width' | awk '{print $2}')
H=$(timeout 12 gz topic -e -n 1 -t /robot_0/scan/points 2>/dev/null | grep -m1 '^height' | awk '{print $2}')
[[ "${W:-0}" -gt 0 ]] || fail "lidar produced no points"
# Single ring is required: a multi-ring lidar cannot feed 2D SLAM through a
# height band (each ring truncates at a different range). See "Gazebo lidar" in
# docs/operations/known-issues.md.
[[ "${H:-0}" == "1" ]] || fail "lidar must be single-ring for 2D SLAM, got height=$H"
echo "   ok: $W beams, $H ring"

echo "== 6. depth camera measures range =="
# adapters/perception/depth_projection.py reads this buffer directly, so its
# shape is an interface: 32-bit floats in metres, one per colour pixel. A driver
# that switched to 16UC1 millimetres would still publish, and every detection marker
# would land 1000x too far away.
DEPTH="$(timeout 12 gz topic -e -n 1 -t /robot_0/camera/depth_image 2>/dev/null \
         | grep -a -E '^(width|height|step|pixel_format_type)')"
DW=$(echo "$DEPTH" | grep -m1 '^width' | awk '{print $2}')
DSTEP=$(echo "$DEPTH" | grep -m1 '^step' | awk '{print $2}')
echo "$DEPTH" | grep -q 'R_FLOAT32' || fail "depth is not R_FLOAT32 metres: $DEPTH"
[[ "${DSTEP:-0}" -eq $(( ${DW:-0} * 4 )) ]] || fail "depth row is $DSTEP bytes for $DW px"
echo "   ok: ${DW}px rows of float32 metres"

echo "== 7. robot drives =="
BEFORE=$(timeout 8 gz topic -e -n 1 -t /robot_0/odom 2>/dev/null | grep -A2 position | grep 'x:' | awk '{print $2}')
timeout 5 gz topic -t /robot_0/cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.6}' >/dev/null 2>&1 &
sleep 4
AFTER=$(timeout 8 gz topic -e -n 1 -t /robot_0/odom 2>/dev/null | grep -A2 position | grep 'x:' | awk '{print $2}')
python3 -c "
import sys
b,a=float('${BEFORE:-0}'),float('${AFTER:-0}')
print(f'   moved {a-b:.2f} m')
sys.exit(0 if a-b > 0.5 else 1)
" || fail "robot did not move (before=$BEFORE after=$AFTER)"

echo
echo "PASS: headless simulation integration test"
