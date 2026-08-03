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
python3 "$SIM/scenario/spawn_fleet.py" --config "$REPO/study/4robot.yaml" || fail "spawn"
sleep 10

echo "== 4. sensors publish =="
TOPICS="$(timeout 10 gz topic -l 2>/dev/null)"
for t in scan/points proximity_scan odom imu camera/image_raw ground_truth; do
  echo "$TOPICS" | grep -q "/robot_0/$t" || fail "missing topic /robot_0/$t"
done
echo "   ok: lidar, odom, imu, camera, ground truth"

echo "== 5. lidar produces data =="
W=$(timeout 12 gz topic -e -n 1 -t /robot_0/scan/points 2>/dev/null | grep -m1 '^width' | awk '{print $2}')
H=$(timeout 12 gz topic -e -n 1 -t /robot_0/scan/points 2>/dev/null | grep -m1 '^height' | awk '{print $2}')
[[ "${W:-0}" -gt 0 ]] || fail "lidar produced no points"
# Single ring is required: a multi-ring lidar cannot feed 2D SLAM through a
# height band (each ring truncates at a different range). See KNOWN_ISSUES.md.
[[ "${H:-0}" == "1" ]] || fail "lidar must be single-ring for 2D SLAM, got height=$H"
echo "   ok: $W beams, $H ring"

echo "== 6. robot drives =="
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
