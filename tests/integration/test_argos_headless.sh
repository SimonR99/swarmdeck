#!/usr/bin/env bash
# The ARGoS backend runs headless, deterministically, and produces sensor data.
#
# The companion to test_sim_headless.sh, which covers the legacy Gazebo path.
# Neither the ROS stack nor Ultra-Fusion is involved: the capture step binds the
# bridge socket itself and speaks the protocol, so what this proves is that the
# simulator, the world, the robot plugins and the render all work. Whether ROS
# then does the right thing with the messages is test_launch_files_build.py's
# job and the live stack's.
#
# Needs: the ARGoS fork installed or built (docs/architecture/simulation.md),
# SwarmDeck's own module in $SWARMDECK_ARGOS_BUILD, numpy and pillow.
set -o pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SIM="$REPO/swarmdeck_ros/src/swarmdeck_sim"
SCENARIO="$SIM/scenario"
WORK="$(mktemp -d)"
ARGOS_BUILD="${SWARMDECK_ARGOS_BUILD:-$REPO/argos/build}"

cleanup() { rm -rf "$WORK"; }
trap cleanup EXIT

fail() { echo "FAIL: $*" >&2; exit 1; }

command -v argos3 >/dev/null \
  || fail "argos3 not on PATH; see docs/architecture/simulation.md"
[[ -f "$ARGOS_BUILD/libswarmdeck_argos.so" ]] \
  || fail "no libswarmdeck_argos.so in $ARGOS_BUILD; run
    cmake -S argos -B argos/build -DCMAKE_BUILD_TYPE=Release && cmake --build argos/build -j"

echo "== 1. deterministic world generation =="
python3 "$SCENARIO/make_argos_world.py" --seed 20260801 -o "$WORK/a/indoor.gltf" >/dev/null \
  || fail "world generation"
python3 "$SCENARIO/make_argos_world.py" --seed 20260801 -o "$WORK/b/indoor.gltf" >/dev/null
cmp -s "$WORK/a/indoor.gltf" "$WORK/b/indoor.gltf" || fail "world glTF not deterministic (NFR-5)"
cmp -s "$WORK/a/indoor.bin"  "$WORK/b/indoor.bin"  || fail "world buffer not deterministic (NFR-5)"
python3 -c "import json,sys; json.load(open(sys.argv[1]))" "$WORK/a/indoor.gltf" \
  || fail "world glTF is not valid JSON"
echo "   ok: byte-identical across runs, valid glTF"

echo "== 2. every detection-target model exists =="
python3 - "$SCENARIO" "$REPO/argos/assets/props" <<'PY' || fail "missing prop models"
import sys
from pathlib import Path
sys.path.insert(0, sys.argv[1])
from make_argos_world import PROPS
props = Path(sys.argv[2])
missing = [n for n in PROPS if not (props / f"{n}.glb").is_file()]
if missing:
    raise SystemExit(f"missing prop models: {missing}")
print(f"   ok: {len(PROPS)} prop models, one per catalog class")
PY

echo "== 3. a planar lidar is refused rather than silently useless =="
# Ultra-Fusion is lidar-inertial. Given one ring it never converges, every
# robot's odometry stays invalid and the fleet stands still, which presents as
# a bridge fault rather than a sensor one.
python3 - "$SCENARIO" "$REPO/configs/4robot.yaml" "$WORK" <<'PY' \
  || fail "a planar lidar was accepted"
import sys
from pathlib import Path
import yaml
sys.path.insert(0, sys.argv[1])
import make_argos_session as mas
cfg = yaml.safe_load(Path(sys.argv[2]).read_text())
cfg["fleet"]["lidar"] = {"profile": "generic_2d"}
out = Path(sys.argv[3]) / "planar.yaml"
out.write_text(yaml.safe_dump(cfg))
try:
    mas.generate_argos_xml(out)
except ValueError as exc:
    assert "vlp16" in str(exc), f"the refusal must name the fix: {exc}"
    print("   ok: refused, and named the fix")
    raise SystemExit(0)
raise SystemExit("a single-ring lidar was accepted")
PY

echo "== 4. headless simulation runs and every robot reports =="
# run_visual_test.py IS the check: it starts ARGoS, speaks the bridge protocol,
# and fails when a robot produces no RGB frame, no depth frame or no lidar
# returns. Those are the failures that are otherwise completely silent.
OUT="$WORK/visual"
SWARMDECK_ARGOS_BUILD="$ARGOS_BUILD" \
  timeout 400 python3 "$REPO/tests/integration/run_visual_test.py" \
    --ticks 20 --outdir "$OUT" || fail "visual capture"
[[ -f "$OUT/fleet_visual_dashboard.png" ]] || fail "no dashboard written"
echo "   ok: every robot produced RGB, depth and lidar returns"

echo
echo "PASS: the ARGoS backend runs headless and deterministically"
