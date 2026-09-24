#!/usr/bin/env bash
set -euo pipefail

# Remote acceptance is opt-in. Set SOURCE to build from a clean planning
# export, or set IMAGE to use an image already present on benchbot.
: "${REMOTE:=sroy@benchbot.yannbouteiller.com}"
: "${PRODUCTION:=/home/sroy/workspaces/swarmdeck}"
: "${MISSION:=$(cat /proc/sys/kernel/random/uuid)}"
: "${ROBOT:=mola_acceptance_robot}"
: "${NAME:=swarmdeck-mola-acceptance-$$}"

if [[ -n "${SOURCE:-}" && -z "${IMAGE:-}" ]]; then
  IMAGE=swarmdeck-mapping:mola-acceptance
fi
if [[ -z "${SOURCE:-}" && -z "${IMAGE:-}" ]]; then
  echo 'set IMAGE for a prebuilt remote image, or SOURCE for an explicit build export' >&2
  exit 2
fi

# SSH joins command arguments into a remote shell command; preserve empty
# SOURCE and quote paths explicitly before that second shell parses them.
remote_command=$(printf '%q ' bash -s -- "${SOURCE:-}" "$PRODUCTION" "$IMAGE" "$MISSION" "$ROBOT" "$NAME")
ssh -o BatchMode=yes -o ConnectTimeout=10 "$REMOTE" "$remote_command" <<'REMOTE_SCRIPT'
set -euo pipefail
source=$1
production=$2
image=$3
mission=$4
robot=$5
name=$6

test -d "$production"
if [[ -n "$source" ]]; then
  test -d "$source"
  source_manifest=$(find "$source" -type f -printf '%P\0' \
    | sort -z \
    | while IFS= read -r -d '' relative; do sha256sum "$source/$relative"; done \
    | sha256sum | cut -d' ' -f1)
  printf 'planning source: %s (tree provenance %s)\n' "$source" "$source_manifest"
  docker build --pull=false -f "$source/deploy/docker/Dockerfile.mapping" -t "$image" "$source"
else
  printf 'using prebuilt image: %s\n' "$image"
fi
printf 'production checkout observed read-only: %s\n' "$production"
docker image inspect --format '{{.Id}}' "$image"

tmp=$(mktemp -d)
cleanup() {
  if [[ "$?" -ne 0 ]]; then docker logs --tail 30 "$name" 2>/dev/null || true; fi
  docker rm -f "$name" >/dev/null 2>&1 || true
  rm -rf "$tmp"
}
trap cleanup EXIT
mkdir -p "$tmp/fixture" "$tmp/maps/$mission/$robot/geometry/chunks"

docker run --rm --init --network none --user "$(id -u):$(id -g)" \
  -e PYTHONPATH=/opt/swarmdeck -v "$tmp/fixture:/tmp/fixture" "$image" \
  python3 /mapping_ws/src/swarmdeck_mapping/test/make_fixture.py /tmp/fixture
cp "$tmp/fixture/snapshot.json" "$tmp/maps/$mission/$robot/snapshot.json"
cp "$tmp/fixture/chunks/"* "$tmp/maps/$mission/$robot/geometry/chunks/"

docker run -d --init --network none --user "$(id -u):$(id -g)" --name "$name" \
  -e "SWARMDECK_MISSION_ID=$mission" \
  -v "$tmp/maps:/maps" "$image" swarmdeck-mola-worker \
  --maps-root /maps --timeout 30 --poll 1 --retry 2 \
  --max-output-bytes 268435456 --keep-generations 2 >/dev/null

for _ in $(seq 1 30); do
  test -s "$tmp/maps/$mission/$robot/mola/index.json" && break
  sleep 1
done
test -s "$tmp/maps/$mission/$robot/mola/index.json"
first=$(sha256sum "$tmp/maps/$mission/$robot/snapshot.json" | cut -d' ' -f1)
first_geometry=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["artifacts"][0]["geometry_revision"])' "$tmp/maps/$mission/$robot/mola/index.json")

# Keep the expected directory but remove its payloads. A successful correction
# must reuse the native resident map rather than reading geometry again.
mv "$tmp/maps/$mission/$robot/geometry/chunks" "$tmp/maps/$mission/$robot/geometry/chunks-held"
mkdir "$tmp/maps/$mission/$robot/geometry/chunks"

python3 - "$tmp/maps/$mission/$robot/snapshot.json" <<'PY'
import hashlib, json, os, sys, tempfile
path = sys.argv[1]
value = json.load(open(path))
manifest = value['manifests'][0]
manifest['graph_revision']['revision'] += 1
for submap in manifest['submaps']:
    submap['pose_revision'] = dict(manifest['graph_revision'])
manifest['submaps'][0]['T_component_submap'][0][3] = 1.0
value['snapshot_id'] = hashlib.sha256(json.dumps(
    value['manifests'], sort_keys=True, separators=(',', ':')).encode()).hexdigest()
with tempfile.NamedTemporaryFile('w', dir=os.path.dirname(path), prefix='.snapshot-', delete=False) as stream:
    json.dump(value, stream, sort_keys=True, separators=(',', ':'))
    stream.flush()
    os.fsync(stream.fileno())
os.replace(stream.name, path)
PY
second=$(sha256sum "$tmp/maps/$mission/$robot/snapshot.json" | cut -d' ' -f1)
test "$first" != "$second"

for _ in $(seq 1 30); do
  current=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["source_sha256"])' "$tmp/maps/$mission/$robot/mola/index.json")
  test "$current" = "$second" && break
  sleep 1
done
test "$current" = "$second"
# The product describes itself: mola/source.json holds the exact bytes the
# index digest names, so readers never depend on snapshot.json.
published_source=$(sha256sum "$tmp/maps/$mission/$robot/mola/source.json" | cut -d' ' -f1)
test "$published_source" = "$second"
second_geometry=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["artifacts"][0]["geometry_revision"])' "$tmp/maps/$mission/$robot/mola/index.json")
test "$first_geometry" = "$second_geometry"

native_count=$(docker top "$name" -eo pid,args | grep -Ec 'swarmdeck-mola-import --serve' || true)
test "$native_count" -eq 1
printf 'PASS mission=%s robot=%s native_processes=%s geometry_revision=%s\n' \
  "$mission" "$robot" "$native_count" "$second_geometry"
REMOTE_SCRIPT
