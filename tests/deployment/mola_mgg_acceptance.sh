#!/usr/bin/env bash
# Cross-image, synthetic map acceptance. No ROS network or motion controller.
set -euo pipefail

mapping_image=${MAPPING_IMAGE:-swarmdeck-mapping:local}
mgg_image=${MGG_IMAGE:-swarmdeck-mgg:local}
fixture_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
scratch=$(mktemp -d)
trap 'rm -rf -- "$scratch"' EXIT
mkdir "$scratch/maps"

docker image inspect "$mapping_image" "$mgg_image" \
  --format '{{.Id}} {{json .RepoTags}}'
docker run --rm --network none --user "$(id -u):$(id -g)" \
  --entrypoint bash \
  -v "$fixture_dir/mola_mgg_fixture.py:/test/mola_mgg_fixture.py:ro" \
  -v "$fixture_dir/mola_planner_acceptance.py:/test/mola_planner_acceptance.py:ro" \
  -v "$scratch/maps:/output" "$mapping_image" -lc '
    set -e
    source /mapping_ws/install/setup.bash
    set -u
    PYTHONPATH=/app:/opt/swarmdeck python3 /test/mola_mgg_fixture.py --output /output
  '

for case_name in initial reused corrected; do
  docker run --rm --network none --user "$(id -u):$(id -g)" \
    --entrypoint bash -v "$scratch/maps:/fixtures:ro" "$mgg_image" -lc '
      set -e
      source /opt/ros/jazzy/setup.bash
      source /opt/mgg/ros2/install/setup.bash
      set -u
      probe=/opt/mgg/ros2/build/mgg_map_octomap/mola_map_probe
      test -x "$probe"
      exec "$probe" "/fixtures/$1" "/fixtures/$1/probe.json"
    ' bash "$case_name" > "$scratch/$case_name.json"
  python3 - "$scratch/maps/$case_name/probe.json" \
    "$scratch/$case_name.json" "$case_name" <<'PY'
import json
import sys
from pathlib import Path

request, result = (json.loads(Path(path).read_text()) for path in sys.argv[1:3])
assert result["status"] == "ready", result
assert result["voxels"] == request["expected_voxels"], result
print(json.dumps({"case": sys.argv[3], **result}))
PY
done
