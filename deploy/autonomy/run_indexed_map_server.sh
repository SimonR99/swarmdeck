#!/usr/bin/env bash
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/swarmdeck-mgg-msgs/local_setup.bash
export PYTHONPATH="/opt/swarmdeck${PYTHONPATH:+:${PYTHONPATH}}"
exec python3 /opt/swarmdeck/deploy/autonomy/indexed_map_server.py "$@"
