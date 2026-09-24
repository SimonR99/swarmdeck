#!/bin/bash
# Run inside the C-SLAM image, with CSLAM_SOURCE pointing at patched sources.
set -euo pipefail
source_root=${CSLAM_SOURCE:-/cslam_ws/src/cslam}
g++ -std=c++17 "$(dirname "$0")/pose_graph_unchanged.cpp" \
  -I"$source_root/include" -I/opt/ros/jazzy/include -I/usr/include/eigen3 \
  -L/opt/ros/jazzy/lib/x86_64-linux-gnu \
  -Wl,-rpath,/opt/ros/jazzy/lib/x86_64-linux-gnu \
  -lgtsam -ltbb -o /tmp/pose_graph_unchanged_test
/tmp/pose_graph_unchanged_test
