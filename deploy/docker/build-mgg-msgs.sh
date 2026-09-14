#!/usr/bin/env bash
set -eo pipefail

revision="${1:?MGG revision is required}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
git clone --branch ros2 --single-branch \
  https://github.com/MISTLab/MGGPlanner.git /tmp/mgg-msgs-source
git -C /tmp/mgg-msgs-source checkout "$revision"
git -C /tmp/mgg-msgs-source apply /tmp/mgg-objective-msgs.patch
git -C /tmp/mgg-msgs-source apply /tmp/mgg-indexed-map-msgs.patch
# The native partial-route patch also changes planner sources. Contract-only
# images apply just its PlanObjective service hunk so every ROS participant
# generates the same type hash without building MGG itself.
git -C /tmp/mgg-msgs-source apply \
  --include=ros2/src/mgg_msgs/srv/PlanObjective.srv \
  /tmp/mgg-partial-route-navigate.patch
# Generate the route-validation interface without compiling planner sources.
git -C /tmp/mgg-msgs-source apply \
  --include=ros2/src/mgg_msgs/CMakeLists.txt \
  --include=ros2/src/mgg_msgs/srv/ValidateObjectiveRoute.srv \
  /tmp/mgg-route-validation.patch
# Generate the rolling Home continuation contract in every image which hosts
# an adapter or indexed-map service. The planner implementation remains in the
# native MGG image; these filtered hunks keep the ROS service type hashes equal.
git -C /tmp/mgg-msgs-source apply \
  --include=ros2/src/mgg_msgs/CMakeLists.txt \
  --include=ros2/src/mgg_msgs/srv/PlanObjective.srv \
  --include=ros2/src/mgg_msgs/srv/RefineObjectiveRoute.srv \
  /tmp/mgg-home-rolling-followup.patch
cd /tmp/mgg-msgs-source/ros2
colcon build --packages-select mgg_msgs --merge-install \
  --executor sequential \
  --install-base /opt/swarmdeck-mgg-msgs \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
