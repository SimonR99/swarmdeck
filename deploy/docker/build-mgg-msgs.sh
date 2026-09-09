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
cd /tmp/mgg-msgs-source/ros2
colcon build --packages-select mgg_msgs --merge-install \
  --executor sequential \
  --install-base /opt/swarmdeck-mgg-msgs \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
