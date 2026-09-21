#!/usr/bin/env bash
set -eo pipefail

revision="${1:?MGG revision is required}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
# SwarmDeck runs the `ros2` branch of MGGPlanner.
# Contract-only images build just the generated mgg_msgs package from it,
# so every ROS participant ends up with identical service type hashes
# without compiling the planner itself.
git clone --branch ros2 \
  https://github.com/MISTLab/MGGPlanner.git /tmp/mgg-msgs-source
git -C /tmp/mgg-msgs-source checkout "$revision"
cd /tmp/mgg-msgs-source/ros2
colcon build --packages-select mgg_msgs --merge-install \
  --executor sequential \
  --install-base /opt/swarmdeck-mgg-msgs \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
