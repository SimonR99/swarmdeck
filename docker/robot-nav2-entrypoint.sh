#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /nav_ws/install/setup.bash

exec "$@"
