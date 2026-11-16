#!/bin/sh
# Copyright 2026 Ahmed Wafdy
# SPDX-License-Identifier: Apache-2.0
#
# Run on the Pi 5 inside the unpacked source tarball (see pack.sh). Builds the 4 MUST Autoware
# safety packages and their 39 workspace dependencies against the ros2-jazzy APK.
# TODO(verify): not run on the board yet.
#
# Env: JOBS (default 2) — parallel packages; rosidl builds are RAM-hungry.
set -eu

JOBS="${JOBS:-2}"

# ROS 2 setup scripts reference unset variables, so relax -u while sourcing.
set +u
. /opt/ros/jazzy/setup.sh
set -u

export CC=clang CXX=clang++
export MAKEFLAGS="-j${JOBS}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ENABLE_AGNOCAST   # autoware_agnocast_wrapper: plain-rclcpp mode

colcon build \
  --base-paths src \
  --parallel-workers "${JOBS}" \
  --continue-on-error \
  --packages-up-to \
    autoware_mrm_handler \
    autoware_mrm_emergency_stop_operator \
    autoware_mrm_comfortable_stop_operator \
    autoware_vehicle_cmd_gate \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release

echo "Build done. Smoke test:"
echo "  . install/setup.sh && ros2 run autoware_mrm_handler autoware_mrm_handler_node"
