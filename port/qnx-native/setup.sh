#!/bin/sh
# Copyright 2026 Ahmed Wafdy
# SPDX-License-Identifier: Apache-2.0
#
# Run on the Pi 5 (QNX 8.0.5, QSTI image). Installs the on-target toolchain and the deps of the
# 4 MUST Autoware packages from repo.oss.qnx.com. Package names checked against the 8.0.5 and
# 8.0.3 APKINDEX on 2026-11-09. TODO(verify): not run on the board yet.
set -eu

sudo apk update
sudo apk add \
  ros2-jazzy \
  build-base clang cmake ninja python3-dev git \
  boost-dev eigen-dev fmt-dev glog-dev gflags-dev tinyxml2-dev

# Sanity checks: toolchain and ROS 2 are usable.
clang++ --version
cmake --version
python3 --version
test -f /opt/ros/jazzy/setup.sh
echo "setup OK"
