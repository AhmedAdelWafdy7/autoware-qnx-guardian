#!/usr/bin/env bash
# Copyright 2026 Ahmed Wafdy
# SPDX-License-Identifier: Apache-2.0
#
# RSS oracle test: compare the Guardian RSS formulas with intel/ad-rss-lib.
#
# ad-rss-lib (LGPL-2.1) and its ad_physics dependency are downloaded into the build directory
# at pinned versions, built there, and linked only into this host test. They are never copied
# into the repository and never linked into the Guardian or any QNX binary.
#
# Needs: git, cmake, a C++17 compiler, Boost headers and spdlog
#   (Ubuntu: sudo apt install libboost-dev libspdlog-dev).
#
# Usage: guardian/test/rss_oracle/run.sh [build-dir]   (default: guardian/build/rss-oracle)

set -euo pipefail

# Pinned upstream versions.
readonly AD_RSS_URL=https://github.com/intel/ad-rss-lib.git
readonly AD_RSS_TAG=v5.0.0
readonly AD_RSS_SHA=75631789b80ae58d9872486560aa74cd2c8a1d49
# carla-simulator/map at the commit ad-rss-lib v5.0.0 pins as its dependencies/map submodule.
readonly MAP_URL=https://github.com/carla-simulator/map.git
readonly MAP_SHA=2da527150aa3e4464ef7d27cbea4e407b033b1cb

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
guardian_dir="$(cd "${here}/../.." && pwd)"
work="$(mkdir -p "${1:-${guardian_dir}/build/rss-oracle}" && cd "${1:-${guardian_dir}/build/rss-oracle}" && pwd)"
prefix="${work}/install"
jobs="${JOBS:-$(nproc)}"

fetch() {  # fetch <url> <sha> <dir>: shallow checkout of one commit
  local url=$1 sha=$2 dir=$3
  if [[ "$(git -C "${dir}" rev-parse HEAD 2>/dev/null || true)" == "${sha}" ]]; then
    return
  fi
  rm -rf "${dir}"
  git init -q "${dir}"
  git -C "${dir}" fetch -q --depth 1 "${url}" "${sha}"
  git -C "${dir}" checkout -q FETCH_HEAD
}

build_dep() {  # build_dep <source-dir> <name>: configure, build and install into ${prefix}
  local src=$1 name=$2
  if [[ -f "${prefix}/.${name}.done" ]]; then
    return
  fi
  cmake -S "${src}" -B "${work}/build-${name}" -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${prefix}" -DCMAKE_PREFIX_PATH="${prefix}" \
    -DBUILD_TESTING=OFF -DBUILD_PYTHON_BINDING=OFF -DBUILD_APIDOC=OFF \
    -DCMAKE_CXX_FLAGS="-Wno-error=maybe-uninitialized"
  # ad-rss-lib builds with -Werror; GCC 13 + Boost 1.83 report a false maybe-uninitialized
  # inside boost/geometry/.../graham_andrew.hpp (Ubuntu 24.04). Host oracle build only.
  cmake --build "${work}/build-${name}" -j "${jobs}"
  cmake --install "${work}/build-${name}"
  touch "${prefix}/.${name}.done"
}

fetch "${MAP_URL}" "${MAP_SHA}" "${work}/src/map"
fetch "${AD_RSS_URL}" "${AD_RSS_SHA}" "${work}/src/ad-rss-lib"
build_dep "${work}/src/map/ad_physics" ad_physics
build_dep "${work}/src/ad-rss-lib/ad_rss" ad_rss

cmake -S "${here}" -B "${work}/build-oracle" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="${prefix}" -DGUARDIAN_DIR="${guardian_dir}"
cmake --build "${work}/build-oracle" -j "${jobs}"
echo "ad-rss-lib ${AD_RSS_TAG} (${AD_RSS_SHA:0:7}), ad_physics ${MAP_SHA:0:7}"
"${work}/build-oracle/rss_oracle_test"
