#!/usr/bin/env bash
# Copyright 2026 Ahmed Wafdy
# SPDX-License-Identifier: Apache-2.0
#
# Linux check for the port: fetch the pinned upstream sources, apply our patches and build the
# four MUST safety packages with ROS 2 Jazzy. Proves the patches apply to the pins and do not
# break the Linux build, and that no PCL ends up in the dependency closure.
#
# Runs in a ros:jazzy container (CI: .github/workflows/port-linux.yml). Locally:
#   docker run --rm -v "$PWD:/ws" -w /ws ros:jazzy port/linux-build.sh
#
# Usage: port/linux-build.sh [work-dir]   (default: port/build-linux)

set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
work="${1:-${here}/build-linux}"
mkdir -p "${work}"
work="$(cd "${work}" && pwd)"
jobs="${JOBS:-$(nproc)}"

# Repos the MUST packages need. The other entries in autoware.repos (fmt, qnx build files)
# are for the QNX side only.
repos=(autoware_universe autoware_core autoware_utils autoware_cmake autoware_msgs
       autoware_internal_msgs autoware_adapi_msgs tier4_autoware_msgs diagnostics geographic_info)
base_paths=(
  src/autoware_adapi_msgs src/autoware_cmake src/autoware_core src/autoware_internal_msgs
  src/autoware_msgs src/autoware_universe src/autoware_utils src/tier4_autoware_msgs
  src/diagnostics/diagnostic_updater src/geographic_info/geographic_msgs
)
targets=(
  autoware_mrm_handler autoware_mrm_emergency_stop_operator
  autoware_mrm_comfortable_stop_operator autoware_vehicle_cmd_gate
)

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

echo "== fetch pinned sources"
mkdir -p "${work}/src"
for name in "${repos[@]}"; do
  read -r url version < <(python3 - "${here}/autoware.repos" "${name}" <<'EOF'
import sys, yaml
r = yaml.safe_load(open(sys.argv[1]))["repositories"][sys.argv[2]]
print(r["url"], r["version"])
EOF
)
  dst="${work}/src/${name}"
  if [ "$(git -C "${dst}" rev-parse HEAD 2>/dev/null)" != "${version}" ]; then
    rm -rf "${dst}"
    git init -q "${dst}"
    git -C "${dst}" fetch -q --depth 1 "${url}" "${version}"
    git -C "${dst}" -c advice.detachedHead=false checkout -q FETCH_HEAD
  fi
  git -C "${dst}" reset -q --hard "${version}"   # drop patches from an earlier run
  git -C "${dst}" clean -qfdx
  echo "${name} ${version}"
done

echo "== apply patches"
for dir in "${here}"/patches/*/; do
  repo="$(basename "${dir}")"
  for p in "${dir}"*.patch; do
    git -C "${work}/src/${repo}" apply --check "${p}"
    git -C "${work}/src/${repo}" apply "${p}"
    echo "applied ${repo}/$(basename "${p}")"
  done
done

cd "${work}"
mapfile -t paths < <(colcon list --base-paths "${base_paths[@]}" \
  --packages-up-to "${targets[@]}" --paths-only)
mapfile -t names < <(colcon list --base-paths "${base_paths[@]}" \
  --packages-up-to "${targets[@]}" --names-only)
echo "== closure: ${#names[@]} packages"

if printf '%s\n' "${names[@]}" | grep -qiE 'pcl'; then
  echo "error: PCL package in the closure:"; printf '%s\n' "${names[@]}" | grep -iE 'pcl'
  exit 1
fi

echo "== rosdep"
if [ "$(id -u)" -eq 0 ]; then
  apt-get update -qq
  rosdep update --rosdistro jazzy -q
  rosdep install -y -q --rosdistro jazzy --ignore-src --from-paths "${paths[@]}"
else
  rosdep install -s --rosdistro jazzy --ignore-src --from-paths "${paths[@]}" || true
fi

echo "== colcon build"
unset ENABLE_AGNOCAST   # autoware_agnocast_wrapper: plain rclcpp mode, as on QNX
colcon build \
  --base-paths "${base_paths[@]}" \
  --packages-up-to "${targets[@]}" \
  --parallel-workers "${jobs}" \
  --event-handlers console_cohesion+ summary+ \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release

echo "== check: no PCL linked"
if find install -name '*.so' -exec ldd {} + 2>/dev/null | grep -q 'libpcl'; then
  echo "error: a built library links PCL"; exit 1
fi

echo "== check: targets installed"
for t in "${targets[@]}"; do
  test -d "install/${t}" || { echo "error: ${t} not installed"; exit 1; }
  echo "ok ${t}"
done
