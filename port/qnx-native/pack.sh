#!/usr/bin/env bash
# Copyright 2026 Ahmed Wafdy
# SPDX-License-Identifier: Apache-2.0
#
# Host side: pack only the source packages the 4 MUST Autoware packages need (43 packages,
# patches 0001-0004 already applied in port/src) plus setup.sh/build.sh into one tarball for
# the Pi. Needs colcon (any ROS 2 host).
#
# Usage: port/qnx-native/pack.sh   -> port/qnx-native/out/aw-safety-src.tar.gz
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
port="$(cd "${here}/.." && pwd)"
out="${here}/out"
stage="${out}/stage"

base_paths=(
  src/autoware_adapi_msgs src/autoware_cmake src/autoware_core src/autoware_internal_msgs
  src/autoware_msgs src/autoware_universe src/autoware_utils src/tier4_autoware_msgs
  src/diagnostics/diagnostic_updater src/geographic_info/geographic_msgs
)
targets=(
  autoware_mrm_handler autoware_mrm_emergency_stop_operator
  autoware_mrm_comfortable_stop_operator autoware_vehicle_cmd_gate
)

cd "${port}"
mapfile -t paths < <(colcon list --base-paths "${base_paths[@]}" \
  --packages-up-to "${targets[@]}" --paths-only)
echo "packing ${#paths[@]} packages"

rm -rf "${stage}"
mkdir -p "${stage}/src"
for p in "${paths[@]}"; do
  mkdir -p "${stage}/$(dirname "${p}")"
  cp -a "${p}" "${stage}/${p}"
done
cp "${here}/setup.sh" "${here}/build.sh" "${stage}/"

tar -czf "${out}/aw-safety-src.tar.gz" -C "${stage}" .
rm -rf "${stage}"
ls -la "${out}/aw-safety-src.tar.gz"
