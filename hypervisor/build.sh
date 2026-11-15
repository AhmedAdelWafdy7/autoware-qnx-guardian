#!/usr/bin/env bash
# Copyright 2026 Ahmed Wafdy
# SPDX-License-Identifier: Apache-2.0
#
# Build the QNX Hypervisor host image for the Raspberry Pi 5, with the QNX guest inside it.
# Runs on an x86_64 Linux host with QNX SDP 8.0 and these Software Center packages:
#   com.qnx.qnx800.target.hypervisor.group, com.qnx.qnx800.bsp.hw.raspberrypi_bcm2712_rpi5,
#   com.qnx.qnx800.bsp.hypervisor_guest_arm
# The QNX BSPs are unpacked into $WORK at build time; nothing from them is stored in this repo.
#
# Output: $OUT/ifs-rpi5-hyp.bin  (copy to the SD card boot partition, config.txt:
#         kernel=ifs-rpi5-hyp.bin)
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
QNX_ROOT="${QNX_ROOT:-$HOME/qnx800}"
WORK="${WORK:-$HERE/work}"
OUT="${OUT:-$HERE/out}"

pick_zip() {  # newest BSP zip matching a pattern
  local z
  z="$(ls -1 "$QNX_ROOT"/bsp/$1 2>/dev/null | sort | tail -n 1 || true)"
  [[ -n "$z" ]] || { echo "error: no $1 in $QNX_ROOT/bsp (install the BSP package)" >&2; exit 1; }
  echo "$z"
}
RPI5_BSP_ZIP="${RPI5_BSP_ZIP:-$(pick_zip 'BSP_raspberrypi-bcm2712-rpi5_*.zip')}"
GUEST_BSP_ZIP="${GUEST_BSP_ZIP:-$(pick_zip 'BSP_hyp-guest-arm_*.zip')}"

# shellcheck disable=SC1091
source "$QNX_ROOT/qnxsdp-env.sh" >/dev/null
[[ -x "$QNX_TARGET/aarch64le/sbin/qvm" ]] || {
  echo "error: qvm not found; install com.qnx.qnx800.target.hypervisor.group" >&2; exit 1; }

rm -rf "$WORK" && mkdir -p "$WORK/rpi5" "$WORK/guest" "$WORK/guest-out" "$OUT"
echo "== BSPs: $(basename "$RPI5_BSP_ZIP"), $(basename "$GUEST_BSP_ZIP")"
python3 -m zipfile -e "$RPI5_BSP_ZIP" "$WORK/rpi5"   # no unzip on the build host
python3 -m zipfile -e "$GUEST_BSP_ZIP" "$WORK/guest"

# ---- QNX guest image (stock guest-1 for now; Guardian + ROS 2 are added later) ----
echo "== Building the QNX guest image"
make -C "$WORK/guest" > "$WORK/guest-make.log" 2>&1 || {
  tail -n 30 "$WORK/guest-make.log"; exit 1; }
cp "$WORK/guest/images/guest-1/qnx800-guest-1.ifs" "$WORK/guest-out/qnx-guardian.ifs"
cp "$HERE/guests/qnx-guardian/qnx-guardian.qvmconf" "$WORK/guest-out/"

# ---- Hypervisor host image ----
echo "== Building the Pi 5 BSP"
make -C "$WORK/rpi5" > "$WORK/rpi5-make.log" 2>&1 || { tail -n 30 "$WORK/rpi5-make.log"; exit 1; }

IMAGES="$WORK/rpi5/images"
HOST_BUILD="$IMAGES/rpi5-hyp.build"
# Enable the hypervisor in the startup program; fail loudly if the BSP line has changed.
grep -q '^ *startup-bcm2712-rpi5 ' "$IMAGES/rpi5.build" || {
  echo "error: startup line not found in rpi5.build" >&2; exit 1; }
sed 's/^\( *startup-bcm2712-rpi5\) /\1 -Q enable /' "$IMAGES/rpi5.build" > "$HOST_BUILD"
sed "s|@GUEST_DIR@|$WORK/guest-out|g" "$HERE/host/hyp-additions.build" >> "$HOST_BUILD"
grep -n 'startup-bcm2712-rpi5' "$HOST_BUILD"

echo "== mkifs host image"
(cd "$IMAGES" && mkifs -v -r../install "$HOST_BUILD" "$OUT/ifs-rpi5-hyp.bin" \
  > "$WORK/mkifs-host.log" 2>&1) || { tail -n 30 "$WORK/mkifs-host.log"; exit 1; }

ls -l "$OUT/ifs-rpi5-hyp.bin"
sha256sum "$OUT/ifs-rpi5-hyp.bin" | tee "$OUT/ifs-rpi5-hyp.bin.sha256"
