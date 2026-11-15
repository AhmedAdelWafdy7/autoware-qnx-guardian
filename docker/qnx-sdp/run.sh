#!/usr/bin/env bash
# Copyright 2026 Ahmed Wafdy
# SPDX-License-Identifier: Apache-2.0
#
# Run a command in the private QNX SDP image, with the repo at /ws and the QNX licence
# (~/.qnx) mounted. Read-write: the licence manager writes heartbeats there.
#
# Usage: docker/qnx-sdp/run.sh <command...>
#   docker/qnx-sdp/run.sh bash -c 'cd guardian && cmake --preset qnx-aarch64le -G Ninja && cmake --build --preset qnx-aarch64le'
# Env:   QNX_IMAGE (default qnx-sdp:8.0), QNX_LICENSE_DIR (default ~/.qnx)

set -euo pipefail

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
image="${QNX_IMAGE:-qnx-sdp:8.0}"
lic="${QNX_LICENSE_DIR:-${HOME}/.qnx}"

test -d "${lic}/license" || { echo "error: no QNX licence at ${lic}/license" >&2; exit 1; }

tty=()
[ -t 0 ] && tty=(-it)

exec docker run --rm "${tty[@]}" \
  --hostname "$(hostname)" \
  --user "$(id -u):$(id -g)" \
  -v "${repo}:/ws" \
  -v "${lic}:/home/qnx/.qnx" \
  -w /ws \
  "${image}" "$@"
