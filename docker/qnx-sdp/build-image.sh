#!/usr/bin/env bash
# Copyright 2026 Ahmed Wafdy
# SPDX-License-Identifier: Apache-2.0
#
# Build the private QNX SDP image on the self-hosted runner, from the local SDP install.
# The image stays in this machine's Docker; never push it (the SDP is proprietary).
#
# Usage: docker/qnx-sdp/build-image.sh [tag]   (default tag: qnx-sdp:8.0)
# Env:   QNX_SDP_ROOT  SDP install (default ~/qnx800)

set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
sdp="${QNX_SDP_ROOT:-${HOME}/qnx800}"
tag="${1:-qnx-sdp:8.0}"

test -f "${sdp}/qnxsdp-env.sh" || { echo "error: no QNX SDP at ${sdp}" >&2; exit 1; }

# The SDP is a named build context: only the paths the Dockerfile copies are sent
# (not images/, which holds the 12 GB BSP card images).
docker buildx build \
  --load \
  --build-context "sdp=${sdp}" \
  --build-arg "UID=$(id -u)" \
  --build-arg "GID=$(id -g)" \
  --label "org.opencontainers.image.description=Private QNX SDP 8.0 build image (do not push)" \
  -t "${tag}" \
  "${here}"

docker image ls "${tag}"
