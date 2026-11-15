# Copyright 2026 Ahmed Wafdy
# SPDX-License-Identifier: Apache-2.0
#
# Private QNX SDP 8.0 build image for the self-hosted runner.
#
# The SDP is proprietary: this image is built on the runner from a local SDP install
# (build-image.sh) and is never pushed to a registry. The QNX licence is not in the image; it is
# mounted from ~/.qnx at run time.
#
# Contents: QNX SDP host tools + target (no BSP card images), CMake + Ninja for the Guardian
# presets, and python3 for the hypervisor build (it unpacks the BSP zips with zipfile).

FROM ubuntu:22.04

ARG UID=1000
ARG GID=1000
# CMake presets version 6 needs CMake >= 3.25; Ubuntu 22.04 ships 3.22.
ARG CMAKE_VERSION=3.28.4
ARG NINJA_VERSION=1.11.1.1

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      bash ca-certificates file git python3 python3-pip \
 && rm -rf /var/lib/apt/lists/* \
 && pip3 install --no-cache-dir "cmake==${CMAKE_VERSION}" "ninja==${NINJA_VERSION}"

# Same layout as qnxsdp-env.sh, under /opt/qnx800. bsp/ holds the BSP zips the hypervisor
# build unpacks.
COPY --from=sdp host   /opt/qnx800/host
COPY --from=sdp target /opt/qnx800/target
COPY --from=sdp bsp    /opt/qnx800/bsp
COPY --from=sdp qnxsdp-env.sh /opt/qnx800/qnxsdp-env.sh

RUN groupadd -g "${GID}" qnx && useradd -m -u "${UID}" -g "${GID}" qnx

# What `source qnxsdp-env.sh` exports, so tools work without sourcing it.
ENV QNX_HOST=/opt/qnx800/host/linux/x86_64 \
    QNX_TARGET=/opt/qnx800/target/qnx \
    QNX_CONFIGURATION=/home/qnx/.qnx \
    QNX_CONFIGURATION_EXCLUSIVE=/home/qnx/.qnx \
    MAKEFLAGS=-I/opt/qnx800/target/qnx/usr/include \
    PYTHONDONTWRITEBYTECODE=1 \
    PATH=/opt/qnx800/host/linux/x86_64/usr/bin:/opt/qnx800/host/common/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    QNX_ROOT=/opt/qnx800

USER qnx
WORKDIR /ws
