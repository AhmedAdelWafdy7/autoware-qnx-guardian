# Private QNX SDP 8.0 build image

The QNX cross-build in CI (`.github/workflows/qnx-build.yml`) runs in this image on the
self-hosted runner. The image is built on that machine from its own QNX SDP install and is
never pushed to a registry: the SDP is proprietary and licensed per user.

| File | What it does |
|---|---|
| `Dockerfile` | Ubuntu 22.04 + the SDP `host/`, `target/` and `bsp/` under `/opt/qnx800`, the variables `qnxsdp-env.sh` sets, CMake 3.28 and Ninja |
| `build-image.sh` | Builds `qnx-sdp:8.0` from `~/qnx800` (or `QNX_SDP_ROOT`) |
| `run.sh` | Runs a command in the image with the repo at `/ws` and `~/.qnx` mounted |

The licence is not in the image. `run.sh` mounts `~/.qnx` read-write (the licence manager
writes heartbeats there) and keeps the host's hostname.

## Set up on the runner

Needs Docker with buildx, and the runner user in the `docker` group:

```bash
sudo apt-get install -y docker.io docker-buildx
sudo usermod -aG docker "$USER"      # then restart the runner service and log in again
docker/qnx-sdp/build-image.sh        # about 5 min; image is 5.7 GB
```

Rebuild the image after installing or updating SDP packages.

## Use

```bash
docker/qnx-sdp/run.sh bash -c 'cd guardian && cmake --preset qnx-aarch64le -G Ninja && cmake --build --preset qnx-aarch64le'
```
