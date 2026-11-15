# Native build of the Autoware safety layer on the Pi 5 (QNX 8.0.5)

Gate check 3 of W1: `autoware_mrm_handler` (and the other three MUST packages) built against
the `ros2-jazzy` APK, then run.

## Why native and not a cross-build

Checked 2026-11-09 against the binary repo at `https://repo.oss.qnx.com/8.0.5/` (and `8.0.3/`):

- `ros2-jazzy` (`20260128-r1`, 8.0.5/extra, 47 MB, maintainer Felix Lionardo) is a full
  development install under `/opt/ros/jazzy`: 308 ROS packages, headers and CMake configs,
  `rclcpp`, `rclcpp_components`, `rmw_cyclonedds_cpp` (CycloneDDS 0.10.5), `rmw_fastrtps_cpp`,
  `tf2_ros`, `tf2_eigen`, `message_filters`, `class_loader`, `logging_demo`, `diagnostic_msgs`.
- It was built for **Python 3.14** with the repo's **clang 22 / libc++** toolchain. Our SDP 8.0
  ships Python 3.13 and the x86_64 build host has 3.10, so a cross-build would need a matching
  host Python 3.14 venv plus a hand-made sysroot just for rosidl. The repo already has the
  on-target toolchain (`build-base`, `clang`, `cmake`, `ninja`, `python3-dev`), so we build on
  the board.
- **Not in the APK:** `diagnostic_updater` (ros/diagnostics 4.2.7) and `geographic_msgs`
  (ros-geographic-info 1.0.6, needed by `autoware_adapi_v1_msgs`). Both are built from
  source in the same workspace (pinned in `../autoware.repos`).
- Third-party deps come from `8.0.3/extra`: boost 1.89, eigen 5.0.1, fmt 12.1.0, glog 0.6.0,
  gflags 2.2.2, tinyxml2 11.0.0.

## Steps

```sh
# 1. Host (this repo, sources imported in port/src, patches applied):
port/qnx-native/pack.sh                 # -> port/qnx-native/out/aw-safety-src.tar.gz

# 2. Copy to the Pi (scp over the LAN, or a USB stick):
scp port/qnx-native/out/aw-safety-src.tar.gz qnxuser@<pi>:/data/home/qnxuser/   # TODO(verify) home path

# 3. Pi, once:
sh setup.sh                             # sudo apk add the toolchain + deps

# 4. Pi:
mkdir -p ~/aw && cd ~/aw && tar xzf ~/aw-safety-src.tar.gz
sh build.sh                             # colcon build of the 4 MUST packages (43 pkgs total)
```

Every step is `TODO(verify)` until it has run on the board. Expected trouble spots:
clang target flags for QNX, `ament` finding Python 3.14, RAM during the rosidl builds (keep
`--parallel-workers` low), and `glog`/`fmt` versions newer than the ones Autoware is tested with.
