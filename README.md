# Autoware QNX Guardian

The Autoware Foundation’s Safety Island showed Autoware control can run on an RTOS (Zephyr), on separate boards. We bring this to QNX 8.0 and go further: Autoware’s own safe-stop stack running on a real ROS 2 runtime, supervising Autoware on a single SoC with the QNX Hypervisor, with automatic takeover when Linux fails.

[![guardian-host](https://github.com/AhmedAdelWafdy7/autoware-qnx-guardian/actions/workflows/guardian-host.yml/badge.svg)](https://github.com/AhmedAdelWafdy7/autoware-qnx-guardian/actions/workflows/guardian-host.yml)
[![qnx-build](https://github.com/AhmedAdelWafdy7/autoware-qnx-guardian/actions/workflows/qnx-build.yml/badge.svg)](https://github.com/AhmedAdelWafdy7/autoware-qnx-guardian/actions/workflows/qnx-build.yml)

## How it fits together

Autoware runs in a Linux guest and does the driving: perception, localization, planning.
The guest image is built with Yocto from
[meta-autoware](https://github.com/AhmedAdelWafdy7/meta-autoware) (Autoware 1.9.0,
ROS 2 Jazzy, Yocto scarthgap).
A QNX 8.0 guest on the same Raspberry Pi 5 checks its work in two layers:

1. Autoware's own safety packages, ported to QNX and running on ROS 2 Jazzy:
   `mrm_handler`, the emergency and comfortable stop operators, `vehicle_cmd_gate`,
   `control_validator`, and a pure pursuit + PID follower.
2. The Guardian, a small QNX-native process. It runs a 100 Hz loop at a priority above
   every ROS 2 and DDS thread and has the last word on each command sent to the vehicle.

```
 Raspberry Pi 5, QNX Hypervisor 8.0

 Linux guest                    QNX 8.0 guest
 Autoware (perception,  ROS 2   Autoware safety layer ──► Guardian ──► vehicle command
 localization, planning) ─────► (ROS 2 on QNX)            (QNX-native)
```

Why two layers? A ROS 2 executor is not hard real-time. The ported safety layer gives us
Autoware's tested stop logic; the Guardian gives us deadlines. If Autoware stops sending
heartbeats, if the safety layer stalls, or if vehicle feedback goes stale, the Guardian brings
the car to a stop by itself.

The simulator is AWSIM, connected over ROS 2.

## The Guardian

Source is in [`guardian/`](guardian/). In short:

- A `CLOCK_MONOTONIC` timer delivers a pulse each cycle; the loop waits in
  `MsgReceivePulse()` at `SCHED_FIFO` with absolute deadlines.
- Heartbeats from Autoware and from the safety layer time out after 100 ms. A frozen or
  jumping sequence number is counted as a fault.
- Each command is passed, clamped or rejected, and every clamp and reject is logged with
  its reason. Checks cover steering angle, rate and sudden jumps, a speed-dependent lateral
  acceleration limit, longitudinal acceleration, and speed.
- States are NORMAL, DEGRADED, MRM_COMFORTABLE, MRM_EMERGENCY and STOPPED. An illegal
  transition is treated as a fault and goes to MRM_EMERGENCY.
- During a stop it holds the steering, ramps the braking up with limited jerk, and never
  reduces braking that Autoware already asked for.
- No allocation after start-up, no exceptions, no blocking calls in the loop. Events go to a
  lock-free ring buffer instead of `printf`.

All the limits and where their values come from are in
[`limits.hpp`](guardian/include/guardian/limits.hpp).

## Status

Done:
- Guardian logic with 60 unit tests, run in CI on x86_64 and arm64
- Guardian cross-compiled for QNX 8.0 (aarch64le and x86_64) in CI on a QNX SDP 8.0 machine

In progress:
- Running the Guardian on QNX hardware (Pi 5) and in QEMU
- ROS 2 Jazzy for QNX 8.0, built with [qnx-ports](https://github.com/qnx-ports/build-files)
- Porting the Autoware safety packages to QNX 8.0

Planned:
- `/dev/guardian` resource manager and CPU pinning
- QNX Hypervisor on the Pi 5 with Linux and QNX guests
- Closed loop with AWSIM, fault injection, and QNX vs Linux latency measurements

## Where QNX is used

| What | File |
|---|---|
| Channels, pulses, `timer_create`, `timer_settime` with `TIMER_ABSTIME`, `timer_getoverrun` | [`guardian/src/platform/main_qnx.cpp`](guardian/src/platform/main_qnx.cpp) |
| QNX SDP 8.0 toolchains (`gcc_ntoaarch64le`, `gcc_ntox86_64`) | [`guardian/cmake/`](guardian/cmake/) |
| CI build with QNX SDP 8.0; checks the binaries use the QNX APIs | [`.github/workflows/qnx-build.yml`](.github/workflows/qnx-build.yml) |

More entries will be added as the port, the resource manager and the hypervisor setup land.

## Building

Unit tests on a Linux host (no QNX SDP needed):

```bash
git clone git@github.com:AhmedAdelWafdy7/autoware-qnx-guardian.git
cd autoware-qnx-guardian/guardian
sudo apt install -y cmake ninja-build g++ googletest
cmake --preset host-test -G Ninja
cmake --build --preset host-test
ctest --preset host-test
```

Cross-build for QNX 8.0 on a Linux x86_64 host with QNX SDP 8.0 installed:

```bash
source ~/qnx800/qnxsdp-env.sh
cd guardian
cmake --preset qnx-aarch64le      # or qnx-x86_64
cmake --build --preset qnx-aarch64le
```

The binary ends up in `build/qnx-aarch64le/guardian`.

## Upstream projects

These are used by URL and pinned version. None of their code is copied into this repository.

- [meta-autoware](https://github.com/AhmedAdelWafdy7/meta-autoware): Yocto layer for the
  Linux guest, at `3e30392`
- [Autoware Universe](https://github.com/autowarefoundation/autoware_universe) 0.52.0 and
  [Autoware Core](https://github.com/autowarefoundation/autoware_core) 1.9.0
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
- [qnx-ports/build-files](https://github.com/qnx-ports/build-files): ROS 2 for QNX 8.0

## Disclaimer

This is a hackathon prototype for the QNX Global Developer Hackathon 2026. It has only been
run against a simulator, it is not certified to ISO 26262 or any other safety standard, and
it must not be used on a real vehicle.

## License

Copyright 2026 Ahmed Wafdy. Apache License 2.0, see [LICENSE](LICENSE).

QNX is a trademark of BlackBerry Limited. Autoware is a trademark of the Autoware Foundation.
This project is not affiliated with either.
