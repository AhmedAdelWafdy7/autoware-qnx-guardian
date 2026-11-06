// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0
//
// QNX 8.0 Guardian loop: a CLOCK_MONOTONIC timer delivers a pulse to a private channel every
// period (FR-OUT-2); the Guardian thread blocks in MsgReceivePulse() at SCHED_FIFO.
// Absolute first expiry + it_interval gives drift-free periodic wake-ups.
//
// Compiles with QNX SDP 8.0 (q++ 12.2.0, gcc_ntoaarch64le + gcc_ntox86_64) under -Werror.
// Needs _QNX_SOURCE (set in CMakeLists.txt): strict -std=c++17 hides the POSIX timer API and
// SIGEV_PULSE_INIT in the QNX headers.
// TODO(verify): not yet run on target (Pi 5 / QEMU).
// Planned next: /dev/guardian resource manager thread (FR-OBS-1), runmask pinning,
// bridge inputs (TODO(bridge)).
//
// Usage: guardian [--cycles N] [--prio P]

#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/neutrino.h>
#include <sys/siginfo.h>
#include <time.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "cycle_stats.hpp"
#include "guardian/guardian.hpp"
#include "guardian/limits.hpp"

namespace
{
constexpr std::int64_t kNsPerSec = 1'000'000'000;
constexpr int kDefaultPriority = 80;  ///< above all ROS 2 / DDS threads; docs/architecture.md
constexpr int kTimerPulseCode = _PULSE_CODE_MINAVAIL;
// QNX defines TIMER_ABSTIME as 0x80000000U but timer_settime() takes an int flags argument.
constexpr int kTimerAbsTime = static_cast<int>(TIMER_ABSTIME);

std::int64_t now_ns() noexcept
{
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::int64_t>(ts.tv_sec) * kNsPerSec + ts.tv_nsec;
}

timespec to_timespec(std::int64_t ns) noexcept
{
  timespec ts{};
  ts.tv_sec = static_cast<time_t>(ns / kNsPerSec);
  ts.tv_nsec = static_cast<long>(ns % kNsPerSec);
  return ts;
}
}  // namespace

int main(int argc, char ** argv)
{
  long cycles = 1000;
  int prio = kDefaultPriority;
  for (int i = 1; i + 1 < argc; i += 2) {
    if (std::strcmp(argv[i], "--cycles") == 0) {
      cycles = std::strtol(argv[i + 1], nullptr, 10);
    } else if (std::strcmp(argv[i], "--prio") == 0) {
      prio = static_cast<int>(std::strtol(argv[i + 1], nullptr, 10));
    }
  }

  // ---- init phase (may allocate / fail) ----
  const int prio_min = sched_get_priority_min(SCHED_FIFO);
  const int prio_max = sched_get_priority_max(SCHED_FIFO);
  if (prio < prio_min || prio > prio_max) {
    std::fprintf(stderr, "error: --prio %d outside SCHED_FIFO range [%d, %d]\n", prio, prio_min,
                 prio_max);
    return 1;
  }
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    std::fprintf(stderr, "warn: mlockall: %s\n", std::strerror(errno));
  }
  sched_param sp{};
  sp.sched_priority = prio;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != EOK) {
    std::fprintf(stderr, "error: SCHED_FIFO %d failed\n", prio);
    return 1;
  }

  const int chid = ChannelCreate(0);
  if (chid == -1) {
    std::perror("ChannelCreate");
    return 1;
  }
  const int coid = ConnectAttach(0, 0, chid, _NTO_SIDE_CHANNEL, 0);
  if (coid == -1) {
    std::perror("ConnectAttach");
    return 1;
  }
  sigevent ev{};
  // Pulse priority field is a short; prio was range-checked above.
  SIGEV_PULSE_INIT(&ev, coid, static_cast<short>(prio), kTimerPulseCode, 0);

  timer_t timer_id{};
  if (timer_create(CLOCK_MONOTONIC, &ev, &timer_id) == -1) {
    std::perror("timer_create");
    return 1;
  }

  static guardian::GuardianEventLog log;
  static guardian::Guardian g(&log);
  static guardian::platform::CycleStats stats;
  guardian::CycleInputs in{};

  std::int64_t deadline = now_ns() + guardian::limits::kCyclePeriod_ns;
  itimerspec its{};
  its.it_value = to_timespec(deadline);
  its.it_interval = to_timespec(guardian::limits::kCyclePeriod_ns);
  if (timer_settime(timer_id, kTimerAbsTime, &its, nullptr) == -1) {
    std::perror("timer_settime");
    return 1;
  }

  // ---- RT phase: no allocation, no blocking except MsgReceivePulse ----
  _pulse pulse{};
  for (long c = 0; c < cycles; ++c) {
    if (MsgReceivePulse(chid, &pulse, sizeof(pulse), nullptr) == -1) {
      in.internal_fault = true;  // fail safe: Guardian goes to MRM
    } else if (pulse.code != kTimerPulseCode) {
      --c;  // not our timer; wait again
      continue;
    }
    const std::int64_t woke = now_ns();
    const std::int64_t lateness = woke - deadline;
    stats.record(lateness);

    const int overruns = timer_getoverrun(timer_id);
    in.now_ns = woke;
    in.deadline_missed = overruns > 0 || lateness > guardian::limits::kCyclePeriod_ns;
    (void)g.step(in);  // TODO(bridge): read inputs, publish output

    deadline += guardian::limits::kCyclePeriod_ns * (1 + (overruns > 0 ? overruns : 0));
  }

  timer_delete(timer_id);
  ConnectDetach(coid);
  ChannelDestroy(chid);

  stats.print(stdout, "qnx");
  std::printf("state=%s events_dropped=%llu\n", guardian::to_string(g.state()),
              static_cast<unsigned long long>(log.dropped()));
  return 0;
}
