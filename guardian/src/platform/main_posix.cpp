// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0
//
// POSIX (Linux) Guardian loop: absolute-deadline clock_nanosleep on CLOCK_MONOTONIC.
// Exists for host development and as the Linux baseline in the QNX-vs-Linux benchmark
// (NFR-T-3). The QNX build uses main_qnx.cpp (timer_create + pulse) instead.
//
// Skeleton: inputs are not wired to the bridge yet (TODO(bridge)), so the Guardian stays in
// its power-on STOPPED state; the loop timing is real.
//
// Usage: guardian [--cycles N] [--prio P]

#include <sched.h>
#include <sys/mman.h>
#include <time.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "cycle_stats.hpp"
#include "guardian/guardian.hpp"
#include "guardian/limits.hpp"

namespace
{
constexpr std::int64_t kNsPerSec = 1'000'000'000;

/// Default SCHED_FIFO priority. Must stay above every ROS 2 / DDS thread
/// (see docs/architecture.md). TODO(verify) against the final priority map.
constexpr int kDefaultPriority = 80;

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

  // Init phase: everything that may allocate or fail happens here.
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    std::fprintf(stderr, "warn: mlockall: %s\n", std::strerror(errno));
  }
  sched_param sp{};
  sp.sched_priority = prio;
  if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
    std::fprintf(stderr, "warn: SCHED_FIFO %d: %s (running with default policy)\n", prio,
                 std::strerror(errno));
  }

  static guardian::GuardianEventLog log;
  static guardian::Guardian g(&log);
  static guardian::platform::CycleStats stats;
  guardian::CycleInputs in{};

  std::int64_t deadline = now_ns() + guardian::limits::kCyclePeriod_ns;
  for (long c = 0; c < cycles; ++c) {
    const timespec ts = to_timespec(deadline);
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {
    }
    const std::int64_t woke = now_ns();
    const std::int64_t lateness = woke - deadline;
    stats.record(lateness);

    in.now_ns = woke;
    in.deadline_missed = lateness > guardian::limits::kCyclePeriod_ns;
    (void)g.step(in);  // TODO(bridge): read inputs, publish output

    deadline += guardian::limits::kCyclePeriod_ns;
  }

  stats.print(stdout, "posix");
  std::printf("state=%s events_dropped=%llu\n", guardian::to_string(g.state()),
              static_cast<unsigned long long>(log.dropped()));
  return 0;
}
