// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0
//
// RTA Switch: immediate switch to Recovery on RSS UNSAFE, return to the Complex Function only
// after the hysteresis window, window restarts on any UNSAFE, invalid outcomes fail safe.

#include <gtest/gtest.h>

#include <cstring>

#include "guardian/limits.hpp"
#include "guardian/rta_switch.hpp"

using guardian::RtaMode;
using guardian::RtaSwitch;
using guardian::TimeNs;
using guardian::rss::Outcome;
namespace L = guardian::limits;

namespace
{
constexpr TimeNs kT0 = 1'000'000'000;
constexpr TimeNs kDt = L::kCyclePeriod_ns;

/// Step `sw` with `o` from t (exclusive) for `duration` ns, one call per cycle. Returns end time.
TimeNs run(RtaSwitch & sw, Outcome o, TimeNs t, TimeNs duration)
{
  const TimeNs end = t + duration;
  while (t < end) {
    t += kDt;
    sw.step(o, t);
  }
  return t;
}
}  // namespace

TEST(RtaSwitch, PowerOnIsComplexFunction)
{
  RtaSwitch sw;
  EXPECT_EQ(sw.mode(), RtaMode::kComplexFunction);
  EXPECT_EQ(sw.switches_to_recovery(), 0UL);
}

TEST(RtaSwitch, SafeAndRestrictStayInComplexFunction)
{
  RtaSwitch sw;
  EXPECT_EQ(sw.step(Outcome::kSafe, kT0), RtaMode::kComplexFunction);
  EXPECT_EQ(sw.step(Outcome::kRestrict, kT0 + kDt), RtaMode::kComplexFunction);
}

TEST(RtaSwitch, UnsafeSwitchesToRecoveryInTheSameCycle)
{
  RtaSwitch sw;
  sw.step(Outcome::kSafe, kT0);
  EXPECT_EQ(sw.step(Outcome::kUnsafe, kT0 + kDt), RtaMode::kRecovery);
  EXPECT_EQ(sw.switches_to_recovery(), 1UL);
  EXPECT_EQ(sw.satisfied_for_ns(kT0 + kDt), 0);
}

TEST(RtaSwitch, ReturnsOnlyAfterHysteresisWindow)
{
  RtaSwitch sw;
  TimeNs t = kT0;
  sw.step(Outcome::kUnsafe, t);
  // Satisfied for one cycle less than the window: still Recovery.
  t = run(sw, Outcome::kSafe, t, L::kRtaHysteresis_ns);
  EXPECT_EQ(sw.mode(), RtaMode::kRecovery);
  EXPECT_EQ(sw.satisfied_for_ns(t), L::kRtaHysteresis_ns - kDt);
  // One more satisfied cycle completes the window.
  t += kDt;
  EXPECT_EQ(sw.step(Outcome::kSafe, t), RtaMode::kComplexFunction);
}

TEST(RtaSwitch, RestrictCountsTowardsTheWindow)
{
  RtaSwitch sw;
  TimeNs t = kT0;
  sw.step(Outcome::kUnsafe, t);
  t = run(sw, Outcome::kRestrict, t, L::kRtaHysteresis_ns + kDt);
  EXPECT_EQ(sw.mode(), RtaMode::kComplexFunction);
}

TEST(RtaSwitch, UnsafeRestartsTheWindow)
{
  RtaSwitch sw;
  TimeNs t = kT0;
  sw.step(Outcome::kUnsafe, t);
  t = run(sw, Outcome::kSafe, t, L::kRtaHysteresis_ns - 5 * kDt);
  t += kDt;
  sw.step(Outcome::kUnsafe, t);  // flicker
  t = run(sw, Outcome::kSafe, t, L::kRtaHysteresis_ns);
  EXPECT_EQ(sw.mode(), RtaMode::kRecovery);  // window restarted at the flicker
  t += kDt;
  EXPECT_EQ(sw.step(Outcome::kSafe, t), RtaMode::kComplexFunction);
  EXPECT_EQ(sw.switches_to_recovery(), 1UL);  // the flicker was already in Recovery
}

TEST(RtaSwitch, FlickeringObjectNeverReturnsToComplexFunction)
{
  // UNSAFE every 50 cycles (0.5 s) — shorter than the window, so the switch stays in Recovery.
  RtaSwitch sw;
  TimeNs t = kT0;
  for (int i = 0; i < 1000; ++i) {
    t += kDt;
    sw.step((i % 50) == 0 ? Outcome::kUnsafe : Outcome::kSafe, t);
    ASSERT_EQ(sw.mode(), RtaMode::kRecovery) << "cycle " << i;
  }
}

TEST(RtaSwitch, InvalidOutcomeIsTreatedAsUnsafe)
{
  RtaSwitch sw;
  const auto raw = static_cast<std::uint8_t>(0xAB);
  Outcome bad;
  std::memcpy(&bad, &raw, sizeof(bad));
  EXPECT_EQ(sw.step(bad, kT0), RtaMode::kRecovery);
}

TEST(RtaSwitch, ClockGoingBackwardsDoesNotShortenTheWindow)
{
  RtaSwitch sw;
  sw.step(Outcome::kUnsafe, kT0);
  sw.step(Outcome::kSafe, kT0 + kDt);
  EXPECT_EQ(sw.step(Outcome::kSafe, kT0 - 10 * L::kRtaHysteresis_ns), RtaMode::kRecovery);
}

TEST(RtaSwitch, ModeNames)
{
  EXPECT_STREQ(guardian::to_string(RtaMode::kComplexFunction), "COMPLEX_FUNCTION");
  EXPECT_STREQ(guardian::to_string(RtaMode::kRecovery), "RECOVERY");
}
