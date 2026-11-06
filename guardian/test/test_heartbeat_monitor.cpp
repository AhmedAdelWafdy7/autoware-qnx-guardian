// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0
//
// FR-HB-1, -2, -5.

#include <gtest/gtest.h>

#include <cstdint>

#include "guardian/heartbeat_monitor.hpp"
#include "guardian/limits.hpp"

using guardian::HeartbeatMonitor;
using guardian::HeartbeatSample;
using guardian::Source;
namespace L = guardian::limits;

namespace
{
HeartbeatSample hb(std::uint32_t seq, guardian::TimeNs t)
{
  HeartbeatSample s;
  s.valid = true;
  s.seq = seq;
  s.rx_time_ns = t;
  return s;
}
}  // namespace

TEST(Heartbeat, NotAliveBeforeFirstBeat)
{
  HeartbeatMonitor m;
  EXPECT_FALSE(m.alive(Source::kDoer, 0));
  m.update(Source::kDoer, HeartbeatSample{});  // invalid sample ignored
  EXPECT_FALSE(m.alive(Source::kDoer, 0));
}

TEST(Heartbeat, AliveUntilTimeoutThenDead)
{
  HeartbeatMonitor m;
  m.update(Source::kDoer, hb(1, 1000));
  EXPECT_TRUE(m.alive(Source::kDoer, 1000));
  EXPECT_TRUE(m.alive(Source::kDoer, 1000 + L::kHeartbeatTimeout_ns));
  EXPECT_FALSE(m.alive(Source::kDoer, 1000 + L::kHeartbeatTimeout_ns + 1));
  EXPECT_FALSE(m.alive(Source::kDoer, 999));  // time went backwards: not trusted
}

TEST(Heartbeat, SourcesAreIndependent)
{
  HeartbeatMonitor m;
  m.update(Source::kDoer, hb(1, 1000));
  EXPECT_TRUE(m.alive(Source::kDoer, 1000));
  EXPECT_FALSE(m.alive(Source::kSafetyLayer, 1000));
  EXPECT_FALSE(m.alive(static_cast<Source>(9), 1000));
  m.update(static_cast<Source>(9), hb(1, 1000));  // ignored, no crash
}

TEST(Heartbeat, AdvancingSeqRefreshes)
{
  HeartbeatMonitor m;
  m.update(Source::kDoer, hb(1, 0));
  m.update(Source::kDoer, hb(2, 90'000'000));
  EXPECT_TRUE(m.alive(Source::kDoer, 150'000'000));
}

TEST(Heartbeat, FrozenSeqDoesNotRefresh)
{
  HeartbeatMonitor m;
  m.update(Source::kSafetyLayer, hb(5, 0));
  m.update(Source::kSafetyLayer, hb(5, 50'000'000));  // new message, same seq
  m.update(Source::kSafetyLayer, hb(5, 90'000'000));
  EXPECT_EQ(m.freeze_count(Source::kSafetyLayer), 2U);
  EXPECT_FALSE(m.alive(Source::kSafetyLayer, L::kHeartbeatTimeout_ns + 1));
}

TEST(Heartbeat, SameSampleTwiceIsNothingNew)
{
  HeartbeatMonitor m;
  m.update(Source::kDoer, hb(1, 100));
  m.update(Source::kDoer, hb(1, 100));
  EXPECT_EQ(m.freeze_count(Source::kDoer), 0U);
}

TEST(Heartbeat, LargeJumpCountedButStillAlive)
{
  HeartbeatMonitor m;
  m.update(Source::kDoer, hb(1, 0));
  m.update(Source::kDoer, hb(1 + L::kMaxHeartbeatSeqJump + 1, 10));
  EXPECT_EQ(m.jump_count(Source::kDoer), 1U);
  EXPECT_TRUE(m.alive(Source::kDoer, 10));
  m.update(Source::kDoer, hb(1 + 2 * L::kMaxHeartbeatSeqJump + 1, 20));  // exactly the limit
  EXPECT_EQ(m.jump_count(Source::kDoer), 1U);
}

TEST(Heartbeat, SeqWrapAroundIsNormal)
{
  HeartbeatMonitor m;
  m.update(Source::kDoer, hb(0xFFFFFFFFU, 0));
  m.update(Source::kDoer, hb(0U, 10));
  EXPECT_EQ(m.jump_count(Source::kDoer), 0U);
  EXPECT_EQ(m.freeze_count(Source::kDoer), 0U);
  EXPECT_TRUE(m.alive(Source::kDoer, 10));
}
