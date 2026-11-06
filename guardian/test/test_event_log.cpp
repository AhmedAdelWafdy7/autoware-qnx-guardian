// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <thread>

#include "guardian/event_log.hpp"

using guardian::Event;
using guardian::EventCode;
using guardian::EventLog;

TEST(EventLog, FifoOrder)
{
  EventLog<4> log;
  Event e;
  for (int i = 0; i < 3; ++i) {
    e.value = i;
    EXPECT_TRUE(log.push(e));
  }
  for (int i = 0; i < 3; ++i) {
    ASSERT_TRUE(log.pop(e));
    EXPECT_EQ(e.value, i);
  }
  EXPECT_FALSE(log.pop(e));
}

TEST(EventLog, FullDropsAndCountsNeverBlocks)
{
  EventLog<4> log;
  Event e;
  for (int i = 0; i < 4; ++i) {
    EXPECT_TRUE(log.push(e));
  }
  EXPECT_FALSE(log.push(e));
  EXPECT_FALSE(log.push(e));
  EXPECT_EQ(log.dropped(), 2U);
}

TEST(EventLog, WrapsAround)
{
  EventLog<4> log;
  Event e;
  for (int i = 0; i < 100; ++i) {
    e.value = i;
    ASSERT_TRUE(log.push(e));
    ASSERT_TRUE(log.pop(e));
    EXPECT_EQ(e.value, i);
  }
}

TEST(EventLog, SpscConcurrentNoLossNoReorder)
{
  static EventLog<1024> log;
  constexpr int kN = 200000;
  std::thread producer([] {
    Event e;
    e.code = EventCode::kClamp;
    for (int i = 0; i < kN; ++i) {
      e.value = i;
      while (!log.push(e)) {
        std::this_thread::yield();
      }
    }
  });
  int expected = 0;
  Event e;
  while (expected < kN) {
    if (log.pop(e)) {
      ASSERT_EQ(e.value, expected);
      ++expected;
    }
  }
  producer.join();
}
