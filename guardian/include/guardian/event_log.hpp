// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0
//
// Lock-free single-producer / single-consumer event log (FR-OBS-2, NFR-Q-1).
// Producer: the Guardian RT cycle (never blocks, never allocates; drops and counts on full).
// Consumer: a low-priority logger thread or the /dev/guardian resource manager.

#ifndef GUARDIAN__EVENT_LOG_HPP_
#define GUARDIAN__EVENT_LOG_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "guardian/types.hpp"

namespace guardian
{

enum class EventCode : std::uint16_t
{
  kStateChange = 1,   ///< a = old state, b = new state
  kClamp = 2,         ///< reasons = reason:: flags
  kReject = 3,        ///< reasons = reason:: flags
  kDeadlineMiss = 4,  ///< value = lateness [ns]
  kHeartbeatLost = 5, ///< a = Source
  kIllegalTransition = 6,
};

struct Event
{
  TimeNs time_ns{0};
  EventCode code{EventCode::kStateChange};
  std::uint8_t a{0};
  std::uint8_t b{0};
  std::uint32_t reasons{0};
  std::int64_t value{0};
};

template <std::size_t Capacity>
class EventLog
{
  static_assert(Capacity >= 2U && (Capacity & (Capacity - 1U)) == 0U, "power of two");

public:
  /// Producer side. Returns false (and counts a drop) if the buffer is full.
  bool push(const Event & e) noexcept
  {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t tail = tail_.load(std::memory_order_acquire);
    if (head - tail >= Capacity) {
      dropped_.fetch_add(1U, std::memory_order_relaxed);
      return false;
    }
    buf_[head & (Capacity - 1U)] = e;
    head_.store(head + 1U, std::memory_order_release);
    return true;
  }

  /// Consumer side. Returns false if empty.
  bool pop(Event & out) noexcept
  {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    const std::size_t head = head_.load(std::memory_order_acquire);
    if (tail == head) {
      return false;
    }
    out = buf_[tail & (Capacity - 1U)];
    tail_.store(tail + 1U, std::memory_order_release);
    return true;
  }

  std::uint64_t dropped() const noexcept { return dropped_.load(std::memory_order_relaxed); }

private:
  Event buf_[Capacity]{};
  alignas(64) std::atomic<std::size_t> head_{0};
  alignas(64) std::atomic<std::size_t> tail_{0};
  alignas(64) std::atomic<std::uint64_t> dropped_{0};
};

}  // namespace guardian

#endif  // GUARDIAN__EVENT_LOG_HPP_
