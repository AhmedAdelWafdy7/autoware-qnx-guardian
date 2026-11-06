// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0
//
// Heartbeat supervision for the Autoware doer (Linux) and the ported ROS 2 safety layer
// (QNX) — FR-HB-1..5. Single-threaded: owned by the Guardian cycle; the bridge hands over
// the latest sample per source each cycle (see bridge/). Allocation-free, noexcept.

#ifndef GUARDIAN__HEARTBEAT_MONITOR_HPP_
#define GUARDIAN__HEARTBEAT_MONITOR_HPP_

#include <cstdint>

#include "guardian/types.hpp"

namespace guardian
{

/// Latest heartbeat seen by the bridge for one source.
struct HeartbeatSample
{
  bool valid{false};       ///< false until the first heartbeat ever arrives
  std::uint32_t seq{0};    ///< publisher sequence counter
  TimeNs rx_time_ns{0};    ///< receive time (CLOCK_MONOTONIC)
};

class HeartbeatMonitor
{
public:
  /// Feed the latest sample for `src`. A sample with the same receive time as last cycle is
  /// "nothing new". A new message whose seq did not advance is a frozen publisher (FR-HB-5)
  /// and does NOT refresh liveness.
  void update(Source src, const HeartbeatSample & sample) noexcept;

  /// True if a fresh, advancing heartbeat from `src` arrived within the timeout.
  bool alive(Source src, TimeNs now_ns) const noexcept;

  std::uint32_t freeze_count(Source src) const noexcept { return ch(src).freezes; }
  std::uint32_t jump_count(Source src) const noexcept { return ch(src).jumps; }

private:
  struct Channel
  {
    bool seen{false};
    std::uint32_t last_seq{0};
    TimeNs last_rx_ns{0};      ///< last receive time of any message
    TimeNs last_alive_ns{0};   ///< last receive time of an *advancing* message
    std::uint32_t freezes{0};
    std::uint32_t jumps{0};
  };

  const Channel & ch(Source s) const noexcept { return ch_[static_cast<std::uint32_t>(s)]; }
  Channel & ch(Source s) noexcept { return ch_[static_cast<std::uint32_t>(s)]; }

  Channel ch_[kNumSources]{};
};

}  // namespace guardian

#endif  // GUARDIAN__HEARTBEAT_MONITOR_HPP_
