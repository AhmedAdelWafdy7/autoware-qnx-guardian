// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0

#include "guardian/heartbeat_monitor.hpp"

#include "guardian/limits.hpp"

namespace guardian
{

void HeartbeatMonitor::update(Source src, const HeartbeatSample & s) noexcept
{
  if (static_cast<std::uint32_t>(src) >= kNumSources || !s.valid) {
    return;
  }
  Channel & c = ch(src);
  if (c.seen && s.rx_time_ns == c.last_rx_ns) {
    return;  // same sample as last cycle: nothing new
  }
  if (!c.seen) {
    c.seen = true;
    c.last_seq = s.seq;
    c.last_rx_ns = s.rx_time_ns;
    c.last_alive_ns = s.rx_time_ns;
    return;
  }
  c.last_rx_ns = s.rx_time_ns;
  const std::uint32_t delta = s.seq - c.last_seq;  // unsigned: wraps correctly
  if (delta == 0U) {
    ++c.freezes;  // publisher alive but stuck: do not refresh liveness
    return;
  }
  if (delta > limits::kMaxHeartbeatSeqJump) {
    ++c.jumps;  // counted (FR-HB-5); still a live, advancing publisher
  }
  c.last_seq = s.seq;
  c.last_alive_ns = s.rx_time_ns;
}

bool HeartbeatMonitor::alive(Source src, TimeNs now_ns) const noexcept
{
  if (static_cast<std::uint32_t>(src) >= kNumSources) {
    return false;
  }
  const Channel & c = ch(src);
  return c.seen && c.last_alive_ns <= now_ns &&
         (now_ns - c.last_alive_ns) <= limits::kHeartbeatTimeout_ns;
}

}  // namespace guardian
