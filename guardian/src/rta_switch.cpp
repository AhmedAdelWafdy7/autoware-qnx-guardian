// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0

#include "guardian/rta_switch.hpp"

#include "guardian/limits.hpp"

namespace guardian
{

const char * to_string(RtaMode m) noexcept
{
  switch (m) {
    case RtaMode::kComplexFunction:
      return "COMPLEX_FUNCTION";
    case RtaMode::kRecovery:
      return "RECOVERY";
  }
  return "UNKNOWN";
}

RtaMode RtaSwitch::step(rss::Outcome outcome, TimeNs now_ns) noexcept
{
  const bool satisfied = outcome == rss::Outcome::kSafe || outcome == rss::Outcome::kRestrict;

  if (!satisfied) {
    satisfied_ = false;
    if (mode_ != RtaMode::kRecovery) {
      mode_ = RtaMode::kRecovery;
      ++to_recovery_;
    }
    return mode_;
  }

  if (!satisfied_) {
    satisfied_ = true;
    satisfied_since_ns_ = now_ns;
  }
  if (mode_ == RtaMode::kRecovery && satisfied_for_ns(now_ns) >= limits::kRtaHysteresis_ns) {
    mode_ = RtaMode::kComplexFunction;
  }
  return mode_;
}

TimeNs RtaSwitch::satisfied_for_ns(TimeNs now_ns) const noexcept
{
  if (!satisfied_ || now_ns < satisfied_since_ns_) {
    return 0;  // a clock going backwards never shortens the window
  }
  return now_ns - satisfied_since_ns_;
}

}  // namespace guardian
