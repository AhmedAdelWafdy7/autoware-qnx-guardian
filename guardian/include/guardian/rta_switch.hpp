// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0
//
// RTA Switch (ASTM F3269): chooses between the Complex Function (Autoware, through the ported
// safety layer) and the Recovery Function, driven by the RSS outcome on the independent channel.
//
// Switching rule (Eberhart et al., IEEE IV 2023):
//   * RSS UNSAFE                      -> RECOVERY at once (same cycle).
//   * RECOVERY -> COMPLEX_FUNCTION    only after RSS has not been UNSAFE for
//                                     limits::kRtaHysteresis_ns without a break.
//   * Any UNSAFE restarts the hysteresis window.
// RESTRICT counts as satisfied: the situation is safe, only the command is clamped (and the
// Guardian applies the clamp on every output anyway).
//
// This is independent of the MRM state machine: the RTA Switch never latches. Faults of the
// Doer, the ROS 2 layer or the Guardian itself are handled by the state machine (MRM latch,
// STOPPED until operator reset). Guardian::step() combines both.
//
// Power-on mode is COMPLEX_FUNCTION: the Guardian powers on in STOPPED, so nothing drives until
// an operator reset, and the switch is stepped every cycle, so an UNSAFE at reset time puts it
// into RECOVERY in that same cycle.
//
// Pure, allocation-free and noexcept: safe for the RT path.

#ifndef GUARDIAN__RTA_SWITCH_HPP_
#define GUARDIAN__RTA_SWITCH_HPP_

#include <cstdint>

#include "guardian/rss/rss.hpp"
#include "guardian/types.hpp"

namespace guardian
{

enum class RtaMode : std::uint8_t
{
  kComplexFunction = 0,  ///< Autoware's (validated, RSS-clamped) command drives
  kRecovery = 1,         ///< RSS violated: Recovery Function brakes
};

const char * to_string(RtaMode m) noexcept;

class RtaSwitch
{
public:
  /// Run one cycle with this cycle's RSS outcome. Returns the new mode.
  /// An outcome value outside the enum is treated as UNSAFE (fail safe).
  RtaMode step(rss::Outcome outcome, TimeNs now_ns) noexcept;

  RtaMode mode() const noexcept { return mode_; }

  /// Time RSS has been continuously not UNSAFE [ns], 0 if UNSAFE this cycle.
  TimeNs satisfied_for_ns(TimeNs now_ns) const noexcept;

  unsigned long switches_to_recovery() const noexcept { return to_recovery_; }

private:
  RtaMode mode_{RtaMode::kComplexFunction};
  bool satisfied_{false};
  TimeNs satisfied_since_ns_{0};
  unsigned long to_recovery_{0};
};

}  // namespace guardian

#endif  // GUARDIAN__RTA_SWITCH_HPP_
