// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0
//
// Final command validation (FR-CV-1..4). Independent of Autoware's control_validator and
// vehicle_cmd_gate (defense in depth). Pure, allocation-free, noexcept.
//
// Order of checks:
//   REJECT (input unusable):  non-finite field, stale/future receive time, no fresh vehicle
//                             status, impossible single-cycle steering step (spike).
//   CLAMP  (input usable but outside the envelope), applied in this order so the final
//          output always satisfies the hard bounds:
//            1. steering rate (step vs. previous output)
//            2. physical steering angle
//            3. speed-dependent steering bound from a_lat = v^2 tan(delta) / L
//            4. longitudinal acceleration, 5. speed
//   PASS   otherwise.
// On REJECT the returned command is the previous output (caller decides what to do).

#ifndef GUARDIAN__VALIDATOR_HPP_
#define GUARDIAN__VALIDATOR_HPP_

#include <cstdint>

#include "guardian/types.hpp"

namespace guardian
{

struct ValidationResult
{
  Verdict verdict{Verdict::kReject};
  std::uint32_t reasons{reason::kNone};  ///< reason:: bit flags
  Command command{};                     ///< command to use (clamped, or previous on REJECT)
};

/// Maximum steering angle [rad] allowed at `speed_mps` by the lateral-acceleration limit.
double max_steer_for_speed(double speed_mps) noexcept;

/// Validate `in` against the envelope.
/// @param previous_output  command output in the previous cycle (rate/spike reference)
/// @param status           latest vehicle status
/// @param now_ns           current CLOCK_MONOTONIC time
ValidationResult validate(
  const Command & in, const Command & previous_output, const VehicleStatus & status,
  TimeNs now_ns) noexcept;

}  // namespace guardian

#endif  // GUARDIAN__VALIDATOR_HPP_
