// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0
//
// Plain-data types shared by the Guardian RT core. No ROS 2 types here: the bridge converts
// autoware_control_msgs::msg::Control etc. into these at the boundary, so the RT core builds
// without ROS 2 (on QNX and on the Linux host for tests/benchmarks).
//
// All times are CLOCK_MONOTONIC nanoseconds taken on the Guardian's host when the data was
// *received* — never ROS message stamps, which may be sim time (AWSIM) or another clock.

#ifndef GUARDIAN__TYPES_HPP_
#define GUARDIAN__TYPES_HPP_

#include <cstdint>

namespace guardian
{

/// Monotonic time in nanoseconds (CLOCK_MONOTONIC).
using TimeNs = std::int64_t;

/// Actuation command, mirrors the fields of autoware_control_msgs::msg::Control used here.
struct Command
{
  double steering_angle_rad{0.0};  ///< front tire angle, left positive
  double steering_rate_radps{0.0};
  double speed_mps{0.0};           ///< target longitudinal velocity
  double accel_mps2{0.0};          ///< target longitudinal acceleration
  TimeNs rx_time_ns{0};            ///< receive time on the Guardian host
};

/// Measured vehicle state (from AWSIM / vehicle interface).
struct VehicleStatus
{
  double speed_mps{0.0};
  double steering_angle_rad{0.0};
  TimeNs rx_time_ns{0};
};

/// Which supervised component a heartbeat belongs to.
enum class Source : std::uint8_t
{
  kDoer = 0,         ///< Autoware on Linux
  kSafetyLayer = 1,  ///< ported Autoware safety layer (ROS 2) on QNX
};
inline constexpr std::uint32_t kNumSources = 2U;

/// Guardian MRM state machine (FR-MRM-3).
enum class State : std::uint8_t
{
  kNormal = 0,
  kDegraded = 1,        ///< doer lost; Autoware's ported MRM on QNX is in charge (FR-MRM-1)
  kMrmComfortable = 2,  ///< Guardian's own comfortable stop (FR-MRM-2)
  kMrmEmergency = 3,    ///< Guardian's own emergency stop
  kStopped = 4,         ///< latched; operator reset required
};
inline constexpr std::uint32_t kNumStates = 5U;

const char * to_string(State s) noexcept;

/// Validator outcome (FR-CV-2). Never silent: CLAMP and REJECT are always logged.
enum class Verdict : std::uint8_t
{
  kPass = 0,
  kClamp = 1,
  kReject = 2,
};

const char * to_string(Verdict v) noexcept;

/// Bit flags explaining a CLAMP or REJECT.
namespace reason
{
inline constexpr std::uint32_t kNone = 0U;
inline constexpr std::uint32_t kNonFinite = 1U << 0U;       ///< NaN / Inf in any field
inline constexpr std::uint32_t kStale = 1U << 1U;           ///< command older than limit
inline constexpr std::uint32_t kSteerSpike = 1U << 2U;      ///< impossible single-cycle step
inline constexpr std::uint32_t kSteerAngle = 1U << 3U;      ///< |delta| > physical limit
inline constexpr std::uint32_t kSteerRate = 1U << 4U;       ///< step > rate limit * period
inline constexpr std::uint32_t kLatAccel = 1U << 5U;        ///< v^2 tan(delta)/L too high
inline constexpr std::uint32_t kLonAccelHigh = 1U << 6U;
inline constexpr std::uint32_t kLonAccelLow = 1U << 7U;
inline constexpr std::uint32_t kSpeed = 1U << 8U;
inline constexpr std::uint32_t kNoVehicleStatus = 1U << 9U;  ///< status stale: cannot check
}  // namespace reason

}  // namespace guardian

#endif  // GUARDIAN__TYPES_HPP_
