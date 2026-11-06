// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0

#include "guardian/validator.hpp"

#include <cmath>

#include "guardian/limits.hpp"

namespace guardian
{

const char * to_string(Verdict v) noexcept
{
  switch (v) {
    case Verdict::kPass:
      return "PASS";
    case Verdict::kClamp:
      return "CLAMP";
    case Verdict::kReject:
      return "REJECT";
  }
  return "UNKNOWN";
}

namespace
{
constexpr double kCyclePeriod_s = static_cast<double>(limits::kCyclePeriod_ns) * 1e-9;

/// Below this speed the lateral-acceleration bound exceeds the physical steering limit
/// anyway; skipping it avoids dividing by ~0.
constexpr double kLatBoundMinSpeed_mps = 0.1;

bool finite(const Command & c) noexcept
{
  return std::isfinite(c.steering_angle_rad) && std::isfinite(c.steering_rate_radps) &&
         std::isfinite(c.speed_mps) && std::isfinite(c.accel_mps2);
}

/// Clamp `value` to [lo, hi]; set `flag` in `reasons` if it changed.
double clamp_flag(
  double value, double lo, double hi, std::uint32_t flag, std::uint32_t & reasons) noexcept
{
  if (value < lo) {
    reasons |= flag;
    return lo;
  }
  if (value > hi) {
    reasons |= flag;
    return hi;
  }
  return value;
}

bool age_ok(TimeNs rx, TimeNs now, TimeNs max_age) noexcept
{
  return rx <= now && (now - rx) <= max_age;
}
}  // namespace

double max_steer_for_speed(double speed_mps) noexcept
{
  const double v = std::fabs(speed_mps);
  if (!(v >= kLatBoundMinSpeed_mps)) {  // also catches NaN
    return limits::kMaxSteerAngle_rad;
  }
  const double bound =
    std::atan(limits::kMaxLatAccel_mps2 * limits::kWheelbase_m / (v * v));
  return bound < limits::kMaxSteerAngle_rad ? bound : limits::kMaxSteerAngle_rad;
}

ValidationResult validate(
  const Command & in, const Command & previous_output, const VehicleStatus & status,
  TimeNs now_ns) noexcept
{
  ValidationResult r;
  r.command = previous_output;

  // ---- REJECT checks --------------------------------------------------------
  std::uint32_t reject = reason::kNone;
  if (!finite(in)) {
    reject |= reason::kNonFinite;
  }
  if (!age_ok(in.rx_time_ns, now_ns, limits::kCommandMaxAge_ns)) {
    reject |= reason::kStale;
  }
  if (!age_ok(status.rx_time_ns, now_ns, limits::kVehicleStatusMaxAge_ns) ||
      !std::isfinite(status.speed_mps)) {
    reject |= reason::kNoVehicleStatus;
  }
  if ((reject & reason::kNonFinite) == 0U &&
      std::fabs(in.steering_angle_rad - previous_output.steering_angle_rad) >
        limits::kSteerSpikeReject_rad) {
    reject |= reason::kSteerSpike;
  }
  if (reject != reason::kNone) {
    r.verdict = Verdict::kReject;
    r.reasons = reject;
    return r;
  }

  // ---- CLAMP checks ---------------------------------------------------------
  std::uint32_t clamp = reason::kNone;
  Command out = in;

  const double max_step = limits::kMaxSteerRate_radps * kCyclePeriod_s;
  const double prev = previous_output.steering_angle_rad;
  out.steering_angle_rad =
    clamp_flag(out.steering_angle_rad, prev - max_step, prev + max_step, reason::kSteerRate, clamp);
  out.steering_rate_radps = clamp_flag(
    out.steering_rate_radps, -limits::kMaxSteerRate_radps, limits::kMaxSteerRate_radps,
    reason::kSteerRate, clamp);

  out.steering_angle_rad = clamp_flag(
    out.steering_angle_rad, -limits::kMaxSteerAngle_rad, limits::kMaxSteerAngle_rad,
    reason::kSteerAngle, clamp);

  const double lat_bound = max_steer_for_speed(status.speed_mps);
  out.steering_angle_rad =
    clamp_flag(out.steering_angle_rad, -lat_bound, lat_bound, reason::kLatAccel, clamp);

  out.accel_mps2 = clamp_flag(
    out.accel_mps2, limits::kMinLonAccel_mps2, limits::kMaxLonAccel_mps2,
    out.accel_mps2 > 0.0 ? reason::kLonAccelHigh : reason::kLonAccelLow, clamp);

  out.speed_mps =
    clamp_flag(out.speed_mps, -limits::kMaxSpeed_mps, limits::kMaxSpeed_mps, reason::kSpeed, clamp);

  r.command = out;
  r.reasons = clamp;
  r.verdict = (clamp == reason::kNone) ? Verdict::kPass : Verdict::kClamp;
  return r;
}

}  // namespace guardian
