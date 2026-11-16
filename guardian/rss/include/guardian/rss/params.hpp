// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0
//
// Every RSS parameter in one place. Units are in the name suffix; every value states its source.
// RSS model: Shalev-Shwartz, Shammah, Shashua, "On a Formal Model of Safe and Scalable
// Self-driving Cars", arXiv:1708.06374.
//
// Changing a value here needs a justification comment and passing oracle tests (ad-rss-lib).
//
// All accelerations are positive magnitudes. Response time rho: during rho a road user may still
// accelerate at lon_accel_max / lat_accel_max towards the other; after rho it brakes with at least
// lon_brake_min / lat_brake_min. A front vehicle may brake with at most lon_brake_max.

#ifndef GUARDIAN__RSS__PARAMS_HPP_
#define GUARDIAN__RSS__PARAMS_HPP_

#include <cstdint>

#include "guardian/limits.hpp"

namespace guardian::rss
{

/// RSS dynamics of one road user.
struct Dynamics
{
  double response_time_s;
  double lon_accel_max_mps2;
  double lon_brake_min_mps2;          ///< minimum braking it is guaranteed to apply
  double lon_brake_min_correct_mps2;  ///< same, when it drives in its correct lane (oncoming)
  double lon_brake_max_mps2;          ///< maximum braking it may apply (as a front vehicle)
  double lat_accel_max_mps2;
  double lat_brake_min_mps2;
  double lat_fluctuation_margin_m;    ///< mu: lateral position noise; both users' halves add up
};

namespace params
{

/// Upper bound on the objects evaluated per cycle. Bounds the RSS loop (no allocation, fixed
/// work). The Input Manager keeps the nearest kMaxObjects; passing more is a caller error and
/// is treated as UNSAFE. 64 covers the AWSIM Nishi-Shinjuku NPC traffic around the ego.
/// TODO(verify): worst-case execution time of 64 objects on the Pi 5 (bench/).
inline constexpr std::uint32_t kMaxObjects = 64U;

/// Small negative ego speeds are odometry noise at standstill (AWSIM reports a few cm/s).
/// Up to this much reverse speed counts as 0; more means the ego is reversing, which this RSS
/// model does not cover, so the situation is invalid (UNSAFE).
inline constexpr double kSpeedNoise_mps = 0.1;

/// Ego vehicle (supervised by the Guardian).
inline constexpr Dynamics kEgo{
  // rho: Guardian detection (one 10 ms cycle) + maximum input age accepted by the time
  // safeguard (100 ms) + brake actuation lag. TODO(verify): against the measured fault->brake
  // latency (logic analyzer) and the Recovery brake ramp, see kRecoveryNote below.
  0.5,
  // The Guardian clamps the ego's commanded acceleration to this (RESTRICT), so the
  // response-time assumption holds by construction. Above Autoware's planning limit
  // (motion velocity smoother max_acc = 1.0 m/s^2, TODO(verify) at the pinned tag), so it only
  // binds when the Doer misbehaves.
  2.0,
  // Braking the Recovery Function guarantees: the Guardian emergency stop
  // (limits::kMrmEmergencyDecel_mps2 = -2.5). Tied by static_assert below.
  2.5,
  // The ego is assumed to drive in its correct lane; no harder braking is assumed there.
  2.5,
  // Strongest braking the Guardian lets through (limits::kMinLonAccel_mps2 = -8.0).
  8.0,
  // Lateral values: ad-rss-lib / CARLA RssSensor defaults. TODO(verify) against the pinned
  // ad-rss-lib release once the oracle test fetches it.
  0.2,
  0.8,
  0.1,
};

/// Other vehicles. Starting values from the ad-rss-lib / CARLA RssSensor defaults for other
/// vehicles. TODO(verify) against the pinned release; tune with the nominal-route intervention
/// rate (RSS must not stop the car in normal traffic).
inline constexpr Dynamics kVehicle{
  2.0,  // rho
  3.5,  // lon accel max
  4.0,  // lon brake min
  3.0,  // lon brake min (correct lane)
  8.0,  // lon brake max
  0.2,  // lat accel max
  0.8,  // lat brake min
  0.1,  // mu
};

/// Pedestrians. Own starting values, not from ad-rss-lib (which uses a separate unstructured
/// model for pedestrians): a walking pedestrian (<= 2 m/s) can start or stop within about 1 s
/// in any direction, so lateral and longitudinal values are equal. TODO(verify): tune in the
/// pedestrian scenarios.
inline constexpr Dynamics kPedestrian{
  0.5,  // rho
  2.0,  // lon accel max
  2.0,  // lon brake min
  2.0,  // lon brake min (correct lane)
  2.0,  // lon brake max
  2.0,  // lat accel max
  2.0,  // lat brake min
  0.1,  // mu
};

/// Unknown objects: the worst case of kVehicle and kPedestrian in every field.
inline constexpr Dynamics kUnknown{
  2.0,  // rho: max
  3.5,  // lon accel max: max
  2.0,  // lon brake min: min
  2.0,  // lon brake min (correct lane): min
  8.0,  // lon brake max: max
  2.0,  // lat accel max: max
  0.8,  // lat brake min: min
  0.1,  // mu: max
};

// kRecoveryNote: RSS assumes the ego reaches -lon_brake_min within rho after UNSAFE. The Guardian
// MRM ramp (limits::kMrmJerk_mps3 = 1.5 m/s^3) would need (2.0 + 2.5) / 1.5 = 3 s from full
// acceleration, not 0.5 s. Resolved in Guardian::step(): every output (any actor, MRM included)
// is clamped to the RSS acceleration bound without a ramp, and the RTA Switch's Recovery
// commands limits::kRtaRecoveryDecel_mps2 as a step. The remaining lag is the actuator's,
// covered by rho. TODO(verify): measured fault->brake latency (logic analyzer) < rho.

// ---- consistency checks ----
constexpr bool well_formed(const Dynamics & d)
{
  return d.response_time_s >= 0.0 && d.lon_accel_max_mps2 >= 0.0 &&
         d.lon_brake_min_correct_mps2 > 0.0 &&
         d.lon_brake_min_correct_mps2 <= d.lon_brake_min_mps2 &&
         d.lon_brake_min_mps2 <= d.lon_brake_max_mps2 && d.lat_accel_max_mps2 >= 0.0 &&
         d.lat_brake_min_mps2 > 0.0 && d.lat_fluctuation_margin_m >= 0.0;
}
static_assert(well_formed(kEgo), "kEgo: need 0 < brake_min_correct <= brake_min <= brake_max");
static_assert(well_formed(kVehicle), "kVehicle malformed");
static_assert(well_formed(kPedestrian), "kPedestrian malformed");
static_assert(well_formed(kUnknown), "kUnknown malformed");
static_assert(kEgo.lon_brake_min_mps2 <= -limits::kMrmEmergencyDecel_mps2,
              "RSS must not assume more ego braking than the Guardian emergency stop applies");
static_assert(kEgo.lon_brake_min_mps2 <= -limits::kRtaRecoveryDecel_mps2,
              "RSS must not assume more ego braking than the RTA Recovery applies");
static_assert(kEgo.lon_brake_max_mps2 <= -limits::kMinLonAccel_mps2,
              "ego brake_max above the strongest braking the validator lets through");
static_assert(kEgo.lon_accel_max_mps2 <= limits::kMaxLonAccel_mps2,
              "ego accel bound looser than the validator envelope");

}  // namespace params
}  // namespace guardian::rss

#endif  // GUARDIAN__RSS__PARAMS_HPP_
