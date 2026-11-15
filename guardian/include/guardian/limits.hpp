// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0
//
// Every safety limit used by the Guardian RT core, in one place.
// Units are in the name suffix (_rad, _mps, _mps2, _ns, ...). Each value states its source.
//
// Design rule: the Guardian is a *backstop*. Its envelope is set at or slightly outside the
// limits of Autoware's own autoware_vehicle_cmd_gate (nominal profile, Autoware 1.9.0,
// config/vehicle_cmd_gate.param.yaml), so in normal operation the gate filters first and the
// Guardian never fights it. A Guardian CLAMP therefore means the upstream gate failed or was
// bypassed — which is exactly what we want to log.
//
// Vehicle values are the AWSIM sample vehicle. TODO(verify): confirm against the
// vehicle_info.param.yaml actually launched on the Linux side.

#ifndef GUARDIAN__LIMITS_HPP_
#define GUARDIAN__LIMITS_HPP_

#include <cstdint>

namespace guardian::limits
{

// ---------------------------------------------------------------- timing ----
/// Guardian cycle period. Target loop of 10 ms (100 Hz).
inline constexpr std::int64_t kCyclePeriod_ns = 10'000'000;

/// Heartbeat timeout for the Autoware doer (Linux) and the ROS 2 safety layer (QNX).
/// Default 100 ms = 10 Guardian cycles; Autoware publishes control at 30-50 Hz, so this
/// tolerates ~3-5 lost messages before acting.
inline constexpr std::int64_t kHeartbeatTimeout_ns = 100'000'000;

/// Maximum age of the latest safety-layer command (receive time, CLOCK_MONOTONIC on QNX).
/// Same as the heartbeat timeout: a live node publishing stale commands is a fault.
inline constexpr std::int64_t kCommandMaxAge_ns = 100'000'000;

/// Maximum age of the vehicle status (speed, steering) used by the validator and the
/// standstill check. Without fresh speed we cannot apply speed-dependent limits.
inline constexpr std::int64_t kVehicleStatusMaxAge_ns = 100'000'000;

/// Allowed jump in heartbeat sequence numbers before it is counted as an anomaly.
/// A few lost DDS samples are normal; a larger jump suggests a restarted publisher.
inline constexpr std::uint32_t kMaxHeartbeatSeqJump = 10U;

// --------------------------------------------------------------- vehicle ----
/// Wheelbase [m]. Used by the lateral-acceleration bound. 2.79 m is one of the vehicle_info
/// wheel_base values in autoware_universe 0.52.0 (others: 2.74 m).
/// TODO(verify): take the exact value from the AWSIM vehicle description used in the demo.
inline constexpr double kWheelbase_m = 2.79;

/// Physical steering (tire) angle limit [rad]. vehicle_info max_steer_angle = 0.70 rad.
/// The gate's steer_cmd_lim is 1.0 rad, so the vehicle limit is the binding one.
inline constexpr double kMaxSteerAngle_rad = 0.70;

// ------------------------------------------------------- command envelope ----
/// Steering rate limit [rad/s]. Gate nominal steer_rate_lim = 1.0 rad/s; +20 % margin.
inline constexpr double kMaxSteerRate_radps = 1.2;

/// Lateral acceleration limit [m/s^2] for a_lat = v^2 * tan(delta) / L.
/// Gate nominal lat_acc_lim = 5.0 m/s^2; +10 % margin.
inline constexpr double kMaxLatAccel_mps2 = 5.5;

/// Longitudinal acceleration limits [m/s^2]. Gate nominal lon_acc_lim = 5.0; +10 %.
inline constexpr double kMaxLonAccel_mps2 = 5.5;
/// Strongest braking command accepted [m/s^2]. ~0.8 g, above any Autoware MRM/AEB request
/// at 1.9.0 (emergency stop -2.5, gate emergency -2.4); harder requests are saturated.
inline constexpr double kMinLonAccel_mps2 = -8.0;

/// Speed command limit [m/s]. Gate nominal vel_lim = 25 m/s; +10 %.
inline constexpr double kMaxSpeed_mps = 27.5;

/// A single-cycle steering step larger than this is physically impossible for the actuator
/// (0.35 rad in 10 ms = 35 rad/s, ~30x the rate limit) and is treated as corrupted input:
/// REJECT, not CLAMP (demo fault "steering spike").
inline constexpr double kSteerSpikeReject_rad = 0.35;

/// Consecutive REJECTs that escalate to Guardian MRM: 5 cycles = 50 ms.
inline constexpr std::uint32_t kMaxConsecutiveRejects = 5U;

// -------------------------------------------------------------- MRM ---------
/// Guardian built-in comfortable stop deceleration [m/s^2]. Matches Autoware
/// mrm_comfortable_stop_operator min_acceleration (-1.0) so both layers stop alike.
inline constexpr double kMrmComfortableDecel_mps2 = -1.0;

/// Guardian built-in emergency stop deceleration [m/s^2]. Matches Autoware
/// mrm_emergency_stop_operator target_acceleration (-2.5).
inline constexpr double kMrmEmergencyDecel_mps2 = -2.5;

/// Jerk used to ramp into the MRM deceleration [m/s^3]. Matches
/// mrm_emergency_stop_operator target_jerk (-1.5). Used for both MRM levels so the Guardian
/// reaches its decel in < 2 s even for comfortable stop (the ROS 2 layer is gone; a slow
/// 0.3 m/s^3 ramp would add metres of stopping distance).
inline constexpr double kMrmJerk_mps3 = 1.5;

/// Brake-hold acceleration once STOPPED [m/s^2]. Gate stop_hold_acceleration = -1.5.
inline constexpr double kStopHoldAccel_mps2 = -1.5;

/// Speed below which the vehicle counts as standing still [m/s].
inline constexpr double kStoppedSpeed_mps = 0.05;

/// Time the vehicle must stay below kStoppedSpeed_mps before MRM -> STOPPED.
/// Gate stop_check_duration = 1.0 s.
inline constexpr std::int64_t kStoppedHold_ns = 1'000'000'000;

// ------------------------------------------------- RSS / RTA Switch -------
/// Maximum age of the independent object channel (AWSIM -> QNX, ego odometry + objects).
/// Same budget as the command and heartbeat (100 ms): RSS on older data is not trustworthy.
/// Stale -> RSS UNSAFE for that cycle and MRM_EMERGENCY (the Guardian has lost its own view).
/// TODO(verify): AWSIM object publish rate (expected 10-30 Hz) fits inside this budget.
inline constexpr std::int64_t kIndependentMaxAge_ns = 100'000'000;

/// RTA Switch hysteresis: control returns to the Complex Function only after RSS has not been
/// UNSAFE for this long (Eberhart et al., IEEE IV 2023: "robustly satisfied"). 1 s = 2x the ego
/// response time (rss::params::kEgo, 0.5 s) and 100 Guardian cycles, so a flickering object
/// cannot toggle the switch at cycle rate. TODO(verify): tune in the pedestrian scenarios.
inline constexpr std::int64_t kRtaHysteresis_ns = 1'000'000'000;

/// Deceleration the Recovery Function commands while the RTA Switch is in Recovery [m/s^2].
/// Applied as a step, no jerk ramp: RSS assumes the ego brakes with at least
/// kEgo.lon_brake_min within its response time (static_assert in rss/params.hpp).
/// Same value as the Guardian emergency stop and Autoware mrm_emergency_stop_operator (-2.5).
inline constexpr double kRtaRecoveryDecel_mps2 = kMrmEmergencyDecel_mps2;

}  // namespace guardian::limits

#endif  // GUARDIAN__LIMITS_HPP_
