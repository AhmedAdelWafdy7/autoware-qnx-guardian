// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0
//
// Minimal RSS (Responsibility-Sensitive Safety) core for the Guardian Safety Monitor.
// Dependency-free, allocation-free, noexcept; loops bounded by params::kMaxObjects.
// Evaluated on the independent object channel (AWSIM -> QNX), never only on Autoware's perception.
//
// Pipeline per cycle:
//   1. make_situation(): one object in the ego frame -> projected RSS situation
//      (straight-road approximation: the ego heading is the lane direction).
//   2. evaluate_object(): longitudinal + lateral safe distances. Dangerous = both unsafe.
//   3. evaluate(): all objects + the Doer's commanded acceleration -> Outcome + bounds.
//
// Outcome rule:
//   UNSAFE   invalid input, or a dangerous object for which the ego must brake (object ahead,
//            oncoming, or overlapping alongside). Accel upper bound = -ego brake_min. The RTA
//            Switch hands over to the Recovery Function.
//   RESTRICT no ego-braking case, but the Doer's command was clamped to the ego's RSS bounds
//            (more acceleration than the response-time assumption allows, or braking harder
//            than brake_max), or a dangerous object is behind (the ego must not brake harder
//            than brake_max; reported, the bounds already enforce it).
//   SAFE     otherwise.
//
// Deliberate simplifications vs ad-rss-lib (all on the conservative side):
//   - No blame time: every dangerous object ahead requires braking, whichever distance became
//     unsafe last. ad-rss-lib may only require a lateral response in a cut-in.
//   - No lateral response is commanded (the Guardian does not steer). Lateral danger alongside
//     is handled by stopping.
//   - The ego is assumed to drive in its correct lane.
// The oracle tests therefore compare safe distances, not proper responses.

#ifndef GUARDIAN__RSS__RSS_HPP_
#define GUARDIAN__RSS__RSS_HPP_

#include <cstddef>
#include <cstdint>

#include "guardian/rss/params.hpp"

namespace guardian::rss
{

/// RSS outcome per cycle. RESTRICT and UNSAFE are always logged by the caller.
enum class Outcome : std::uint8_t
{
  kSafe = 0,
  kRestrict = 1,
  kUnsafe = 2,
};

const char * to_string(Outcome o) noexcept;

enum class ObjectClass : std::uint8_t
{
  kVehicle = 0,
  kPedestrian = 1,
  kUnknown = 2,
};

/// Dynamics used for an object class.
const Dynamics & dynamics_for(ObjectClass c) noexcept;

/// Bit flags explaining a RESTRICT or UNSAFE.
namespace reason
{
inline constexpr std::uint32_t kNone = 0U;
inline constexpr std::uint32_t kInvalidInput = 1U << 0U;     ///< NaN/Inf, reversing ego, bad sizes
inline constexpr std::uint32_t kTooManyObjects = 1U << 1U;   ///< count > params::kMaxObjects
inline constexpr std::uint32_t kDangerousAhead = 1U << 2U;   ///< ego must brake (same direction)
inline constexpr std::uint32_t kDangerousOncoming = 1U << 3U;
inline constexpr std::uint32_t kDangerousAlongside = 1U << 4U;  ///< longitudinal overlap
inline constexpr std::uint32_t kDangerousBehind = 1U << 5U;  ///< ego must not brake > brake_max
inline constexpr std::uint32_t kAccelClamped = 1U << 6U;     ///< command above an RSS bound
inline constexpr std::uint32_t kBrakeClamped = 1U << 7U;     ///< command below an RSS bound
/// Set by the caller (Guardian), not by evaluate(): the independent channel is missing or
/// stale, so RSS cannot be evaluated this cycle and is treated as UNSAFE.
inline constexpr std::uint32_t kNoIndependentData = 1U << 8U;
}  // namespace reason

// ------------------------------------------------------------ safe distances ----
// Pure formulas, exposed for the oracle tests. Speeds are magnitudes >= 0 unless noted.
// Invalid input (non-finite, negative speed or parameter) returns +infinity: never safe.

/// Distance a road user covers: rho at full acceleration, then braking with `brake` to stop.
double stopping_distance(double v_mps, double rho_s, double accel_mps2, double brake_mps2) noexcept;

/// Longitudinal safe distance, both driving in the same direction (paper Def. 1).
double lon_safe_distance_same_direction(
  double v_rear_mps, const Dynamics & rear, double v_front_mps, const Dynamics & front) noexcept;

/// Longitudinal safe distance, driving towards each other (paper Def. 4). `correct` drives in its
/// correct lane and brakes with lon_brake_min_correct; `other` brakes with lon_brake_min.
double lon_safe_distance_opposite_direction(
  double v_correct_mps, const Dynamics & correct, double v_other_mps,
  const Dynamics & other) noexcept;

/// Lateral safe distance (paper Def. 6). Lateral speeds are signed, positive = to the left.
/// `left` is the road user on the left. mu = mean of both fluctuation margins.
double lat_safe_distance(
  double v_left_lat_mps, const Dynamics & left, double v_right_lat_mps,
  const Dynamics & right) noexcept;

// -------------------------------------------------------------- situations ----

/// Ego state and footprint. The ego frame is base_link: x forward, y left.
struct EgoState
{
  double speed_mps{0.0};      ///< longitudinal speed (odometry)
  double lat_speed_mps{0.0};  ///< lateral speed, positive left (odometry, usually ~0)
  double front_m{0.0};        ///< base_link -> front bumper
  double rear_m{0.0};         ///< base_link -> rear bumper
  double half_width_m{0.0};
};

/// One object from the independent channel, in the ego frame.
struct Object
{
  ObjectClass cls{ObjectClass::kUnknown};
  double x_m{0.0};  ///< footprint centre
  double y_m{0.0};
  double yaw_rad{0.0};  ///< relative to the ego heading
  double length_m{0.0};
  double width_m{0.0};
  double vx_mps{0.0};  ///< ground velocity, ego-frame axes
  double vy_mps{0.0};
};

/// Longitudinal relation of one object to the ego.
enum class LonCase : std::uint8_t
{
  kEgoBehind = 0,   ///< object ahead, same direction: ego is the rear vehicle
  kOncoming = 1,    ///< object ahead, moving towards the ego
  kEgoInFront = 2,  ///< object behind the ego
};

/// Projected RSS situation for one object.
struct Situation
{
  bool valid{false};
  ObjectClass cls{ObjectClass::kUnknown};
  LonCase lon_case{LonCase::kEgoBehind};
  double lon_gap_m{0.0};          ///< bumper to bumper; <= 0 means longitudinal overlap
  double ego_lon_speed_mps{0.0};  ///< >= 0
  double obj_lon_speed_mps{0.0};  ///< >= 0, along its own driving direction for this case
  bool object_left{false};
  double lat_gap_m{0.0};          ///< side to side; <= 0 means lateral overlap
  double ego_lat_speed_mps{0.0};  ///< positive left
  double obj_lat_speed_mps{0.0};  ///< positive left
};

/// Project an ego-frame object into an RSS situation. Returns valid = false on bad input.
Situation make_situation(const EgoState & ego, const Object & obj) noexcept;

/// Per-object result.
struct ObjectResult
{
  bool valid{false};
  bool lon_safe{false};
  bool lat_safe{false};
  double lon_safe_distance_m{0.0};
  double lat_safe_distance_m{0.0};
  bool dangerous() const noexcept { return !valid || (!lon_safe && !lat_safe); }
};

ObjectResult evaluate_object(const Situation & s) noexcept;

/// Result for the whole world in one cycle.
struct Result
{
  Outcome outcome{Outcome::kUnsafe};
  std::uint32_t reasons{reason::kNone};
  double accel_min_mps2{0.0};  ///< allowed ego acceleration range (proper response)
  double accel_max_mps2{0.0};
  double accel_mps2{0.0};      ///< commanded acceleration clamped into the range
  std::int32_t worst_index{-1};    ///< first object that caused UNSAFE / RESTRICT, -1 if none
  std::uint32_t num_dangerous{0U};
};

/// Evaluate `count` situations against the Doer's commanded acceleration.
/// `situations` may be null only when count == 0.
Result evaluate(
  const Situation * situations, std::size_t count, double commanded_accel_mps2) noexcept;

}  // namespace guardian::rss

#endif  // GUARDIAN__RSS__RSS_HPP_
