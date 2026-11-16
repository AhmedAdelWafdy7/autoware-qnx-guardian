// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0

#include "guardian/rss/rss.hpp"

#include <cmath>
#include <limits>

namespace guardian::rss
{
namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();

bool finite(double v) noexcept { return std::isfinite(v); }

bool valid_dynamics(const Dynamics & d) noexcept
{
  return finite(d.response_time_s) && d.response_time_s >= 0.0 && finite(d.lon_accel_max_mps2) &&
         d.lon_accel_max_mps2 >= 0.0 && finite(d.lon_brake_min_mps2) &&
         d.lon_brake_min_mps2 > 0.0 && finite(d.lon_brake_min_correct_mps2) &&
         d.lon_brake_min_correct_mps2 > 0.0 && finite(d.lon_brake_max_mps2) &&
         d.lon_brake_max_mps2 > 0.0 && finite(d.lat_accel_max_mps2) &&
         d.lat_accel_max_mps2 >= 0.0 && finite(d.lat_brake_min_mps2) &&
         d.lat_brake_min_mps2 > 0.0 && finite(d.lat_fluctuation_margin_m) &&
         d.lat_fluctuation_margin_m >= 0.0;
}

/// Signed lateral displacement along axis u of a road user that moves towards the other one
/// (direction `toward` = +1 or -1): rho accelerating towards it, then braking with `brake`.
/// The braking phase only counts while it still moves towards the other one: moving away, it
/// may brake laterally as hard as it likes, so the worst case is an immediate stop
/// (same rule as ad-rss-lib calculateLateralDistanceOffsetAfterStatedBrakingPattern).
double lateral_displacement(
  double v_u, double rho, double accel, double brake, double toward) noexcept
{
  const double v_rho = v_u + rho * toward * accel;
  double d = rho * (v_u + v_rho) / 2.0;
  if (v_rho * toward > 0.0) {
    d += toward * v_rho * v_rho / (2.0 * brake);
  }
  return d;
}
}  // namespace

const char * to_string(Outcome o) noexcept
{
  switch (o) {
    case Outcome::kSafe:
      return "SAFE";
    case Outcome::kRestrict:
      return "RESTRICT";
    case Outcome::kUnsafe:
      return "UNSAFE";
  }
  return "INVALID";
}

const Dynamics & dynamics_for(ObjectClass c) noexcept
{
  switch (c) {
    case ObjectClass::kVehicle:
      return params::kVehicle;
    case ObjectClass::kPedestrian:
      return params::kPedestrian;
    case ObjectClass::kUnknown:
      break;
  }
  return params::kUnknown;  // also for out-of-range values: worst case
}

// ------------------------------------------------------------ safe distances ----

double stopping_distance(double v_mps, double rho_s, double accel_mps2, double brake_mps2) noexcept
{
  if (!finite(v_mps) || !finite(rho_s) || !finite(accel_mps2) || !finite(brake_mps2) ||
      v_mps < 0.0 || rho_s < 0.0 || accel_mps2 < 0.0 || brake_mps2 <= 0.0) {
    return kInf;
  }
  const double v_rho = v_mps + rho_s * accel_mps2;
  return v_mps * rho_s + 0.5 * accel_mps2 * rho_s * rho_s + v_rho * v_rho / (2.0 * brake_mps2);
}

double lon_safe_distance_same_direction(
  double v_rear_mps, const Dynamics & rear, double v_front_mps, const Dynamics & front) noexcept
{
  if (!valid_dynamics(rear) || !valid_dynamics(front) || !finite(v_front_mps) ||
      v_front_mps < 0.0) {
    return kInf;
  }
  const double rear_d = stopping_distance(
    v_rear_mps, rear.response_time_s, rear.lon_accel_max_mps2, rear.lon_brake_min_mps2);
  const double front_d = v_front_mps * v_front_mps / (2.0 * front.lon_brake_max_mps2);
  const double d = rear_d - front_d;  // +inf stays +inf
  return d > 0.0 ? d : 0.0;
}

double lon_safe_distance_opposite_direction(
  double v_correct_mps, const Dynamics & correct, double v_other_mps,
  const Dynamics & other) noexcept
{
  if (!valid_dynamics(correct) || !valid_dynamics(other)) {
    return kInf;
  }
  return stopping_distance(
           v_correct_mps, correct.response_time_s, correct.lon_accel_max_mps2,
           correct.lon_brake_min_correct_mps2) +
         stopping_distance(
           v_other_mps, other.response_time_s, other.lon_accel_max_mps2,
           other.lon_brake_min_mps2);
}

double lat_safe_distance(
  double v_left_lat_mps, const Dynamics & left, double v_right_lat_mps,
  const Dynamics & right) noexcept
{
  if (!valid_dynamics(left) || !valid_dynamics(right) || !finite(v_left_lat_mps) ||
      !finite(v_right_lat_mps)) {
    return kInf;
  }
  // Axis u points from the left road user to the right one (u = -y).
  // Worst case: the left one accelerates towards the right (+u), the right one towards the
  // left (-u), each for its own response time, then both brake laterally to zero.
  const double left_u = lateral_displacement(
    -v_left_lat_mps, left.response_time_s, left.lat_accel_max_mps2, left.lat_brake_min_mps2,
    1.0);
  const double right_u = lateral_displacement(
    -v_right_lat_mps, right.response_time_s, right.lat_accel_max_mps2,
    right.lat_brake_min_mps2, -1.0);
  // Paper Def. 6: mu + [closing]+. ad-rss-lib computes [closing + mu]+, which is smaller when
  // both move apart; we keep the paper's form (never less conservative).
  const double mu = 0.5 * (left.lat_fluctuation_margin_m + right.lat_fluctuation_margin_m);
  const double closing = left_u - right_u;
  return mu + (closing > 0.0 ? closing : 0.0);
}

// -------------------------------------------------------------- situations ----

Situation make_situation(const EgoState & ego, const Object & obj) noexcept
{
  Situation s;
  const bool all_finite = finite(ego.speed_mps) && finite(ego.lat_speed_mps) &&
                          finite(ego.front_m) && finite(ego.rear_m) &&
                          finite(ego.half_width_m) && finite(obj.x_m) && finite(obj.y_m) &&
                          finite(obj.yaw_rad) && finite(obj.length_m) && finite(obj.width_m) &&
                          finite(obj.vx_mps) && finite(obj.vy_mps);
  if (!all_finite || ego.front_m < 0.0 || ego.rear_m < 0.0 || ego.half_width_m < 0.0 ||
      obj.length_m < 0.0 || obj.width_m < 0.0 || ego.speed_mps < -params::kSpeedNoise_mps) {
    return s;  // valid = false
  }

  s.cls = obj.cls;
  s.ego_lon_speed_mps = ego.speed_mps > 0.0 ? ego.speed_mps : 0.0;
  s.ego_lat_speed_mps = ego.lat_speed_mps;
  s.obj_lat_speed_mps = obj.vy_mps;

  // Extent of the (rotated) object footprint along the ego axes.
  const double c = std::fabs(std::cos(obj.yaw_rad));
  const double sn = std::fabs(std::sin(obj.yaw_rad));
  const double half_x = 0.5 * (c * obj.length_m + sn * obj.width_m);
  const double half_y = 0.5 * (sn * obj.length_m + c * obj.width_m);

  // Longitudinal: compare with the centre of the ego footprint.
  const double ego_centre_x = 0.5 * (ego.front_m - ego.rear_m);
  if (obj.x_m >= ego_centre_x) {
    s.lon_gap_m = (obj.x_m - half_x) - ego.front_m;
    if (obj.vx_mps >= 0.0) {
      s.lon_case = LonCase::kEgoBehind;
      s.obj_lon_speed_mps = obj.vx_mps;
    } else {
      s.lon_case = LonCase::kOncoming;
      s.obj_lon_speed_mps = -obj.vx_mps;
    }
  } else {
    s.lon_gap_m = -ego.rear_m - (obj.x_m + half_x);
    s.lon_case = LonCase::kEgoInFront;
    // An object behind moving backwards (away) is treated as standing: conservative.
    s.obj_lon_speed_mps = obj.vx_mps > 0.0 ? obj.vx_mps : 0.0;
  }

  // Lateral.
  s.object_left = obj.y_m >= 0.0;
  s.lat_gap_m = std::fabs(obj.y_m) - half_y - ego.half_width_m;

  s.valid = true;
  return s;
}

ObjectResult evaluate_object(const Situation & s) noexcept
{
  ObjectResult r;
  if (!s.valid || !finite(s.lon_gap_m) || !finite(s.lat_gap_m)) {
    return r;  // valid = false
  }
  const Dynamics & ego = params::kEgo;
  const Dynamics & obj = dynamics_for(s.cls);

  switch (s.lon_case) {
    case LonCase::kEgoBehind:
      r.lon_safe_distance_m =
        lon_safe_distance_same_direction(s.ego_lon_speed_mps, ego, s.obj_lon_speed_mps, obj);
      break;
    case LonCase::kOncoming:
      r.lon_safe_distance_m =
        lon_safe_distance_opposite_direction(s.ego_lon_speed_mps, ego, s.obj_lon_speed_mps, obj);
      break;
    case LonCase::kEgoInFront:
      r.lon_safe_distance_m =
        lon_safe_distance_same_direction(s.obj_lon_speed_mps, obj, s.ego_lon_speed_mps, ego);
      break;
    default:
      return r;  // unknown enum value: invalid
  }

  r.lat_safe_distance_m =
    s.object_left ? lat_safe_distance(s.obj_lat_speed_mps, obj, s.ego_lat_speed_mps, ego)
                  : lat_safe_distance(s.ego_lat_speed_mps, ego, s.obj_lat_speed_mps, obj);

  if (!finite(r.lon_safe_distance_m) || !finite(r.lat_safe_distance_m)) {
    return ObjectResult{};  // a formula rejected its input
  }
  r.lon_safe = s.lon_gap_m > r.lon_safe_distance_m;
  r.lat_safe = s.lat_gap_m > r.lat_safe_distance_m;
  r.valid = true;
  return r;
}

Result evaluate(
  const Situation * situations, std::size_t count, double commanded_accel_mps2) noexcept
{
  const Dynamics & ego = params::kEgo;
  Result res;
  res.accel_min_mps2 = -ego.lon_brake_max_mps2;
  res.accel_max_mps2 = ego.lon_accel_max_mps2;

  bool unsafe = false;
  std::int32_t first_unsafe = -1;
  std::int32_t first_restrict = -1;

  if (count > params::kMaxObjects) {
    unsafe = true;
    res.reasons |= reason::kTooManyObjects;
  } else if ((situations == nullptr && count > 0U) || !finite(commanded_accel_mps2)) {
    unsafe = true;
    res.reasons |= reason::kInvalidInput;
  } else {
    for (std::size_t i = 0U; i < count; ++i) {  // bounded: count <= kMaxObjects
      const Situation & s = situations[i];
      const ObjectResult r = evaluate_object(s);
      const auto idx = static_cast<std::int32_t>(i);
      if (!r.valid) {
        unsafe = true;
        res.reasons |= reason::kInvalidInput;
        if (first_unsafe < 0) {
          first_unsafe = idx;
        }
        continue;
      }
      if (!r.dangerous()) {
        continue;
      }
      ++res.num_dangerous;
      if (s.lon_gap_m <= 0.0) {
        unsafe = true;
        res.reasons |= reason::kDangerousAlongside;
      } else if (s.lon_case == LonCase::kEgoBehind) {
        unsafe = true;
        res.reasons |= reason::kDangerousAhead;
      } else if (s.lon_case == LonCase::kOncoming) {
        unsafe = true;
        res.reasons |= reason::kDangerousOncoming;
      } else {
        // Ego in front: it must not brake harder than brake_max. The default range already
        // enforces that; reported so it is never silent.
        res.reasons |= reason::kDangerousBehind;
        if (first_restrict < 0) {
          first_restrict = idx;
        }
        continue;
      }
      if (first_unsafe < 0) {
        first_unsafe = idx;
      }
    }
  }

  if (unsafe) {
    res.accel_max_mps2 = -ego.lon_brake_min_mps2;  // proper response: brake at least brake_min
  }

  if (!finite(commanded_accel_mps2)) {
    res.accel_mps2 = res.accel_max_mps2;
  } else if (commanded_accel_mps2 > res.accel_max_mps2) {
    res.accel_mps2 = res.accel_max_mps2;
    res.reasons |= reason::kAccelClamped;
  } else if (commanded_accel_mps2 < res.accel_min_mps2) {
    res.accel_mps2 = res.accel_min_mps2;
    res.reasons |= reason::kBrakeClamped;
  } else {
    res.accel_mps2 = commanded_accel_mps2;
  }

  if (unsafe) {
    res.outcome = Outcome::kUnsafe;
    res.worst_index = first_unsafe;
  } else if ((res.reasons & (reason::kDangerousBehind | reason::kAccelClamped |
                              reason::kBrakeClamped)) != 0U) {
    res.outcome = Outcome::kRestrict;
    res.worst_index = first_restrict;
  } else {
    res.outcome = Outcome::kSafe;
  }
  return res;
}

}  // namespace guardian::rss
