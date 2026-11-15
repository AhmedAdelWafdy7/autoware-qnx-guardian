// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0
//
// Oracle test: the Guardian's RSS safe distances against intel/ad-rss-lib v5.0.0
// (ad::rss::structured::calculateSafe*Distance*), on the Guardian's own parameter sets and on
// randomly generated situations (fixed seed, so every run checks the same cases).
//
// Conventions that differ between the two and are converted here:
//   - ad-rss-lib stores braking as negative accelerations; the Guardian stores magnitudes.
//   - ad-rss-lib's lateral axis points to the right; the Guardian's (base_link y) to the left.
//
// Expected result:
//   - Longitudinal, same and opposite direction: equal within kTol.
//   - Lateral: equal within kTol when the two road users close in. When they move apart,
//     ad-rss-lib gives max(closing + mu, 0) and the Guardian gives mu (paper Def. 6). The
//     Guardian is then larger, i.e. more conservative, and the test checks exactly that.

#include <gtest/gtest.h>

#include <ad/rss/core/RelativeObjectState.hpp>
#include <ad/rss/core/RelativeObjectStateValidInputRange.hpp>
#include <ad/rss/structured/RssFormulas.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

#include "guardian/rss/params.hpp"
#include "guardian/rss/rss.hpp"

namespace grss = guardian::rss;
namespace P = guardian::rss::params;
namespace phys = ad::physics;
using ad::rss::core::RelativeObjectState;

namespace
{
/// Absolute tolerance [m]. Both sides use double; differences come only from operation order.
constexpr double kTol = 1e-6;
/// ad-rss-lib caps speed after accelerating at max_speed_on_acceleration; the Guardian does
/// not. Generated speeds stay well below this so the cap never applies.
constexpr double kMaxSpeedOnAccel_mps = 100.0;
constexpr int kRandomCases = 20000;

/// Build an ad-rss-lib object state that passes withinValidInputRange().
/// lat_speed is in the Guardian convention (positive = left).
RelativeObjectState make_state(const grss::Dynamics & d, double lon_speed, double lat_speed)
{
  RelativeObjectState s;
  s.object_type = ad::rss::world::ObjectType::OtherVehicle;

  auto & dyn = s.dynamics;
  dyn.alpha_lon.accel_max = phys::Acceleration(d.lon_accel_max_mps2);
  dyn.alpha_lon.brake_min = phys::Acceleration(-d.lon_brake_min_mps2);
  dyn.alpha_lon.brake_min_correct = phys::Acceleration(-d.lon_brake_min_correct_mps2);
  dyn.alpha_lon.brake_max = phys::Acceleration(-d.lon_brake_max_mps2);
  dyn.alpha_lat.accel_max = phys::Acceleration(d.lat_accel_max_mps2);
  dyn.alpha_lat.brake_min = phys::Acceleration(-d.lat_brake_min_mps2);
  dyn.lateral_fluctuation_margin = phys::Distance(d.lat_fluctuation_margin_m);
  dyn.response_time = phys::Duration(d.response_time_s);
  dyn.max_speed_on_acceleration = phys::Speed(kMaxSpeedOnAccel_mps);
  dyn.min_longitudinal_safety_distance = phys::Distance(0.0);
  // Unstructured settings are not used by the structured formulas, but must be in range.
  // Values from ad-rss-lib's own test support (getObjectRssDynamics).
  auto & u = dyn.unstructured_settings;
  u.pedestrian_turning_radius = phys::Distance(2.0);
  u.drive_away_max_angle = phys::Angle(2.4);
  u.vehicle_yaw_rate_change = phys::AngularAcceleration(0.3);
  u.vehicle_min_radius = phys::Distance(3.5);
  u.vehicle_trajectory_calculation_step = phys::Duration(0.2);
  u.vehicle_front_intermediate_yaw_rate_change_ratio_steps = 3;
  u.vehicle_back_intermediate_yaw_rate_change_ratio_steps = 0;
  u.vehicle_brake_intermediate_acceleration_steps = 1;
  u.vehicle_continue_forward_intermediate_acceleration_steps = 0;
  u.vehicle_continue_forward_intermediate_yaw_rate_change_ratio_steps = 3;
  u.pedestrian_continue_forward_intermediate_heading_change_ratio_steps = 3;
  u.pedestrian_continue_forward_intermediate_acceleration_steps = 0;
  u.pedestrian_brake_intermediate_acceleration_steps = 3;
  u.pedestrian_front_intermediate_heading_change_ratio_steps = 4;
  u.pedestrian_back_intermediate_heading_change_ratio_steps = 0;

  auto & v = s.structured_object_state.velocity;
  v.speed_lon_min = phys::Speed(lon_speed);
  v.speed_lon_max = phys::Speed(lon_speed);
  v.speed_lat_min = phys::Speed(-lat_speed);  // ad-rss-lib: positive = right
  v.speed_lat_max = phys::Speed(-lat_speed);
  s.structured_object_state.distance_to_enter_intersection = phys::Distance(0.0);
  s.structured_object_state.distance_to_leave_intersection = phys::Distance(1000.0);
  s.structured_object_state.has_priority = false;
  s.structured_object_state.is_in_correct_lane = true;

  auto & o = s.unstructured_object_state;
  o.yaw = phys::Angle(0.0);
  o.steering_angle = phys::Angle(0.0);
  o.dimension.length = phys::Distance(4.0);
  o.dimension.width = phys::Distance(2.0);
  o.yaw_rate = phys::AngularVelocity(0.0);
  o.center_point.x = phys::Distance(0.0);
  o.center_point.y = phys::Distance(0.0);
  const phys::Speed speed(std::hypot(lon_speed, lat_speed));
  o.speed_range.minimum = speed;
  o.speed_range.maximum = speed;
  return s;
}

struct Pair
{
  grss::Dynamics a;
  double a_lon;
  double a_lat;
  grss::Dynamics b;
  double b_lon;
  double b_lat;
};

/// Run all three formulas on one pair and compare. `a` is rear / correct-lane / left,
/// `b` is front / oncoming / right. Returns the number of failed comparisons.
int compare(const Pair & p, const char * label)
{
  const RelativeObjectState a = make_state(p.a, p.a_lon, p.a_lat);
  const RelativeObjectState b = make_state(p.b, p.b_lon, p.b_lat);
  EXPECT_TRUE(withinValidInputRange(a, false)) << label;
  EXPECT_TRUE(withinValidInputRange(b, false)) << label;
  int failures = 0;

  // Longitudinal, same direction: b leads, a follows.
  {
    phys::Distance oracle(0.0);
    const bool ok =
      ad::rss::structured::calculateSafeLongitudinalDistanceSameDirection(b, a, oracle);
    const double ours = grss::lon_safe_distance_same_direction(p.a_lon, p.a, p.b_lon, p.b);
    EXPECT_TRUE(ok) << label;
    const double diff = std::fabs(ours - static_cast<double>(oracle));
    EXPECT_LE(diff, kTol) << label << " lon same: ours=" << ours
                          << " ad-rss=" << static_cast<double>(oracle);
    failures += (ok && diff <= kTol) ? 0 : 1;
  }
  // Longitudinal, opposite direction: a drives in its correct lane, b is oncoming.
  {
    phys::Distance oracle(0.0);
    const bool ok =
      ad::rss::structured::calculateSafeLongitudinalDistanceOppositeDirection(a, b, oracle);
    const double ours = grss::lon_safe_distance_opposite_direction(p.a_lon, p.a, p.b_lon, p.b);
    EXPECT_TRUE(ok) << label;
    const double diff = std::fabs(ours - static_cast<double>(oracle));
    EXPECT_LE(diff, kTol) << label << " lon opposite: ours=" << ours
                          << " ad-rss=" << static_cast<double>(oracle);
    failures += (ok && diff <= kTol) ? 0 : 1;
  }
  // Lateral: a is on the left, b on the right.
  {
    phys::Distance oracle(0.0);
    const bool ok = ad::rss::structured::calculateSafeLateralDistance(a, b, oracle);
    const double ours = grss::lat_safe_distance(p.a_lat, p.a, p.b_lat, p.b);
    const double theirs = static_cast<double>(oracle);
    const double mu = 0.5 * (p.a.lat_fluctuation_margin_m + p.b.lat_fluctuation_margin_m);
    EXPECT_TRUE(ok) << label;
    bool pass = false;
    if (theirs >= mu - kTol) {
      // Closing in (or not moving apart): same formula.
      pass = std::fabs(ours - theirs) <= kTol;
    } else {
      // Moving apart: ad-rss-lib subtracts the separation from mu, the Guardian keeps mu.
      pass = std::fabs(ours - mu) <= kTol && ours >= theirs;
    }
    EXPECT_TRUE(pass) << label << " lat: ours=" << ours << " ad-rss=" << theirs
                      << " mu=" << mu;
    failures += (ok && pass) ? 0 : 1;
  }
  return failures;
}

const grss::Dynamics * const kParamSets[] = {&P::kEgo, &P::kVehicle, &P::kPedestrian, &P::kUnknown};
const char * const kParamNames[] = {"ego", "vehicle", "pedestrian", "unknown"};
}  // namespace

TEST(RssOracle, GuardianParameterSets)
{
  const double lon_speeds[] = {0.0, 1.0, 5.0, 13.9, 27.5};
  const double lat_speeds[] = {-1.0, -0.2, 0.0, 0.2, 1.0};
  int cases = 0;
  int failures = 0;
  for (std::size_t i = 0; i < 4; ++i) {
    for (std::size_t j = 0; j < 4; ++j) {
      for (double va : lon_speeds) {
        for (double vb : lon_speeds) {
          for (double la : lat_speeds) {
            for (double lb : lat_speeds) {
              char label[96];
              std::snprintf(label, sizeof(label), "%s(%.1f,%.1f) vs %s(%.1f,%.1f)",
                            kParamNames[i], va, la, kParamNames[j], vb, lb);
              failures += compare({*kParamSets[i], va, la, *kParamSets[j], vb, lb}, label);
              ++cases;
            }
          }
        }
      }
    }
  }
  std::printf("parameter sets: %d cases x 3 formulas, %d mismatches\n", cases, failures);
  EXPECT_EQ(failures, 0);
}

TEST(RssOracle, RandomSituations)
{
  std::mt19937_64 rng(20261109ULL);  // fixed seed: reproducible
  auto uni = [&rng](double lo, double hi) {
    return std::uniform_real_distribution<double>(lo, hi)(rng);
  };
  auto dynamics = [&uni]() {
    grss::Dynamics d{};
    d.response_time_s = uni(0.05, 3.0);
    d.lon_accel_max_mps2 = uni(0.0, 5.0);
    d.lon_brake_min_correct_mps2 = uni(0.5, 6.0);
    d.lon_brake_min_mps2 = d.lon_brake_min_correct_mps2 + uni(0.0, 3.0);
    d.lon_brake_max_mps2 = d.lon_brake_min_mps2 + uni(0.0, 4.0);
    d.lat_accel_max_mps2 = uni(0.0, 3.0);
    d.lat_brake_min_mps2 = uni(0.1, 3.0);
    d.lat_fluctuation_margin_m = uni(0.0, 1.0);
    return d;
  };
  int failures = 0;
  for (int k = 0; k < kRandomCases; ++k) {
    // Speeds stay below kMaxSpeedOnAccel_mps even after rho at full acceleration (40 + 3*5).
    const Pair p{dynamics(), uni(0.0, 40.0), uni(-3.0, 3.0),
                 dynamics(), uni(0.0, 40.0), uni(-3.0, 3.0)};
    char label[32];
    std::snprintf(label, sizeof(label), "random #%d", k);
    failures += compare(p, label);
    if (failures > 20) {
      FAIL() << "too many mismatches, stopping";
    }
  }
  std::printf("random: %d cases x 3 formulas, %d mismatches (tolerance %.0e m)\n", kRandomCases,
              failures, kTol);
  EXPECT_EQ(failures, 0);
}
