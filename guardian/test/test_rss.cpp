// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0
//
// RSS core: safe-distance formulas (hand-computed values), situation projection, and the
// SAFE / RESTRICT / UNSAFE outcome per cycle.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include "guardian/rss/params.hpp"
#include "guardian/rss/rss.hpp"

namespace rss = guardian::rss;
namespace P = guardian::rss::params;
namespace R = guardian::rss::reason;
using rss::LonCase;
using rss::ObjectClass;
using rss::Outcome;

namespace
{
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kTol = 1e-9;

// Ego footprint used in the scenarios: base_link at the rear axle.
rss::EgoState ego(double speed = 10.0)
{
  rss::EgoState e;
  e.speed_mps = speed;
  e.front_m = 3.8;
  e.rear_m = 1.0;
  e.half_width_m = 0.9;
  return e;
}

rss::Object vehicle(double x, double y, double vx, double vy = 0.0)
{
  rss::Object o;
  o.cls = ObjectClass::kVehicle;
  o.x_m = x;
  o.y_m = y;
  o.length_m = 4.0;
  o.width_m = 1.8;
  o.vx_mps = vx;
  o.vy_mps = vy;
  return o;
}

rss::Object pedestrian(double x, double y, double vx, double vy)
{
  rss::Object o = vehicle(x, y, vx, vy);
  o.cls = ObjectClass::kPedestrian;
  o.length_m = 0.5;
  o.width_m = 0.5;
  return o;
}

rss::Result run(const rss::EgoState & e, const rss::Object & o, double accel = 0.0)
{
  const rss::Situation s = rss::make_situation(e, o);
  return rss::evaluate(&s, 1U, accel);
}
}  // namespace

// ------------------------------------------------------------ safe distances ----

TEST(RssDistance, StoppingDistance)
{
  // v = 10, rho = 0.5, a = 2, b = 2.5: 10*0.5 + 0.5*2*0.25 + 11^2/5 = 5 + 0.25 + 24.2
  EXPECT_NEAR(rss::stopping_distance(10.0, 0.5, 2.0, 2.5), 29.45, kTol);
  // Standing, no response time: zero.
  EXPECT_NEAR(rss::stopping_distance(0.0, 0.0, 2.0, 2.5), 0.0, kTol);
}

TEST(RssDistance, StoppingDistanceRejectsBadInput)
{
  EXPECT_EQ(rss::stopping_distance(-0.1, 0.5, 2.0, 2.5), kInf);
  EXPECT_EQ(rss::stopping_distance(10.0, -0.5, 2.0, 2.5), kInf);
  EXPECT_EQ(rss::stopping_distance(10.0, 0.5, -2.0, 2.5), kInf);
  EXPECT_EQ(rss::stopping_distance(10.0, 0.5, 2.0, 0.0), kInf);
  EXPECT_EQ(rss::stopping_distance(kNan, 0.5, 2.0, 2.5), kInf);
  EXPECT_EQ(rss::stopping_distance(10.0, 0.5, 2.0, kInf), kInf);
}

TEST(RssDistance, SameDirectionEqualSpeeds)
{
  // Ego (rear) 10 m/s, vehicle (front) 10 m/s: 29.45 - 10^2 / (2*8) = 29.45 - 6.25
  EXPECT_NEAR(
    rss::lon_safe_distance_same_direction(10.0, P::kEgo, 10.0, P::kVehicle), 23.2, kTol);
}

TEST(RssDistance, SameDirectionZeroSpeeds)
{
  // Both standing: only the ego's response-time creep, 0.5*2*0.25 + 1^2/5 = 0.45
  EXPECT_NEAR(rss::lon_safe_distance_same_direction(0.0, P::kEgo, 0.0, P::kVehicle), 0.45, kTol);
}

TEST(RssDistance, SameDirectionFasterFrontIsZero)
{
  EXPECT_EQ(rss::lon_safe_distance_same_direction(0.0, P::kEgo, 20.0, P::kVehicle), 0.0);
}

TEST(RssDistance, SameDirectionNegativeSpeedNeverSafe)
{
  EXPECT_EQ(rss::lon_safe_distance_same_direction(-1.0, P::kEgo, 10.0, P::kVehicle), kInf);
  EXPECT_EQ(rss::lon_safe_distance_same_direction(10.0, P::kEgo, -1.0, P::kVehicle), kInf);
  EXPECT_EQ(rss::lon_safe_distance_same_direction(kNan, P::kEgo, 10.0, P::kVehicle), kInf);
}

TEST(RssDistance, SameDirectionGrowsWithRearSpeed)
{
  double prev = rss::lon_safe_distance_same_direction(0.0, P::kEgo, 5.0, P::kVehicle);
  for (int i = 1; i <= 30; ++i) {
    const double d =
      rss::lon_safe_distance_same_direction(static_cast<double>(i), P::kEgo, 5.0, P::kVehicle);
    EXPECT_GE(d, prev);
    prev = d;
  }
}

TEST(RssDistance, OppositeDirection)
{
  // Ego in its correct lane: 29.45 (brake_min_correct 2.5).
  // Vehicle: rho 2, a 3.5, b 4: 10*2 + 0.5*3.5*4 + 17^2/8 = 20 + 7 + 36.125 = 63.125
  EXPECT_NEAR(
    rss::lon_safe_distance_opposite_direction(10.0, P::kEgo, 10.0, P::kVehicle), 92.575, kTol);
}

TEST(RssDistance, OppositeDirectionNegativeSpeedNeverSafe)
{
  EXPECT_EQ(rss::lon_safe_distance_opposite_direction(10.0, P::kEgo, -1.0, P::kVehicle), kInf);
}

TEST(RssDistance, LateralBothStill)
{
  // Ego on the left, vehicle on the right, no lateral speed.
  // Ego: v_rho = 0.1, 0.5*0.1/2 + 0.01/1.6 = 0.03125 towards the right.
  // Vehicle: v_rho = -0.4, 2*(-0.4)/2 - 0.16/1.6 = -0.5 (towards the left).
  // mu 0.1 + 0.53125
  EXPECT_NEAR(rss::lat_safe_distance(0.0, P::kEgo, 0.0, P::kVehicle), 0.63125, kTol);
}

TEST(RssDistance, LateralMovingApartIsOnlyMargin)
{
  // Left vehicle moving left at 2 m/s: it cannot close the gap within its response time.
  EXPECT_NEAR(rss::lat_safe_distance(2.0, P::kVehicle, 0.0, P::kEgo), 0.1, kTol);
}

TEST(RssDistance, LateralNoBrakingPhaseWhenMovingAway)
{
  // Pedestrian on the left walking right at 2 m/s: v_rho = 3, 0.5*5/2 + 9/4 = 3.5.
  // Vehicle on the right moving right at 1 m/s: v_rho = 0.6 (still away), 2*1.6/2 = 1.6 and no
  // braking phase (it may stop at once). mu 0.1 + (3.5 - 1.6)
  EXPECT_NEAR(rss::lat_safe_distance(-2.0, P::kPedestrian, -1.0, P::kVehicle), 2.0, kTol);
}

TEST(RssDistance, LateralMirrorSymmetric)
{
  // Mirroring the scene (swap sides, negate lateral speeds) gives the same distance.
  for (double v1 : {-1.5, -0.3, 0.0, 0.4, 1.2}) {
    for (double v2 : {-1.0, 0.0, 0.7}) {
      EXPECT_NEAR(
        rss::lat_safe_distance(v1, P::kEgo, v2, P::kPedestrian),
        rss::lat_safe_distance(-v2, P::kPedestrian, -v1, P::kEgo), kTol);
    }
  }
}

TEST(RssDistance, LateralNonFiniteNeverSafe)
{
  EXPECT_EQ(rss::lat_safe_distance(kNan, P::kEgo, 0.0, P::kVehicle), kInf);
}

TEST(RssDistance, MalformedDynamicsNeverSafe)
{
  rss::Dynamics bad = P::kVehicle;
  bad.lon_brake_max_mps2 = 0.0;
  EXPECT_EQ(rss::lon_safe_distance_same_direction(10.0, P::kEgo, 10.0, bad), kInf);
  bad = P::kVehicle;
  bad.lat_brake_min_mps2 = kNan;
  EXPECT_EQ(rss::lat_safe_distance(0.0, P::kEgo, 0.0, bad), kInf);
}

// -------------------------------------------------------------- situations ----

TEST(RssSituation, ObjectAheadSameDirection)
{
  const rss::Situation s = rss::make_situation(ego(), vehicle(30.0, 0.0, 10.0));
  ASSERT_TRUE(s.valid);
  EXPECT_EQ(s.lon_case, LonCase::kEgoBehind);
  EXPECT_NEAR(s.lon_gap_m, 30.0 - 2.0 - 3.8, kTol);
  EXPECT_NEAR(s.lat_gap_m, 0.0 - 0.9 - 0.9, kTol);
  EXPECT_DOUBLE_EQ(s.obj_lon_speed_mps, 10.0);
}

TEST(RssSituation, ObjectAheadMovingTowardsUsIsOncoming)
{
  const rss::Situation s = rss::make_situation(ego(), vehicle(40.0, 0.0, -8.0));
  ASSERT_TRUE(s.valid);
  EXPECT_EQ(s.lon_case, LonCase::kOncoming);
  EXPECT_DOUBLE_EQ(s.obj_lon_speed_mps, 8.0);
}

TEST(RssSituation, ObjectBehind)
{
  const rss::Situation s = rss::make_situation(ego(), vehicle(-8.0, 0.0, 10.0));
  ASSERT_TRUE(s.valid);
  EXPECT_EQ(s.lon_case, LonCase::kEgoInFront);
  EXPECT_NEAR(s.lon_gap_m, -1.0 - (-8.0 + 2.0), kTol);
}

TEST(RssSituation, ObjectBehindReversingAwayCountsAsStanding)
{
  const rss::Situation s = rss::make_situation(ego(), vehicle(-8.0, 0.0, -3.0));
  ASSERT_TRUE(s.valid);
  EXPECT_DOUBLE_EQ(s.obj_lon_speed_mps, 0.0);
}

TEST(RssSituation, RotatedFootprint)
{
  rss::Object o = vehicle(20.0, 5.0, 0.0);
  o.length_m = 4.0;
  o.width_m = 2.0;
  o.yaw_rad = std::acos(0.0);  // pi/2, crossing: length along the ego y axis
  const rss::Situation s = rss::make_situation(ego(), o);
  ASSERT_TRUE(s.valid);
  EXPECT_NEAR(s.lon_gap_m, 20.0 - 1.0 - 3.8, 1e-6);
  EXPECT_NEAR(s.lat_gap_m, 5.0 - 2.0 - 0.9, 1e-6);
  EXPECT_TRUE(s.object_left);
}

TEST(RssSituation, StandstillNoiseIsZeroSpeed)
{
  const rss::Situation s = rss::make_situation(ego(-0.05), vehicle(30.0, 0.0, 0.0));
  ASSERT_TRUE(s.valid);
  EXPECT_DOUBLE_EQ(s.ego_lon_speed_mps, 0.0);
}

TEST(RssSituation, ReversingEgoIsInvalid)
{
  EXPECT_FALSE(rss::make_situation(ego(-0.5), vehicle(30.0, 0.0, 0.0)).valid);
}

TEST(RssSituation, BadInputIsInvalid)
{
  EXPECT_FALSE(rss::make_situation(ego(kNan), vehicle(30.0, 0.0, 0.0)).valid);
  EXPECT_FALSE(rss::make_situation(ego(), vehicle(kInf, 0.0, 0.0)).valid);
  rss::Object o = vehicle(30.0, 0.0, 0.0);
  o.length_m = -1.0;
  EXPECT_FALSE(rss::make_situation(ego(), o).valid);
  rss::EgoState e = ego();
  e.half_width_m = -0.1;
  EXPECT_FALSE(rss::make_situation(e, vehicle(30.0, 0.0, 0.0)).valid);
}

// ---------------------------------------------------------------- outcomes ----

TEST(RssOutcome, EmptyWorldIsSafe)
{
  const rss::Result r = rss::evaluate(nullptr, 0U, 0.5);
  EXPECT_EQ(r.outcome, Outcome::kSafe);
  EXPECT_EQ(r.reasons, R::kNone);
  EXPECT_DOUBLE_EQ(r.accel_mps2, 0.5);
  EXPECT_EQ(r.worst_index, -1);
}

TEST(RssOutcome, FollowingAtSafeGapIsSafe)
{
  // Gap 24.2 m > 23.2 m safe distance (both 10 m/s).
  const rss::Result r = run(ego(), vehicle(30.0, 0.0, 10.0));
  EXPECT_EQ(r.outcome, Outcome::kSafe);
  EXPECT_EQ(r.num_dangerous, 0U);
}

TEST(RssOutcome, FollowingTooCloseIsUnsafe)
{
  // Gap 22.2 m < 23.2 m and same lane: dangerous, ego must brake.
  const rss::Result r = run(ego(), vehicle(28.0, 0.0, 10.0), 0.5);
  EXPECT_EQ(r.outcome, Outcome::kUnsafe);
  EXPECT_NE(r.reasons & R::kDangerousAhead, 0U);
  EXPECT_DOUBLE_EQ(r.accel_max_mps2, -P::kEgo.lon_brake_min_mps2);
  EXPECT_DOUBLE_EQ(r.accel_mps2, -P::kEgo.lon_brake_min_mps2);
  EXPECT_EQ(r.worst_index, 0);
}

TEST(RssOutcome, AdjacentLaneCloseIsSafe)
{
  // Longitudinally too close but laterally 1.7 m apart (> 0.63 m): not dangerous.
  EXPECT_EQ(run(ego(), vehicle(10.0, 3.5, 10.0)).outcome, Outcome::kSafe);
}

TEST(RssOutcome, OncomingInOurLaneIsUnsafe)
{
  // Gap 54.2 m < 92.575 m.
  const rss::Result r = run(ego(), vehicle(60.0, 0.0, -10.0));
  EXPECT_EQ(r.outcome, Outcome::kUnsafe);
  EXPECT_NE(r.reasons & R::kDangerousOncoming, 0U);
}

TEST(RssOutcome, OncomingInAdjacentLaneIsSafe)
{
  EXPECT_EQ(run(ego(), vehicle(40.0, 3.5, -10.0)).outcome, Outcome::kSafe);
}

TEST(RssOutcome, PedestrianStepsTowardsPathIsUnsafe)
{
  // Pedestrian 20 m ahead, 3 m left, walking right at 1.5 m/s.
  // Lateral gap 3 - 0.25 - 0.9 = 1.85 m < 2.69375 m; longitudinal 15.95 m < 29.45 m.
  const rss::Result r = run(ego(), pedestrian(20.0, 3.0, 0.0, -1.5));
  EXPECT_EQ(r.outcome, Outcome::kUnsafe);
  EXPECT_NE(r.reasons & R::kDangerousAhead, 0U);
}

TEST(RssOutcome, PedestrianStandingOnSidewalkIsSafe)
{
  // Same place, standing: lateral safe distance 0.63125 m < 1.85 m.
  EXPECT_EQ(run(ego(), pedestrian(20.0, 3.0, 0.0, 0.0)).outcome, Outcome::kSafe);
}

TEST(RssOutcome, OverlapAlongsideIsUnsafe)
{
  // Object footprint overlapping the ego footprint.
  const rss::Result r = run(ego(), vehicle(1.4, 0.5, 10.0));
  EXPECT_EQ(r.outcome, Outcome::kUnsafe);
  EXPECT_NE(r.reasons & R::kDangerousAlongside, 0U);
}

TEST(RssOutcome, CloseVehicleBehindIsRestrictNotUnsafe)
{
  // Tailgater 5 m behind at the same speed: dangerous, but braking is not the ego's response.
  const rss::Result r = run(ego(), vehicle(-8.0, 0.0, 10.0), 0.3);
  EXPECT_EQ(r.outcome, Outcome::kRestrict);
  EXPECT_NE(r.reasons & R::kDangerousBehind, 0U);
  EXPECT_DOUBLE_EQ(r.accel_mps2, 0.3);
  EXPECT_DOUBLE_EQ(r.accel_min_mps2, -P::kEgo.lon_brake_max_mps2);
  EXPECT_EQ(r.worst_index, 0);
}

TEST(RssOutcome, AccelAboveResponseTimeBoundIsClamped)
{
  const rss::Result r = rss::evaluate(nullptr, 0U, 3.0);
  EXPECT_EQ(r.outcome, Outcome::kRestrict);
  EXPECT_NE(r.reasons & R::kAccelClamped, 0U);
  EXPECT_DOUBLE_EQ(r.accel_mps2, P::kEgo.lon_accel_max_mps2);
  EXPECT_EQ(r.worst_index, -1);
}

TEST(RssOutcome, BrakeHarderThanMaxIsClamped)
{
  const rss::Result r = rss::evaluate(nullptr, 0U, -9.0);
  EXPECT_EQ(r.outcome, Outcome::kRestrict);
  EXPECT_NE(r.reasons & R::kBrakeClamped, 0U);
  EXPECT_DOUBLE_EQ(r.accel_mps2, -P::kEgo.lon_brake_max_mps2);
}

TEST(RssOutcome, StrongerBrakingThanRequiredIsKeptWhenUnsafe)
{
  // The Doer already brakes harder than brake_min: keep its command.
  const rss::Result r = run(ego(), vehicle(28.0, 0.0, 10.0), -4.0);
  EXPECT_EQ(r.outcome, Outcome::kUnsafe);
  EXPECT_DOUBLE_EQ(r.accel_mps2, -4.0);
}

TEST(RssOutcome, UnsafeWinsOverRestrictAndPointsAtUnsafeObject)
{
  rss::Situation s[2] = {
    rss::make_situation(ego(), vehicle(-8.0, 0.0, 10.0)),  // dangerous behind
    rss::make_situation(ego(), vehicle(28.0, 0.0, 10.0)),  // dangerous ahead
  };
  const rss::Result r = rss::evaluate(s, 2U, 0.0);
  EXPECT_EQ(r.outcome, Outcome::kUnsafe);
  EXPECT_EQ(r.worst_index, 1);
  EXPECT_EQ(r.num_dangerous, 2U);
}

// --------------------------------------------------------------- fail-safe ----

TEST(RssFailSafe, InvalidSituationIsUnsafe)
{
  rss::Situation s[2] = {
    rss::make_situation(ego(), vehicle(80.0, 0.0, 10.0)),  // safe
    rss::make_situation(ego(-1.0), vehicle(80.0, 0.0, 10.0)),  // invalid
  };
  const rss::Result r = rss::evaluate(s, 2U, 0.0);
  EXPECT_EQ(r.outcome, Outcome::kUnsafe);
  EXPECT_NE(r.reasons & R::kInvalidInput, 0U);
  EXPECT_EQ(r.worst_index, 1);
}

TEST(RssFailSafe, NonFiniteCommandIsUnsafe)
{
  const rss::Result r = rss::evaluate(nullptr, 0U, kNan);
  EXPECT_EQ(r.outcome, Outcome::kUnsafe);
  EXPECT_NE(r.reasons & R::kInvalidInput, 0U);
  EXPECT_DOUBLE_EQ(r.accel_mps2, -P::kEgo.lon_brake_min_mps2);
}

TEST(RssFailSafe, NullWithCountIsUnsafe)
{
  EXPECT_EQ(rss::evaluate(nullptr, 1U, 0.0).outcome, Outcome::kUnsafe);
}

TEST(RssFailSafe, TooManyObjectsIsUnsafe)
{
  rss::Situation s[P::kMaxObjects + 1U]{};
  const rss::Result r = rss::evaluate(s, P::kMaxObjects + 1U, 0.0);
  EXPECT_EQ(r.outcome, Outcome::kUnsafe);
  EXPECT_NE(r.reasons & R::kTooManyObjects, 0U);
}

TEST(RssFailSafe, MaxObjectsAllSafeIsSafe)
{
  rss::Situation s[P::kMaxObjects];
  for (auto & x : s) {
    x = rss::make_situation(ego(), vehicle(200.0, 0.0, 10.0));
  }
  EXPECT_EQ(rss::evaluate(s, P::kMaxObjects, 0.0).outcome, Outcome::kSafe);
}

TEST(RssFailSafe, HandBuiltSituationWithNegativeSpeedIsUnsafe)
{
  rss::Situation s = rss::make_situation(ego(), vehicle(80.0, 0.0, 10.0));
  s.obj_lon_speed_mps = -1.0;
  EXPECT_FALSE(rss::evaluate_object(s).valid);
  EXPECT_EQ(rss::evaluate(&s, 1U, 0.0).outcome, Outcome::kUnsafe);
}

TEST(RssFailSafe, UnknownEnumValuesAreHandled)
{
  rss::Situation s = rss::make_situation(ego(), vehicle(80.0, 0.0, 10.0));
  s.lon_case = static_cast<LonCase>(7);
  EXPECT_EQ(rss::evaluate(&s, 1U, 0.0).outcome, Outcome::kUnsafe);
  // An out-of-range class gets the worst-case dynamics.
  EXPECT_EQ(&rss::dynamics_for(static_cast<ObjectClass>(9)), &P::kUnknown);
}

TEST(RssFailSafe, OutcomeNames)
{
  EXPECT_STREQ(rss::to_string(Outcome::kSafe), "SAFE");
  EXPECT_STREQ(rss::to_string(Outcome::kRestrict), "RESTRICT");
  EXPECT_STREQ(rss::to_string(Outcome::kUnsafe), "UNSAFE");
  EXPECT_STREQ(rss::to_string(static_cast<Outcome>(9)), "INVALID");
}
