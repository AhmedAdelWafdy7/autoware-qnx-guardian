// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0
//
// FR-CV-1..4: PASS / CLAMP / REJECT, each with its reason.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "guardian/limits.hpp"
#include "guardian/validator.hpp"

using guardian::Command;
using guardian::validate;
using guardian::Verdict;
using guardian::VehicleStatus;
namespace L = guardian::limits;
namespace R = guardian::reason;

namespace
{
constexpr guardian::TimeNs kNow = 10'000'000'000;  // 10 s
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

Command cmd(double steer = 0.0, double accel = 0.0, double speed = 5.0)
{
  Command c;
  c.steering_angle_rad = steer;
  c.accel_mps2 = accel;
  c.speed_mps = speed;
  c.rx_time_ns = kNow - 5'000'000;
  return c;
}

VehicleStatus status(double speed = 5.0)
{
  VehicleStatus s;
  s.speed_mps = speed;
  s.rx_time_ns = kNow - 5'000'000;
  return s;
}
}  // namespace

TEST(Validator, NominalPasses)
{
  const auto r = validate(cmd(0.01, 0.5), cmd(), status(), kNow);
  EXPECT_EQ(r.verdict, Verdict::kPass);
  EXPECT_EQ(r.reasons, R::kNone);
  EXPECT_DOUBLE_EQ(r.command.steering_angle_rad, 0.01);
  EXPECT_DOUBLE_EQ(r.command.accel_mps2, 0.5);
}

TEST(Validator, NonFiniteRejected)
{
  for (double bad : {kNan, kInf, -kInf}) {
    Command c = cmd();
    c.steering_angle_rad = bad;
    EXPECT_EQ(validate(c, cmd(), status(), kNow).reasons & R::kNonFinite, R::kNonFinite);
    c = cmd();
    c.accel_mps2 = bad;
    EXPECT_EQ(validate(c, cmd(), status(), kNow).verdict, Verdict::kReject);
    c = cmd();
    c.speed_mps = bad;
    EXPECT_EQ(validate(c, cmd(), status(), kNow).verdict, Verdict::kReject);
    c = cmd();
    c.steering_rate_radps = bad;
    EXPECT_EQ(validate(c, cmd(), status(), kNow).verdict, Verdict::kReject);
  }
}

TEST(Validator, RejectReturnsPreviousOutput)
{
  Command prev = cmd(0.1, -0.3);
  Command c = cmd();
  c.accel_mps2 = kNan;
  const auto r = validate(c, prev, status(), kNow);
  EXPECT_EQ(r.verdict, Verdict::kReject);
  EXPECT_DOUBLE_EQ(r.command.steering_angle_rad, 0.1);
  EXPECT_DOUBLE_EQ(r.command.accel_mps2, -0.3);
}

TEST(Validator, StaleAndFutureCommandsRejected)
{
  Command c = cmd();
  c.rx_time_ns = kNow - L::kCommandMaxAge_ns - 1;
  EXPECT_EQ(validate(c, cmd(), status(), kNow).reasons, R::kStale);
  c.rx_time_ns = kNow - L::kCommandMaxAge_ns;  // boundary: still OK
  EXPECT_EQ(validate(c, cmd(), status(), kNow).verdict, Verdict::kPass);
  c.rx_time_ns = kNow + 1;  // from the future: clock inconsistency
  EXPECT_EQ(validate(c, cmd(), status(), kNow).reasons, R::kStale);
}

TEST(Validator, MissingVehicleStatusRejected)
{
  VehicleStatus s = status();
  s.rx_time_ns = kNow - L::kVehicleStatusMaxAge_ns - 1;
  EXPECT_EQ(validate(cmd(), cmd(), s, kNow).reasons, R::kNoVehicleStatus);
  s = status();
  s.speed_mps = kNan;
  EXPECT_EQ(validate(cmd(), cmd(), s, kNow).reasons, R::kNoVehicleStatus);
}

TEST(Validator, SteeringSpikeRejected)
{
  const auto r = validate(cmd(L::kSteerSpikeReject_rad + 0.01), cmd(0.0), status(), kNow);
  EXPECT_EQ(r.verdict, Verdict::kReject);
  EXPECT_EQ(r.reasons, R::kSteerSpike);
  // Spike is relative to the previous output, in both directions.
  EXPECT_EQ(validate(cmd(-0.3), cmd(0.1), status(), kNow).reasons, R::kSteerSpike);
}

TEST(Validator, SteeringRateClamped)
{
  const double max_step = L::kMaxSteerRate_radps * 0.01;  // per 10 ms cycle
  const auto r = validate(cmd(0.1), cmd(0.0), status(1.0), kNow);
  EXPECT_EQ(r.verdict, Verdict::kClamp);
  EXPECT_EQ(r.reasons & R::kSteerRate, R::kSteerRate);
  EXPECT_NEAR(r.command.steering_angle_rad, max_step, 1e-12);

  Command c = cmd(0.0);
  c.steering_rate_radps = -5.0;
  const auto r2 = validate(c, cmd(0.0), status(), kNow);
  EXPECT_EQ(r2.verdict, Verdict::kClamp);
  EXPECT_DOUBLE_EQ(r2.command.steering_rate_radps, -L::kMaxSteerRate_radps);
}

TEST(Validator, PhysicalSteeringAngleClamped)
{
  // Previous output already at the limit, request a bit more (within rate), slow speed.
  const auto r = validate(cmd(L::kMaxSteerAngle_rad + 0.005), cmd(L::kMaxSteerAngle_rad),
                          status(0.0), kNow);
  EXPECT_EQ(r.verdict, Verdict::kClamp);
  EXPECT_EQ(r.reasons, R::kSteerAngle);
  EXPECT_DOUBLE_EQ(r.command.steering_angle_rad, L::kMaxSteerAngle_rad);
}

TEST(Validator, SpeedDependentLateralBound)
{
  // a_lat = v^2 tan(delta) / L  ->  delta_max = atan(a_lat_max L / v^2)
  const double v = 20.0;
  const double bound = std::atan(L::kMaxLatAccel_mps2 * L::kWheelbase_m / (v * v));
  EXPECT_NEAR(guardian::max_steer_for_speed(v), bound, 1e-12);
  EXPECT_NEAR(guardian::max_steer_for_speed(-v), bound, 1e-12);
  EXPECT_DOUBLE_EQ(guardian::max_steer_for_speed(0.0), L::kMaxSteerAngle_rad);
  EXPECT_DOUBLE_EQ(guardian::max_steer_for_speed(kNan), L::kMaxSteerAngle_rad);
  EXPECT_DOUBLE_EQ(guardian::max_steer_for_speed(1.0), L::kMaxSteerAngle_rad);

  const auto r = validate(cmd(bound + 0.005), cmd(bound), status(v), kNow);
  EXPECT_EQ(r.verdict, Verdict::kClamp);
  EXPECT_EQ(r.reasons, R::kLatAccel);
  EXPECT_NEAR(r.command.steering_angle_rad, bound, 1e-12);
  // Resulting lateral acceleration is within the limit.
  EXPECT_LE(v * v * std::tan(r.command.steering_angle_rad) / L::kWheelbase_m,
            L::kMaxLatAccel_mps2 + 1e-9);
}

TEST(Validator, LongitudinalAndSpeedClamped)
{
  auto r = validate(cmd(0.0, 9.0), cmd(), status(), kNow);
  EXPECT_EQ(r.reasons, R::kLonAccelHigh);
  EXPECT_DOUBLE_EQ(r.command.accel_mps2, L::kMaxLonAccel_mps2);

  r = validate(cmd(0.0, -20.0), cmd(), status(), kNow);
  EXPECT_EQ(r.reasons, R::kLonAccelLow);
  EXPECT_DOUBLE_EQ(r.command.accel_mps2, L::kMinLonAccel_mps2);

  r = validate(cmd(0.0, 0.0, 40.0), cmd(), status(), kNow);
  EXPECT_EQ(r.reasons, R::kSpeed);
  EXPECT_DOUBLE_EQ(r.command.speed_mps, L::kMaxSpeed_mps);

  r = validate(cmd(0.0, 0.0, -40.0), cmd(), status(), kNow);
  EXPECT_DOUBLE_EQ(r.command.speed_mps, -L::kMaxSpeed_mps);
}

TEST(Validator, AutowareMrmDecelerationsPassUnclamped)
{
  // The Guardian must not fight Autoware's own MRM requests (backstop design).
  EXPECT_EQ(validate(cmd(0.0, -2.5, 0.0), cmd(), status(), kNow).verdict, Verdict::kPass);
  EXPECT_EQ(validate(cmd(0.0, -2.4, 0.0), cmd(), status(), kNow).verdict, Verdict::kPass);
}

TEST(Validator, VerdictToString)
{
  EXPECT_STREQ(guardian::to_string(Verdict::kPass), "PASS");
  EXPECT_STREQ(guardian::to_string(Verdict::kClamp), "CLAMP");
  EXPECT_STREQ(guardian::to_string(Verdict::kReject), "REJECT");
  EXPECT_STREQ(guardian::to_string(static_cast<Verdict>(9)), "UNKNOWN");
}
