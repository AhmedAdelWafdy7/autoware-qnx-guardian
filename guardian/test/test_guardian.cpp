// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0
//
// Whole-cycle behaviour: arbitration (FR-MRM-4), Guardian MRM (FR-MRM-2), heartbeat loss
// reaction time (FR-HB-3 / NFR-T-2), REJECT escalation (FR-CV-6), logging (FR-CV-2).

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "guardian/guardian.hpp"
#include "guardian/limits.hpp"

using guardian::Actor;
using guardian::CycleInputs;
using guardian::CycleOutput;
using guardian::Event;
using guardian::EventCode;
using guardian::Guardian;
using guardian::GuardianEventLog;
using guardian::State;
using guardian::TimeNs;
using guardian::Verdict;
namespace L = guardian::limits;

namespace
{
/// Simulated world: everything healthy unless a test breaks it.
class World
{
public:
  bool doer_up{true};
  bool safety_up{true};
  bool status_up{true};
  double speed{5.0};
  double steer_cmd{0.0};
  double accel_cmd{0.3};

  CycleInputs next()
  {
    t_ += L::kCyclePeriod_ns;
    ++seq_;
    CycleInputs in;
    in.now_ns = t_;
    if (doer_up) {
      doer_ = {true, seq_, t_};
    }
    if (safety_up) {
      safety_ = {true, seq_, t_};
      cmd_.steering_angle_rad = steer_cmd;
      cmd_.accel_mps2 = accel_cmd;
      cmd_.speed_mps = 5.0;
      cmd_.rx_time_ns = t_;
    }
    in.doer = doer_;
    in.safety_layer = safety_;
    in.have_command = true;
    in.command = cmd_;
    if (status_up) {
      status_.speed_mps = speed;
      status_.rx_time_ns = t_;
    }
    in.have_status = true;
    in.status = status_;
    return in;
  }

  TimeNs now() const { return t_; }

private:
  TimeNs t_{1'000'000'000};
  std::uint32_t seq_{0};
  guardian::HeartbeatSample doer_{};
  guardian::HeartbeatSample safety_{};
  guardian::Command cmd_{};
  guardian::VehicleStatus status_{};
};

CycleOutput arm(Guardian & g, World & w)
{
  CycleInputs in = w.next();
  in.operator_reset = true;
  return g.step(in);
}

int count(GuardianEventLog & log, EventCode code)
{
  int n = 0;
  Event e;
  while (log.pop(e)) {
    n += (e.code == code) ? 1 : 0;
  }
  return n;
}
}  // namespace

TEST(Guardian, PowerOnIsStoppedAndHoldsBrake)
{
  Guardian g;
  World w;
  const auto out = g.step(w.next());
  EXPECT_EQ(out.state, State::kStopped);
  EXPECT_EQ(out.actor, Actor::kGuardianMrm);
  EXPECT_DOUBLE_EQ(out.command.accel_mps2, L::kStopHoldAccel_mps2);
}

TEST(Guardian, ResetWithoutInputsStaysStopped)
{
  Guardian g;
  CycleInputs in;
  in.now_ns = 1'000'000'000;
  in.operator_reset = true;
  EXPECT_EQ(g.step(in).state, State::kStopped);
}

TEST(Guardian, ArmedPassesSafetyLayerCommand)
{
  Guardian g;
  World w;
  const auto out = arm(g, w);
  EXPECT_EQ(out.state, State::kNormal);
  EXPECT_EQ(out.actor, Actor::kSafetyLayer);
  EXPECT_EQ(out.verdict, Verdict::kPass);
  EXPECT_DOUBLE_EQ(out.command.accel_mps2, 0.3);
}

TEST(Guardian, DoerLossWithinTimeoutPlusOneCycleGoesDegraded)
{
  Guardian g;
  World w;
  arm(g, w);
  w.doer_up = false;
  const TimeNs lost_at = w.now();
  CycleOutput out;
  do {
    out = g.step(w.next());
  } while (out.state == State::kNormal && w.now() < lost_at + 1'000'000'000);
  EXPECT_EQ(out.state, State::kDegraded);
  EXPECT_LE(w.now() - lost_at, L::kHeartbeatTimeout_ns + L::kCyclePeriod_ns);  // NFR-T-2
  // Autoware's ported MRM (safety layer) still drives (FR-MRM-1).
  EXPECT_EQ(out.actor, Actor::kSafetyLayer);
}

TEST(Guardian, SafetyLayerLossGuardianTakesOverAndStops)
{
  GuardianEventLog log;
  Guardian g(&log);
  World w;
  arm(g, w);
  w.steer_cmd = 0.0;
  g.step(w.next());

  w.safety_up = false;
  const TimeNs lost_at = w.now();
  CycleOutput out;
  do {
    out = g.step(w.next());
  } while (out.state == State::kNormal);
  EXPECT_EQ(out.state, State::kMrmComfortable);
  EXPECT_LE(w.now() - lost_at, L::kHeartbeatTimeout_ns + L::kCyclePeriod_ns);
  EXPECT_EQ(out.actor, Actor::kGuardianMrm);

  // Deceleration ramps with bounded jerk to the comfortable target and never releases.
  double prev = out.command.accel_mps2;
  for (int i = 0; i < 300; ++i) {
    out = g.step(w.next());
    EXPECT_LE(out.command.accel_mps2, prev + 1e-12);
    EXPECT_GE(prev - out.command.accel_mps2, -1e-12);
    EXPECT_LE(prev - out.command.accel_mps2, L::kMrmJerk_mps3 * 0.01 + 1e-12);
    EXPECT_DOUBLE_EQ(out.command.speed_mps, 0.0);
    EXPECT_DOUBLE_EQ(out.command.steering_angle_rad, 0.0);  // lane hold
    prev = out.command.accel_mps2;
  }
  EXPECT_DOUBLE_EQ(out.command.accel_mps2, L::kMrmComfortableDecel_mps2);

  // Vehicle comes to rest -> STOPPED after the hold time, then brake hold.
  w.speed = 0.0;
  const TimeNs still_at = w.now();
  do {
    out = g.step(w.next());
  } while (out.state != State::kStopped && w.now() < still_at + 5'000'000'000);
  EXPECT_EQ(out.state, State::kStopped);
  EXPECT_GE(w.now() - still_at, L::kStoppedHold_ns);
  EXPECT_DOUBLE_EQ(out.command.accel_mps2, L::kStopHoldAccel_mps2);
  EXPECT_GE(count(log, EventCode::kHeartbeatLost), 1);
}

TEST(Guardian, StoppedIsLatchedUntilReset)
{
  Guardian g;
  World w;
  arm(g, w);
  w.safety_up = false;
  for (int i = 0; i < 20; ++i) {
    g.step(w.next());
  }
  w.speed = 0.0;
  for (int i = 0; i < 150; ++i) {
    g.step(w.next());
  }
  ASSERT_EQ(g.state(), State::kStopped);
  w.safety_up = true;  // everything healthy again
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(g.step(w.next()).state, State::kStopped);
  }
  EXPECT_EQ(arm(g, w).state, State::kNormal);
}

TEST(Guardian, SteeringSpikeRejectedHeldAndLogged)
{
  GuardianEventLog log;
  Guardian g(&log);
  World w;
  arm(g, w);
  w.steer_cmd = 0.5;  // spike from 0.0
  const auto out = g.step(w.next());
  EXPECT_EQ(out.verdict, Verdict::kReject);
  EXPECT_EQ(out.actor, Actor::kGuardianHold);
  EXPECT_DOUBLE_EQ(out.command.steering_angle_rad, 0.0);
  EXPECT_EQ(out.state, State::kNormal);
  EXPECT_EQ(count(log, EventCode::kReject), 1);
}

TEST(Guardian, ConsecutiveRejectsEscalateToMrm)
{
  Guardian g;
  World w;
  arm(g, w);
  w.steer_cmd = 0.5;
  CycleOutput out;
  for (std::uint32_t i = 0; i < L::kMaxConsecutiveRejects; ++i) {
    out = g.step(w.next());
  }
  EXPECT_EQ(out.state, State::kMrmComfortable);
}

TEST(Guardian, IsolatedRejectsDoNotEscalate)
{
  Guardian g;
  World w;
  arm(g, w);
  for (int i = 0; i < 20; ++i) {
    w.steer_cmd = (i % 2 == 0) ? 0.5 : 0.0;  // reject, pass, reject, ...
    g.step(w.next());
  }
  EXPECT_EQ(g.state(), State::kNormal);
}

TEST(Guardian, ClampIsLoggedAndApplied)
{
  GuardianEventLog log;
  Guardian g(&log);
  World w;
  arm(g, w);
  w.accel_cmd = 9.0;
  const auto out = g.step(w.next());
  EXPECT_EQ(out.verdict, Verdict::kClamp);
  EXPECT_DOUBLE_EQ(out.command.accel_mps2, L::kMaxLonAccel_mps2);
  EXPECT_EQ(count(log, EventCode::kClamp), 1);
}

TEST(Guardian, DeadlineMissGoesEmergency)
{
  GuardianEventLog log;
  Guardian g(&log);
  World w;
  arm(g, w);
  CycleInputs in = w.next();
  in.deadline_missed = true;
  const auto out = g.step(in);
  EXPECT_EQ(out.state, State::kMrmEmergency);
  EXPECT_EQ(out.actor, Actor::kGuardianMrm);
  EXPECT_EQ(count(log, EventCode::kDeadlineMiss), 1);
}

TEST(Guardian, InternalFaultGoesEmergency)
{
  Guardian g;
  World w;
  arm(g, w);
  CycleInputs in = w.next();
  in.internal_fault = true;
  EXPECT_EQ(g.step(in).state, State::kMrmEmergency);
}

TEST(Guardian, StaleVehicleStatusGoesEmergency)
{
  Guardian g;
  World w;
  arm(g, w);
  w.status_up = false;
  CycleOutput out;
  for (int i = 0; i < 12; ++i) {
    out = g.step(w.next());
  }
  EXPECT_EQ(out.state, State::kMrmEmergency);
}

TEST(Guardian, EmergencyNeverReleasesHarderBraking)
{
  Guardian g;
  World w;
  arm(g, w);
  w.accel_cmd = -4.0;  // Autoware braking hard (passes: within envelope)
  g.step(w.next());
  CycleInputs in = w.next();
  in.deadline_missed = true;
  const auto out = g.step(in);
  ASSERT_EQ(out.state, State::kMrmEmergency);
  EXPECT_DOUBLE_EQ(out.command.accel_mps2, -4.0);  // not released to -2.5
}

TEST(Guardian, NonFiniteSpeedIsNotFreshStatus)
{
  Guardian g;
  World w;
  arm(g, w);
  w.speed = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(g.step(w.next()).state, State::kMrmEmergency);
}
