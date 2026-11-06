// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0

#include "guardian/guardian.hpp"

#include <cmath>

#include "guardian/limits.hpp"

namespace guardian
{

namespace
{
constexpr double kCyclePeriod_s = static_cast<double>(limits::kCyclePeriod_ns) * 1e-9;

bool fresh(TimeNs rx, TimeNs now, TimeNs max_age) noexcept
{
  return rx <= now && (now - rx) <= max_age;
}
}  // namespace

void Guardian::log(
  TimeNs t, EventCode code, std::uint8_t a, std::uint8_t b, std::uint32_t reasons,
  std::int64_t value) noexcept
{
  if (log_ != nullptr) {
    Event e;
    e.time_ns = t;
    e.code = code;
    e.a = a;
    e.b = b;
    e.reasons = reasons;
    e.value = value;
    (void)log_->push(e);  // drop is counted inside the log; never blocks the cycle
  }
}

Command Guardian::mrm_command(State s, TimeNs now_ns) const noexcept
{
  Command c;
  c.rx_time_ns = now_ns;
  c.speed_mps = 0.0;
  c.steering_rate_radps = 0.0;
  // Hold lane: keep the last commanded steering angle (no planner available to us).
  c.steering_angle_rad = have_last_out_ ? last_out_.steering_angle_rad : 0.0;

  if (s == State::kStopped) {
    c.accel_mps2 = limits::kStopHoldAccel_mps2;
    return c;
  }
  const double target = (s == State::kMrmEmergency) ? limits::kMrmEmergencyDecel_mps2
                                                     : limits::kMrmComfortableDecel_mps2;
  const double prev = have_last_out_ ? last_out_.accel_mps2 : 0.0;
  if (prev > target) {
    // Ramp toward the target deceleration with bounded jerk.
    const double next = prev - limits::kMrmJerk_mps3 * kCyclePeriod_s;
    c.accel_mps2 = next > target ? next : target;
  } else {
    // Already braking at least this hard: never release the brake during MRM.
    c.accel_mps2 = prev < limits::kMinLonAccel_mps2 ? limits::kMinLonAccel_mps2 : prev;
  }
  return c;
}

CycleOutput Guardian::step(const CycleInputs & in) noexcept
{
  const TimeNs now = in.now_ns;

  // 1. Heartbeats.
  hb_.update(Source::kDoer, in.doer);
  hb_.update(Source::kSafetyLayer, in.safety_layer);
  const bool doer_alive = hb_.alive(Source::kDoer, now);
  const bool safety_alive = hb_.alive(Source::kSafetyLayer, now);
  if (doer_was_alive_ && !doer_alive) {
    log(now, EventCode::kHeartbeatLost, static_cast<std::uint8_t>(Source::kDoer), 0U, 0U, 0);
  }
  if (safety_was_alive_ && !safety_alive) {
    log(now, EventCode::kHeartbeatLost, static_cast<std::uint8_t>(Source::kSafetyLayer), 0U,
        0U, 0);
  }
  doer_was_alive_ = doer_alive;
  safety_was_alive_ = safety_alive;

  // 2. Vehicle status and standstill timer.
  const bool status_fresh =
    in.have_status && std::isfinite(in.status.speed_mps) &&
    fresh(in.status.rx_time_ns, now, limits::kVehicleStatusMaxAge_ns);
  if (status_fresh && std::fabs(in.status.speed_mps) < limits::kStoppedSpeed_mps) {
    if (!still_) {
      still_ = true;
      still_since_ns_ = now;
    }
  } else {
    still_ = false;
  }
  const bool stopped_for_hold = still_ && (now - still_since_ns_) >= limits::kStoppedHold_ns;

  // 3. Validate the safety layer's command against the previous output.
  Command reference = last_out_;
  if (!have_last_out_) {
    reference.steering_angle_rad = status_fresh ? in.status.steering_angle_rad : 0.0;
  }
  ValidationResult v;  // default: REJECT
  if (in.have_command) {
    v = validate(in.command, reference, in.status, now);
  }
  const bool command_fresh =
    in.have_command && fresh(in.command.rx_time_ns, now, limits::kCommandMaxAge_ns);
  if (v.verdict == Verdict::kReject) {
    if (reject_streak_ < limits::kMaxConsecutiveRejects) {
      ++reject_streak_;
    }
  } else {
    reject_streak_ = 0U;
  }

  // 4. State machine.
  Health h;
  h.doer_alive = doer_alive;
  h.safety_layer_alive = safety_alive;
  h.command_fresh = command_fresh;
  h.vehicle_status_fresh = status_fresh;
  h.reject_limit_reached = reject_streak_ >= limits::kMaxConsecutiveRejects;
  h.internal_fault = in.internal_fault;
  h.deadline_missed = in.deadline_missed;
  h.stopped_for_hold_time = stopped_for_hold;
  h.operator_reset = in.operator_reset;

  const State before = sm_.state();
  const unsigned long illegal_before = sm_.illegal_requests();
  const State state = sm_.step(h);
  if (state != before) {
    log(now, EventCode::kStateChange, static_cast<std::uint8_t>(before),
        static_cast<std::uint8_t>(state), 0U, 0);
  }
  if (sm_.illegal_requests() != illegal_before) {
    log(now, EventCode::kIllegalTransition, static_cast<std::uint8_t>(before),
        static_cast<std::uint8_t>(state), 0U, 0);
  }
  if (in.deadline_missed) {
    log(now, EventCode::kDeadlineMiss, 0U, 0U, 0U, 0);
  }

  // 5. Arbitration -> final command.
  CycleOutput out;
  out.state = state;
  out.verdict = v.verdict;
  out.reasons = v.reasons;

  if (state == State::kNormal || state == State::kDegraded) {
    if (v.verdict == Verdict::kReject) {
      log(now, EventCode::kReject, 0U, 0U, v.reasons, 0);
      out.command = reference;
      out.command.rx_time_ns = now;
      out.actor = Actor::kGuardianHold;
    } else {
      if (v.verdict == Verdict::kClamp) {
        log(now, EventCode::kClamp, 0U, 0U, v.reasons, 0);
      }
      out.command = v.command;
      out.actor = Actor::kSafetyLayer;
    }
  } else {
    out.command = mrm_command(state, now);
    out.actor = Actor::kGuardianMrm;
  }

  last_out_ = out.command;
  have_last_out_ = true;
  return out;
}

}  // namespace guardian
