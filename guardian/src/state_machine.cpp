// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0

#include "guardian/state_machine.hpp"

#include <cstdint>

namespace guardian
{

const char * to_string(State s) noexcept
{
  switch (s) {
    case State::kNormal:
      return "NORMAL";
    case State::kDegraded:
      return "DEGRADED";
    case State::kMrmComfortable:
      return "MRM_COMFORTABLE";
    case State::kMrmEmergency:
      return "MRM_EMERGENCY";
    case State::kStopped:
      return "STOPPED";
  }
  return "UNKNOWN";
}

namespace
{
// kAllowed[from][to]. Self-loops allowed. See header for the rules.
constexpr bool kAllowed[kNumStates][kNumStates] = {
  //               NORMAL DEGRADED MRM_C  MRM_E  STOPPED
  /* NORMAL   */ {true, true, true, true, false},
  /* DEGRADED */ {true, true, true, true, false},
  /* MRM_C    */ {false, false, true, true, true},
  /* MRM_E    */ {false, false, false, true, true},
  /* STOPPED  */ {true, false, false, false, true},
};

constexpr std::uint32_t idx(State s) noexcept { return static_cast<std::uint32_t>(s); }

constexpr bool valid(State s) noexcept { return idx(s) < kNumStates; }

constexpr bool is_mrm(State s) noexcept
{
  return s == State::kMrmComfortable || s == State::kMrmEmergency;
}
}  // namespace

bool is_allowed_transition(State from, State to) noexcept
{
  if (!valid(from) || !valid(to)) {
    return false;
  }
  return kAllowed[idx(from)][idx(to)];
}

State decide(State current, const Health & h) noexcept
{
  if (!valid(current)) {
    return State::kMrmEmergency;  // unknown state -> fail safe
  }

  if (current == State::kStopped) {
    const bool all_healthy = h.doer_alive && h.safety_layer_alive && h.command_fresh &&
                             h.vehicle_status_fresh && !h.internal_fault &&
                             !h.deadline_missed && !h.reject_limit_reached;
    return (h.operator_reset && all_healthy) ? State::kNormal : State::kStopped;
  }

  // Emergency class: the Guardian itself or its view of the vehicle is compromised.
  const bool emergency = h.internal_fault || h.deadline_missed || !h.vehicle_status_fresh;

  if (is_mrm(current)) {
    // Latched until standstill. Standstill needs trustworthy speed feedback.
    if (h.stopped_for_hold_time && h.vehicle_status_fresh) {
      return State::kStopped;
    }
    return emergency ? State::kMrmEmergency : current;
  }

  // current is NORMAL or DEGRADED.
  if (emergency) {
    return State::kMrmEmergency;
  }
  // The ROS 2 safety layer is not trustworthy: the Guardian takes over (FR-MRM-2).
  if (!h.safety_layer_alive || !h.command_fresh || h.reject_limit_reached) {
    return State::kMrmComfortable;
  }
  // Doer gone but safety layer healthy: Autoware's ported MRM drives the stop (FR-MRM-1).
  if (!h.doer_alive) {
    return State::kDegraded;
  }
  return State::kNormal;
}

void StateMachine::enter(State target) noexcept
{
  if (target != state_) {
    state_ = target;
    ++transitions_;
  }
}

State StateMachine::step(const Health & health) noexcept
{
  const State target = decide(state_, health);
  if (is_allowed_transition(state_, target)) {
    enter(target);
  } else {
    // decide() must never produce this; treat as internal error.
    ++illegal_requests_;
    enter(state_ == State::kStopped ? State::kStopped : State::kMrmEmergency);
  }
  return state_;
}

bool StateMachine::request(State target) noexcept
{
  // Leaving STOPPED is only possible through step() with operator_reset and full health.
  const bool leaves_stopped = state_ == State::kStopped && target != State::kStopped;
  if (!leaves_stopped && is_allowed_transition(state_, target)) {
    enter(target);
    return true;
  }
  ++illegal_requests_;
  enter(state_ == State::kStopped ? State::kStopped : State::kMrmEmergency);
  return false;
}

}  // namespace guardian
