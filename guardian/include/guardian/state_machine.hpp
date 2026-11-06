// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0
//
// Guardian MRM state machine (FR-MRM-3, FR-MRM-5).
//
//   NORMAL ──► DEGRADED ──► MRM_COMFORTABLE ──► MRM_EMERGENCY ──► STOPPED
//     ▲  ◄──────┘  │                │  ▲                            │
//     │            └────────────────┼──┘ (any fault class may       │
//     │                             └──► skip ahead, never back)    │
//     └──────────────── operator reset (only from STOPPED) ─────────┘
//
// Rules:
//  * Only DEGRADED may return to NORMAL on its own (doer heartbeat back).
//  * Once a Guardian MRM has started it is latched until standstill: MRM states never
//    de-escalate (comfortable may escalate to emergency).
//  * STOPPED is left only by an explicit operator reset while every input is healthy.
//  * Any transition not in the allowed table is an internal error -> MRM_EMERGENCY
//    (fail safe), except from STOPPED, which stays STOPPED.
//
// Pure, allocation-free and noexcept: safe for the RT path.

#ifndef GUARDIAN__STATE_MACHINE_HPP_
#define GUARDIAN__STATE_MACHINE_HPP_

#include "guardian/types.hpp"

namespace guardian
{

/// Health snapshot evaluated once per cycle. true = condition present.
struct Health
{
  bool doer_alive{false};             ///< Autoware (Linux) heartbeat fresh
  bool safety_layer_alive{false};     ///< ported ROS 2 safety layer heartbeat fresh
  bool command_fresh{false};          ///< latest safety-layer command within max age
  bool vehicle_status_fresh{false};   ///< speed/steering feedback within max age
  bool reject_limit_reached{false};   ///< N consecutive validator REJECTs
  bool internal_fault{false};         ///< Guardian self-check failed
  bool deadline_missed{false};        ///< Guardian cycle overran
  bool stopped_for_hold_time{false};  ///< |v| < threshold for the hold time
  bool operator_reset{false};         ///< reset requested (via /dev/guardian)
};

/// True if `from -> to` is in the allowed transition table (self-loops are allowed).
bool is_allowed_transition(State from, State to) noexcept;

/// The state the Guardian *wants* to be in, given the current state and health.
/// Always returns a state reachable from `current` per the table above.
State decide(State current, const Health & health) noexcept;

/// Stateful wrapper: applies decide(), re-checks the table, counts transitions/errors.
class StateMachine
{
public:
  State state() const noexcept { return state_; }

  /// Run one cycle. Returns the new state.
  State step(const Health & health) noexcept;

  /// Request an explicit transition (used by tests and fault injection). An illegal
  /// request is an internal error: the machine goes to MRM_EMERGENCY (or stays STOPPED)
  /// and the error counter increments. Returns true if the request was legal.
  /// request() can never leave STOPPED: only step() with operator_reset and full health can.
  bool request(State target) noexcept;

  unsigned long transitions() const noexcept { return transitions_; }
  unsigned long illegal_requests() const noexcept { return illegal_requests_; }

private:
  void enter(State target) noexcept;

  /// Power-on state is STOPPED (fail safe): the Guardian arms only after an operator reset
  /// with every input healthy, never on stale or absent data.
  State state_{State::kStopped};
  unsigned long transitions_{0};
  unsigned long illegal_requests_{0};
};

}  // namespace guardian

#endif  // GUARDIAN__STATE_MACHINE_HPP_
