// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0
//
// FR-MRM-3 / FR-MRM-5: every transition, legal and illegal, is tested.

#include <gtest/gtest.h>

#include <cstdint>

#include "guardian/state_machine.hpp"

using guardian::decide;
using guardian::Health;
using guardian::is_allowed_transition;
using guardian::State;
using guardian::StateMachine;

namespace
{
constexpr State kAll[] = {State::kNormal, State::kDegraded, State::kMrmComfortable,
                          State::kMrmEmergency, State::kStopped};

Health healthy()
{
  Health h;
  h.doer_alive = true;
  h.safety_layer_alive = true;
  h.command_fresh = true;
  h.vehicle_status_fresh = true;
  return h;
}

/// Drive a fresh machine into `target` through legal steps.
StateMachine at(State target)
{
  StateMachine sm;  // power-on: STOPPED
  if (target == State::kStopped) {
    return sm;
  }
  Health h = healthy();
  h.operator_reset = true;
  sm.step(h);  // -> NORMAL
  EXPECT_EQ(sm.state(), State::kNormal);
  if (target != State::kNormal) {
    EXPECT_TRUE(sm.request(target));
  }
  EXPECT_EQ(sm.state(), target);
  return sm;
}
}  // namespace

// ---------------------------------------------------------------- table -----

TEST(TransitionTable, MatchesSpecExactly)
{
  // Expected[from][to], written independently of the implementation.
  const bool expected[5][5] = {
    // to: N      D      MC     ME     S
    {true, true, true, true, false},     // from NORMAL
    {true, true, true, true, false},     // from DEGRADED
    {false, false, true, true, true},    // from MRM_COMFORTABLE (no de-escalation)
    {false, false, false, true, true},   // from MRM_EMERGENCY (no de-escalation)
    {true, false, false, false, true},   // from STOPPED (reset only)
  };
  for (std::uint32_t f = 0; f < 5U; ++f) {
    for (std::uint32_t t = 0; t < 5U; ++t) {
      EXPECT_EQ(is_allowed_transition(kAll[f], kAll[t]), expected[f][t])
        << guardian::to_string(kAll[f]) << " -> " << guardian::to_string(kAll[t]);
    }
  }
}

TEST(TransitionTable, OutOfRangeStatesAreNeverAllowed)
{
  const auto bogus = static_cast<State>(42);
  for (State s : kAll) {
    EXPECT_FALSE(is_allowed_transition(bogus, s));
    EXPECT_FALSE(is_allowed_transition(s, bogus));
  }
}

TEST(TransitionTable, DecideOnlyEverProducesAllowedTransitions)
{
  // Exhaustive over all 2^9 health combinations and all states.
  for (State s : kAll) {
    for (std::uint32_t bits = 0; bits < (1U << 9U); ++bits) {
      Health h;
      h.doer_alive = (bits & 1U) != 0U;
      h.safety_layer_alive = (bits & 2U) != 0U;
      h.command_fresh = (bits & 4U) != 0U;
      h.vehicle_status_fresh = (bits & 8U) != 0U;
      h.reject_limit_reached = (bits & 16U) != 0U;
      h.internal_fault = (bits & 32U) != 0U;
      h.deadline_missed = (bits & 64U) != 0U;
      h.stopped_for_hold_time = (bits & 128U) != 0U;
      h.operator_reset = (bits & 256U) != 0U;
      const State t = decide(s, h);
      EXPECT_TRUE(is_allowed_transition(s, t))
        << guardian::to_string(s) << " -> " << guardian::to_string(t) << " bits=" << bits;
    }
  }
}

// ------------------------------------------------------------- decide() -----

TEST(Decide, UnknownStateFailsSafeToEmergency)
{
  EXPECT_EQ(decide(static_cast<State>(99), healthy()), State::kMrmEmergency);
}

TEST(Decide, HealthyStaysNormal) { EXPECT_EQ(decide(State::kNormal, healthy()), State::kNormal); }

TEST(Decide, DoerLostGivesDegradedSoAutowareMrmActs)
{
  Health h = healthy();
  h.doer_alive = false;
  EXPECT_EQ(decide(State::kNormal, h), State::kDegraded);
}

TEST(Decide, DegradedRecoversWhenDoerReturns)
{
  EXPECT_EQ(decide(State::kDegraded, healthy()), State::kNormal);
}

TEST(Decide, SafetyLayerLostGivesGuardianComfortableMrm)
{
  for (State s : {State::kNormal, State::kDegraded}) {
    Health h = healthy();
    h.safety_layer_alive = false;
    EXPECT_EQ(decide(s, h), State::kMrmComfortable);
    h = healthy();
    h.command_fresh = false;
    EXPECT_EQ(decide(s, h), State::kMrmComfortable);
    h = healthy();
    h.reject_limit_reached = true;
    EXPECT_EQ(decide(s, h), State::kMrmComfortable);
  }
}

TEST(Decide, EmergencyClassFaults)
{
  for (State s : {State::kNormal, State::kDegraded, State::kMrmComfortable}) {
    Health h = healthy();
    h.internal_fault = true;
    EXPECT_EQ(decide(s, h), State::kMrmEmergency);
    h = healthy();
    h.deadline_missed = true;
    EXPECT_EQ(decide(s, h), State::kMrmEmergency);
    h = healthy();
    h.vehicle_status_fresh = false;
    EXPECT_EQ(decide(s, h), State::kMrmEmergency);
  }
}

TEST(Decide, MrmIsLatchedEvenIfEverythingRecovers)
{
  EXPECT_EQ(decide(State::kMrmComfortable, healthy()), State::kMrmComfortable);
  EXPECT_EQ(decide(State::kMrmEmergency, healthy()), State::kMrmEmergency);
}

TEST(Decide, MrmEndsInStoppedOnlyWithTrustedStandstill)
{
  Health h = healthy();
  h.stopped_for_hold_time = true;
  EXPECT_EQ(decide(State::kMrmComfortable, h), State::kStopped);
  EXPECT_EQ(decide(State::kMrmEmergency, h), State::kStopped);
  h.vehicle_status_fresh = false;  // cannot trust "stopped"
  EXPECT_EQ(decide(State::kMrmComfortable, h), State::kMrmEmergency);
  EXPECT_EQ(decide(State::kMrmEmergency, h), State::kMrmEmergency);
}

TEST(Decide, StandstillInNormalDoesNotStop)
{
  Health h = healthy();
  h.stopped_for_hold_time = true;  // e.g. waiting at a red light
  EXPECT_EQ(decide(State::kNormal, h), State::kNormal);
}

TEST(Decide, StoppedNeedsResetAndFullHealth)
{
  EXPECT_EQ(decide(State::kStopped, healthy()), State::kStopped);  // no reset
  Health h = healthy();
  h.operator_reset = true;
  EXPECT_EQ(decide(State::kStopped, h), State::kNormal);

  // Each missing condition blocks the reset.
  Health bad[7];
  for (Health & b : bad) {
    b = h;
  }
  bad[0].doer_alive = false;
  bad[1].safety_layer_alive = false;
  bad[2].command_fresh = false;
  bad[3].vehicle_status_fresh = false;
  bad[4].internal_fault = true;
  bad[5].deadline_missed = true;
  bad[6].reject_limit_reached = true;
  for (const Health & b : bad) {
    EXPECT_EQ(decide(State::kStopped, b), State::kStopped);
  }
}

TEST(Decide, ResetOutsideStoppedIsIgnored)
{
  Health h = healthy();
  h.operator_reset = true;
  EXPECT_EQ(decide(State::kMrmComfortable, h), State::kMrmComfortable);
  EXPECT_EQ(decide(State::kMrmEmergency, h), State::kMrmEmergency);
}

// --------------------------------------------------------- StateMachine -----

TEST(StateMachine, PowerOnStateIsStopped)
{
  StateMachine sm;
  EXPECT_EQ(sm.state(), State::kStopped);
  sm.step(healthy());  // healthy but no reset: still stopped
  EXPECT_EQ(sm.state(), State::kStopped);
}

TEST(StateMachine, EveryLegalRequestSucceeds)
{
  for (State from : kAll) {
    for (State to : kAll) {
      if (!is_allowed_transition(from, to)) {
        continue;
      }
      if (from == State::kStopped && to != State::kStopped) {
        continue;  // covered by RequestCannotLeaveStopped
      }
      StateMachine sm = at(from);
      EXPECT_TRUE(sm.request(to)) << guardian::to_string(from) << "->" << guardian::to_string(to);
      EXPECT_EQ(sm.state(), to);
      EXPECT_EQ(sm.illegal_requests(), 0UL);
    }
  }
}

TEST(StateMachine, EveryIllegalRequestFailsSafe)
{
  for (State from : kAll) {
    for (State to : kAll) {
      if (is_allowed_transition(from, to)) {
        continue;
      }
      StateMachine sm = at(from);
      EXPECT_FALSE(sm.request(to)) << guardian::to_string(from) << "->" << guardian::to_string(to);
      const State expected = (from == State::kStopped) ? State::kStopped : State::kMrmEmergency;
      EXPECT_EQ(sm.state(), expected);
      EXPECT_EQ(sm.illegal_requests(), 1UL);
    }
  }
}

TEST(StateMachine, RequestCannotLeaveStopped)
{
  for (State to : {State::kNormal, State::kDegraded, State::kMrmComfortable,
                   State::kMrmEmergency}) {
    StateMachine sm;
    EXPECT_FALSE(sm.request(to));
    EXPECT_EQ(sm.state(), State::kStopped);
  }
}

TEST(StateMachine, BogusRequestFailsSafe)
{
  StateMachine sm = at(State::kNormal);
  EXPECT_FALSE(sm.request(static_cast<State>(7)));
  EXPECT_EQ(sm.state(), State::kMrmEmergency);
}

TEST(StateMachine, FullLifecycleCountsTransitions)
{
  StateMachine sm;
  Health h = healthy();
  h.operator_reset = true;
  sm.step(h);
  EXPECT_EQ(sm.state(), State::kNormal);
  h = healthy();
  h.doer_alive = false;
  sm.step(h);
  EXPECT_EQ(sm.state(), State::kDegraded);
  h.safety_layer_alive = false;
  sm.step(h);
  EXPECT_EQ(sm.state(), State::kMrmComfortable);
  h.deadline_missed = true;
  sm.step(h);
  EXPECT_EQ(sm.state(), State::kMrmEmergency);
  h = healthy();
  h.stopped_for_hold_time = true;
  sm.step(h);
  EXPECT_EQ(sm.state(), State::kStopped);
  EXPECT_EQ(sm.transitions(), 5UL);
  EXPECT_EQ(sm.illegal_requests(), 0UL);
}

TEST(StateMachine, ToStringCoversAllStates)
{
  EXPECT_STREQ(guardian::to_string(State::kNormal), "NORMAL");
  EXPECT_STREQ(guardian::to_string(State::kDegraded), "DEGRADED");
  EXPECT_STREQ(guardian::to_string(State::kMrmComfortable), "MRM_COMFORTABLE");
  EXPECT_STREQ(guardian::to_string(State::kMrmEmergency), "MRM_EMERGENCY");
  EXPECT_STREQ(guardian::to_string(State::kStopped), "STOPPED");
  EXPECT_STREQ(guardian::to_string(static_cast<State>(99)), "UNKNOWN");
}
