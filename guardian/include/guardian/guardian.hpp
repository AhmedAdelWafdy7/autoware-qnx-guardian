// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0
//
// One Guardian cycle: heartbeats -> validation -> state machine -> final command.
// Platform-independent; the QNX (timer + pulse) and POSIX (clock_nanosleep) loops in
// src/platform/ call step() once per period. Allocation-free and noexcept after construction.
//
// Arbitration (FR-MRM-4, see docs/safety-concept.md):
//   NORMAL / DEGRADED   the ported Autoware safety layer drives; its command is validated.
//                       In DEGRADED that command *is* Autoware's MRM (mrm_handler on QNX).
//                       On a single REJECT the previous output is held for one cycle.
//   MRM_* / STOPPED     the Guardian ignores the safety layer and outputs its own command.
// The Guardian is the only publisher of the final vehicle command (FR-OUT-1), so the two
// layers can never command the vehicle at the same time.

#ifndef GUARDIAN__GUARDIAN_HPP_
#define GUARDIAN__GUARDIAN_HPP_

#include <cstdint>

#include "guardian/event_log.hpp"
#include "guardian/heartbeat_monitor.hpp"
#include "guardian/state_machine.hpp"
#include "guardian/types.hpp"
#include "guardian/validator.hpp"

namespace guardian
{

inline constexpr std::size_t kEventLogCapacity = 1024U;
using GuardianEventLog = EventLog<kEventLogCapacity>;

/// Everything the Guardian sees in one cycle (latest values handed over by the bridge).
struct CycleInputs
{
  TimeNs now_ns{0};
  HeartbeatSample doer{};
  HeartbeatSample safety_layer{};
  bool have_command{false};
  Command command{};  ///< latest command from the ported safety layer (vehicle_cmd_gate out)
  bool have_status{false};
  VehicleStatus status{};
  bool operator_reset{false};
  bool deadline_missed{false};  ///< set by the platform loop
  bool internal_fault{false};   ///< set by the platform loop / self checks
};

/// Who produced the final command this cycle.
enum class Actor : std::uint8_t
{
  kSafetyLayer = 0,   ///< validated command from the ported Autoware safety layer
  kGuardianHold = 1,  ///< previous output held after a REJECT
  kGuardianMrm = 2,   ///< Guardian built-in MRM / standstill hold
};

struct CycleOutput
{
  Command command{};
  State state{State::kStopped};
  Verdict verdict{Verdict::kReject};
  std::uint32_t reasons{reason::kNone};
  Actor actor{Actor::kGuardianMrm};
};

class Guardian
{
public:
  /// @param log optional event log (may be nullptr); must outlive the Guardian.
  explicit Guardian(GuardianEventLog * log = nullptr) noexcept : log_(log) {}

  CycleOutput step(const CycleInputs & in) noexcept;

  State state() const noexcept { return sm_.state(); }
  const HeartbeatMonitor & heartbeats() const noexcept { return hb_; }
  std::uint32_t reject_streak() const noexcept { return reject_streak_; }

private:
  Command mrm_command(State s, TimeNs now_ns) const noexcept;
  void log(TimeNs t, EventCode code, std::uint8_t a, std::uint8_t b, std::uint32_t reasons,
           std::int64_t value) noexcept;

  GuardianEventLog * log_;
  HeartbeatMonitor hb_{};
  StateMachine sm_{};
  Command last_out_{};
  bool have_last_out_{false};
  std::uint32_t reject_streak_{0};
  bool still_{false};
  TimeNs still_since_ns_{0};
  bool doer_was_alive_{false};
  bool safety_was_alive_{false};
};

}  // namespace guardian

#endif  // GUARDIAN__GUARDIAN_HPP_
