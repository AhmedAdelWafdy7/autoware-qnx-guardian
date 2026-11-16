// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0
//
// One Guardian cycle: heartbeats -> validation -> RSS + RTA Switch -> state machine ->
// final command.
// Platform-independent; the QNX (timer + pulse) and POSIX (clock_nanosleep) loops in
// src/platform/ call step() once per period. Allocation-free and noexcept after construction.
//
// Arbitration (see docs/safety-concept.md):
//   NORMAL / DEGRADED   the ported Autoware safety layer drives; its command is validated.
//                       In DEGRADED that command *is* Autoware's MRM (mrm_handler on QNX).
//                       On a single REJECT the previous output is held for one cycle.
//   MRM_* / STOPPED     the Guardian ignores the safety layer and outputs its own command.
//   RTA Switch          in NORMAL / DEGRADED, RSS UNSAFE on the independent channel switches to
//                       Recovery: the safety layer's (validated) steering is kept, the
//                       longitudinal command becomes a step brake (limits::kRtaRecoveryDecel).
//                       Back to the Complex Function after the hysteresis window (rta_switch.hpp).
//   RSS envelope        every output, whichever actor produced it, is clamped to this cycle's
//                       RSS acceleration bounds (no ramp), so an RSS-required brake is never
//                       softened by the MRM jerk ramp or a held command.
// The Guardian is the only publisher of the final vehicle command, so the two
// layers can never command the vehicle at the same time.

#ifndef GUARDIAN__GUARDIAN_HPP_
#define GUARDIAN__GUARDIAN_HPP_

#include <cstdint>

#include "guardian/event_log.hpp"
#include "guardian/heartbeat_monitor.hpp"
#include "guardian/rss/rss.hpp"
#include "guardian/rta_switch.hpp"
#include "guardian/state_machine.hpp"
#include "guardian/types.hpp"
#include "guardian/validator.hpp"

namespace guardian
{

inline constexpr std::size_t kEventLogCapacity = 1024U;
using GuardianEventLog = EventLog<kEventLogCapacity>;

/// Independent channel (AWSIM -> QNX, bypassing Autoware perception), in the ego frame.
/// The bridge keeps at most rss::params::kMaxObjects (the nearest ones) and sets count.
struct IndependentWorld
{
  TimeNs rx_time_ns{0};    ///< receive time on the Guardian host
  rss::EgoState ego{};     ///< ego speed from the independent odometry + footprint (config)
  std::uint32_t count{0};  ///< valid entries in objects[]; > kMaxObjects is UNSAFE
  rss::Object objects[rss::params::kMaxObjects]{};
};

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
  bool have_world{false};
  IndependentWorld world{};
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
  kGuardianRss = 3,   ///< RTA Switch in Recovery: RSS proper-response brake
};

struct CycleOutput
{
  Command command{};
  State state{State::kStopped};
  Verdict verdict{Verdict::kReject};
  std::uint32_t reasons{reason::kNone};
  Actor actor{Actor::kGuardianMrm};
  rss::Outcome rss_outcome{rss::Outcome::kUnsafe};
  std::uint32_t rss_reasons{rss::reason::kNone};
  RtaMode rta_mode{RtaMode::kComplexFunction};
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
  const RtaSwitch & rta() const noexcept { return rta_; }

private:
  Command mrm_command(State s, TimeNs now_ns) const noexcept;
  rss::Result evaluate_rss(const CycleInputs & in, bool world_fresh,
                           double commanded_accel_mps2) noexcept;
  void log(TimeNs t, EventCode code, std::uint8_t a, std::uint8_t b, std::uint32_t reasons,
           std::int64_t value) noexcept;

  GuardianEventLog * log_;
  HeartbeatMonitor hb_{};
  StateMachine sm_{};
  RtaSwitch rta_{};
  rss::Situation situations_[rss::params::kMaxObjects]{};  ///< scratch, no allocation
  rss::Outcome last_rss_outcome_{rss::Outcome::kSafe};
  std::uint32_t last_rss_reasons_{rss::reason::kNone};
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
