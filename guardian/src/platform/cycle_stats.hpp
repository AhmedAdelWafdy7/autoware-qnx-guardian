// Copyright 2026 Ahmed Wafdy
// SPDX-License-Identifier: Apache-2.0
//
// Wake-up latency statistics for the Guardian loop (NFR-T-1, NFR-T-3). Fixed-size, no
// allocation: safe to update from the RT cycle. Identical on QNX and Linux so the
// benchmark compares like with like.

#ifndef GUARDIAN__PLATFORM__CYCLE_STATS_HPP_
#define GUARDIAN__PLATFORM__CYCLE_STATS_HPP_

#include <cstdint>
#include <cstdio>

namespace guardian::platform
{

class CycleStats
{
public:
  static constexpr std::uint32_t kBuckets = 2000U;  ///< 1 us buckets, 0..1999 us
  static constexpr std::int64_t kBucket_ns = 1000;

  /// Record lateness = actual wake time - deadline [ns] (negative clamps to 0).
  void record(std::int64_t lateness_ns) noexcept
  {
    if (lateness_ns < 0) {
      lateness_ns = 0;
    }
    ++count_;
    if (lateness_ns > max_ns_) {
      max_ns_ = lateness_ns;
    }
    sum_ns_ += lateness_ns;
    const std::int64_t b = lateness_ns / kBucket_ns;
    if (b >= static_cast<std::int64_t>(kBuckets)) {
      ++overflow_;
    } else {
      ++hist_[b];
    }
  }

  /// Percentile in microseconds (upper bucket edge). Not RT: call after the run.
  std::int64_t percentile_us(double p) const noexcept
  {
    if (count_ == 0U) {
      return 0;
    }
    const auto target = static_cast<std::uint64_t>(p * static_cast<double>(count_));
    std::uint64_t acc = 0U;
    for (std::uint32_t i = 0U; i < kBuckets; ++i) {
      acc += hist_[i];
      if (acc > target) {
        return static_cast<std::int64_t>(i) + 1;
      }
    }
    return max_ns_ / 1000;
  }

  void print(std::FILE * f, const char * label) const noexcept
  {
    std::fprintf(
      f,
      "%s cycles=%llu mean_us=%.2f p50_us=%lld p99_us=%lld p99.9_us=%lld max_us=%.1f "
      "over_2ms=%llu\n",
      label, static_cast<unsigned long long>(count_),
      count_ ? static_cast<double>(sum_ns_) / static_cast<double>(count_) / 1000.0 : 0.0,
      static_cast<long long>(percentile_us(0.50)), static_cast<long long>(percentile_us(0.99)),
      static_cast<long long>(percentile_us(0.999)), static_cast<double>(max_ns_) / 1000.0,
      static_cast<unsigned long long>(overflow_));
  }

private:
  std::uint64_t hist_[kBuckets]{};
  std::uint64_t count_{0};
  std::uint64_t overflow_{0};
  std::int64_t sum_ns_{0};
  std::int64_t max_ns_{0};
};

}  // namespace guardian::platform

#endif  // GUARDIAN__PLATFORM__CYCLE_STATS_HPP_
