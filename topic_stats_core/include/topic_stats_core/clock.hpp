// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef TOPIC_STATS_CORE__CLOCK_HPP_
#define TOPIC_STATS_CORE__CLOCK_HPP_

#include <mutex>

#include "topic_stats_core/types.hpp"

namespace topic_stats_core
{

/// Source of time for the collector.
///
/// The collector samples time itself rather than accepting timestamps from ingest adapters, which
/// keeps the Recorder interface trivial on the hot path. Injecting the clock is what buys back
/// determinism in tests, and is also the seam an offline trace replay would drive.
///
/// Two clocks are needed, not one. Periods must come from a monotonic clock so that a system time
/// step does not manufacture an enormous or negative measurement, but take age has to be measured
/// against the publisher's system-clock source timestamp.
class Clock
{
public:
  virtual ~Clock() = default;
  virtual MonoTime monotonic() const = 0;
  virtual SysTime system() const = 0;
};

/// Real time.
class SystemClock : public Clock
{
public:
  MonoTime monotonic() const override
  {
    return MonoClock::now();
  }

  SysTime system() const override
  {
    return SysClock::now();
  }
};

/// Test and replay clock. Time only moves when told to.
class ManualClock : public Clock
{
public:
  MonoTime monotonic() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return mono_;
  }

  SysTime system() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return sys_;
  }

  /// Advance both clocks by the same amount.
  void advance(Duration by)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    mono_ += by;
    sys_ += by;
  }

  /// Move the system clock without moving the monotonic clock, to exercise the reason there are
  /// two of them.
  void advance_system_only(Duration by)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sys_ += by;
  }

  void set_system(SysTime to)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sys_ = to;
  }

private:
  mutable std::mutex mutex_;
  MonoTime mono_{};
  SysTime sys_{};
};

}  // namespace topic_stats_core

#endif  // TOPIC_STATS_CORE__CLOCK_HPP_
