// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef TOPIC_STATS_CORE__TESTING__RECORDING_SINK_HPP_
#define TOPIC_STATS_CORE__TESTING__RECORDING_SINK_HPP_

#include <mutex>
#include <vector>

#include "topic_stats_core/sink.hpp"
#include "topic_stats_core/types.hpp"

namespace topic_stats_core::testing
{

/// Keeps every report it is handed, for tests to assert against.
///
/// Lives in the public headers rather than a test directory so that ingest and egress packages can
/// use it without depending on this package's test sources.
class RecordingSink : public StatsSink
{
public:
  void publish(const StatsReport & report) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    reports_.push_back(report);
  }

  std::vector<StatsReport> reports() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return reports_;
  }

  size_t count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return reports_.size();
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    reports_.clear();
  }

private:
  mutable std::mutex mutex_;
  std::vector<StatsReport> reports_;
};

}  // namespace topic_stats_core::testing

#endif  // TOPIC_STATS_CORE__TESTING__RECORDING_SINK_HPP_
