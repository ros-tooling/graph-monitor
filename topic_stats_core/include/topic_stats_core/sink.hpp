// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef TOPIC_STATS_CORE__SINK_HPP_
#define TOPIC_STATS_CORE__SINK_HPP_

#include "topic_stats_core/types.hpp"

namespace topic_stats_core
{

/// Everything an egress adapter is allowed to do.
///
/// Sinks receive plain structs, never ROS messages, so that the message contract stays confined to
/// the one sink that speaks it. Called from whatever thread drives reporting, one call at a time.
///
/// A sink must not block for long and must not throw. It is called from a context that may be
/// inside the middleware's own call stack, depending on the ingest adapter in use.
class StatsSink
{
public:
  virtual ~StatsSink() = default;
  virtual void publish(const StatsReport & report) = 0;
};

/// Discards everything. Useful for measuring ingest overhead in isolation.
class NullSink : public StatsSink
{
public:
  void publish(const StatsReport &) override
  {}
};

}  // namespace topic_stats_core

#endif  // TOPIC_STATS_CORE__SINK_HPP_
