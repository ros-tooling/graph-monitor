// SPDX-FileCopyrightText: 2024 Bonsai Robotics, Inc.
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "topic_stats_ros/stats_message.hpp"

#include <chrono>
#include <cstdint>
#include <utility>

namespace topic_stats_ros
{

namespace
{

constexpr int64_t kNanosecondsPerSecond = 1000 * 1000 * 1000;

/// Splits nanoseconds into the (int32 sec, uint32 nanosec) pair the ROS time messages use.
///
/// Floors rather than truncates, because the nanosecond field is unsigned: -0.5s is (-1, 5e8), not
/// (0, -5e8). This matters for take age, which goes negative whenever publisher and subscriber
/// clocks disagree. Note that rmw_time_from_nsec cannot be used here, as rmw_time_t's fields are
/// both unsigned and it turns any negative input into an enormous positive duration.
void split_nanoseconds(int64_t nanoseconds, int32_t & seconds, uint32_t & remainder)
{
  int64_t whole = nanoseconds / kNanosecondsPerSecond;
  int64_t fraction = nanoseconds % kNanosecondsPerSecond;
  if (fraction < 0) {
    whole -= 1;
    fraction += kNanosecondsPerSecond;
  }
  seconds = static_cast<int32_t>(whole);
  remainder = static_cast<uint32_t>(fraction);
}

builtin_interfaces::msg::Duration duration_message(topic_stats_core::Duration duration)
{
  builtin_interfaces::msg::Duration msg;
  split_nanoseconds(duration.count(), msg.sec, msg.nanosec);
  return msg;
}

builtin_interfaces::msg::Time time_message(topic_stats_core::SysTime time)
{
  const auto since_epoch = std::chrono::duration_cast<topic_stats_core::Duration>(time.time_since_epoch());
  builtin_interfaces::msg::Time msg;
  split_nanoseconds(since_epoch.count(), msg.sec, msg.nanosec);
  return msg;
}

}  // namespace

bool to_statistic_type(topic_stats_core::StatKind kind, uint8_t & statistic_type)
{
  using rosgraph_monitor_msgs::msg::TopicStatistic;
  switch (kind) {
    case topic_stats_core::StatKind::PublishedPeriod:
      statistic_type = TopicStatistic::PUBLISHED_PERIOD;
      return true;
    case topic_stats_core::StatKind::ReceivedPeriod:
      statistic_type = TopicStatistic::RECEIVED_PERIOD;
      return true;
    case topic_stats_core::StatKind::TakeAge:
      statistic_type = TopicStatistic::TAKE_AGE;
      return true;
    case topic_stats_core::StatKind::CallbackDuration:
      // No constant for this yet. The RMW ingest cannot observe callbacks anyway.
      return false;
  }
  return false;
}

rosgraph_monitor_msgs::msg::TopicStatistics to_message(
  const topic_stats_core::NodeReport & node_report, topic_stats_core::SysTime timestamp)
{
  rosgraph_monitor_msgs::msg::TopicStatistics msg;
  msg.timestamp = time_message(timestamp);
  msg.statistics.reserve(node_report.samples.size());

  for (const auto & sample : node_report.samples) {
    uint8_t statistic_type = 0;
    if (!to_statistic_type(sample.stat, statistic_type)) {
      continue;
    }
    rosgraph_monitor_msgs::msg::TopicStatistic stat;
    stat.statistic_type = statistic_type;
    stat.node_name = sample.node_name;
    stat.topic_name = sample.topic_name;
    stat.window_count = static_cast<int32_t>(sample.window_count);
    stat.mean = duration_message(sample.mean);
    stat.min = duration_message(sample.min);
    stat.max = duration_message(sample.max);
    msg.statistics.push_back(std::move(stat));
  }
  return msg;
}

}  // namespace topic_stats_ros
