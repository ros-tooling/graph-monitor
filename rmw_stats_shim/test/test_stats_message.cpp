// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <utility>

#include "rmw_stats_shim/stats_message.hpp"

using rmw_stats_shim::to_message;
using rmw_stats_shim::to_statistic_type;
using rosgraph_monitor_msgs::msg::TopicStatistic;
using topic_stats_core::Duration;
using topic_stats_core::NodeReport;
using topic_stats_core::StatKind;
using topic_stats_core::StatSample;
using topic_stats_core::SysTime;

namespace
{

Duration ms(int64_t count)
{
  return std::chrono::duration_cast<Duration>(std::chrono::milliseconds(count));
}

StatSample sample(StatKind stat, Duration mean, Duration min, Duration max)
{
  StatSample out;
  out.stat = stat;
  out.node_name = "/ns/talker";
  out.topic_name = "/chatter";
  out.window_count = 7;
  out.mean = mean;
  out.min = min;
  out.max = max;
  return out;
}

NodeReport report_with(StatSample one)
{
  NodeReport report;
  report.node_name = "/ns/talker";
  report.samples.push_back(std::move(one));
  return report;
}

}  // namespace

TEST(StatisticType, core_kinds_map_onto_message_constants)
{
  uint8_t type = 255;
  ASSERT_TRUE(to_statistic_type(StatKind::PublishedPeriod, type));
  EXPECT_EQ(type, TopicStatistic::PUBLISHED_PERIOD);
  ASSERT_TRUE(to_statistic_type(StatKind::ReceivedPeriod, type));
  EXPECT_EQ(type, TopicStatistic::RECEIVED_PERIOD);
  ASSERT_TRUE(to_statistic_type(StatKind::TakeAge, type));
  EXPECT_EQ(type, TopicStatistic::TAKE_AGE);
}

TEST(StatisticType, statistics_the_message_cannot_express_are_dropped_not_mangled)
{
  // The core can measure more than this message carries. Silently reusing another constant would
  // corrupt the monitor's deadline checks.
  uint8_t type = 255;
  EXPECT_FALSE(to_statistic_type(StatKind::CallbackDuration, type));
}

TEST(ToMessage, carries_identity_window_and_all_three_statistics)
{
  const auto msg = to_message(report_with(sample(StatKind::PublishedPeriod, ms(100), ms(90), ms(120))), SysTime{});
  ASSERT_EQ(msg.statistics.size(), 1u);
  const auto & stat = msg.statistics.front();
  EXPECT_EQ(stat.statistic_type, TopicStatistic::PUBLISHED_PERIOD);
  EXPECT_EQ(stat.node_name, "/ns/talker");
  EXPECT_EQ(stat.topic_name, "/chatter");
  EXPECT_EQ(stat.window_count, 7);
  EXPECT_EQ(stat.mean.sec, 0);
  EXPECT_EQ(stat.mean.nanosec, 100u * 1000u * 1000u);
  EXPECT_EQ(stat.min.nanosec, 90u * 1000u * 1000u);
  EXPECT_EQ(stat.max.nanosec, 120u * 1000u * 1000u);
}

TEST(ToMessage, durations_over_a_second_split_correctly)
{
  const auto msg = to_message(report_with(sample(StatKind::ReceivedPeriod, ms(2500), ms(2500), ms(2500))), SysTime{});
  ASSERT_EQ(msg.statistics.size(), 1u);
  EXPECT_EQ(msg.statistics.front().mean.sec, 2);
  EXPECT_EQ(msg.statistics.front().mean.nanosec, 500u * 1000u * 1000u);
}

TEST(ToMessage, negative_take_age_survives_the_conversion)
{
  // Take age goes negative when publisher and subscriber clocks disagree. The nanosec field is
  // unsigned, so the seconds must floor: -0.5s is (-1, 5e8). Truncating instead, or routing this
  // through rmw_time_from_nsec whose fields are both unsigned, turns clock skew into an enormous
  // positive latency.
  const auto msg = to_message(report_with(sample(StatKind::TakeAge, ms(-500), ms(-500), ms(-500))), SysTime{});
  ASSERT_EQ(msg.statistics.size(), 1u);
  const auto & mean = msg.statistics.front().mean;
  EXPECT_EQ(mean.sec, -1);
  EXPECT_EQ(mean.nanosec, 500u * 1000u * 1000u);
}

TEST(ToMessage, exactly_negative_whole_seconds_do_not_borrow)
{
  const auto msg = to_message(report_with(sample(StatKind::TakeAge, ms(-2000), ms(-2000), ms(-2000))), SysTime{});
  ASSERT_EQ(msg.statistics.size(), 1u);
  EXPECT_EQ(msg.statistics.front().mean.sec, -2);
  EXPECT_EQ(msg.statistics.front().mean.nanosec, 0u);
}

TEST(ToMessage, timestamp_comes_from_the_snapshot)
{
  const auto msg =
    to_message(report_with(sample(StatKind::PublishedPeriod, ms(1), ms(1), ms(1))), SysTime{} + ms(1500));
  EXPECT_EQ(msg.timestamp.sec, 1);
  EXPECT_EQ(msg.timestamp.nanosec, 500u * 1000u * 1000u);
}

TEST(ToMessage, a_report_of_only_inexpressible_statistics_yields_no_statistics)
{
  // The sink checks for this and skips publishing rather than emitting an empty message.
  const auto msg = to_message(report_with(sample(StatKind::CallbackDuration, ms(5), ms(5), ms(5))), SysTime{});
  EXPECT_TRUE(msg.statistics.empty());
}
