// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "topic_stats_collector/segment_pool.hpp"
#include "topic_stats_core/clock.hpp"
#include "topic_stats_core/collector.hpp"
#include "topic_stats_core/testing/recording_sink.hpp"
#include "topic_stats_ros/stats_message.hpp"
#include "topic_stats_shm/shm_sink.hpp"

using rosgraph_monitor_msgs::msg::TopicStatistics;
using topic_stats_core::Collector;
using topic_stats_core::Duration;
using topic_stats_core::EndpointDescriptor;
using topic_stats_core::EndpointKind;
using topic_stats_core::ManualClock;
using topic_stats_core::NodeDescriptor;
using topic_stats_core::SnapshotMode;
using topic_stats_core::StatsReport;

namespace tsc = topic_stats_collector;
namespace tss = topic_stats_shm;

namespace
{

Duration ms(int64_t count)
{
  return std::chrono::duration_cast<Duration>(std::chrono::milliseconds(count));
}

std::atomic<int64_t> g_pid_counter{9500000};

/// Drives a real Collector so that the reports under test are the ones the system actually
/// produces, rather than hand-built structs that could quietly diverge from them.
StatsReport realistic_snapshot(Collector & collector, ManualClock & clock, SnapshotMode mode)
{
  const auto node = collector.register_node(NodeDescriptor{"/ns/talker"});
  const auto pub = collector.register_endpoint(node, EndpointDescriptor{EndpointKind::Publisher, "/chatter", {}});
  const auto sub = collector.register_endpoint(node, EndpointDescriptor{EndpointKind::Subscription, "/incoming", {}});
  const auto other = collector.register_node(NodeDescriptor{"/ns/listener"});
  const auto other_sub =
    collector.register_endpoint(other, EndpointDescriptor{EndpointKind::Subscription, "/chatter", {}});

  clock.set_system(topic_stats_core::SysTime{} + ms(10000));
  collector.record_publish(pub);
  collector.record_take(sub, ms(9800).count());
  collector.record_take(other_sub, std::nullopt);
  clock.advance(ms(100));
  collector.record_publish(pub);
  collector.record_take(sub, ms(11000).count());  // a take age that lands negative, from clock skew
  collector.record_take(other_sub, std::nullopt);

  return collector.snapshot(mode);
}

std::vector<TopicStatistics> messages_from(const StatsReport & report)
{
  std::vector<TopicStatistics> messages;
  for (const auto & node_report : report.nodes) {
    auto msg = topic_stats_ros::to_message(node_report, report.timestamp);
    if (!msg.statistics.empty()) {
      messages.push_back(std::move(msg));
    }
  }
  return messages;
}

/// Flattens every statistic out of a set of messages into a comparable, order-independent form.
std::vector<std::string> flatten(const std::vector<TopicStatistics> & messages)
{
  std::vector<std::string> lines;
  for (const auto & msg : messages) {
    for (const auto & stat : msg.statistics) {
      lines.push_back(
        std::to_string(msg.timestamp.sec) + "." + std::to_string(msg.timestamp.nanosec) + "|" +
        std::to_string(stat.statistic_type) + "|" + stat.node_name + "|" + stat.topic_name + "|" +
        std::to_string(stat.window_count) + "|" + std::to_string(stat.mean.sec) + "." +
        std::to_string(stat.mean.nanosec) + "|" + std::to_string(stat.min.sec) + "." +
        std::to_string(stat.min.nanosec) + "|" + std::to_string(stat.max.sec) + "." + std::to_string(stat.max.nanosec));
    }
  }
  std::sort(lines.begin(), lines.end());
  return lines;
}

}  // namespace

// The whole reason the statistics core, the ingest adapters and the egress adapters are separate:
// swapping the egress must not change what a consumer sees. This drives one real Collector, sends
// the identical snapshot down both paths, and requires the resulting messages to match exactly.
TEST(EgressEquivalence, shared_memory_delivers_what_the_publisher_would_have)
{
  auto clock = std::make_shared<ManualClock>();
  Collector collector(Collector::Options{50}, clock);

  // Taken twice in All mode so both paths get the same content: OnlyChanged consumes.
  const auto snapshot = realistic_snapshot(collector, *clock, SnapshotMode::All);
  ASSERT_FALSE(snapshot.empty());

  // What the publishing egress would put on the wire. It converts each node report with exactly
  // this call, so this is the reference without needing a middleware to observe it.
  const auto expected = flatten(messages_from(snapshot));
  // Two nodes, a published period, a received period, a take age, and a second received period.
  // Pinned so the comparison below cannot pass by trivially matching one lonely sample.
  ASSERT_EQ(expected.size(), 4u);

  // The same snapshot through shared memory and back out of a collector.
  tss::ProcessIdentity identity = tss::current_process_identity();
  identity.pid = g_pid_counter.fetch_add(1);
  tss::SharedMemorySink::Options sink_options;
  sink_options.segment_name = tss::segment_name_for(identity);
  std::string error;
  auto sink = tss::SharedMemorySink::create(sink_options, error);
  ASSERT_NE(sink, nullptr) << error;

  tsc::SegmentPool::Options pool_options;
  pool_options.skip_backlog_on_attach = false;
  pool_options.reclaim_dead_segments = false;
  pool_options.is_alive = [](const std::string &) { return true; };
  tsc::SegmentPool pool(pool_options);

  sink->publish(snapshot);

  std::vector<TopicStatistics> collected;
  const auto result = pool.poll();
  for (const auto & report : result.reports) {
    for (auto & msg : messages_from(report)) {
      collected.push_back(std::move(msg));
    }
  }

  EXPECT_EQ(result.lapped, 0u);
  EXPECT_EQ(result.torn, 0u);
  EXPECT_EQ(sink->unrepresentable_samples(), 0u);
  EXPECT_EQ(sink->truncated_names(), 0u);
  EXPECT_EQ(flatten(collected), expected);

  std::string ignored;
  tss::Segment::unlink(sink_options.segment_name, ignored);
}

TEST(EgressEquivalence, the_shared_memory_path_preserves_negative_take_age)
{
  // Worth its own assertion because it crosses three representations: a signed chrono duration, a
  // signed wire field, and a message whose nanosecond component is unsigned.
  auto clock = std::make_shared<ManualClock>();
  Collector collector(Collector::Options{50}, clock);
  const auto snapshot = realistic_snapshot(collector, *clock, SnapshotMode::All);

  tss::ProcessIdentity identity = tss::current_process_identity();
  identity.pid = g_pid_counter.fetch_add(1);
  tss::SharedMemorySink::Options sink_options;
  sink_options.segment_name = tss::segment_name_for(identity);
  std::string error;
  auto sink = tss::SharedMemorySink::create(sink_options, error);
  ASSERT_NE(sink, nullptr) << error;

  tsc::SegmentPool::Options pool_options;
  pool_options.skip_backlog_on_attach = false;
  pool_options.reclaim_dead_segments = false;
  pool_options.is_alive = [](const std::string &) { return true; };
  tsc::SegmentPool pool(pool_options);

  sink->publish(snapshot);
  const auto result = pool.poll();

  bool saw_negative = false;
  for (const auto & report : result.reports) {
    for (const auto & node : report.nodes) {
      for (const auto & sample : node.samples) {
        if (sample.stat == topic_stats_core::StatKind::TakeAge && sample.min < Duration{0}) {
          saw_negative = true;
        }
      }
    }
  }
  EXPECT_TRUE(saw_negative) << "a negative take age did not survive the shared memory crossing";

  std::string ignored;
  tss::Segment::unlink(sink_options.segment_name, ignored);
}
