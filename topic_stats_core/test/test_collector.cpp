// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "topic_stats_core/clock.hpp"
#include "topic_stats_core/collector.hpp"
#include "topic_stats_core/types.hpp"

using topic_stats_core::Collector;
using topic_stats_core::Duration;
using topic_stats_core::EndpointDescriptor;
using topic_stats_core::EndpointId;
using topic_stats_core::EndpointKind;
using topic_stats_core::ManualClock;
using topic_stats_core::NodeDescriptor;
using topic_stats_core::NodeId;
using topic_stats_core::SnapshotMode;
using topic_stats_core::SourceTimestamp;
using topic_stats_core::StatKind;
using topic_stats_core::StatSample;
using topic_stats_core::StatsReport;

namespace
{

Duration ms(int64_t count)
{
  return std::chrono::duration_cast<Duration>(std::chrono::milliseconds(count));
}

SourceTimestamp source_at_ms(int64_t count)
{
  return ms(count).count();
}

std::optional<StatSample> find_sample(
  const StatsReport & report, const std::string & node_name, const std::string & topic_name, StatKind stat)
{
  for (const auto & node : report.nodes) {
    if (node.node_name != node_name) {
      continue;
    }
    for (const auto & sample : node.samples) {
      if (sample.topic_name == topic_name && sample.stat == stat) {
        return sample;
      }
    }
  }
  return std::nullopt;
}

size_t total_samples(const StatsReport & report)
{
  size_t count = 0;
  for (const auto & node : report.nodes) {
    count += node.samples.size();
  }
  return count;
}

/// Collector with a manual clock, a node, and one publisher and one subscription on it.
class CollectorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    clock = std::make_shared<ManualClock>();
    collector = std::make_unique<Collector>(Collector::Options{window_size}, clock);
    node = collector->register_node(NodeDescriptor{"/ns/talker"});
    pub = collector->register_endpoint(node, EndpointDescriptor{EndpointKind::Publisher, "/chatter", {}});
    sub = collector->register_endpoint(node, EndpointDescriptor{EndpointKind::Subscription, "/incoming", {}});
  }

  size_t window_size = 50;
  std::shared_ptr<ManualClock> clock;
  std::unique_ptr<Collector> collector;
  NodeId node;
  EndpointId pub;
  EndpointId sub;
};

}  // namespace

TEST_F(CollectorTest, registration_alone_produces_no_report)
{
  const auto report = collector->snapshot();
  EXPECT_TRUE(report.empty());
  EXPECT_EQ(collector->diagnostics().live_nodes, 1u);
  EXPECT_EQ(collector->diagnostics().live_endpoints, 2u);
}

TEST_F(CollectorTest, first_publish_yields_no_period)
{
  // One event is a baseline, not an interval.
  collector->record_publish(pub);
  EXPECT_TRUE(collector->snapshot().empty());
}

TEST_F(CollectorTest, publish_period_is_the_interval_between_publishes)
{
  collector->record_publish(pub);
  clock->advance(ms(100));
  collector->record_publish(pub);

  const auto report = collector->snapshot();
  const auto sample = find_sample(report, "/ns/talker", "/chatter", StatKind::PublishedPeriod);
  ASSERT_TRUE(sample.has_value());
  EXPECT_EQ(sample->mean, ms(100));
  EXPECT_EQ(sample->min, ms(100));
  EXPECT_EQ(sample->max, ms(100));
  EXPECT_EQ(sample->window_count, 1u);
  EXPECT_EQ(sample->node_name, "/ns/talker");
}

TEST_F(CollectorTest, period_averages_over_the_window)
{
  collector->record_publish(pub);
  for (const auto interval : {ms(100), ms(200), ms(300)}) {
    clock->advance(interval);
    collector->record_publish(pub);
  }

  const auto sample = find_sample(collector->snapshot(), "/ns/talker", "/chatter", StatKind::PublishedPeriod);
  ASSERT_TRUE(sample.has_value());
  EXPECT_EQ(sample->window_count, 3u);
  EXPECT_EQ(sample->mean, ms(200));
  EXPECT_EQ(sample->min, ms(100));
  EXPECT_EQ(sample->max, ms(300));
}

TEST_F(CollectorTest, subscription_reports_received_period)
{
  collector->record_take(sub, std::nullopt);
  clock->advance(ms(40));
  collector->record_take(sub, std::nullopt);

  const auto report = collector->snapshot();
  EXPECT_TRUE(find_sample(report, "/ns/talker", "/incoming", StatKind::ReceivedPeriod).has_value());
  EXPECT_FALSE(find_sample(report, "/ns/talker", "/incoming", StatKind::PublishedPeriod).has_value());
}

TEST_F(CollectorTest, only_changed_snapshot_consumes_measurements)
{
  collector->record_publish(pub);
  clock->advance(ms(100));
  collector->record_publish(pub);

  EXPECT_EQ(total_samples(collector->snapshot()), 1u);
  // Nothing new arrived, so an endpoint that has gone quiet stops being reported. Downstream
  // staleness detection depends on this.
  EXPECT_TRUE(collector->snapshot().empty());

  clock->advance(ms(100));
  collector->record_publish(pub);
  EXPECT_EQ(total_samples(collector->snapshot()), 1u);
}

TEST_F(CollectorTest, all_mode_reports_statistics_that_have_not_changed)
{
  collector->record_publish(pub);
  clock->advance(ms(100));
  collector->record_publish(pub);
  ASSERT_EQ(total_samples(collector->snapshot(SnapshotMode::OnlyChanged)), 1u);

  const auto all = collector->snapshot(SnapshotMode::All);
  const auto sample = find_sample(all, "/ns/talker", "/chatter", StatKind::PublishedPeriod);
  ASSERT_TRUE(sample.has_value());
  EXPECT_EQ(sample->mean, ms(100));
}

TEST_F(CollectorTest, all_mode_still_omits_statistics_with_no_measurements)
{
  // The subscription exists but has never received anything. Reporting a zero for it would look
  // like a real measurement.
  collector->record_publish(pub);
  clock->advance(ms(100));
  collector->record_publish(pub);

  const auto all = collector->snapshot(SnapshotMode::All);
  EXPECT_FALSE(find_sample(all, "/ns/talker", "/incoming", StatKind::ReceivedPeriod).has_value());
  EXPECT_EQ(total_samples(all), 1u);
}

TEST_F(CollectorTest, take_age_is_measured_against_the_source_timestamp)
{
  clock->set_system(topic_stats_core::SysTime{ms(1000)});
  collector->record_take(sub, source_at_ms(400));

  const auto sample = find_sample(collector->snapshot(), "/ns/talker", "/incoming", StatKind::TakeAge);
  ASSERT_TRUE(sample.has_value());
  EXPECT_EQ(sample->mean, ms(600));
  EXPECT_EQ(sample->window_count, 1u);
}

TEST_F(CollectorTest, take_age_needs_only_one_message_unlike_period)
{
  // Age is absolute, not an interval, so the very first take produces one.
  clock->set_system(topic_stats_core::SysTime{ms(500)});
  collector->record_take(sub, source_at_ms(500));

  const auto report = collector->snapshot();
  EXPECT_TRUE(find_sample(report, "/ns/talker", "/incoming", StatKind::TakeAge).has_value());
  EXPECT_FALSE(find_sample(report, "/ns/talker", "/incoming", StatKind::ReceivedPeriod).has_value());
}

TEST_F(CollectorTest, negative_take_age_is_reported_rather_than_discarded)
{
  // Clock skew between hosts. Surfacing it is the point.
  clock->set_system(topic_stats_core::SysTime{ms(100)});
  collector->record_take(sub, source_at_ms(300));

  const auto sample = find_sample(collector->snapshot(), "/ns/talker", "/incoming", StatKind::TakeAge);
  ASSERT_TRUE(sample.has_value());
  EXPECT_EQ(sample->mean, ms(-200));
  EXPECT_EQ(collector->diagnostics().unusable_source_timestamps, 0u);
}

TEST_F(CollectorTest, absent_source_timestamp_produces_no_take_age)
{
  collector->record_take(sub, std::nullopt);
  clock->advance(ms(10));
  collector->record_take(sub, std::nullopt);

  const auto report = collector->snapshot();
  EXPECT_FALSE(find_sample(report, "/ns/talker", "/incoming", StatKind::TakeAge).has_value());
  EXPECT_EQ(collector->diagnostics().unusable_source_timestamps, 0u);
}

TEST_F(CollectorTest, non_positive_source_timestamp_is_counted_as_unusable)
{
  // Several rmw implementations report zero rather than nothing when they have no timestamp.
  clock->set_system(topic_stats_core::SysTime{ms(1000)});
  collector->record_take(sub, 0);

  const auto report = collector->snapshot();
  EXPECT_FALSE(find_sample(report, "/ns/talker", "/incoming", StatKind::TakeAge).has_value());
  EXPECT_EQ(collector->diagnostics().unusable_source_timestamps, 1u);
}

TEST_F(CollectorTest, a_system_clock_step_does_not_corrupt_periods)
{
  // The entire reason the collector holds two clocks. NTP stepping the system clock mid-run must
  // not manufacture a period measurement.
  collector->record_publish(pub);
  clock->advance(ms(100));
  clock->advance_system_only(ms(60 * 60 * 1000));
  collector->record_publish(pub);

  const auto sample = find_sample(collector->snapshot(), "/ns/talker", "/chatter", StatKind::PublishedPeriod);
  ASSERT_TRUE(sample.has_value());
  EXPECT_EQ(sample->mean, ms(100));
}

TEST_F(CollectorTest, report_is_grouped_by_node)
{
  const auto other = collector->register_node(NodeDescriptor{"/ns/listener"});
  const auto other_sub =
    collector->register_endpoint(other, EndpointDescriptor{EndpointKind::Subscription, "/chatter", {}});

  collector->record_publish(pub);
  collector->record_take(other_sub, std::nullopt);
  clock->advance(ms(50));
  collector->record_publish(pub);
  collector->record_take(other_sub, std::nullopt);

  const auto report = collector->snapshot();
  ASSERT_EQ(report.nodes.size(), 2u);
  EXPECT_EQ(total_samples(report), 2u);
  EXPECT_TRUE(find_sample(report, "/ns/talker", "/chatter", StatKind::PublishedPeriod).has_value());
  EXPECT_TRUE(find_sample(report, "/ns/listener", "/chatter", StatKind::ReceivedPeriod).has_value());
}

TEST_F(CollectorTest, snapshot_is_stamped_with_system_time)
{
  clock->set_system(topic_stats_core::SysTime{ms(12345)});
  collector->record_publish(pub);
  clock->advance(ms(10));
  collector->record_publish(pub);

  const auto report = collector->snapshot();
  EXPECT_EQ(report.timestamp, topic_stats_core::SysTime{ms(12355)});
}

TEST_F(CollectorTest, unregistered_endpoint_stops_being_reported)
{
  collector->record_publish(pub);
  clock->advance(ms(100));
  collector->record_publish(pub);
  collector->unregister_endpoint(pub);

  EXPECT_TRUE(collector->snapshot(SnapshotMode::All).empty());
  EXPECT_EQ(collector->diagnostics().live_endpoints, 1u);
}

TEST_F(CollectorTest, records_against_a_stale_endpoint_are_counted_not_crashed)
{
  collector->unregister_endpoint(pub);
  // An ingest adapter can be told about destruction while another thread is mid-publish.
  collector->record_publish(pub);
  collector->record_take(pub, source_at_ms(1));
  collector->record_duration(pub, StatKind::CallbackDuration, ms(5));

  EXPECT_EQ(collector->diagnostics().stale_id_records, 3u);
  EXPECT_TRUE(collector->snapshot(SnapshotMode::All).empty());
}

TEST_F(CollectorTest, records_against_a_never_valid_endpoint_are_counted)
{
  collector->record_publish(EndpointId{});
  EXPECT_EQ(collector->diagnostics().stale_id_records, 1u);
}

TEST_F(CollectorTest, publish_recorded_on_a_subscription_is_an_ingest_bug)
{
  collector->record_publish(sub);
  collector->record_take(pub, std::nullopt);

  EXPECT_EQ(collector->diagnostics().mismatched_records, 2u);
  EXPECT_EQ(collector->diagnostics().stale_id_records, 0u);
  EXPECT_TRUE(collector->snapshot(SnapshotMode::All).empty());
}

TEST_F(CollectorTest, unregistering_a_node_drops_its_endpoints)
{
  // RMW teardown does not guarantee endpoints are destroyed before their node, so the collector
  // cannot rely on the adapter cleaning up first.
  collector->record_publish(pub);
  clock->advance(ms(100));
  collector->record_publish(pub);

  collector->unregister_node(node);
  EXPECT_EQ(collector->diagnostics().live_nodes, 0u);
  EXPECT_EQ(collector->diagnostics().live_endpoints, 0u);
  EXPECT_TRUE(collector->snapshot(SnapshotMode::All).empty());

  collector->record_publish(pub);
  EXPECT_EQ(collector->diagnostics().stale_id_records, 1u);
}

TEST_F(CollectorTest, unregistering_a_node_leaves_other_nodes_endpoints_alone)
{
  const auto other = collector->register_node(NodeDescriptor{"/ns/listener"});
  const auto other_pub = collector->register_endpoint(other, EndpointDescriptor{EndpointKind::Publisher, "/other", {}});

  collector->unregister_node(node);
  EXPECT_EQ(collector->diagnostics().live_endpoints, 1u);

  collector->record_publish(other_pub);
  clock->advance(ms(20));
  collector->record_publish(other_pub);
  EXPECT_TRUE(find_sample(collector->snapshot(), "/ns/listener", "/other", StatKind::PublishedPeriod).has_value());
  EXPECT_EQ(collector->diagnostics().stale_id_records, 0u);
}

TEST_F(CollectorTest, registering_an_endpoint_on_a_stale_node_returns_an_invalid_id)
{
  collector->unregister_node(node);
  const auto orphan = collector->register_endpoint(node, EndpointDescriptor{EndpointKind::Publisher, "/orphan", {}});
  EXPECT_FALSE(orphan.valid());
  EXPECT_EQ(collector->diagnostics().live_endpoints, 0u);
}

TEST_F(CollectorTest, precomputed_durations_are_recorded_under_their_own_statistic)
{
  collector->record_duration(sub, StatKind::CallbackDuration, ms(7));
  collector->record_duration(sub, StatKind::CallbackDuration, ms(3));

  const auto sample = find_sample(collector->snapshot(), "/ns/talker", "/incoming", StatKind::CallbackDuration);
  ASSERT_TRUE(sample.has_value());
  EXPECT_EQ(sample->window_count, 2u);
  EXPECT_EQ(sample->mean, ms(5));
  EXPECT_EQ(sample->min, ms(3));
  EXPECT_EQ(sample->max, ms(7));
}

TEST(CollectorOptions, window_size_bounds_how_many_measurements_are_averaged)
{
  auto clock = std::make_shared<ManualClock>();
  Collector collector(Collector::Options{2}, clock);
  const auto node = collector.register_node(NodeDescriptor{"/n"});
  const auto pub = collector.register_endpoint(node, EndpointDescriptor{EndpointKind::Publisher, "/t", {}});

  collector.record_publish(pub);
  for (const auto interval : {ms(10), ms(10), ms(100), ms(100)}) {
    clock->advance(interval);
    collector.record_publish(pub);
  }

  const auto sample = find_sample(collector.snapshot(), "/n", "/t", StatKind::PublishedPeriod);
  ASSERT_TRUE(sample.has_value());
  EXPECT_EQ(sample->window_count, 2u);
  EXPECT_EQ(sample->mean, ms(100));
}
