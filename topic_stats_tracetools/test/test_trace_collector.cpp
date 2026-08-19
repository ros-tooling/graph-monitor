// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "topic_stats_core/clock.hpp"
#include "topic_stats_core/collector.hpp"
#include "topic_stats_core/testing/recording_sink.hpp"
#include "topic_stats_tracetools/trace_collector.hpp"

using topic_stats_core::Collector;
using topic_stats_core::Duration;
using topic_stats_core::ManualClock;
using topic_stats_core::StatKind;
using topic_stats_core::StatsReport;
using topic_stats_core::testing::RecordingSink;
using topic_stats_tracetools::TraceCollector;

namespace
{

Duration ms(int64_t count)
{
  return std::chrono::duration_cast<Duration>(std::chrono::milliseconds(count));
}

// Handles are only ever compared, never dereferenced, so fabricated addresses are enough to drive
// the whole adapter without rcl, a middleware, or a tracing session anywhere in sight.
const void * handle(uintptr_t value)
{
  return reinterpret_cast<const void *>(value);
}

constexpr auto kNode = 0x1000;
constexpr auto kPub = 0x2000;
constexpr auto kSub = 0x3000;

class TraceCollectorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    clock = std::make_shared<ManualClock>();
    collector = std::make_shared<Collector>(Collector::Options{50}, clock);
    auto owned_sink = std::make_unique<RecordingSink>();
    sink = owned_sink.get();
    adapter = std::make_unique<TraceCollector>(config, collector, std::move(owned_sink));
  }

  /// Everything rcl emits when a node with one publisher and one subscription comes up.
  void bring_up_node()
  {
    adapter->on_node_init(handle(kNode), "talker", "/ns");
    adapter->on_publisher_init(handle(kNode), handle(kPub), "/chatter");
    adapter->on_subscription_init(handle(kNode), handle(kSub), "/incoming");
  }

  std::optional<topic_stats_core::StatSample> last_sample(const std::string & topic, StatKind stat) const
  {
    const auto reports = sink->reports();
    for (auto it = reports.rbegin(); it != reports.rend(); ++it) {
      for (const auto & node : it->nodes) {
        for (const auto & sample : node.samples) {
          if (sample.topic_name == topic && sample.stat == stat) {
            return sample;
          }
        }
      }
    }
    return std::nullopt;
  }

  TraceCollector::Config config;
  std::shared_ptr<ManualClock> clock;
  std::shared_ptr<Collector> collector;
  RecordingSink * sink = nullptr;
  std::unique_ptr<TraceCollector> adapter;
};

}  // namespace

TEST_F(TraceCollectorTest, node_name_is_joined_from_name_and_namespace)
{
  // rcl reports them separately, and the graph monitor keys on the fully qualified name.
  bring_up_node();
  adapter->on_publish(handle(kPub));
  clock->advance(ms(100));
  adapter->on_publish(handle(kPub));
  adapter->report();

  ASSERT_EQ(sink->count(), 1u);
  const auto reports = sink->reports();
  ASSERT_EQ(reports.front().nodes.size(), 1u);
  EXPECT_EQ(reports.front().nodes.front().node_name, "/ns/talker");
}

TEST_F(TraceCollectorTest, a_root_namespace_does_not_produce_a_doubled_slash)
{
  adapter->on_node_init(handle(kNode), "talker", "/");
  adapter->on_publisher_init(handle(kNode), handle(kPub), "/chatter");
  adapter->on_publish(handle(kPub));
  clock->advance(ms(50));
  adapter->on_publish(handle(kPub));
  adapter->report();

  EXPECT_EQ(sink->reports().front().nodes.front().node_name, "/talker");  // NOLINT: copy is fine here
}

TEST_F(TraceCollectorTest, publishes_become_published_period)
{
  bring_up_node();
  adapter->on_publish(handle(kPub));
  clock->advance(ms(100));
  adapter->on_publish(handle(kPub));
  adapter->report();

  const auto sample = last_sample("/chatter", StatKind::PublishedPeriod);
  ASSERT_TRUE(sample.has_value());
  EXPECT_EQ(sample->mean, ms(100));
  EXPECT_EQ(sample->node_name, "/ns/talker");
}

TEST_F(TraceCollectorTest, takes_become_received_period_and_take_age)
{
  bring_up_node();
  clock->set_system(topic_stats_core::SysTime{} + ms(1000));
  adapter->on_take(handle(kSub), ms(900).count(), true);
  clock->advance(ms(100));
  adapter->on_take(handle(kSub), ms(1050).count(), true);
  adapter->report();

  const auto period = last_sample("/incoming", StatKind::ReceivedPeriod);
  ASSERT_TRUE(period.has_value());
  EXPECT_EQ(period->mean, ms(100));

  const auto age = last_sample("/incoming", StatKind::TakeAge);
  ASSERT_TRUE(age.has_value());
  EXPECT_EQ(age->window_count, 2u) << "age is absolute, so both takes yield one";
}

TEST_F(TraceCollectorTest, a_take_that_took_nothing_is_not_a_message)
{
  // The middleware polls far more often than messages arrive. Counting these would report a
  // received period of roughly the polling interval on every topic in the system.
  bring_up_node();
  for (int i = 0; i < 50; ++i) {
    clock->advance(ms(1));
    adapter->on_take(handle(kSub), 0, false);
  }
  adapter->report();

  EXPECT_FALSE(last_sample("/incoming", StatKind::ReceivedPeriod).has_value());
  EXPECT_FALSE(last_sample("/incoming", StatKind::TakeAge).has_value());
  EXPECT_EQ(collector->diagnostics().stale_id_records, 0u);
}

TEST_F(TraceCollectorTest, publishes_on_handles_rcl_never_announced_are_ignored_quietly)
{
  // Real and constant: the middleware has discovery publishers of its own that rcl never
  // initialises, and their tracepoints fire before any node exists.
  adapter->on_publish(handle(0xDEAD));
  adapter->on_take(handle(0xBEEF), 12345, true);
  bring_up_node();
  adapter->on_publish(handle(0xDEAD));

  adapter->report();
  EXPECT_EQ(sink->count(), 0u);
  EXPECT_EQ(collector->diagnostics().stale_id_records, 0u)
    << "unknown handles must not be reported as stale records; they were never ours";
}

TEST_F(TraceCollectorTest, an_endpoint_on_an_unannounced_node_is_skipped)
{
  adapter->on_publisher_init(handle(0xBAD), handle(kPub), "/chatter");
  adapter->on_publish(handle(kPub));
  clock->advance(ms(10));
  adapter->on_publish(handle(kPub));
  adapter->report();
  EXPECT_EQ(sink->count(), 0u);
}

TEST_F(TraceCollectorTest, a_reused_publisher_address_is_reattributed_to_its_new_topic)
{
  // Nothing ever tells this adapter an endpoint went away, so the middleware is free to hand the
  // same address back for a different topic. Keeping the old mapping would attribute the new
  // topic's traffic to the old one.
  bring_up_node();
  adapter->on_publish(handle(kPub));
  clock->advance(ms(100));
  adapter->on_publish(handle(kPub));
  adapter->report();
  ASSERT_TRUE(last_sample("/chatter", StatKind::PublishedPeriod).has_value());

  adapter->on_publisher_init(handle(kNode), handle(kPub), "/replacement");
  adapter->on_publish(handle(kPub));
  clock->advance(ms(200));
  adapter->on_publish(handle(kPub));
  adapter->report();

  const auto replacement = last_sample("/replacement", StatKind::PublishedPeriod);
  ASSERT_TRUE(replacement.has_value());
  EXPECT_EQ(replacement->mean, ms(200));

  // The old endpoint is gone rather than lingering as a second entry for the same handle.
  EXPECT_EQ(collector->diagnostics().live_endpoints, 2u) << "the subscription plus one publisher";
}

TEST_F(TraceCollectorTest, a_reused_node_address_is_reattributed)
{
  bring_up_node();
  adapter->on_node_init(handle(kNode), "listener", "/other");
  adapter->on_publisher_init(handle(kNode), handle(0x9000), "/fresh");
  adapter->on_publish(handle(0x9000));
  clock->advance(ms(100));
  adapter->on_publish(handle(0x9000));
  adapter->report();

  ASSERT_EQ(sink->count(), 1u);
  // Held by value: reports() hands back a copy, so binding a reference into the temporary would
  // dangle the moment the statement ends.
  const auto reports = sink->reports();
  ASSERT_EQ(reports.front().nodes.size(), 1u);
  EXPECT_EQ(reports.front().nodes.front().node_name, "/other/listener");
}

TEST_F(TraceCollectorTest, nothing_is_evicted_when_eviction_is_off)
{
  bring_up_node();
  for (int i = 0; i < 50; ++i) {
    adapter->report();
  }
  EXPECT_EQ(collector->diagnostics().live_endpoints, 2u);
  EXPECT_EQ(adapter->revived_endpoints(), 0u);
}

TEST(TraceCollectorEviction, a_quiet_endpoint_is_dropped_and_brought_back_when_it_speaks_again)
{
  // ros2_tracing has no destruction tracepoints, so without eviction the registry grows for the
  // life of the process. Eviction is only safe because the adapter keeps the descriptor and can
  // register the endpoint again; otherwise a topic that publishes once an hour would be dropped
  // and then never measured again.
  auto clock = std::make_shared<ManualClock>();
  auto collector = std::make_shared<Collector>(Collector::Options{50}, clock);
  auto owned_sink = std::make_unique<RecordingSink>();
  auto * sink = owned_sink.get();

  TraceCollector::Config config;
  config.idle_eviction_reports = 2;
  TraceCollector adapter(config, collector, std::move(owned_sink));

  adapter.on_node_init(handle(kNode), "talker", "/ns");
  adapter.on_publisher_init(handle(kNode), handle(kPub), "/rare");

  adapter.report();
  adapter.report();
  EXPECT_EQ(collector->diagnostics().live_endpoints, 0u) << "the quiet endpoint should have been dropped";

  // An hour later, the topic finally publishes.
  adapter.on_publish(handle(kPub));
  clock->advance(ms(100));
  adapter.on_publish(handle(kPub));
  adapter.report();

  EXPECT_EQ(adapter.revived_endpoints(), 1u);
  EXPECT_EQ(collector->diagnostics().live_endpoints, 1u);
  ASSERT_FALSE(sink->reports().empty());

  bool found = false;
  for (const auto & report : sink->reports()) {
    for (const auto & node : report.nodes) {
      for (const auto & sample : node.samples) {
        if (sample.topic_name == "/rare" && sample.stat == StatKind::PublishedPeriod) {
          EXPECT_EQ(sample.mean, ms(100));
          EXPECT_EQ(node.node_name, "/ns/talker");
          found = true;
        }
      }
    }
  }
  EXPECT_TRUE(found) << "measurement did not resume after the endpoint was brought back";
}
