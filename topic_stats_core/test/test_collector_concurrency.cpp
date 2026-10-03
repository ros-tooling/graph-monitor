// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "topic_stats_core/clock.hpp"
#include "topic_stats_core/collector.hpp"
#include "topic_stats_core/types.hpp"

using topic_stats_core::Collector;
using topic_stats_core::EndpointDescriptor;
using topic_stats_core::EndpointId;
using topic_stats_core::EndpointKind;
using topic_stats_core::NodeDescriptor;
using topic_stats_core::SnapshotMode;
using topic_stats_core::StatKind;
using topic_stats_core::SystemClock;

namespace
{
constexpr auto kRunDuration = std::chrono::milliseconds(300);
}  // namespace

// The predecessor of this collector mutated unsynchronized maps from arbitrary publisher threads
// while a timer thread iterated them, which is a data race that survived only because endpoints are
// almost always created at startup. These tests exist to keep that from coming back, and are most
// valuable run under ThreadSanitizer.

TEST(CollectorConcurrency, recording_on_many_endpoints_while_snapshotting)
{
  Collector collector(Collector::Options{16}, std::make_shared<SystemClock>());
  const auto node = collector.register_node(NodeDescriptor{"/n"});

  constexpr int kEndpoints = 8;
  std::vector<EndpointId> endpoints;
  for (int i = 0; i < kEndpoints; ++i) {
    endpoints.push_back(collector.register_endpoint(
      node, EndpointDescriptor{EndpointKind::Publisher, "/topic_" + std::to_string(i), {}}));
  }

  std::atomic<bool> stop{false};
  std::atomic<uint64_t> snapshots{0};
  std::vector<std::thread> threads;

  for (int i = 0; i < kEndpoints; ++i) {
    threads.emplace_back([&collector, &stop, id = endpoints[i]]() {
      while (!stop.load(std::memory_order_relaxed)) {
        collector.record_publish(id);
      }
    });
  }
  threads.emplace_back([&collector, &stop, &snapshots]() {
    while (!stop.load(std::memory_order_relaxed)) {
      const auto report = collector.snapshot(SnapshotMode::All);
      for (const auto & node_report : report.nodes) {
        for (const auto & sample : node_report.samples) {
          // Touch the contents so a torn read would be observable.
          ASSERT_FALSE(sample.topic_name.empty());
          ASSERT_GT(sample.window_count, 0u);
        }
      }
      snapshots.fetch_add(1, std::memory_order_relaxed);
    }
  });

  std::this_thread::sleep_for(kRunDuration);
  stop = true;
  for (auto & thread : threads) {
    thread.join();
  }

  EXPECT_GT(snapshots.load(), 0u);
  EXPECT_EQ(collector.diagnostics().stale_id_records, 0u);
  EXPECT_EQ(collector.diagnostics().mismatched_records, 0u);
}

TEST(CollectorConcurrency, endpoint_churn_while_recording_and_snapshotting)
{
  // The hard case: registration reallocates the slot table and recycles indices while other
  // threads hold ids. Stale records are expected here; crashes and torn reads are not.
  Collector collector(Collector::Options{8}, std::make_shared<SystemClock>());
  const auto node = collector.register_node(NodeDescriptor{"/n"});

  std::atomic<bool> stop{false};
  std::atomic<uint64_t> churned{0};
  // Published by the churn thread, read by the recorders. Deliberately racy access to a handle,
  // which is exactly what an ingest adapter's own handle map does under load.
  std::atomic<EndpointId> shared_id{EndpointId{}};
  ASSERT_TRUE(shared_id.is_lock_free()) << "EndpointId must be small enough to publish atomically";

  std::vector<std::thread> threads;
  for (int i = 0; i < 4; ++i) {
    threads.emplace_back([&collector, &stop, &shared_id]() {
      while (!stop.load(std::memory_order_relaxed)) {
        collector.record_publish(shared_id.load(std::memory_order_relaxed));
      }
    });
  }
  threads.emplace_back([&collector, &stop, &shared_id, &churned, node]() {
    while (!stop.load(std::memory_order_relaxed)) {
      const auto id = collector.register_endpoint(node, EndpointDescriptor{EndpointKind::Publisher, "/churn", {}});
      shared_id.store(id, std::memory_order_relaxed);
      collector.unregister_endpoint(id);
      churned.fetch_add(1, std::memory_order_relaxed);
    }
  });
  threads.emplace_back([&collector, &stop]() {
    while (!stop.load(std::memory_order_relaxed)) {
      (void)collector.snapshot(SnapshotMode::All);
    }
  });

  std::this_thread::sleep_for(kRunDuration);
  stop = true;
  for (auto & thread : threads) {
    thread.join();
  }

  EXPECT_GT(churned.load(), 0u);
  // Slots must have been recycled rather than accumulating one per churn cycle.
  EXPECT_LE(collector.diagnostics().live_endpoints, 1u);
}

TEST(CollectorConcurrency, node_teardown_while_recording)
{
  Collector collector(Collector::Options{8}, std::make_shared<SystemClock>());

  std::atomic<bool> stop{false};
  std::atomic<EndpointId> shared_id{EndpointId{}};
  std::atomic<uint64_t> cycles{0};

  std::vector<std::thread> threads;
  for (int i = 0; i < 4; ++i) {
    threads.emplace_back([&collector, &stop, &shared_id]() {
      while (!stop.load(std::memory_order_relaxed)) {
        collector.record_take(shared_id.load(std::memory_order_relaxed), 1);
      }
    });
  }
  threads.emplace_back([&collector, &stop, &shared_id, &cycles]() {
    while (!stop.load(std::memory_order_relaxed)) {
      const auto node = collector.register_node(NodeDescriptor{"/transient"});
      const auto id = collector.register_endpoint(node, EndpointDescriptor{EndpointKind::Subscription, "/t", {}});
      shared_id.store(id, std::memory_order_relaxed);
      // Node goes away without its endpoint being unregistered first.
      collector.unregister_node(node);
      cycles.fetch_add(1, std::memory_order_relaxed);
    }
  });

  std::this_thread::sleep_for(kRunDuration);
  stop = true;
  for (auto & thread : threads) {
    thread.join();
  }

  EXPECT_GT(cycles.load(), 0u);
  EXPECT_EQ(collector.diagnostics().live_endpoints, 0u);
  EXPECT_EQ(collector.diagnostics().live_nodes, 0u);
}
