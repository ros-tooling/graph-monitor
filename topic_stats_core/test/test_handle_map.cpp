// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>

#include "topic_stats_core/handle_map.hpp"
#include "topic_stats_core/types.hpp"

using topic_stats_core::EndpointId;
using topic_stats_core::HandleMap;
using topic_stats_core::NodeId;

namespace
{

struct Entry
{
  NodeId node;
  EndpointId endpoint;
};

// Handles are only ever compared, never dereferenced, so fabricated addresses are enough to test
// the mapping without a middleware anywhere in sight.
const void * handle(uintptr_t value)
{
  return reinterpret_cast<const void *>(value);
}

}  // namespace

TEST(HandleMap, maps_a_handle_to_its_id)
{
  HandleMap<NodeId> map;
  EXPECT_TRUE(map.add(handle(1), NodeId{4, 1}));
  const auto found = map.find(handle(1));
  ASSERT_TRUE(found.has_value());
  EXPECT_EQ(*found, (NodeId{4, 1}));
  EXPECT_EQ(map.size(), 1u);
}

TEST(HandleMap, unknown_handle_is_not_an_error)
{
  // The hot path relies on this: the shim's own statistics publishers are never registered, so
  // every publish on them lands here.
  HandleMap<NodeId> map;
  EXPECT_FALSE(map.find(handle(1)).has_value());
  EXPECT_FALSE(map.remove(handle(1)).has_value());
}

TEST(HandleMap, adding_a_handle_twice_keeps_the_first_id)
{
  HandleMap<NodeId> map;
  ASSERT_TRUE(map.add(handle(1), NodeId{4, 1}));
  EXPECT_FALSE(map.add(handle(1), NodeId{9, 9}));
  EXPECT_EQ(*map.find(handle(1)), (NodeId{4, 1}));
  EXPECT_EQ(map.size(), 1u);
}

TEST(HandleMap, remove_returns_what_it_removed)
{
  HandleMap<NodeId> map;
  map.add(handle(1), NodeId{4, 1});
  const auto removed = map.remove(handle(1));
  ASSERT_TRUE(removed.has_value());
  EXPECT_EQ(*removed, (NodeId{4, 1}));
  EXPECT_EQ(map.size(), 0u);
  EXPECT_FALSE(map.find(handle(1)).has_value());
}

TEST(HandleMap, recycled_handle_addresses_map_to_the_new_id)
{
  // The middleware is free to hand back the same address for a new entity once the old one is
  // destroyed, so a stale entry must not survive removal.
  HandleMap<NodeId> map;
  map.add(handle(1), NodeId{4, 1});
  map.remove(handle(1));
  map.add(handle(1), NodeId{4, 2});
  EXPECT_EQ(*map.find(handle(1)), (NodeId{4, 2}));
}

TEST(HandleMap, remove_if_drops_a_nodes_endpoints)
{
  // RMW teardown does not guarantee endpoints are destroyed before their node, so without this the
  // map would grow for the life of any process that churns nodes.
  HandleMap<Entry> map;
  const NodeId doomed{1, 0};
  const NodeId survivor{2, 0};
  map.add(handle(10), Entry{doomed, EndpointId{10, 0}});
  map.add(handle(11), Entry{doomed, EndpointId{11, 0}});
  map.add(handle(12), Entry{survivor, EndpointId{12, 0}});

  const auto removed = map.remove_if([&](const Entry & entry) { return entry.node == doomed; });
  EXPECT_EQ(removed.size(), 2u);
  EXPECT_EQ(map.size(), 1u);
  EXPECT_TRUE(map.find(handle(12)).has_value());
  EXPECT_FALSE(map.find(handle(10)).has_value());
  EXPECT_FALSE(map.find(handle(11)).has_value());
}

TEST(HandleMap, remove_if_matching_nothing_removes_nothing)
{
  HandleMap<Entry> map;
  map.add(handle(10), Entry{NodeId{1, 0}, EndpointId{10, 0}});
  const auto removed = map.remove_if([](const Entry &) { return false; });
  EXPECT_TRUE(removed.empty());
  EXPECT_EQ(map.size(), 1u);
}

TEST(HandleMap, concurrent_lookup_during_registration_and_teardown)
{
  // RMW entities are created and destroyed on arbitrary threads while messages flow on others.
  // Getting this wrong is how the previous implementation raced. Most valuable under TSan.
  HandleMap<Entry> map;
  constexpr uintptr_t kStable = 1;
  map.add(handle(kStable), Entry{NodeId{1, 0}, EndpointId{1, 0}});

  std::atomic<bool> stop{false};
  std::atomic<uint64_t> hits{0};
  std::vector<std::thread> threads;

  for (int i = 0; i < 4; ++i) {
    threads.emplace_back([&map, &stop, &hits]() {
      while (!stop.load(std::memory_order_relaxed)) {
        if (map.find(handle(kStable)).has_value()) {
          hits.fetch_add(1, std::memory_order_relaxed);
        }
      }
    });
  }
  threads.emplace_back([&map, &stop]() {
    uintptr_t next = 100;
    while (!stop.load(std::memory_order_relaxed)) {
      map.add(handle(next), Entry{NodeId{2, 0}, EndpointId{2, 0}});
      map.remove(handle(next));
      map.remove_if([](const Entry & entry) { return entry.node == NodeId{2, 0}; });
      next = (next > 10000) ? 100 : next + 1;
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  stop = true;
  for (auto & thread : threads) {
    thread.join();
  }

  EXPECT_GT(hits.load(), 0u);
  // The stable entry belongs to a different node and must have survived every remove_if sweep.
  EXPECT_TRUE(map.find(handle(kStable)).has_value());
}
