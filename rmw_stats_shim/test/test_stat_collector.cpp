// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "rmw_stats_shim/stat_collector.hpp"

using rmw_stats_shim::EndpointStatistics;
using rmw_stats_shim::EndpointType;
using rmw_stats_shim::StatCollector;
using std::chrono::milliseconds;

namespace
{

constexpr size_t NUM_SUBSCRIPTIONS = 64;
constexpr size_t NUM_READERS = 4;
constexpr size_t CHURN_ITERATIONS = 2000;

rmw_node_t make_node()
{
  rmw_node_t node{};
  node.name = "node";
  node.namespace_ = "/";
  return node;
}

}  // namespace

TEST(StatCollector, ConcurrentSubscriptionChurnAndReceive)
{
  auto & collector = StatCollector::instance();
  rmw_node_t node = make_node();

  std::vector<std::string> topics;
  std::vector<rmw_subscription_t> subscriptions(NUM_SUBSCRIPTIONS);
  topics.reserve(NUM_SUBSCRIPTIONS);
  for (size_t i = 0; i < NUM_SUBSCRIPTIONS; ++i) {
    topics.push_back("/topic_" + std::to_string(i));
    subscriptions[i].topic_name = topics.back().c_str();
  }

  std::atomic<bool> done = false;
  std::vector<std::thread> readers;
  for (size_t r = 0; r < NUM_READERS; ++r) {
    readers.emplace_back([&] {
      rmw_message_info_t info{};
      while (!done) {
        for (auto & sub : subscriptions) {
          collector.onReceive(&sub, &info);
        }
      }
    });
  }
  std::thread stats([&] {
    while (!done) {
      collector.publishStatistics();
    }
  });

  for (size_t i = 0; i < CHURN_ITERATIONS; ++i) {
    for (auto & sub : subscriptions) {
      collector.addSubscription(&sub, &node);
    }
    for (auto & sub : subscriptions) {
      collector.removeSubscription(&sub);
    }
  }

  done = true;
  for (auto & reader : readers) {
    reader.join();
  }
  stats.join();
  SUCCEED();
}

TEST(EndpointStatistics, MessagesReportWindowMinAndMax)
{
  rmw_node_t node = make_node();
  EndpointStatistics stats(EndpointType::Subscription, "/topic", &node, 10);

  MonoTime t0{};
  // Periods of 10ms, 20ms and 6ms
  for (int offset : {0, 10, 30, 36}) {
    stats.onMessage(t0 + milliseconds{offset});
  }
  auto period = stats.periodMsg();
  EXPECT_EQ(period.window_count, 3);
  EXPECT_EQ(period.mean.nanosec, 12000000u);
  EXPECT_EQ(period.min.nanosec, 6000000u);
  EXPECT_EQ(period.max.nanosec, 20000000u);

  for (int age : {7, 3, 15}) {
    stats.onAge(milliseconds{age});
  }
  auto age = stats.ageMsg();
  EXPECT_EQ(age.window_count, 3);
  EXPECT_EQ(age.mean.nanosec, 8333333u);
  EXPECT_EQ(age.min.nanosec, 3000000u);
  EXPECT_EQ(age.max.nanosec, 15000000u);
}
