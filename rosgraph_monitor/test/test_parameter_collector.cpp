// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

/**
 * Tests for the parameter collector.
 *
 * The collector is driven through a fake parameter service so the concurrency cap, expiry and
 * failure handling can be exercised without a ROS graph, and responses can be delivered in
 * whatever order a test needs.
 */

#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rclcpp/logger.hpp"
#include "rosgraph_monitor/parameter_collector.hpp"

using rcl_interfaces::msg::ParameterDescriptor;
using rcl_interfaces::msg::ParameterType;
using rcl_interfaces::msg::ParameterValue;
using rosgraph_monitor::NodeParameters;
using rosgraph_monitor::ParameterCollector;
using rosgraph_monitor::ParameterServiceClient;

namespace
{

/// A parameter service whose responses are delivered by the test, not by a node.
class FakeParameterService : public ParameterServiceClient
{
public:
  bool is_ready(const std::string & node_name) override
  {
    return unreachable.count(node_name) == 0;
  }

  void list_parameters(const std::string & node_name, NamesCallback callback) override
  {
    list_calls.push_back(node_name);
    pending_list[node_name] = std::move(callback);
  }

  void forget(const std::string & node_name) override
  {
    forgotten.push_back(node_name);
  }

  /// Deliver the response the collector is waiting on for a node.
  void reply_list(const std::string & node, std::optional<std::vector<std::string>> names)
  {
    auto it = pending_list.find(node);
    ASSERT_NE(it, pending_list.end()) << "no outstanding request for " << node;
    auto callback = std::move(it->second);
    pending_list.erase(it);
    callback(std::move(names));
  }

  std::vector<std::string> list_calls;
  std::vector<std::string> forgotten;
  std::set<std::string> unreachable;

private:
  std::map<std::string, NamesCallback> pending_list;
};

class ParameterCollectorTest : public testing::Test
{
protected:
  void make_collector(ParameterCollector::Options options = ParameterCollector::Options{})
  {
    collector_ = std::make_unique<ParameterCollector>(
      service_,
      [this](const std::string & node_name, NodeParameters parameters) {
        completed_.emplace_back(node_name, std::move(parameters));
      },
      [this]() { return now_; },
      rclcpp::get_logger("test_parameter_collector"),
      options);
  }

  void advance(std::chrono::nanoseconds delta)
  {
    now_ += delta;
  }

  std::shared_ptr<FakeParameterService> service_ = std::make_shared<FakeParameterService>();
  std::unique_ptr<ParameterCollector> collector_;
  std::vector<std::pair<std::string, NodeParameters>> completed_;
  std::chrono::nanoseconds now_{0};
};

}  // namespace

// ---------------------------------------------------------------------------
// Observing a node
// ---------------------------------------------------------------------------

TEST_F(ParameterCollectorTest, reports_the_parameters_a_node_lists)
{
  make_collector();
  collector_->request("/talker");

  ASSERT_EQ(service_->list_calls, std::vector<std::string>{"/talker"});
  EXPECT_TRUE(completed_.empty()) << "nothing to report until the node answers";

  service_->reply_list("/talker", std::vector<std::string>{"gain", "rate"});

  ASSERT_EQ(completed_.size(), 1u);
  EXPECT_EQ(completed_[0].first, "/talker");
  ASSERT_EQ(completed_[0].second.descriptors.size(), 2u);
  EXPECT_EQ(completed_[0].second.descriptors[0].name, "gain");
  EXPECT_EQ(completed_[0].second.descriptors[1].name, "rate");
  EXPECT_EQ(collector_->active_count(), 0u) << "slot released once the node is done";
}

TEST_F(ParameterCollectorTest, a_node_with_no_parameters_completes_as_an_empty_observation)
{
  make_collector();
  collector_->request("/bare");
  service_->reply_list("/bare", std::vector<std::string>{});

  ASSERT_EQ(completed_.size(), 1u) << "having no parameters is an answer, not a failure";
  EXPECT_TRUE(completed_[0].second.empty());
  EXPECT_EQ(collector_->active_count(), 0u);
}

TEST_F(ParameterCollectorTest, requesting_a_node_twice_does_not_duplicate_work)
{
  make_collector();
  collector_->request("/talker");
  collector_->request("/talker");
  EXPECT_EQ(service_->list_calls.size(), 1u);
  EXPECT_EQ(collector_->pending_count(), 0u);
}

TEST_F(ParameterCollectorTest, an_unreachable_node_does_not_hold_a_slot)
{
  make_collector();
  service_->unreachable.insert("/gone");
  collector_->request("/gone");

  EXPECT_TRUE(service_->list_calls.empty());
  EXPECT_EQ(collector_->active_count(), 0u);
  EXPECT_EQ(collector_->pending_count(), 0u);
}

TEST_F(ParameterCollectorTest, a_node_that_becomes_reachable_is_observed_when_asked_again)
{
  // The collector does not retry on its own; asking again is the caller's job, and this is
  // what makes that work.
  make_collector();
  service_->unreachable.insert("/late");
  collector_->request("/late");
  ASSERT_TRUE(service_->list_calls.empty());

  service_->unreachable.erase("/late");
  collector_->request("/late");

  ASSERT_EQ(service_->list_calls, std::vector<std::string>{"/late"});
  service_->reply_list("/late", std::vector<std::string>{"gain"});
  ASSERT_EQ(completed_.size(), 1u);
  EXPECT_EQ(completed_[0].second.descriptors.size(), 1u);
}

// ---------------------------------------------------------------------------
// Concurrency cap
// ---------------------------------------------------------------------------

TEST_F(ParameterCollectorTest, observes_at_most_max_concurrent_nodes)
{
  ParameterCollector::Options options;
  options.max_concurrent = 2;
  make_collector(options);

  for (const auto & node : {"/a", "/b", "/c", "/d"}) {
    collector_->request(node);
  }

  EXPECT_EQ(collector_->active_count(), 2u);
  EXPECT_EQ(collector_->pending_count(), 2u);
  EXPECT_EQ(service_->list_calls.size(), 2u) << "only in-flight nodes issue requests";
}

TEST_F(ParameterCollectorTest, finishing_a_node_admits_a_queued_one)
{
  ParameterCollector::Options options;
  options.max_concurrent = 1;
  make_collector(options);

  collector_->request("/first");
  collector_->request("/second");
  ASSERT_EQ(service_->list_calls, std::vector<std::string>{"/first"});

  service_->reply_list("/first", std::vector<std::string>{});

  ASSERT_EQ(service_->list_calls.size(), 2u);
  EXPECT_EQ(service_->list_calls[1], "/second");
  EXPECT_EQ(collector_->pending_count(), 0u);
}

// ---------------------------------------------------------------------------
// Failures and nodes going away
// ---------------------------------------------------------------------------

TEST_F(ParameterCollectorTest, a_failed_list_frees_the_slot)
{
  ParameterCollector::Options options;
  options.max_concurrent = 1;
  make_collector(options);
  collector_->request("/broken");
  collector_->request("/next");

  service_->reply_list("/broken", std::nullopt);

  EXPECT_TRUE(completed_.empty()) << "nothing to report for a node that would not answer";
  ASSERT_EQ(service_->list_calls.size(), 2u) << "the queued node still gets its turn";
  EXPECT_EQ(service_->list_calls[1], "/next");
}

TEST_F(ParameterCollectorTest, a_cancelled_node_reports_nothing_when_its_reply_lands)
{
  make_collector();
  collector_->request("/leaving");

  // The node drops out of the graph while the list request is outstanding.
  collector_->cancel("/leaving");
  service_->reply_list("/leaving", std::vector<std::string>{"gain"});

  EXPECT_TRUE(completed_.empty()) << "a response for an abandoned observation is discarded";
  EXPECT_EQ(collector_->active_count(), 0u);
  EXPECT_EQ(service_->forgotten, std::vector<std::string>{"/leaving"});
}

TEST_F(ParameterCollectorTest, cancelling_admits_a_queued_node)
{
  ParameterCollector::Options options;
  options.max_concurrent = 1;
  make_collector(options);
  collector_->request("/leaving");
  collector_->request("/waiting");

  collector_->cancel("/leaving");

  ASSERT_EQ(service_->list_calls.size(), 2u);
  EXPECT_EQ(service_->list_calls[1], "/waiting");
}

TEST_F(ParameterCollectorTest, cancelling_a_queued_node_removes_it)
{
  ParameterCollector::Options options;
  options.max_concurrent = 1;
  make_collector(options);
  collector_->request("/first");
  collector_->request("/doomed");

  collector_->cancel("/doomed");
  service_->reply_list("/first", std::vector<std::string>{});

  EXPECT_EQ(service_->list_calls.size(), 1u) << "a cancelled node is never started";
  EXPECT_EQ(collector_->pending_count(), 0u);
}

TEST_F(ParameterCollectorTest, an_observation_that_stops_answering_expires)
{
  ParameterCollector::Options options;
  options.max_concurrent = 1;
  options.timeout = std::chrono::seconds(5);
  make_collector(options);

  collector_->request("/silent");
  collector_->request("/waiting");
  ASSERT_EQ(collector_->active_count(), 1u);

  advance(std::chrono::seconds(4));
  collector_->tick();
  EXPECT_EQ(collector_->active_count(), 1u) << "not yet past the timeout";

  advance(std::chrono::seconds(2));
  collector_->tick();

  EXPECT_TRUE(completed_.empty());
  ASSERT_EQ(service_->list_calls.size(), 2u) << "the slot is freed for the queued node";
  EXPECT_EQ(service_->list_calls[1], "/waiting");
}

TEST_F(ParameterCollectorTest, a_reply_after_expiry_is_discarded)
{
  ParameterCollector::Options options;
  options.timeout = std::chrono::seconds(5);
  make_collector(options);

  collector_->request("/slow");
  advance(std::chrono::seconds(6));
  collector_->tick();

  service_->reply_list("/slow", std::vector<std::string>{"gain"});

  EXPECT_TRUE(completed_.empty());
}

TEST_F(ParameterCollectorTest, a_node_can_be_observed_again_after_being_cancelled)
{
  make_collector();
  collector_->request("/flaky");
  collector_->cancel("/flaky");

  collector_->request("/flaky");
  ASSERT_EQ(service_->list_calls.size(), 2u);

  service_->reply_list("/flaky", std::vector<std::string>{});
  ASSERT_EQ(completed_.size(), 1u);
  EXPECT_EQ(completed_[0].first, "/flaky");
}

// ---------------------------------------------------------------------------
// Re-entrancy
// ---------------------------------------------------------------------------

namespace
{

/// A client that answers on the calling thread, before its request method returns.
/// Perfectly legal, and what a cache or an in-process node would do. The collector must not be
/// holding its lock when it calls out, or the response handler deadlocks against it.
class SynchronousParameterService : public ParameterServiceClient
{
public:
  bool is_ready(const std::string &) override
  {
    return true;
  }

  void list_parameters(const std::string &, NamesCallback callback) override
  {
    callback(std::vector<std::string>{"gain"});
  }

  void forget(const std::string &) override
  {}
};

}  // namespace

TEST(ParameterCollectorReentrancyTest, a_client_that_answers_inline_does_not_deadlock)
{
  std::vector<std::pair<std::string, NodeParameters>> completed;
  ParameterCollector collector(
    std::make_shared<SynchronousParameterService>(),
    [&completed](const std::string & node_name, NodeParameters parameters) {
      completed.emplace_back(node_name, std::move(parameters));
    },
    []() { return std::chrono::nanoseconds{0}; },
    rclcpp::get_logger("test_parameter_collector"));

  // Run on another thread so a regression is a failure rather than a hung test binary.
  auto done = std::async(std::launch::async, [&collector]() { collector.request("/inline"); });
  ASSERT_EQ(done.wait_for(std::chrono::seconds(5)), std::future_status::ready)
    << "collector deadlocked against a synchronous client";
  done.get();

  ASSERT_EQ(completed.size(), 1u);
  EXPECT_EQ(completed[0].first, "/inline");
  ASSERT_EQ(completed[0].second.descriptors.size(), 1u);
  EXPECT_EQ(completed[0].second.descriptors[0].name, "gain");
}
