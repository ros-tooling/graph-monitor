// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

/**
 * Tests for the parameter collector.
 *
 * The collector is driven through a fake parameter service so the sequencing, concurrency cap
 * and failure handling can be exercised without a ROS graph, and responses can be delivered in
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
#include "rclcpp/logger.hpp"
#include "rosgraph_monitor/parameter_collector.hpp"

using rcl_interfaces::msg::ParameterDescriptor;
using rcl_interfaces::msg::ParameterValue;
using rosgraph_monitor::NodeParameters;
using rosgraph_monitor::pair_parameters;
using rosgraph_monitor::ParameterCollector;
using rosgraph_monitor::ParameterServiceClient;

namespace
{

ParameterDescriptor descriptor(const std::string & name, uint8_t type = 1)
{
  ParameterDescriptor d;
  d.name = name;
  d.type = type;
  return d;
}

ParameterValue int_value(int64_t v)
{
  ParameterValue value;
  value.type = 2;  // PARAMETER_INTEGER
  value.integer_value = v;
  return value;
}

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

  void describe_parameters(
    const std::string & node_name, const std::vector<std::string> & names, DescriptorsCallback callback) override
  {
    describe_calls.push_back({node_name, names});
    pending_describe[node_name] = std::move(callback);
  }

  void get_parameters(
    const std::string & node_name, const std::vector<std::string> & names, ValuesCallback callback) override
  {
    get_calls.push_back({node_name, names});
    pending_get[node_name] = std::move(callback);
  }

  void forget(const std::string & node_name) override
  {
    forgotten.push_back(node_name);
  }

  // Deliver a response the collector is waiting on.
  void reply_list(const std::string & node, std::optional<std::vector<std::string>> names)
  {
    auto callback = take(pending_list, node);
    callback(std::move(names));
  }

  void reply_describe(const std::string & node, std::optional<std::vector<ParameterDescriptor>> descriptors)
  {
    auto callback = take(pending_describe, node);
    callback(std::move(descriptors));
  }

  void reply_get(const std::string & node, std::optional<std::vector<ParameterValue>> values)
  {
    auto callback = take(pending_get, node);
    callback(std::move(values));
  }

  bool awaiting_list(const std::string & node) const
  {
    return pending_list.count(node) > 0;
  }

  bool awaiting_describe(const std::string & node) const
  {
    return pending_describe.count(node) > 0;
  }

  bool awaiting_get(const std::string & node) const
  {
    return pending_get.count(node) > 0;
  }

  std::vector<std::string> list_calls;
  std::vector<std::pair<std::string, std::vector<std::string>>> describe_calls;
  std::vector<std::pair<std::string, std::vector<std::string>>> get_calls;
  std::vector<std::string> forgotten;
  std::set<std::string> unreachable;

private:
  template <typename Map>
  typename Map::mapped_type take(Map & map, const std::string & node)
  {
    auto it = map.find(node);
    EXPECT_NE(it, map.end()) << "no outstanding request for " << node;
    auto callback = std::move(it->second);
    map.erase(it);
    return callback;
  }

  std::map<std::string, NamesCallback> pending_list;
  std::map<std::string, DescriptorsCallback> pending_describe;
  std::map<std::string, ValuesCallback> pending_get;
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
// pair_parameters
// ---------------------------------------------------------------------------

TEST(PairParametersTest, pairs_equal_length_responses)
{
  const auto paired = pair_parameters({descriptor("a"), descriptor("b")}, {int_value(1), int_value(2)});
  ASSERT_EQ(paired.descriptors.size(), 2u);
  ASSERT_EQ(paired.values.size(), 2u);
  EXPECT_EQ(paired.descriptors[1].name, "b");
  EXPECT_EQ(paired.values[1].integer_value, 2);
}

TEST(PairParametersTest, drops_the_tail_when_responses_disagree)
{
  // A parameter removed between the describe and the get shortens one response but not the
  // other. Pairing past that point would attach a value to the wrong parameter.
  const auto paired = pair_parameters({descriptor("a"), descriptor("b"), descriptor("c")}, {int_value(1)});
  ASSERT_EQ(paired.descriptors.size(), 1u);
  ASSERT_EQ(paired.values.size(), 1u);
  EXPECT_EQ(paired.descriptors[0].name, "a");
}

TEST(PairParametersTest, empty_responses_pair_to_nothing)
{
  const auto paired = pair_parameters({}, {});
  EXPECT_TRUE(paired.empty());
}

// ---------------------------------------------------------------------------
// The three-call sequence
// ---------------------------------------------------------------------------

TEST_F(ParameterCollectorTest, runs_list_describe_get_in_order)
{
  make_collector();
  collector_->request("/talker");

  ASSERT_EQ(service_->list_calls, std::vector<std::string>{"/talker"});
  EXPECT_TRUE(service_->describe_calls.empty()) << "describe must wait for the names";

  service_->reply_list("/talker", std::vector<std::string>{"gain", "rate"});
  ASSERT_EQ(service_->describe_calls.size(), 1u);
  EXPECT_EQ(service_->describe_calls[0].second, (std::vector<std::string>{"gain", "rate"}));
  EXPECT_TRUE(service_->get_calls.empty()) << "get must wait for the descriptors";

  service_->reply_describe("/talker", std::vector<ParameterDescriptor>{descriptor("gain"), descriptor("rate")});
  ASSERT_EQ(service_->get_calls.size(), 1u);

  service_->reply_get("/talker", std::vector<ParameterValue>{int_value(7), int_value(8)});

  ASSERT_EQ(completed_.size(), 1u);
  EXPECT_EQ(completed_[0].first, "/talker");
  ASSERT_EQ(completed_[0].second.descriptors.size(), 2u);
  EXPECT_EQ(completed_[0].second.descriptors[0].name, "gain");
  EXPECT_EQ(completed_[0].second.values[1].integer_value, 8);
  EXPECT_EQ(collector_->active_count(), 0u) << "slot released once the node is done";
}

TEST_F(ParameterCollectorTest, a_node_with_no_parameters_completes_without_more_calls)
{
  make_collector();
  collector_->request("/bare");
  service_->reply_list("/bare", std::vector<std::string>{});

  EXPECT_TRUE(service_->describe_calls.empty());
  EXPECT_TRUE(service_->get_calls.empty());
  ASSERT_EQ(completed_.size(), 1u);
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

TEST_F(ParameterCollectorTest, a_failed_describe_frees_the_slot)
{
  make_collector();
  collector_->request("/talker");
  service_->reply_list("/talker", std::vector<std::string>{"gain"});
  service_->reply_describe("/talker", std::nullopt);

  EXPECT_TRUE(completed_.empty());
  EXPECT_EQ(collector_->active_count(), 0u);
}

TEST_F(ParameterCollectorTest, values_failing_still_reports_the_descriptors)
{
  // The interface is still worth knowing even when the values could not be read.
  make_collector();
  collector_->request("/talker");
  service_->reply_list("/talker", std::vector<std::string>{"gain"});
  service_->reply_describe("/talker", std::vector<ParameterDescriptor>{descriptor("gain")});
  service_->reply_get("/talker", std::nullopt);

  ASSERT_EQ(completed_.size(), 1u);
  EXPECT_EQ(completed_[0].second.descriptors.size(), 1u);
  EXPECT_TRUE(completed_[0].second.values.empty());
}

TEST_F(ParameterCollectorTest, a_cancelled_node_reports_nothing_when_its_reply_lands)
{
  make_collector();
  collector_->request("/leaving");
  service_->reply_list("/leaving", std::vector<std::string>{"gain"});

  // The node drops out of the graph while the describe is outstanding.
  collector_->cancel("/leaving");
  service_->reply_describe("/leaving", std::vector<ParameterDescriptor>{descriptor("gain")});

  EXPECT_TRUE(completed_.empty()) << "a response for an abandoned observation is discarded";
  EXPECT_TRUE(service_->get_calls.empty()) << "the sequence does not continue after cancelling";
  EXPECT_EQ(collector_->active_count(), 0u);
}

TEST_F(ParameterCollectorTest, cancelling_admits_a_queued_node)
{
  ParameterCollector::Options options;
  options.max_concurrent = 1;
  make_collector(options);
  collector_->request("/leaving");
  collector_->request("/waiting");

  collector_->cancel("/leaving");
  collector_->tick();

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

  EXPECT_TRUE(service_->describe_calls.empty());
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

  void describe_parameters(
    const std::string &, const std::vector<std::string> & names, DescriptorsCallback callback) override
  {
    std::vector<ParameterDescriptor> descriptors;
    for (const auto & name : names) {
      descriptors.push_back(descriptor(name));
    }
    callback(std::move(descriptors));
  }

  void get_parameters(const std::string &, const std::vector<std::string> & names, ValuesCallback callback) override
  {
    std::vector<ParameterValue> values(names.size(), int_value(1));
    callback(std::move(values));
  }
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
  EXPECT_EQ(completed[0].second.values.size(), 1u);
}
