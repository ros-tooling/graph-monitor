// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "gmock/gmock.h"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_monitor/rclcpp_parameter_client.hpp"

using rosgraph_monitor::RclcppParameterServiceClient;

namespace
{

/// A client built on a node whose context has been shut down.
/// Each test gets its own context, so shutting one down leaves the others usable.
class InvalidContextClient : public testing::Test
{
protected:
  void SetUp() override
  {
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);

    rclcpp::NodeOptions options;
    options.context(context_);
    node_ = std::make_shared<rclcpp::Node>("param_client_test_node", options);

    client_ = std::make_unique<RclcppParameterServiceClient>(
      node_->get_node_base_interface(),
      node_->get_node_topics_interface(),
      node_->get_node_graph_interface(),
      node_->get_node_services_interface());

    context_->shutdown("test teardown");
  }

  void TearDown() override
  {
    client_.reset();
    node_.reset();
    context_.reset();
  }

  rclcpp::Context::SharedPtr context_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<RclcppParameterServiceClient> client_;
};

TEST_F(InvalidContextClient, IsReadyReportsFalse)
{
  EXPECT_FALSE(client_->is_ready("/some_node"));
}

TEST_F(InvalidContextClient, ListParametersCallsBackOnceWithNullopt)
{
  int calls = 0;
  std::optional<std::vector<std::string>> result{std::vector<std::string>{}};
  client_->list_parameters("/some_node", [&](std::optional<std::vector<std::string>> names) {
    ++calls;
    result = std::move(names);
  });

  EXPECT_EQ(calls, 1);
  EXPECT_EQ(result, std::nullopt);
}

TEST_F(InvalidContextClient, DescribeParametersCallsBackOnceWithNullopt)
{
  using Descriptors = std::vector<rcl_interfaces::msg::ParameterDescriptor>;

  int calls = 0;
  std::optional<Descriptors> result{Descriptors{}};
  client_->describe_parameters("/some_node", {"a", "b"}, [&](std::optional<Descriptors> descriptors) {
    ++calls;
    result = std::move(descriptors);
  });

  EXPECT_EQ(calls, 1);
  EXPECT_EQ(result, std::nullopt);
}

TEST_F(InvalidContextClient, ForgetDoesNotThrow)
{
  EXPECT_NO_THROW(client_->forget("/some_node"));
}

/// A failed construction is not cached, so the client is usable again once a valid context exists.
TEST(ValidContextClient, IsReadyReportsFalseForAbsentNode)
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);

  rclcpp::NodeOptions options;
  options.context(context);
  auto node = std::make_shared<rclcpp::Node>("param_client_retry_test_node", options);

  RclcppParameterServiceClient client(
    node->get_node_base_interface(),
    node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  EXPECT_FALSE(client.is_ready("/node_that_does_not_exist"));

  context->shutdown("test teardown");
}

}  // namespace
