// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "rosgraph_monitor/rclcpp_parameter_client.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace rosgraph_monitor
{

RclcppParameterServiceClient::RclcppParameterServiceClient(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services)
: node_base_(std::move(node_base))
, node_topics_(std::move(node_topics))
, node_graph_(std::move(node_graph))
, node_services_(std::move(node_services))
{}

std::shared_ptr<rclcpp::AsyncParametersClient> RclcppParameterServiceClient::client_for(const std::string & node_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = clients_.find(node_name);
  if (it != clients_.end()) {
    return it->second;
  }
  auto client =
    std::make_shared<rclcpp::AsyncParametersClient>(node_base_, node_topics_, node_graph_, node_services_, node_name);
  clients_.emplace(node_name, client);
  return client;
}

bool RclcppParameterServiceClient::is_ready(const std::string & node_name)
{
  return client_for(node_name)->service_is_ready();
}

void RclcppParameterServiceClient::list_parameters(const std::string & node_name, NamesCallback callback)
{
  // Empty prefixes and depth 0 means every parameter the node has.
  client_for(node_name)->list_parameters(
    {}, 0, [callback](std::shared_future<rcl_interfaces::msg::ListParametersResult> future) {
      try {
        callback(future.get().names);
      } catch (const std::exception &) {
        // The node went away, or the call was interrupted. Either way there is nothing to
        // report, and the collector treats a failure the same as an absent node.
        callback(std::nullopt);
      }
    });
}

void RclcppParameterServiceClient::describe_parameters(
  const std::string & node_name, const std::vector<std::string> & names, DescriptorsCallback callback)
{
  client_for(node_name)->describe_parameters(
    names, [callback](std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>> future) {
      try {
        callback(future.get());
      } catch (const std::exception &) {
        callback(std::nullopt);
      }
    });
}

void RclcppParameterServiceClient::get_parameters(
  const std::string & node_name, const std::vector<std::string> & names, ValuesCallback callback)
{
  client_for(node_name)->get_parameters(names, [callback](std::shared_future<std::vector<rclcpp::Parameter>> future) {
    try {
      const auto parameters = future.get();
      std::vector<rcl_interfaces::msg::ParameterValue> values;
      values.reserve(parameters.size());
      for (const auto & parameter : parameters) {
        values.push_back(parameter.get_value_message());
      }
      callback(std::move(values));
    } catch (const std::exception &) {
      callback(std::nullopt);
    }
  });
}

void RclcppParameterServiceClient::forget(const std::string & node_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  clients_.erase(node_name);
}

}  // namespace rosgraph_monitor
