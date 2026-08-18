// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef ROSGRAPH_MONITOR__RCLCPP_PARAMETER_CLIENT_HPP_
#define ROSGRAPH_MONITOR__RCLCPP_PARAMETER_CLIENT_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rosgraph_monitor/parameter_collector.hpp"

namespace rosgraph_monitor
{

/// @brief Talks to real nodes' parameter services on behalf of the collector.
///
/// A thin adapter over rclcpp::AsyncParametersClient. Every call uses the callback form, so
/// responses arrive on the executor and no thread is spent waiting on a node that may never
/// answer. One client is kept per observed node and dropped when the node goes away.
class RclcppParameterServiceClient : public ParameterServiceClient
{
public:
  RclcppParameterServiceClient(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services);

  bool is_ready(const std::string & node_name) override;
  void list_parameters(const std::string & node_name, NamesCallback callback) override;
  void describe_parameters(
    const std::string & node_name, const std::vector<std::string> & names, DescriptorsCallback callback) override;
  void get_parameters(
    const std::string & node_name, const std::vector<std::string> & names, ValuesCallback callback) override;
  void forget(const std::string & node_name) override;

private:
  std::shared_ptr<rclcpp::AsyncParametersClient> client_for(const std::string & node_name);

  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_;
  const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_;

  std::mutex mutex_;
  std::unordered_map<std::string, std::shared_ptr<rclcpp::AsyncParametersClient>> clients_;
};

}  // namespace rosgraph_monitor

#endif  // ROSGRAPH_MONITOR__RCLCPP_PARAMETER_CLIENT_HPP_
