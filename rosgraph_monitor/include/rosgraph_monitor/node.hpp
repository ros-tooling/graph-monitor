// SPDX-FileCopyrightText: 2024 Bonsai Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef ROSGRAPH_MONITOR__NODE_HPP_
#define ROSGRAPH_MONITOR__NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/node.hpp"
#include "rosgraph_monitor/monitor.hpp"
#include "rosgraph_monitor/rosgraph_monitor_generated_parameters.hpp"
#include "rosgraph_monitor_msgs/msg/topic_statistics.hpp"
#include "rosgraph_msgs/msg/graph.hpp"

namespace rosgraph_monitor
{

constexpr int SERVICE_TIMEOUT_S = 5;

class Node : public rclcpp::Node
{
private:
  static GraphMonitorConfiguration create_graph_monitor_config(const rosgraph_monitor::Params & params);

public:
  explicit Node(const rclcpp::NodeOptions & options);

protected:
  void update_params(const rosgraph_monitor::Params & params);
  void on_topic_statistics(const rosgraph_monitor_msgs::msg::TopicStatistics::SharedPtr msg);
  void publish_diagnostics();
  void publish_rosgraph(rosgraph_msgs::msg::Graph rosgraph_msg);
  QueryParamsReturnType query_params(
    const std::string & node_name, std::function<void(const rcl_interfaces::msg::ListParametersResult &)> callback);

  rosgraph_monitor::ParamListener param_listener_;
  rosgraph_monitor::Params params_;

  rclcpp::Subscription<rosgraph_monitor_msgs::msg::TopicStatistics>::SharedPtr sub_topic_statistics_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
  rclcpp::Publisher<rosgraph_msgs::msg::Graph>::SharedPtr pub_rosgraph_;
  rclcpp::TimerBase::SharedPtr timer_publish_report_;

  // Declared/constructed last, in case our callbacks need the above endpoints
  RosGraphMonitor graph_monitor_;
};

}  // namespace rosgraph_monitor

#endif  // ROSGRAPH_MONITOR__NODE_HPP_
