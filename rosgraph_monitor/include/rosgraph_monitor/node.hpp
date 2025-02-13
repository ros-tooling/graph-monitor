// Copyright 2024, Bonsai Robotics, Inc - All Rights Reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROSGRAPH_MONITOR__NODE_HPP_
#define ROSGRAPH_MONITOR__NODE_HPP_

#include <memory>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/node.hpp"
#include "rosgraph_monitor_msgs/msg/topic_statistics.hpp"

#include "rosgraph_monitor/rosgraph_monitor_generated_parameters.hpp"
#include "rosgraph_monitor/graph_analyzer.hpp"
#include "rosgraph_monitor/monitor.hpp"

namespace rosgraph_monitor
{

class Node : public rclcpp::Node
{
public:
  explicit Node(const rclcpp::NodeOptions & options);

protected:
  rcl_interfaces::msg::SetParametersResult on_parameter_event(
    const std::vector<rclcpp::Parameter> & parameters);
  void on_new_params();
  void on_topic_statistics(const rosgraph_monitor_msgs::msg::TopicStatistics::SharedPtr msg);
  void publish_diagnostics();

  std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> param_cb_handle_;
  std::shared_ptr<rosgraph_monitor::ParamListener> param_listener_;
  rosgraph_monitor::Params params_;

  RosGraphMonitor graph_monitor_;
  GraphAnalyzer graph_analyzer_;

  rclcpp::TimerBase::SharedPtr timer_publish_report_;
  rclcpp::Subscription<rosgraph_monitor_msgs::msg::TopicStatistics>::SharedPtr
    sub_topic_statistics_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostic_agg_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_diagnostic_toplevel_;
};

}  // namespace rosgraph_monitor

#endif  // ROSGRAPH_MONITOR__NODE_HPP_
