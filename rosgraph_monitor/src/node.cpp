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

#include "rosgraph_monitor/node.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rosgraph_monitor_msgs/msg/topic_statistics.hpp"

#include "rosgraph_monitor/rosgraph_monitor_generated_parameters.hpp"

namespace
{

template<typename T>
std::unordered_set<T> vec_to_set(const std::vector<T> & in)
{
  return std::unordered_set<T>(in.begin(), in.end());
}

}  // namespace

namespace rosgraph_monitor
{
GraphMonitorConfiguration Node::create_graph_monitor_config(
  const rosgraph_monitor::Params & params)
{
  const rosgraph_monitor::Params::GraphMonitor & gparms = params.graph_monitor;
  GraphMonitorConfiguration gconf;
  gconf.diagnostic_namespace = gparms.diagnostic_namespace;
  gconf.nodes.ignore_prefixes = gparms.nodes.ignore_prefixes;
  gconf.nodes.warn_only_prefixes = gparms.nodes.warn_only_prefixes;
  gconf.continuity.enable = gparms.continuity.enable;
  gconf.continuity.ignore_subscriber_nodes = vec_to_set(gparms.continuity.ignore_subscriber_nodes);
  gconf.continuity.ignore_topic_types = vec_to_set(gparms.continuity.ignore_topic_types);
  gconf.continuity.ignore_topic_names = vec_to_set(gparms.continuity.ignore_topic_names);
  gconf.topic_statistics.deadline_allowed_error = gparms.topic_statistics.deadline_allowed_error;
  gconf.topic_statistics.stale_timeout =
    std::chrono::milliseconds{gparms.topic_statistics.stale_timeout_ms};
  return gconf;
}

Node::Node(const rclcpp::NodeOptions & options)
: rclcpp::Node("rosgraph_monitor", options),
  param_cb_handle_(get_node_parameters_interface()->add_on_set_parameters_callback(
      std::bind(&Node::on_parameter_event, this, std::placeholders::_1))),
  param_listener_(new rosgraph_monitor::ParamListener(get_node_parameters_interface())),
  params_(param_listener_->get_params()),
  graph_monitor_(
    get_node_graph_interface(),
    [this]() {return get_clock()->now();},
    get_logger().get_child("rosgraph"),
    create_graph_monitor_config(params_)),
  sub_topic_statistics_(
    create_subscription<rosgraph_monitor_msgs::msg::TopicStatistics>(
      "/topic_statistics",
      rclcpp::QoS{10},
      std::bind(&Node::on_topic_statistics, this, std::placeholders::_1))),
  pub_diagnostics_(
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics",
      10)),
  pub_diagnostic_agg_(
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics_agg",
      10)),
  pub_diagnostic_toplevel_(
    create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
      "/diagnostics_toplevel_status",
      10))
{
  graph_analyzer_.init("/Health", params_.graph_analyzer);

  // Don't start evaluation timer until after first configuration of the monitor
  timer_publish_report_ = create_wall_timer(
    std::chrono::milliseconds(params_.diagnostics_publish_period_ms),
    std::bind(&Node::publish_diagnostics, this));
}

rcl_interfaces::msg::SetParametersResult Node::on_parameter_event(
  const std::vector<rclcpp::Parameter> & /* parameters */)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  if (!param_listener_) {
    return result;
  } else if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
  } else {
    RCLCPP_WARN(get_logger(), "Received parameter callback, but parameters weren't outdated");
    result.successful = false;
    return result;
  }

  on_new_params();
  return result;
}

void Node::on_new_params()
{
  graph_monitor_.config() = create_graph_monitor_config(params_);
}

void Node::on_topic_statistics(const rosgraph_monitor_msgs::msg::TopicStatistics::SharedPtr msg)
{
  graph_monitor_.on_topic_statistics(*msg);
}

void Node::publish_diagnostics()
{
  auto diagnostic_array = graph_monitor_.evaluate();

  // TODO(ek) probably possible & cleaner to implement using DiagnosticAggregator with Analyzers
  // Evaluate all diagnostics into final aggregated set for reporting
  for (const auto & status : diagnostic_array->status) {
    if (graph_analyzer_.match(status.name)) {
      auto item = std::make_shared<diagnostic_aggregator::StatusItem>(&status);
      graph_analyzer_.analyze(item);
    }
  }
  auto agg_statuses = graph_analyzer_.report();

  // Evaluate aggregated diagnostics into one single toplevel status "result"
  auto diagnostic_toplevel = *agg_statuses[0];
  diagnostic_msgs::msg::DiagnosticArray diagnostic_agg;
  diagnostic_agg.header.stamp = get_clock()->now();
  for (const auto & status : agg_statuses) {
    diagnostic_agg.status.push_back(*status);
  }
  // Publish both the aggregated statuses and the result
  pub_diagnostic_agg_->publish(diagnostic_agg);
  pub_diagnostic_toplevel_->publish(diagnostic_toplevel);
  pub_diagnostics_->publish(std::move(diagnostic_array));
}

}  // namespace rosgraph_monitor

RCLCPP_COMPONENTS_REGISTER_NODE(rosgraph_monitor::Node)
