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
#include "rosgraph_monitor_msgs/msg/graph.hpp"
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
  gconf.topic_statistics.mandatory_topics = vec_to_set(gparms.topic_statistics.mandatory_topics);
  gconf.topic_statistics.ignore_topics = vec_to_set(gparms.topic_statistics.ignore_topics);
  return gconf;
}

Node::Node(const rclcpp::NodeOptions & options)
: rclcpp::Node("rosgraph_monitor", options),
  param_listener_(get_node_parameters_interface()),
  params_(param_listener_.get_params()),
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
  pub_rosgraph_(
    create_publisher<rosgraph_monitor_msgs::msg::Graph>(
      "/rosgraph",
      rclcpp::QoS(1).transient_local().reliable())),

  timer_publish_report_(
    create_wall_timer(
      std::chrono::milliseconds(params_.diagnostics_publish_period_ms),
      std::bind(&Node::publish_diagnostics, this)))
{
  param_listener_.setUserCallback(std::bind(&Node::update_params, this, std::placeholders::_1));

  // Set up callback to publish rosgraph when nodes change
  graph_monitor_.set_graph_change_callback(std::bind(&Node::publish_rosgraph, this));

  // Publish initial rosgraph state
  publish_rosgraph();
}

void Node::update_params(const rosgraph_monitor::Params & params)
{
  params_ = params;
  graph_monitor_.config() = create_graph_monitor_config(params_);
}

void Node::on_topic_statistics(const rosgraph_monitor_msgs::msg::TopicStatistics::SharedPtr msg)
{
  graph_monitor_.on_topic_statistics(*msg);
}

void Node::publish_diagnostics()
{
  auto diagnostic_array = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
  diagnostic_array->header.stamp = get_clock()->now();
  diagnostic_array->header.frame_id = "rosgraph_monitor";

  graph_monitor_.evaluate(diagnostic_array->status);

  pub_diagnostics_->publish(std::move(diagnostic_array));
}

void Node::publish_rosgraph()
{
  rosgraph_monitor_msgs::msg::Graph rosgraph_msg;
  graph_monitor_.fill_rosgraph_msg(rosgraph_msg);
  pub_rosgraph_->publish(std::move(rosgraph_msg));
}

}  // namespace rosgraph_monitor

RCLCPP_COMPONENTS_REGISTER_NODE(rosgraph_monitor::Node)
