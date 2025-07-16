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

#include "rosgraph_monitor/monitor.hpp"

#include "rclcpp/logging.hpp"


std::size_t std::hash<RosRmwGid>::operator()(
  const RosRmwGid & id) const noexcept
{
  constexpr std::size_t u64s = sizeof(uint64_t);
  static_assert(sizeof(RosRmwGid) == (3 * u64s) || sizeof(RosRmwGid) == (2 * u64s));
  if (sizeof(RosRmwGid) == (2 * u64s)) {
    uint64_t d0, d1;
    std::memcpy(&d0, id.data() + (0 * u64s), u64s);
    std::memcpy(&d1, id.data() + (1 * u64s), u64s);
    return d0 + d1;
  } else if (sizeof(RosRmwGid) == (3 * u64s)) {
    uint64_t d0, d1, d2;
    std::memcpy(&d0, id.data() + (0 * u64s), u64s);
    std::memcpy(&d1, id.data() + (1 * u64s), u64s);
    std::memcpy(&d2, id.data() + (2 * u64s), u64s);
    return d0 + d1 + d2;
  }
  return 0;
}

std::size_t std::hash<std::pair<std::string, std::string>>::operator()(
  const std::pair<std::string, std::string> & value) const noexcept
{
  std::size_t h1 = std::hash<std::string>{}(value.first);
  std::size_t h2 = std::hash<std::string>{}(value.second);
  // Cribbed from boost::hash_combine
  return h1 ^ (h2 << 1);
}

namespace
{

bool match_any_prefixes(const std::vector<std::string> & prefixes, const std::string & value)
{
  for (const std::string & prefix : prefixes) {
    if (value.compare(0, prefix.size(), prefix) == 0) {
      return true;
    }
  }
  return false;
}

}  // namespace


namespace rosgraph_monitor
{

std::string gid_to_str(const uint8_t gid[RMW_GID_STORAGE_SIZE])
{
  std::string result;
  result.resize(24 * 2 + 23);
  snprintf(&result[0], 3, "%02x", gid[0]);  // NOLINT(runtime/printf)
  size_t pos = 2;
  for (size_t i = 1; i < 24; i++) {
    snprintf(&result[pos], 4, ".%02x", gid[i]);  // NOLINT(runtime/printf)
    pos += 3;
  }
  return result;
}

std::string gid_to_str(const RosRmwGid & gid)
{
  return gid_to_str(&gid[0]);
}

RosGraphMonitor::EndpointTracking::EndpointTracking(
  const std::string & topic_name,
  const rclcpp::TopicEndpointInfo & info,
  const rclcpp::Time & now)
: topic_name(topic_name),
  node_name(info.node_namespace() == "/" ?
    info.node_namespace() + info.node_name() :
    info.node_namespace() + "/" + info.node_name()),
  info(info),
  last_stats_timestamp(now)
{
}

RosGraphMonitor::RosGraphMonitor(
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
  std::function<rclcpp::Time()> now_fn,
  rclcpp::Logger logger,
  GraphMonitorConfiguration config)
: config_(config),
  now_fn_(now_fn),
  node_graph_(node_graph),
  logger_(logger),
  graph_change_event_(node_graph->get_graph_event())
{
  update_graph();
  watch_thread_ = std::thread(std::bind(&RosGraphMonitor::watch_for_updates, this));
}

RosGraphMonitor::~RosGraphMonitor()
{
  shutdown_.store(true);
  // Set the predicate and notify the condition_variable to wake up the watch thread immediately
  graph_change_event_->set();
  node_graph_->notify_shutdown();
  update_event_.set();
  watch_thread_.join();
}

void RosGraphMonitor::update_graph()
{
  auto node_names = node_graph_->get_node_names();
  track_node_updates(node_names);

  const auto topics_and_types = node_graph_->get_topic_names_and_types();
  track_endpoint_updates(topics_and_types);
}

bool RosGraphMonitor::ignore_node(const std::string & node_name)
{
  if (node_name == "_NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_") {
    return true;
  }
  if (match_any_prefixes(config_.nodes.ignore_prefixes, node_name)) {
    auto [it, inserted] = ignored_nodes_.insert(node_name);
    if (inserted) {
      RCLCPP_DEBUG(logger_, "Ignoring new node: %s", node_name.c_str());
    }
    return true;
  }
  return false;
}

void RosGraphMonitor::track_node_updates(
  const std::vector<std::string> & observed_node_names)
{
  // Mark all stale as base state
  for (auto & [node_name, tracking] : nodes_) {
    tracking.stale = true;
  }
  // Look at current node list, detect new and returned
  for (const auto & node_name : observed_node_names) {
    if (ignore_node(node_name)) {
      continue;
    }

    auto [it, inserted] = nodes_.emplace(node_name, NodeTracking{});
    if (inserted) {
      RCLCPP_DEBUG(logger_, "New node: %s", node_name.c_str());
    } else {
      NodeTracking & tracking = it->second;
      tracking.stale = false;
      if (tracking.missing) {
        RCLCPP_INFO(logger_, "Node %s came back", node_name.c_str());
        tracking.missing = false;
        returned_nodes_.insert(node_name);
      }
    }
  }
  // Check which nodes are still stale - they weren't observed
  for (auto & [node_name, tracking] : nodes_) {
    if (tracking.stale && !tracking.missing) {
      RCLCPP_WARN(logger_, "Node %s went missing", node_name.c_str());
      tracking.missing = true;
      returned_nodes_.erase(node_name);
    }
  }
}

std::optional<RosGraphMonitor::EndpointTrackingMap::iterator> RosGraphMonitor::add_publisher(
  const std::string & topic_name, const rclcpp::TopicEndpointInfo & info)
{
  EndpointTracking proposed_tracking(topic_name, info, now_fn_());
  if (ignore_node(proposed_tracking.node_name)) {
    return std::nullopt;
  }
  auto [it, inserted] = publishers_.emplace(info.endpoint_gid(), proposed_tracking);
  auto & [gid, tracking] = *it;
  publisher_lookup_.insert_or_assign(std::make_pair(tracking.node_name, tracking.topic_name), gid);
  if (inserted) {
    RCLCPP_DEBUG(
      logger_, "New Publisher: %s::%s (%s)",
      tracking.node_name.c_str(), tracking.topic_name.c_str(),
      gid_to_str(tracking.info.endpoint_gid()).c_str());
  }
  return it;
}

std::optional<RosRmwGid> RosGraphMonitor::lookup_publisher(
  const std::string & node_name, const std::string & topic_name) const
{
  try {
    return publisher_lookup_.at(std::make_pair(node_name, topic_name));
  } catch (const std::out_of_range &) {
    return std::nullopt;
  }
}

std::optional<RosGraphMonitor::EndpointTrackingMap::iterator> RosGraphMonitor::add_subscription(
  const std::string & topic_name, const rclcpp::TopicEndpointInfo & info)
{
  EndpointTracking proposed_tracking(topic_name, info, now_fn_());
  if (ignore_node(proposed_tracking.node_name)) {
    return std::nullopt;
  }
  auto [it, inserted] = subscriptions_.emplace(info.endpoint_gid(), proposed_tracking);
  auto & [gid, tracking] = *it;
  subscription_lookup_.insert_or_assign(
    std::make_pair(tracking.node_name, tracking.topic_name),
    gid);
  if (inserted) {
    RCLCPP_DEBUG(
      logger_, "New Subscription: %s::%s (%s)",
      tracking.node_name.c_str(), tracking.topic_name.c_str(),
      gid_to_str(tracking.info.endpoint_gid()).c_str());
  }
  return it;
}

std::optional<RosRmwGid> RosGraphMonitor::lookup_subscription(
  const std::string & node_name, const std::string & topic_name) const
{
  try {
    return subscription_lookup_.at(std::make_pair(node_name, topic_name));
  } catch (const std::out_of_range &) {
    return std::nullopt;
  }
}

bool RosGraphMonitor::topic_period_ok(
  const rosgraph_monitor_msgs::msg::TopicStatistic & stat, const rclcpp::Duration & deadline) const
{
  const std::chrono::nanoseconds measured_period =
    rclcpp::Duration(stat.mean).to_chrono<std::chrono::nanoseconds>();
  const std::chrono::nanoseconds chrono_deadline = deadline.to_chrono<std::chrono::nanoseconds>();
  if (measured_period < chrono_deadline) {
    return true;
  }
  const auto period_error = measured_period - chrono_deadline;
  const auto allowed_error = chrono_deadline * config_.topic_statistics.deadline_allowed_error;
  return period_error <= allowed_error;
}


void RosGraphMonitor::track_endpoint_updates(const TopicsToTypes & observed_topics_and_types)
{
  // Mark all stale as base state
  for (auto & [gid, tracking] : publishers_) {
    tracking.stale = true;
  }
  for (auto & [gid, tracking] : subscriptions_) {
    tracking.stale = true;
  }
  for (auto & [topic_name, counts] : topic_endpoint_counts_) {
    counts.pubs = 0;
    counts.subs = 0;
  }

  // Look over all currently observed topics for endpoint changes
  for (const auto & [topic_name, topic_types] : observed_topics_and_types) {
    // Assumption: "multiple types on the topic" is an error already handled elsewhere
    bool count_topic =
      config_.continuity.ignore_topic_names.count(topic_name) == 0 &&
      config_.continuity.ignore_topic_types.count(topic_types[0]) == 0;
    auto & endpoint_counts = topic_endpoint_counts_[topic_name];

    // Check all publishers
    for (const auto & endpoint_info : node_graph_->get_publishers_info_by_topic(topic_name)) {
      auto maybe_it = add_publisher(topic_name, endpoint_info);
      if (!maybe_it.has_value()) {
        continue;
      }
      auto & [gid, tracking] = **maybe_it;
      if (count_topic) {
        endpoint_counts.pubs++;
      }
      tracking.stale = false;
    }

    // Check all subscriptions
    for (const auto & endpoint_info : node_graph_->get_subscriptions_info_by_topic(topic_name)) {
      auto maybe_it = add_subscription(topic_name, endpoint_info);
      if (!maybe_it.has_value()) {
        continue;
      }
      auto & [gid, tracking] = **maybe_it;

      bool count_subs_from_node =
        config_.continuity.ignore_subscriber_nodes.count(tracking.node_name) == 0;
      if (count_topic && count_subs_from_node) {
        endpoint_counts.subs++;
      }
      tracking.stale = false;
    }
  }

  // Check for any stale endpoints (not seen this iteration) - they're missing.
  // For now just delete them, there isn't a meaningful health case to track at this point
  // Also remove endpoints from missing node, that node missing is the important error.
  for (EndpointTrackingMap * endpoints : {&publishers_, &subscriptions_}) {
    for (auto it = endpoints->begin(); it != endpoints->end(); ) {
      auto & [gid, tracking] = *it;
      const auto node_it = nodes_.find(tracking.node_name);
      bool node_not_tracked = node_it == nodes_.end();
      bool node_missing = node_not_tracked ? true : node_it->second.missing;
      if (node_missing || tracking.stale) {
        it = endpoints->erase(it);
      } else {
        it++;
      }
    }
  }

  // Super basic graph continuity test - does not yet account for QoS mismatch
  if (config_.continuity.enable) {
    for (auto it = topic_endpoint_counts_.begin(); it != topic_endpoint_counts_.end(); ) {
      auto & [topic_name, counts] = *it;
      // Check counts to see if any pubs or subs don't have matches
      if (counts.pubs > 0 && counts.subs == 0) {
        pubs_with_no_subs_.insert(topic_name);
      }
      if (counts.subs > 0 && counts.pubs == 0) {
        subs_with_no_pubs_.insert(topic_name);
      }
      // Delete any lingering tracking with no matches at all, the topic doesn't exist anymore
      if (counts.pubs == 0 && counts.subs == 0) {
        pubs_with_no_subs_.erase(topic_name);
        subs_with_no_pubs_.erase(topic_name);
        it = topic_endpoint_counts_.erase(it);
      } else {
        it++;
      }
    }
  }
}

std::unique_ptr<diagnostic_msgs::msg::DiagnosticArray> RosGraphMonitor::evaluate()
{
  auto now = now_fn_();
  auto msg = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
  msg->header.stamp = now;

  using diagnostic_msgs::msg::DiagnosticStatus;

  // Nodes
  for (const auto & [node_name, node_info] : nodes_) {
    if (node_info.missing) {
      if (match_any_prefixes(config_.nodes.warn_only_prefixes, node_name)) {
        msg->status.push_back(
          statusMsg(
            DiagnosticStatus::WARN,
            "Optional node missing: " + node_name,
            "Node::" + node_name));
      } else {
        msg->status.push_back(
          statusMsg(
            DiagnosticStatus::ERROR,
            "Required node missing: " + node_name,
            "Node::" + node_name));
      }
    }
  }
  for (const std::string & node_name : returned_nodes_) {
    msg->status.push_back(
      statusMsg(
        DiagnosticStatus::OK,
        "Node OK: " + node_name,
        "Node::" + node_name));
  }

  // Continuity
  for (const auto & [topic_name, counts] : topic_endpoint_counts_) {
    if (counts.pubs > 0 && subs_with_no_pubs_.erase(topic_name) > 0) {
      msg->status.push_back(
        statusMsg(
          DiagnosticStatus::OK,
          "Dead sink: cleared. Topic now has publisher(s)",
          "Continuity::" + topic_name));
    }
    if (counts.subs > 0 && pubs_with_no_subs_.erase(topic_name) > 0) {
      msg->status.push_back(
        statusMsg(
          DiagnosticStatus::OK,
          "Leaf topic: cleared. Topic now has subscriber(s)",
          "Continuity::" + topic_name));
    }
  }
  for (const auto & topic_name : pubs_with_no_subs_) {
    msg->status.push_back(
      statusMsg(
        DiagnosticStatus::WARN,
        "Leaf topic (No subscriptions)",
        "Continuity::" + topic_name));
  }
  for (const auto & topic_name : subs_with_no_pubs_) {
    msg->status.push_back(
      statusMsg(
        DiagnosticStatus::WARN,
        "Dead sink (No publishers)",
        "Continuity::" + topic_name));
  }

  // Frequency
  auto deadline_not_set = [](const rclcpp::Duration & dur) {
      return rmw_time_equal(dur.to_rmw_time(), RMW_DURATION_INFINITE) ||
             rmw_time_equal(dur.to_rmw_time(), RMW_DURATION_UNSPECIFIED);
    };

  for (const auto & [gid, tracking] : publishers_) {
    auto deadline = tracking.info.qos_profile().deadline();
    const std::string subname = "PublishFrequency::" + tracking.topic_name;
    bool stale = (now - tracking.last_stats_timestamp) > config_.topic_statistics.stale_timeout;
    if (deadline_not_set(deadline)) {
      // No deadline, don't care
    } else if (stale) {
      // Haven't received topic statistics recently enough, likely this means it's not running
      msg->status.push_back(
        statusMsg(
          DiagnosticStatus::ERROR,
          "Stale topic statistics for publisher with deadline",
          subname));
    } else if (!tracking.period_stat.has_value()) {
      // Not stale yet, but also not yet received statistics info. Just waiting.
    } else if (!topic_period_ok(*tracking.period_stat, deadline)) {
      // Have received topic stats and it isn't in good range
      msg->status.push_back(
        statusMsg(
          DiagnosticStatus::WARN,
          "Publisher sending slower than promised deadline",
          subname));
    } else {
      msg->status.push_back(
        statusMsg(
          DiagnosticStatus::OK,
          "Publisher send frequency acceptable",
          subname));
    }
  }
  for (const auto & [gid, tracking] : subscriptions_) {
    auto deadline = tracking.info.qos_profile().deadline();
    const std::string subname = "ReceiveFrequency::" + tracking.topic_name;
    bool stale = (now - tracking.last_stats_timestamp) > config_.topic_statistics.stale_timeout;
    if (deadline_not_set(deadline)) {
      // No deadline, don't care
    } else if (stale) {
      // Haven't received topic statistics recently enough, likely this means it's not running
      msg->status.push_back(
        statusMsg(
          DiagnosticStatus::ERROR,
          "Stale topic statistics for subscription with deadline",
          subname));
    } else if (!tracking.period_stat.has_value()) {
      // Not stale yet, but also not yet received statistics info. Just waiting.
    } else if (!topic_period_ok(*tracking.period_stat, deadline)) {
      // Have received topic stats and it isn't in good range
      msg->status.push_back(
        statusMsg(
          DiagnosticStatus::WARN,
          "Subscription receiving slower than requested deadline",
          subname));
    } else {
      msg->status.push_back(
        statusMsg(
          DiagnosticStatus::OK,
          "Subscription receive frequency acceptable",
          subname));
    }
  }

  return msg;
}

void RosGraphMonitor::watch_for_updates()
{
  const auto wait_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::milliseconds(100));
  while (!shutdown_) {
    if (graph_change_event_->check_and_clear()) {
      update_graph();
      update_event_.set();
    }
    node_graph_->wait_for_graph_change(graph_change_event_, wait_ns);
  }
}

bool RosGraphMonitor::wait_for_update(std::chrono::milliseconds timeout)
{
  update_event_.wait_for(timeout);
  return update_event_.check_and_clear();
}

GraphMonitorConfiguration & RosGraphMonitor::config()
{
  return config_;
}


const GraphMonitorConfiguration & RosGraphMonitor::config() const
{
  return config_;
}

void RosGraphMonitor::on_topic_statistics(const rosgraph_monitor_msgs::msg::TopicStatistics & msg)
{
  for (const auto & stat : msg.statistics) {
    std::optional<RosRmwGid> maybe_gid;
    EndpointTrackingMap * endpoints = nullptr;
    if (stat.statistic_type == rosgraph_monitor_msgs::msg::TopicStatistic::PUBLISHED_PERIOD) {
      maybe_gid = lookup_publisher(stat.node_name, stat.topic_name);
      endpoints = &publishers_;
    } else if (stat.statistic_type == rosgraph_monitor_msgs::msg::TopicStatistic::RECEIVED_PERIOD) {
      maybe_gid = lookup_subscription(stat.node_name, stat.topic_name);
      endpoints = &subscriptions_;
    } else {
      continue;
    }

    if (!maybe_gid.has_value()) {
      continue;
    }
    assert(endpoints != nullptr);
    auto it = endpoints->find(*maybe_gid);
    if (it == endpoints->end()) {
      continue;
    }
    auto & [gid, tracking] = *it;
    tracking.last_stats_timestamp = rclcpp::Time(msg.timestamp, RCL_ROS_TIME);
    tracking.period_stat = stat;
  }
}

diagnostic_msgs::msg::DiagnosticStatus RosGraphMonitor::statusMsg(
  uint8_t level,
  const std::string & message,
  const std::string & subname) const
{
  diagnostic_msgs::msg::DiagnosticStatus msg;
  msg.level = level;
  msg.name = config_.diagnostic_namespace;
  if (!subname.empty()) {
    msg.name += "::" + subname;
  }
  msg.message = message;
  msg.hardware_id = "health";
  return msg;
}


}  // namespace rosgraph_monitor
