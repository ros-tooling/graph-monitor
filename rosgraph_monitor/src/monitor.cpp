// SPDX-FileCopyrightText: 2024 Bonsai Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "rosgraph_monitor/monitor.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"

std::size_t std::hash<RosRmwGid>::operator()(const RosRmwGid & id) const noexcept
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

using ParamQueries = rosgraph_monitor::QueryQueue<std::vector<std::string>>;
using DescriptorQueries = rosgraph_monitor::QueryQueue<std::vector<rcl_interfaces::msg::ParameterDescriptor>>;

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

std::vector<rcl_interfaces::msg::ParameterDescriptor> reconcile_descriptors(
  const std::vector<rcl_interfaces::msg::ParameterDescriptor> & recorded, const std::vector<std::string> & names)
{
  std::vector<rcl_interfaces::msg::ParameterDescriptor> descriptors;
  descriptors.reserve(names.size());
  for (const auto & name : names) {
    const auto known = std::find_if(
      recorded.begin(), recorded.end(), [&name](const auto & descriptor) { return descriptor.name == name; });
    if (known != recorded.end()) {
      descriptors.push_back(*known);
      continue;
    }
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.name = name;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET;
    descriptors.push_back(std::move(descriptor));
  }
  return descriptors;
}

void convert_maybe_inifite_durations(
  const rclcpp::Duration & duration, builtin_interfaces::msg::Duration & msg_duration)
{
  auto rmw_time = duration.to_rmw_time();
  if (rmw_time_equal(rmw_time, RMW_DURATION_INFINITE) || rmw_time_equal(rmw_time, RMW_DURATION_UNSPECIFIED)) {
    msg_duration.sec = 0;
    msg_duration.nanosec = 0;
  } else {
    msg_duration.sec = rmw_time.sec;
    msg_duration.nanosec = rmw_time.nsec;
  }
}

rosgraph_msgs::msg::QoSProfile to_msg(const rclcpp::QoS & qos_profile)
{
  rosgraph_msgs::msg::QoSProfile qos_msg;

  qos_msg.history = static_cast<uint8_t>(qos_profile.history());
  qos_msg.reliability = static_cast<uint8_t>(qos_profile.reliability());
  qos_msg.durability = static_cast<uint8_t>(qos_profile.durability());
  qos_msg.liveliness = static_cast<uint8_t>(qos_profile.liveliness());

  qos_msg.depth = qos_profile.depth();
  // Convert Duration fields - handle infinite durations
  convert_maybe_inifite_durations(qos_profile.deadline(), qos_msg.deadline);
  convert_maybe_inifite_durations(qos_profile.lifespan(), qos_msg.lifespan);
  convert_maybe_inifite_durations(qos_profile.liveliness_lease_duration(), qos_msg.liveliness_lease_duration);

  return qos_msg;
}

RosGraphMonitor::NodeTracking::NodeTracking(const std::string & name)
: name(name)
{}

RosGraphMonitor::EndpointTracking::EndpointTracking(
  const std::string & topic_name, const rclcpp::TopicEndpointInfo & info, const rclcpp::Time & now)
: topic_name(topic_name)
, node_name(
    info.node_namespace() == "/" ? info.node_namespace() + info.node_name()
                                 : info.node_namespace() + "/" + info.node_name())
, info(info)
, last_stats_timestamp(now)
{}

rosgraph_msgs::msg::Topic RosGraphMonitor::EndpointTracking::to_msg()
{
  rosgraph_msgs::msg::Topic topic_msg;
  topic_msg.name = topic_name;
  topic_msg.type.name = info.topic_type();
#ifndef ROS2_HUMBLE
  // Humble's rclcpp does not expose the endpoint's type hash, so it is left at its default there.
  const auto & type_hash = info.topic_type_hash();
  topic_msg.type.hash.version = type_hash.version;
  std::copy(std::begin(type_hash.value), std::end(type_hash.value), topic_msg.type.hash.value.begin());
#endif
  topic_msg.qos = rosgraph_monitor::to_msg(info.qos_profile());
  return topic_msg;
}

RosGraphMonitor::RosGraphMonitor(
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
  std::function<rclcpp::Time()> now_fn,
  rclcpp::Logger logger,
  std::shared_ptr<ParameterServiceClient> parameter_client,
  GraphMonitorConfiguration config,
  GraphChangeCallback change_callback)
: config_(config)
, now_fn_(now_fn)
, node_graph_(node_graph)
, logger_(logger)
, graph_change_event_(node_graph->get_graph_event())
, parameter_client_(parameter_client)
, graph_(GraphTracking{})
, graph_change_callback_(change_callback)
, descriptor_queries_(
    [this](const std::string & node_name, DescriptorQueries::Done done) {
      const auto names = recorded_parameter_names(node_name);
      if (names.empty()) {
        done(std::nullopt);
        return;
      }
      if (!parameter_client_->is_ready(node_name)) {
        RCLCPP_DEBUG(logger_, "Parameter services for %s are not available yet", node_name.c_str());
        done(std::nullopt);
        return;
      }
      parameter_client_->describe_parameters(node_name, names, std::move(done));
    },
    [this](const std::string & node_name, std::vector<rcl_interfaces::msg::ParameterDescriptor> descriptors) {
      on_node_descriptors(node_name, std::move(descriptors));
    },
    DescriptorQueries::Options{
      config.parameters.max_concurrent,
      config.parameters.timeout,
      config.parameters.retry_delay,
      config.parameters.first_retry_delay})
, param_queries_(
    [this](const std::string & node_name, ParamQueries::Done done) {
      if (!parameter_client_->is_ready(node_name)) {
        RCLCPP_DEBUG(logger_, "Parameter services for %s are not available yet", node_name.c_str());
        done(std::nullopt);
        return;
      }
      parameter_client_->list_parameters(node_name, std::move(done));
    },
    [this](const std::string & node_name, std::vector<std::string> parameter_names) {
      on_node_parameters(node_name, std::move(parameter_names));
    },
    ParamQueries::Options{
      config.parameters.max_concurrent,
      config.parameters.timeout,
      config.parameters.retry_delay,
      config.parameters.first_retry_delay})
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
  const auto node_names = node_graph_->get_node_names();
  const auto topics_and_types = node_graph_->get_topic_names_and_types();

  NodeChanges changes;
  {
    auto graph = graph_.lock();
    changes = track_node_updates(node_names, *graph);
    track_endpoint_updates(topics_and_types, *graph);
  }

  // Outside the lock: the query queue's thread and the change callback both take it.
  for (const auto & node_name : changes.to_observe) {
    param_queries_.request(node_name);
  }
  for (const auto & node_name : changes.departed) {
    param_queries_.cancel(node_name);
    descriptor_queries_.cancel(node_name);
    parameter_client_->forget(node_name);
  }
  notify_graph_change();
}

bool RosGraphMonitor::ignore_node(const std::string & node_name, GraphTracking & graph)
{
  if (node_name == "_NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_") {
    return true;
  }
  if (match_any_prefixes(config_.nodes.ignore_prefixes, node_name)) {
    auto [it, inserted] = graph.ignored_nodes.insert(node_name);
    if (inserted) {
      RCLCPP_DEBUG(logger_, "Ignoring new node: %s", node_name.c_str());
    }
    return true;
  }
  return false;
}

RosGraphMonitor::NodeChanges RosGraphMonitor::track_node_updates(
  const std::vector<std::string> & observed_node_names, GraphTracking & graph)
{
  NodeChanges changes;

  // Mark all stale as base state
  for (auto & [node_name, tracking] : graph.nodes) {
    tracking.stale = true;
  }
  // Look at current node list, detect new and returned
  for (const auto & node_name : observed_node_names) {
    if (ignore_node(node_name, graph)) {
      continue;
    }

    NodeTracking tracking{node_name};
    auto [it, inserted] = graph.nodes.emplace(node_name, tracking);

    if (inserted) {
      RCLCPP_DEBUG(logger_, "New node: %s", node_name.c_str());
    } else {
      NodeTracking & tracking = it->second;
      tracking.stale = false;
      if (tracking.missing) {
        RCLCPP_INFO(logger_, "Node %s came back", node_name.c_str());
        tracking.missing = false;
        graph.returned_nodes.insert(node_name);
        tracking.params.reset();
      }
    }
  }
  // Check which nodes are still stale - they weren't observed
  for (auto & [node_name, tracking] : graph.nodes) {
    if (tracking.stale && !tracking.missing) {
      RCLCPP_WARN(logger_, "Node %s went missing", node_name.c_str());
      tracking.missing = true;
      changes.departed.push_back(node_name);
      graph.returned_nodes.erase(node_name);
    } else if (!tracking.missing && !tracking.params.has_value()) {
      changes.to_observe.push_back(node_name);
    }
  }

  return changes;
}

std::optional<RosGraphMonitor::EndpointTrackingMap::iterator> RosGraphMonitor::add_publisher(
  const std::string & topic_name, const rclcpp::TopicEndpointInfo & info, GraphTracking & graph)
{
  EndpointTracking proposed_tracking(topic_name, info, now_fn_());
  if (ignore_node(proposed_tracking.node_name, graph)) {
    return std::nullopt;
  }
  auto [it, inserted] = graph.publishers.emplace(info.endpoint_gid(), proposed_tracking);
  auto & [gid, tracking] = *it;
  graph.publisher_lookup.insert_or_assign(std::make_pair(tracking.node_name, tracking.topic_name), gid);
  if (inserted) {
    RCLCPP_DEBUG(
      logger_,
      "New Publisher: %s::%s (%s)",
      tracking.node_name.c_str(),
      tracking.topic_name.c_str(),
      gid_to_str(tracking.info.endpoint_gid()).c_str());
  }
  return it;
}

std::optional<RosGraphMonitor::EndpointTrackingMap::iterator> RosGraphMonitor::add_subscription(
  const std::string & topic_name, const rclcpp::TopicEndpointInfo & info, GraphTracking & graph)
{
  EndpointTracking proposed_tracking(topic_name, info, now_fn_());
  if (ignore_node(proposed_tracking.node_name, graph)) {
    return std::nullopt;
  }
  auto [it, inserted] = graph.subscriptions.emplace(info.endpoint_gid(), proposed_tracking);
  auto & [gid, tracking] = *it;
  graph.subscription_lookup.insert_or_assign(std::make_pair(tracking.node_name, tracking.topic_name), gid);
  if (inserted) {
    RCLCPP_DEBUG(
      logger_,
      "New Subscription: %s::%s (%s)",
      tracking.node_name.c_str(),
      tracking.topic_name.c_str(),
      gid_to_str(tracking.info.endpoint_gid()).c_str());
  }
  return it;
}

bool RosGraphMonitor::topic_period_ok(
  const rosgraph_monitor_msgs::msg::TopicStatistic & stat, const rclcpp::Duration & deadline) const
{
  const std::chrono::nanoseconds measured_period = rclcpp::Duration(stat.mean).to_chrono<std::chrono::nanoseconds>();
  const std::chrono::nanoseconds chrono_deadline = deadline.to_chrono<std::chrono::nanoseconds>();
  if (measured_period < chrono_deadline) {
    return true;
  }
  const auto period_error = measured_period - chrono_deadline;
  const auto allowed_error = chrono_deadline * config_.topic_statistics.deadline_allowed_error;
  return period_error <= allowed_error;
}

void RosGraphMonitor::track_endpoint_updates(const TopicsToTypes & observed_topics_and_types, GraphTracking & graph)
{
  // Mark all stale as base state
  for (auto & [gid, tracking] : graph.publishers) {
    tracking.stale = true;
  }
  for (auto & [gid, tracking] : graph.subscriptions) {
    tracking.stale = true;
  }
  for (auto & [topic_name, counts] : graph.topic_endpoint_counts) {
    counts.pubs = 0;
    counts.subs = 0;
  }

  // Look over all currently observed topics for endpoint changes
  for (const auto & [topic_name, topic_types] : observed_topics_and_types) {
    // Assumption: "multiple types on the topic" is an error already handled elsewhere
    bool count_topic = config_.continuity.ignore_topic_names.count(topic_name) == 0 &&
                       config_.continuity.ignore_topic_types.count(topic_types[0]) == 0;
    auto & endpoint_counts = graph.topic_endpoint_counts[topic_name];

    // Check all publishers
    for (const auto & endpoint_info : node_graph_->get_publishers_info_by_topic(topic_name)) {
      auto maybe_it = add_publisher(topic_name, endpoint_info, graph);
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
      auto maybe_it = add_subscription(topic_name, endpoint_info, graph);
      if (!maybe_it.has_value()) {
        continue;
      }
      auto & [gid, tracking] = **maybe_it;

      bool count_subs_from_node = config_.continuity.ignore_subscriber_nodes.count(tracking.node_name) == 0;
      if (count_topic && count_subs_from_node) {
        endpoint_counts.subs++;
      }
      tracking.stale = false;
    }
  }

  // Check for any stale endpoints (not seen this iteration) - they're missing.
  // For now just delete them, there isn't a meaningful health case to track at this point
  // Also remove endpoints from missing node, that node missing is the important error.
  for (EndpointTrackingMap * endpoints : {&graph.publishers, &graph.subscriptions}) {
    for (auto it = endpoints->begin(); it != endpoints->end();) {
      auto & [gid, tracking] = *it;
      const auto node_it = graph.nodes.find(tracking.node_name);
      bool node_not_tracked = node_it == graph.nodes.end();
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
    for (auto it = graph.topic_endpoint_counts.begin(); it != graph.topic_endpoint_counts.end();) {
      auto & [topic_name, counts] = *it;
      // Check counts to see if any pubs or subs don't have matches
      if (counts.pubs > 0 && counts.subs == 0) {
        graph.pubs_with_no_subs.insert(topic_name);
      }
      if (counts.subs > 0 && counts.pubs == 0) {
        graph.subs_with_no_pubs.insert(topic_name);
      }
      // Delete any lingering tracking with no matches at all, the topic doesn't exist anymore
      if (counts.pubs == 0 && counts.subs == 0) {
        graph.pubs_with_no_subs.erase(topic_name);
        graph.subs_with_no_pubs.erase(topic_name);
        it = graph.topic_endpoint_counts.erase(it);
      } else {
        it++;
      }
    }
  }
}

void RosGraphMonitor::evaluate(std::vector<diagnostic_msgs::msg::DiagnosticStatus> & status)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  auto now = now_fn_();

  auto graph_guard = graph_.lock();
  auto & graph = *graph_guard;

  // Nodes
  {
    diagnostic_updater::DiagnosticStatusWrapper nodes_status;
    statusWrapper(nodes_status, DiagnosticStatus::OK, "Nodes OK", "nodes");
    size_t missing_optional_nodes = 0;
    size_t missing_required_nodes = 0;
    for (const auto & [node_name, node_info] : graph.nodes) {
      if (node_info.missing) {
        if (match_any_prefixes(config_.nodes.warn_only_prefixes, node_name)) {
          nodes_status.add("Optional node missing", node_name);
          missing_optional_nodes++;
        } else {
          nodes_status.add("Required node missing", node_name);
          missing_required_nodes++;
        }
      }
    }
    for (const std::string & node_name : graph.returned_nodes) {
      nodes_status.addf("Node came back: %s", node_name.c_str());
    }
    if (missing_required_nodes > 0) {
      nodes_status.summaryf(DiagnosticStatus::ERROR, "%d required node(s) missing.", missing_required_nodes);
    }
    if (missing_optional_nodes > 0) {
      nodes_status.mergeSummaryf(DiagnosticStatus::WARN, "%d optional node(s) missing.", missing_optional_nodes);
    }
    status.push_back(nodes_status);
  }

  // Continuity
  {
    diagnostic_updater::DiagnosticStatusWrapper continuity_status;
    statusWrapper(continuity_status, DiagnosticStatus::OK, "Graph continuity OK", "continuity");
    for (const auto & [topic_name, counts] : graph.topic_endpoint_counts) {
      if (counts.pubs > 0 && graph.subs_with_no_pubs.erase(topic_name) > 0) {
        continuity_status.add("Dead sink cleared. Topic now has publisher(s).", topic_name);
      }
      if (counts.subs > 0 && graph.pubs_with_no_subs.erase(topic_name) > 0) {
        continuity_status.add("Leaf topic cleared. Topic now has subscriber(s).", topic_name);
      }
    }
    size_t continuity_issues = 0;
    for (const auto & topic_name : graph.pubs_with_no_subs) {
      continuity_status.add("Leaf topic (No subscriptions): Topic", topic_name);
      continuity_issues++;
    }
    for (const auto & topic_name : graph.subs_with_no_pubs) {
      continuity_status.add("Dead sink (No publishers): Topic", topic_name);
      continuity_issues++;
    }
    if (continuity_issues > 0) {
      continuity_status.summaryf(DiagnosticStatus::WARN, "%d continuity issues detected.", continuity_issues);
    }
    status.push_back(continuity_status);
  }

  // Frequency
  auto deadline_not_set = [](const rclcpp::Duration & dur) {
    return rmw_time_equal(dur.to_rmw_time(), RMW_DURATION_INFINITE) ||
           rmw_time_equal(dur.to_rmw_time(), RMW_DURATION_UNSPECIFIED);
  };
  {
    diagnostic_updater::DiagnosticStatusWrapper pub_freq_status;
    statusWrapper(pub_freq_status, DiagnosticStatus::OK, "Publish frequencies OK", "publish_frequency");

    size_t pub_freq_errors = 0;
    size_t pub_freq_warns = 0;
    for (const auto & [gid, tracking] : graph.publishers) {
      auto deadline = tracking.info.qos_profile().deadline();
      const std::string & topic = tracking.topic_name;
      bool stale = (now - tracking.last_stats_timestamp) > config_.topic_statistics.stale_timeout;

      bool no_deadline = deadline_not_set(deadline);
      bool ignore_deadline = config_.topic_statistics.ignore_topics.count(topic) > 0;
      bool mandatory_deadline = config_.topic_statistics.mandatory_topics.count(topic) > 0;
      bool deadline_not_required = (no_deadline && !mandatory_deadline) || ignore_deadline;

      if (deadline_not_required) {
        // No deadline, don't care
      } else if (stale) {
        // Haven't received topic statistics recently enough, likely this means it's not running
        pub_freq_status.add("Stale topic statistics for publisher with deadline", topic);
        pub_freq_errors++;
      } else if (!tracking.period_stat.has_value()) {
        // Not stale yet, but also not yet received statistics info. Just waiting.
      } else if (!topic_period_ok(*tracking.period_stat, deadline)) {
        // Have received topic stats and it isn't in good range
        pub_freq_status.add("Publisher sending too slowly", topic);
        pub_freq_warns++;
      } else {
        pub_freq_status.add("Publisher frequency OK", topic);
      }
    }
    if (pub_freq_errors > 0) {
      pub_freq_status.summaryf(DiagnosticStatus::ERROR, "Frequency errors detected.", pub_freq_errors);
    }
    if (pub_freq_warns > 0) {
      pub_freq_status.summaryf(DiagnosticStatus::WARN, "Frequency warnings detected.", pub_freq_warns);
    }
    status.push_back(pub_freq_status);
  }
  {
    diagnostic_updater::DiagnosticStatusWrapper sub_freq_status;
    statusWrapper(sub_freq_status, DiagnosticStatus::OK, "Receive frequencies OK", "receive_frequency");
    size_t sub_freq_errors = 0;
    size_t sub_freq_warns = 0;
    for (const auto & [gid, tracking] : graph.subscriptions) {
      auto deadline = tracking.info.qos_profile().deadline();
      const std::string & topic = tracking.topic_name;
      bool stale = (now - tracking.last_stats_timestamp) > config_.topic_statistics.stale_timeout;
      if (deadline_not_set(deadline)) {
        // No deadline, don't care
      } else if (stale) {
        // Haven't received topic statistics recently enough, likely this means it's not running
        sub_freq_status.add("Stale topic statistics for subscription with deadline", topic);
        sub_freq_errors++;
      } else if (!tracking.period_stat.has_value()) {
        // Not stale yet, but also not yet received statistics info. Just waiting.
      } else if (!topic_period_ok(*tracking.period_stat, deadline)) {
        // Have received topic stats and it isn't in good range
        sub_freq_status.add("Subscription receiving too slowly", topic);
        sub_freq_warns++;
      } else {
        sub_freq_status.add("Subscription receive frequency OK", topic);
      }
    }
    if (sub_freq_errors > 0) {
      sub_freq_status.summaryf(DiagnosticStatus::ERROR, "Frequency errors detected.", sub_freq_errors);
    }
    if (sub_freq_warns > 0) {
      sub_freq_status.summaryf(DiagnosticStatus::WARN, "Frequency warnings detected.", sub_freq_warns);
    }
    status.push_back(sub_freq_status);
  }
}

void RosGraphMonitor::watch_for_updates()
{
  const auto wait_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::milliseconds(100));
  while (!shutdown_) {
    if (graph_change_event_->check_and_clear()) {
      // A graph query only fails once the context is gone, so watching ends instead of spinning on a dead context.
      try {
        update_graph();
      } catch (const std::exception & e) {
        RCLCPP_DEBUG(logger_, "Stopping graph watch: %s", e.what());
        break;
      }
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
  auto graph_guard = graph_.lock();
  auto & graph = *graph_guard;

  for (const auto & stat : msg.statistics) {
    RosRmwGid gid;
    EndpointTrackingMap * endpoints = nullptr;
    if (stat.statistic_type == rosgraph_monitor_msgs::msg::TopicStatistic::PUBLISHED_PERIOD) {
      try {
        gid = graph.publisher_lookup.at(std::make_pair(stat.node_name, stat.topic_name));
      } catch (const std::out_of_range &) {
        continue;
      }
      endpoints = &graph.publishers;
    } else if (stat.statistic_type == rosgraph_monitor_msgs::msg::TopicStatistic::RECEIVED_PERIOD) {
      try {
        gid = graph.subscription_lookup.at(std::make_pair(stat.node_name, stat.topic_name));
      } catch (const std::out_of_range &) {
        continue;
      }
      endpoints = &graph.subscriptions;
    } else {
      continue;
    }

    assert(endpoints != nullptr);
    auto it = endpoints->find(gid);
    if (it == endpoints->end()) {
      continue;
    }
    auto & [tracked_gid, tracking] = *it;
    tracking.last_stats_timestamp = rclcpp::Time(msg.timestamp, RCL_ROS_TIME);
    tracking.period_stat = stat;
  }
}

void RosGraphMonitor::statusWrapper(
  diagnostic_updater::DiagnosticStatusWrapper & msg,
  uint8_t level,
  const std::string & message,
  const std::string & subname) const
{
  msg.summary(level, message);
  msg.name = config_.diagnostic_namespace;
  if (!subname.empty()) {
    msg.name += "/" + subname;
  }
  msg.hardware_id = "health";
}

void RosGraphMonitor::fill_rosgraph_msg(rosgraph_msgs::msg::Graph & msg)
{
  msg.nodes.clear();

  auto graph_guard = graph_.lock();
  auto & graph = *graph_guard;
  RCLCPP_DEBUG(logger_, "EVENT rosgraph message with %zu nodes", graph.nodes.size());

  for (const auto & [node_name, node_info] : graph.nodes) {
    if (ignore_node(node_name, graph) || node_info.missing || node_info.stale) {
      continue;
    }

    rosgraph_msgs::msg::Node node_msg;
    node_msg.name = node_name;

    if (node_info.params) {
      node_msg.parameters = node_info.params->descriptors;
      node_msg.parameter_values = node_info.params->values;
    }

    // Add publishers for this node
    for (auto & [gid, tracking] : graph.publishers) {
      if (tracking.node_name == node_name) {
        auto topic_msg = tracking.to_msg();
        node_msg.publishers.push_back(topic_msg);
      }
    }

    // Add subscriptions for this node
    for (auto & [gid, tracking] : graph.subscriptions) {
      if (tracking.node_name == node_name) {
        auto topic_msg = tracking.to_msg();
        node_msg.subscriptions.push_back(topic_msg);
      }
    }
    msg.nodes.push_back(node_msg);
  }
}

void RosGraphMonitor::notify_graph_change()
{
  if (graph_change_callback_) {
    rosgraph_msgs::msg::Graph msg;
    fill_rosgraph_msg(msg);
    graph_change_callback_(msg);
  }
}

std::vector<std::string> RosGraphMonitor::recorded_parameter_names(const std::string & node_name)
{
  std::vector<std::string> names;
  auto graph = graph_.lock();
  const auto it = graph->nodes.find(node_name);
  if (it == graph->nodes.end() || !it->second.params) {
    return names;
  }
  const auto & descriptors = it->second.params->descriptors;
  names.reserve(descriptors.size());
  for (const auto & descriptor : descriptors) {
    names.push_back(descriptor.name);
  }
  return names;
}

void RosGraphMonitor::on_node_parameters(const std::string & node_name, std::vector<std::string> parameter_names)
{
  RCLCPP_DEBUG(logger_, "Observed %zu parameters for node %s", parameter_names.size(), node_name.c_str());

  bool changed = false;
  {
    // Runs on the query queue's thread, while the watch thread may be rebuilding the graph.
    auto graph = graph_.lock();
    auto it = graph->nodes.find(node_name);
    if (it == graph->nodes.end()) {
      // The node left the graph between the request and the response.
      return;
    }
    auto & params = it->second.params;
    // A first observation is a change even when the node has no parameters.
    const bool first_observation = !params.has_value();
    if (first_observation) {
      params.emplace();
    }
    auto descriptors = reconcile_descriptors(params->descriptors, parameter_names);
    changed = first_observation || params->descriptors != descriptors;
    params->descriptors = std::move(descriptors);
  }

  // Outside the lock: the descriptor query's start call takes it.
  if (!parameter_names.empty()) {
    descriptor_queries_.request(node_name);
  }
  if (changed) {
    notify_graph_change();
  }
}

void RosGraphMonitor::on_node_descriptors(
  const std::string & node_name, std::vector<rcl_interfaces::msg::ParameterDescriptor> descriptors)
{
  RCLCPP_DEBUG(logger_, "Observed %zu parameter descriptors for node %s", descriptors.size(), node_name.c_str());

  bool changed = false;
  {
    // Runs on the query queue's thread, while the watch thread may be rebuilding the graph.
    auto graph = graph_.lock();
    auto it = graph->nodes.find(node_name);
    if (it == graph->nodes.end() || !it->second.params) {
      // The node left the graph, or lost its observation, between the request and the response.
      return;
    }
    auto & params = *it->second.params;
    changed = params.descriptors != descriptors;
    params.descriptors = std::move(descriptors);
  }

  if (changed) {
    notify_graph_change();
  }
}

}  // namespace rosgraph_monitor
