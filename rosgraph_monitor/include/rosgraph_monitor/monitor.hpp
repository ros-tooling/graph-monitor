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

#ifndef ROSGRAPH_MONITOR__MONITOR_HPP_
#define ROSGRAPH_MONITOR__MONITOR_HPP_

#include <thread>
#include <map>
#include <memory>
#include <optional>
#include <regex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/time.hpp"
#include "rosgraph_monitor_msgs/msg/topic_statistics.hpp"

#include "rosgraph_monitor/event.hpp"

typedef std::array<uint8_t, RMW_GID_STORAGE_SIZE> RosRmwGid;

/// @brief Provide a std::hash specialization so we can use RMW GID as a map key
template<>
struct std::hash<RosRmwGid>
{
  std::size_t operator()(const RosRmwGid & id) const noexcept;
};

template<>
struct std::hash<std::pair<std::string, std::string>>
{
  std::size_t operator()(const std::pair<std::string, std::string> & value) const noexcept;
};


namespace rosgraph_monitor
{

std::string gid_to_str(const uint8_t gid[RMW_GID_STORAGE_SIZE]);
std::string gid_to_str(const RosRmwGid & gid);

struct GraphMonitorConfiguration
{
  std::string diagnostic_namespace{"Rosgraph"};

  struct NodeChecks
  {
    // Matching nodes will not be considered in any graph analysis
    std::vector<std::string> ignore_prefixes;
    // Downgrade ERROR to WARN for matching nodes when they are missing.
    std::vector<std::string> warn_only_prefixes;
  } nodes;

  struct ContinuityChecks
  {
    // If set, don't perform any continuity checks
    bool enable = true;
    // These nodes don't count for subscriptions when reporting discontinuity
    std::unordered_set<std::string> ignore_subscriber_nodes;
    // Any topics of these types will be ignored entirely for continuity checks
    std::unordered_set<std::string> ignore_topic_types;
    // Any topics with these names will be ignored entirely for continuity checks
    std::unordered_set<std::string> ignore_topic_names;
  } continuity;

  struct TopicStatisticsChecks
  {
    // What fraction of the promised deadline the topic statistics may err by
    // and still be considered compliant.
    // For example if 0.1, then a deadline of 10 milliseconds will be considered OK
    // if average measured interval is 9-11 milliseconds
    // This equates to: expectation of 100Hz will be considered OK from 90.9-111.1Hz
    float deadline_allowed_error = 0.1;
    // For topics whose frequency is tracked, if new statistics are not received within this
    // time frame then the statistic will be reported as stale with an ERROR.
    std::chrono::milliseconds stale_timeout{3000};
  } topic_statistics;
};

/// @brief Monitors the ROS application graph, providing diagnostics about its health.
class RosGraphMonitor
{
public:
  /// @brief Constructor
  /// @param config Includes/excludes the entities to care about in diagnostic reporting
  /// @param now_fn Function to fetch the current time as defined in the owning context
  /// @param node_graph Interface from owning Node to retrieve information about the ROS graph
  /// @param logger
  RosGraphMonitor(
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    std::function<rclcpp::Time()> now_fn,
    rclcpp::Logger logger,
    GraphMonitorConfiguration config = GraphMonitorConfiguration{});

  virtual ~RosGraphMonitor();

  /// @brief Return diagnostics of latest graph understanding
  /// @return A message filled with all current conditions. May be empty array, but never nullptr
  std::unique_ptr<diagnostic_msgs::msg::DiagnosticArray> evaluate();

  /// @brief Wait until next graph update is integrated into the monitor
  /// @param timeout
  /// @return True if there were graph updates detected, or false on timeout
  bool wait_for_update(std::chrono::milliseconds timeout);

  /// @return Mutable reference to the configuration, for updating
  GraphMonitorConfiguration & config();

  /// @return Const reference to configuration
  const GraphMonitorConfiguration & config() const;

  /// @brief Integrate new topic statistics input to determine if topics are meeting contracts.
  /// @param statistics Incoming statistics list
  void on_topic_statistics(const rosgraph_monitor_msgs::msg::TopicStatistics & statistics);

protected:
  /* Types */

  /// @brief Keeps flags for tracking observed nodes over time
  struct NodeTracking
  {
    bool missing = false;
    bool stale = false;
  };

  /// @brief Keeps aggregate info about a topic as a whole over time
  struct TopicTracking
  {
    size_t pubs = 0;
    size_t subs = 0;
  };

  /// @brief Keeps information and flags about observed Publishers/Subscriptions over time
  struct EndpointTracking
  {
    bool stale = false;
    const std::string topic_name;
    const std::string node_name;
    const rclcpp::TopicEndpointInfo info;

    rclcpp::Time last_stats_timestamp;
    std::optional<rosgraph_monitor_msgs::msg::TopicStatistic> period_stat;

    EndpointTracking(
      const std::string & topic_name,
      const rclcpp::TopicEndpointInfo & info,
      const rclcpp::Time & now);
  };

  typedef std::map<std::string, std::vector<std::string>> TopicsToTypes;
  typedef std::unordered_map<RosRmwGid, EndpointTracking> EndpointTrackingMap;
  typedef std::unordered_set<RosRmwGid> EndpointSet;
  typedef std::pair<std::string, std::string> NodeAndTopic;

  /* Methods */

  /// @brief Update internal graph representation and detect changes
  void update_graph();

  /// @brief Called in thread to watch the graph in infinite loop and rebuild tracking on changes.
  void watch_for_updates();

  /// @brief Should we skip tracking this node?
  /// @param node_name
  /// @return Whether to ignore tracking the node
  bool ignore_node(const std::string & node_name);

  /// @brief Check current observed state against our tracked state, updating tracking info
  /// @param observed_node_names
  void track_node_updates(
    const std::vector<std::string> & observed_node_names);

  /// @brief Check current observed state against our tracked state, updating tracking info
  /// @param observed_topics_and_types
  void track_endpoint_updates(const TopicsToTypes & observed_topics_and_types);

  /// @return Iterator to existing or added publisher, or nullopt if node ignored
  std::optional<EndpointTrackingMap::iterator> add_publisher(
    const std::string & topic_name, const rclcpp::TopicEndpointInfo & info);

  /// @return GID of a publisher if found, else nullopt if such an endpoint not tracked.
  std::optional<RosRmwGid> lookup_publisher(
    const std::string & node_name, const std::string & topic_name) const;

  /// @return Iterator to existing or added publisher, or nullopt if node ignored
  std::optional<EndpointTrackingMap::iterator> add_subscription(
    const std::string & topic_name, const rclcpp::TopicEndpointInfo & info);

  /// @return GID of a publisher if found, else nullopt if such an endpoint not tracked.
  std::optional<RosRmwGid> lookup_subscription(
    const std::string & node_name, const std::string & topic_name) const;

  bool topic_period_ok(
    const rosgraph_monitor_msgs::msg::TopicStatistic & stat,
    const rclcpp::Duration & deadline) const;

  diagnostic_msgs::msg::DiagnosticStatus statusMsg(
    uint8_t level,
    const std::string & message,
    const std::string & subname = "") const;

  /* Members */

  // Configuration
  GraphMonitorConfiguration config_;
  std::function<rclcpp::Time()> now_fn_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_;
  rclcpp::Logger logger_;

  // Execution model
  std::atomic_bool shutdown_ = false;
  rclcpp::Event::SharedPtr graph_change_event_;
  std::thread watch_thread_;
  Event update_event_;

  // Graph cache
  std::unordered_map<std::string, NodeTracking> nodes_;
  EndpointTrackingMap publishers_;
  EndpointTrackingMap subscriptions_;
  std::unordered_map<NodeAndTopic, RosRmwGid> publisher_lookup_;
  std::unordered_map<NodeAndTopic, RosRmwGid> subscription_lookup_;

  // Tracking outputs
  std::unordered_set<std::string> ignored_nodes_;
  std::unordered_set<std::string> returned_nodes_;
  std::unordered_map<std::string, TopicTracking> topic_endpoint_counts_;
  std::unordered_set<std::string> pubs_with_no_subs_;  // a.k.a. "leaf topics"
  std::unordered_set<std::string> subs_with_no_pubs_;  // a.k.a. "dead sinks"
};

}  // namespace rosgraph_monitor

#endif  // ROSGRAPH_MONITOR__MONITOR_HPP_
