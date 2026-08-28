// SPDX-FileCopyrightText: 2024 Bonsai Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef ROSGRAPH_MONITOR__MONITOR_HPP_
#define ROSGRAPH_MONITOR__MONITOR_HPP_

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <regex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "rcl_interfaces/msg/list_parameters_result.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/time.hpp"
#include "rosgraph_monitor/event.hpp"
#include "rosgraph_monitor/mutex_protected.hpp"
#include "rosgraph_monitor/parameter_collector.hpp"
#include "rosgraph_monitor/query_queue.hpp"
#include "rosgraph_monitor_msgs/msg/topic_statistics.hpp"
#include "rosgraph_msgs/msg/graph.hpp"
#include "rosgraph_msgs/msg/qo_s_profile.hpp"
#include "rosgraph_msgs/msg/topic.hpp"

typedef std::array<uint8_t, RMW_GID_STORAGE_SIZE> RosRmwGid;

// Optional trigger for monitor to call, to alert owner of updates to the graph
typedef std::function<void(rosgraph_msgs::msg::Graph &)> GraphChangeCallback;

/// @brief Provide a std::hash specialization so we can use RMW GID as a map key
template <>
struct std::hash<RosRmwGid>
{
  std::size_t operator()(const RosRmwGid & id) const noexcept;
};

template <>
struct std::hash<std::pair<std::string, std::string>>
{
  std::size_t operator()(const std::pair<std::string, std::string> & value) const noexcept;
};

namespace rosgraph_monitor
{

std::string gid_to_str(const uint8_t gid[RMW_GID_STORAGE_SIZE]);
std::string gid_to_str(const RosRmwGid & gid);

/// @brief What is recorded about one node's parameters.
struct RecordedParameters
{
  /// One descriptor per parameter name, in the order the parameters are published in.
  std::vector<rcl_interfaces::msg::ParameterDescriptor> descriptors;
  /// The value recorded for a parameter name, absent for a name whose value is not known yet.
  std::unordered_map<std::string, rcl_interfaces::msg::ParameterValue> values;
  /// The names whose values were recorded from parameter events, which outrank query responses.
  std::unordered_set<std::string> event_valued;
};

/// @brief Bring a set of recorded descriptors in line with a freshly observed list of names.
/// @return One descriptor per observed name in the observed order,
/// followed by the kept descriptors the names leave out, in the recorded order.
/// A name already among `recorded` keeps its recorded descriptor, a new name gets a NOT_SET
/// descriptor, and a recorded name absent from both `names` and `keep` is dropped.
std::vector<rcl_interfaces::msg::ParameterDescriptor> reconcile_descriptors(
  const std::vector<rcl_interfaces::msg::ParameterDescriptor> & recorded,
  const std::vector<std::string> & names,
  const std::unordered_set<std::string> & keep);

/// @brief Update a set of recorded descriptors from a freshly described set.
/// @return One descriptor per recorded name, in the recorded order.
/// A recorded name that `described` covers takes the described descriptor, a recorded name absent
/// from `described` keeps its recorded descriptor, and a described name that is not recorded is dropped.
std::vector<rcl_interfaces::msg::ParameterDescriptor> merge_descriptors(
  const std::vector<rcl_interfaces::msg::ParameterDescriptor> & recorded,
  const std::vector<rcl_interfaces::msg::ParameterDescriptor> & described);

/// @brief The names one parameter-value query asked for, paired with the values the node answered.
/// The values are positional against the names.
using ValueResponse = std::pair<std::vector<std::string>, std::vector<rcl_interfaces::msg::ParameterValue>>;

/// @brief Pair each requested name of a parameter-value response with the value at its position.
/// @return One entry per requested name that the response carries a value for.
std::unordered_map<std::string, rcl_interfaces::msg::ParameterValue> pair_values(const ValueResponse & response);

/// @brief Lay recorded values out against a set of descriptors.
/// @return One value per descriptor, in the descriptors' order.
/// Empty when any descriptor's name has no recorded value.
std::vector<rcl_interfaces::msg::ParameterValue> parallel_values(
  const std::vector<rcl_interfaces::msg::ParameterDescriptor> & descriptors,
  const std::unordered_map<std::string, rcl_interfaces::msg::ParameterValue> & values);

struct GraphMonitorConfiguration
{
  std::string diagnostic_namespace{"rosgraph"};

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

  /// Read once at construction.
  struct ParameterObservation
  {
    // How many nodes may have parameter queries in flight at once
    size_t max_concurrent = 4;
    // A query with no response by this point has failed and will be retried
    std::chrono::milliseconds timeout{10000};
    // Longest wait between attempts at one node
    std::chrono::milliseconds retry_delay{5000};
    // Wait before the first retry at a node, doubling per consecutive failure up to retry_delay
    std::chrono::milliseconds first_retry_delay{500};
  } parameters;

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
    // List of topics that must exist and have deadlines
    std::unordered_set<std::string> mandatory_topics;
    // List of topics that should not be considered for frequency checks
    // (e.g. topics that are known to be misconfigured and not meeting their deadlines
    std::unordered_set<std::string> ignore_topics;
  } topic_statistics;
};

/// @brief Monitors the ROS application graph, providing diagnostics about its health.
class RosGraphMonitor
{
public:
  /// @brief Constructor
  /// @param node_graph Interface from owning Node to retrieve information about the ROS graph
  /// @param now_fn Function to fetch the current time as defined in the owning context
  /// @param logger
  /// @param config Includes/excludes the entities to care about in diagnostic reporting
  /// @param parameter_client Interface to query parameters of nodes by name
  RosGraphMonitor(
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    std::function<rclcpp::Time()> now_fn,
    rclcpp::Logger logger,
    std::shared_ptr<ParameterServiceClient> parameter_client,
    GraphMonitorConfiguration config = GraphMonitorConfiguration{},
    GraphChangeCallback change_callback = GraphChangeCallback());

  virtual ~RosGraphMonitor();

  /// @brief Return diagnostics of latest graph understanding
  /// @return A message filled with all current conditions. May be empty array, but never nullptr
  void evaluate(std::vector<diagnostic_msgs::msg::DiagnosticStatus> & status);

  /// @brief Wait until next graph update is integrated into the monitor
  /// @param timeout
  /// @return True if there were graph updates detected, or false on timeout
  bool wait_for_update(std::chrono::milliseconds timeout);

  /// @return Mutable reference to the configuration, fo;r updating
  GraphMonitorConfiguration & config();

  /// @return Const reference to configuration
  const GraphMonitorConfiguration & config() const;

  /// @brief Integrate new topic statistics input to determine if topics are meeting contracts.
  /// @param statistics Incoming statistics list
  void on_topic_statistics(const rosgraph_monitor_msgs::msg::TopicStatistics & statistics);

  /// @brief Integrate a parameter event, updating the recorded parameters of the node it names.
  /// @param event Incoming event
  /// @details Dropped for an untracked node, or for a node whose parameters are not observed yet.
  void on_parameter_event(const rcl_interfaces::msg::ParameterEvent & event);

  /// @brief Fill a Graph message containing current graph state
  void fill_rosgraph_msg(rosgraph_msgs::msg::Graph & msg);

protected:
  /* Types */

  /// @brief Keeps flags for tracking observed nodes over time
  struct NodeTracking
  {
    std::string name;
    bool missing = false;
    bool stale = false;
    // nullopt until a successful observation is made, differentiating "no params" from "don't know"
    std::optional<RecordedParameters> params;

    explicit NodeTracking(const std::string & name);
  };

  /// @brief changes from a single graph update
  struct NodeChanges
  {
    /// Newly seen, seen again after going missing, or an earlier attempt failed
    std::vector<std::string> to_observe;
    /// No longer present, any ongoing observations should be abandoned
    std::vector<std::string> departed;
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

    rosgraph_msgs::msg::Topic to_msg();

    EndpointTracking(const std::string & topic_name, const rclcpp::TopicEndpointInfo & info, const rclcpp::Time & now);
  };

  typedef std::map<std::string, std::vector<std::string>> TopicsToTypes;
  typedef std::unordered_map<RosRmwGid, EndpointTracking> EndpointTrackingMap;
  typedef std::unordered_set<RosRmwGid> EndpointSet;
  typedef std::pair<std::string, std::string> NodeAndTopic;

  struct GraphTracking
  {
    // Graph entities
    std::unordered_map<std::string, NodeTracking> nodes;
    EndpointTrackingMap publishers;
    EndpointTrackingMap subscriptions;

    // Convenience derivations
    std::unordered_map<NodeAndTopic, RosRmwGid> publisher_lookup;
    std::unordered_map<NodeAndTopic, RosRmwGid> subscription_lookup;
    std::unordered_map<std::string, TopicTracking> topic_endpoint_counts;

    // Analysis
    std::unordered_set<std::string> ignored_nodes;
    std::unordered_set<std::string> returned_nodes;
    std::unordered_set<std::string> pubs_with_no_subs;  // a.k.a. "leaf topics"
    std::unordered_set<std::string> subs_with_no_pubs;  // a.k.a. "dead sinks"
  };

  /* Methods */

  /// @brief Update internal graph representation and detect changes
  void update_graph();

  /// @brief Called in thread to watch the graph in infinite loop and rebuild tracking on changes.
  void watch_for_updates();

  /// @brief Should we skip tracking this node?
  /// @param node_name
  /// @return Whether to ignore tracking the node
  bool ignore_node(const std::string & node_name, GraphTracking & graph);

  /// @brief Check current observed state against our tracked state, updating tracking info
  /// @param observed_node_names
  /// @return Which nodes need observing and which have left the graph
  NodeChanges track_node_updates(const std::vector<std::string> & observed_node_names, GraphTracking & graph);

  /// @brief Check current observed state against our tracked state, updating tracking info
  /// @param observed_topics_and_types
  void track_endpoint_updates(const TopicsToTypes & observed_topics_and_types, GraphTracking & graph);

  /// @return Iterator to existing or added publisher, or nullopt if node ignored
  std::optional<EndpointTrackingMap::iterator> add_publisher(
    const std::string & topic_name, const rclcpp::TopicEndpointInfo & info, GraphTracking & graph);

  /// @return Iterator to existing or added publisher, or nullopt if node ignored
  std::optional<EndpointTrackingMap::iterator> add_subscription(
    const std::string & topic_name, const rclcpp::TopicEndpointInfo & info, GraphTracking & graph);

  bool topic_period_ok(
    const rosgraph_monitor_msgs::msg::TopicStatistic & stat, const rclcpp::Duration & deadline) const;

  void statusWrapper(
    diagnostic_updater::DiagnosticStatusWrapper & msg,
    uint8_t level,
    const std::string & message,
    const std::string & subname) const;

  /// @brief Record a node's observed parameters
  /// @note Runs on the query queue's thread
  void on_node_parameters(const std::string & node_name, std::vector<std::string> parameter_names);

  /// @brief Read the parameter names recorded for a node, taking the graph lock
  /// @return The recorded names, empty if the node is untracked or not yet observed
  std::vector<std::string> recorded_parameter_names(const std::string & node_name);

  /// @brief Record a node's observed parameter descriptors
  /// @note Runs on the query queue's thread
  void on_node_descriptors(
    const std::string & node_name, std::vector<rcl_interfaces::msg::ParameterDescriptor> descriptors);

  /// @brief Record a node's observed parameter values, for recorded names with no value yet
  /// @note Runs on the query queue's thread
  void on_node_values(const std::string & node_name, ValueResponse response);

  /// @brief Invoke the graph change callback, if one is set.
  void notify_graph_change();

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

  std::shared_ptr<ParameterServiceClient> parameter_client_;

  /* Three different threads read and write to this state:
   * - the watch thread rebuilding the graph
   * - the query queue's thread writing back parameter results
   * - the caller's thread reading via evaluate() or fill_rosgraph_msg()
   *
   * Mutex never held while invoking graph_change_callback_, which may trigger outside caller to re-enter.
   */
  MutexProtected<GraphTracking> graph_;
  GraphChangeCallback graph_change_callback_;

  // Declared last so queries stop before anything they touch is destroyed.
  // The names queue requests on the descriptor and value queues,
  // so it is declared after both of them and stops first.
  QueryQueue<std::vector<rcl_interfaces::msg::ParameterDescriptor>> descriptor_queries_;
  QueryQueue<ValueResponse> value_queries_;
  QueryQueue<std::vector<std::string>> param_queries_;
};

}  // namespace rosgraph_monitor

#endif  // ROSGRAPH_MONITOR__MONITOR_HPP_
