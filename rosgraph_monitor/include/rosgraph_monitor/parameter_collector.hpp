// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef ROSGRAPH_MONITOR__PARAMETER_COLLECTOR_HPP_
#define ROSGRAPH_MONITOR__PARAMETER_COLLECTOR_HPP_

#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rclcpp/logger.hpp"

namespace rosgraph_monitor
{

/// @brief A node's parameters, as the graph message wants them.
/// @details descriptors and values are parallel: values[i] is the current value of descriptors[i].
///   values may be empty if the node answered the describe but not the get.
struct NodeParameters
{
  std::vector<rcl_interfaces::msg::ParameterDescriptor> descriptors;
  std::vector<rcl_interfaces::msg::ParameterValue> values;

  bool empty() const
  {
    return descriptors.empty();
  }
};

/// @brief Pair a describe response with a get response into parallel lists.
/// @details Both responses are positional against the names that were asked for, and neither
///   value carries a name. A parameter removed between the two calls shortens one response but
///   not the other, so anything past the shorter of the two is dropped rather than mispaired.
///   Descriptors are authoritative for names.
/// @param descriptors Response to describe_parameters, in request order
/// @param values Response to get_parameters, in request order
/// @return Parallel descriptor/value lists, of equal length
NodeParameters pair_parameters(
  const std::vector<rcl_interfaces::msg::ParameterDescriptor> & descriptors,
  const std::vector<rcl_interfaces::msg::ParameterValue> & values);

/// @brief The three parameter service calls the collector needs, as an interface.
/// @details Abstracted so the collector can be tested without a ROS graph, and so the retry and
///   concurrency policy lives in one testable place rather than in the RMW-facing code.
///   Every call is asynchronous: the callback runs when the response arrives, on whatever
///   thread the implementation uses. A std::nullopt result means the call failed or the node
///   is gone; the collector treats it the same either way.
class ParameterServiceClient
{
public:
  virtual ~ParameterServiceClient() = default;

  using NamesCallback = std::function<void(std::optional<std::vector<std::string>>)>;
  using DescriptorsCallback = std::function<void(std::optional<std::vector<rcl_interfaces::msg::ParameterDescriptor>>)>;
  using ValuesCallback = std::function<void(std::optional<std::vector<rcl_interfaces::msg::ParameterValue>>)>;

  /// @return Whether the node is currently reachable. Checked before starting, so an
  ///   unreachable node does not occupy a concurrency slot waiting for a response.
  virtual bool is_ready(const std::string & node_name) = 0;

  virtual void list_parameters(const std::string & node_name, NamesCallback callback) = 0;

  virtual void describe_parameters(
    const std::string & node_name, const std::vector<std::string> & names, DescriptorsCallback callback) = 0;

  virtual void get_parameters(
    const std::string & node_name, const std::vector<std::string> & names, ValuesCallback callback) = 0;

  /// @brief Drop any client state held for a node that has gone away.
  virtual void forget(const std::string & node_name)
  {
    (void)node_name;
  }
};

/// @brief Observes the full parameter state of nodes, a bounded number at a time.
///
/// There is no single call that returns parameters with their descriptors and values, so each
/// node takes three round trips: list the names, describe them, then read them. This runs that
/// sequence for many nodes without letting the number of outstanding requests grow with the
/// size of the graph, and without blocking the caller.
///
/// Thread safe. Callbacks are never invoked while the internal lock is held, so the completion
/// handler is free to call back into the collector.
class ParameterCollector
{
public:
  struct Options
  {
    /// How many nodes may be observed at once. Each observation has at most one request
    /// outstanding, so this also bounds in-flight service requests.
    size_t max_concurrent = 4;
    /// How long a single node's observation may take before it is abandoned. Without this a
    /// node that stops answering mid-sequence would hold its slot indefinitely.
    std::chrono::nanoseconds timeout = std::chrono::seconds(10);
  };

  /// @param node_name Node the parameters belong to
  /// @param parameters What was observed. Empty if the node reported no parameters.
  using CompleteCallback = std::function<void(const std::string & node_name, NodeParameters parameters)>;

  /// @brief Source of the current time, injected so tests can drive expiry directly.
  using NowFunc = std::function<std::chrono::nanoseconds()>;

  ParameterCollector(
    std::shared_ptr<ParameterServiceClient> client,
    CompleteCallback on_complete,
    NowFunc now_fn,
    rclcpp::Logger logger,
    Options options);

  /// @brief Construct with default options
  ParameterCollector(
    std::shared_ptr<ParameterServiceClient> client,
    CompleteCallback on_complete,
    NowFunc now_fn,
    rclcpp::Logger logger);

  /// @brief Ask for a node's parameters. Queued if already at capacity.
  /// @details A node already queued or in flight is ignored, so repeated discovery of the same
  ///   node does not pile up duplicate work.
  void request(const std::string & node_name);

  /// @brief Abandon a node, whether queued or in flight.
  /// @details Called when a node leaves the graph. A response that arrives afterwards is
  ///   discarded, since the observation it belongs to no longer exists.
  void cancel(const std::string & node_name);

  /// @brief Abandon observations that have run past the timeout, and start queued work.
  void tick();

  size_t active_count() const;
  size_t pending_count() const;

private:
  enum class Stage
  {
    Listing,
    Describing,
    Getting
  };

  /// A request to issue once the lock has been released. Collected under the lock and
  /// dispatched outside it, because a client is free to answer synchronously on the calling
  /// thread, and its response handler takes the same lock.
  struct Call
  {
    enum class Kind
    {
      List,
      Describe,
      Get
    };
    Kind kind;
    std::string node_name;
    uint64_t generation;
    std::vector<std::string> names;
  };

  struct Observation
  {
    Stage stage = Stage::Listing;
    std::chrono::nanoseconds started;
    /// Bumped whenever an observation is abandoned, so a late response can tell that the
    /// observation it belongs to is no longer the current one for that node.
    uint64_t generation = 0;
    std::vector<std::string> names;
    std::vector<rcl_interfaces::msg::ParameterDescriptor> descriptors;
  };

  /// @note Caller must hold mutex_. Appends the requests to issue after unlocking.
  void start_next(std::vector<Call> & calls);
  /// @note Caller must hold mutex_. Appends the request to issue after unlocking.
  void begin(const std::string & node_name, std::vector<Call> & calls);
  /// @brief Issue collected requests. Must be called with mutex_ released.
  void dispatch(std::vector<Call> & calls);
  /// @return Whether this response belongs to a live observation
  /// @note Caller must hold mutex_
  bool still_current(const std::string & node_name, uint64_t generation) const;
  /// @note Caller must hold mutex_. Frees the node's slot.
  void finish(const std::string & node_name);

  void on_names(const std::string & node_name, uint64_t generation, std::optional<std::vector<std::string>> names);
  void on_descriptors(
    const std::string & node_name,
    uint64_t generation,
    std::optional<std::vector<rcl_interfaces::msg::ParameterDescriptor>> descriptors);
  void on_values(
    const std::string & node_name,
    uint64_t generation,
    std::optional<std::vector<rcl_interfaces::msg::ParameterValue>> values);

  const std::shared_ptr<ParameterServiceClient> client_;
  const CompleteCallback on_complete_;
  const NowFunc now_fn_;
  rclcpp::Logger logger_;
  const Options options_;

  mutable std::mutex mutex_;
  std::unordered_map<std::string, Observation> active_;
  std::deque<std::string> pending_;
  std::unordered_set<std::string> queued_;
  uint64_t next_generation_ = 1;
};

}  // namespace rosgraph_monitor

#endif  // ROSGRAPH_MONITOR__PARAMETER_COLLECTOR_HPP_
