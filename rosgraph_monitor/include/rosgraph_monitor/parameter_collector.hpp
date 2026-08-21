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
#include "rosgraph_monitor/config.hpp"

namespace rosgraph_monitor
{

/// @brief A node's parameters.
/// @details descriptors and values are parallel: values[i] is the current value of descriptors[i].
///   Values may be empty if not (yet) known.
struct NodeParameters
{
  std::vector<rcl_interfaces::msg::ParameterDescriptor> descriptors;
  std::vector<rcl_interfaces::msg::ParameterValue> values;

  bool empty() const
  {
    return descriptors.empty();
  }
};

/// @brief Describe parameters by name alone, for when nothing else about them is known.
/// @details Listing is the only parameter call that does not have to name what it asks for,
///   so it is the only one that works on a node whose parameters are not known in advance.
///   A name says nothing about a parameter's type, so every descriptor is left unset, and there
///   are no values to report.
/// TODO(troy): describe_parameters and get_parameters will fill in the real types and values.
/// @param names Response to list_parameters
/// @return Descriptors carrying only the names, and no values
NodeParameters parameters_from_names(const std::vector<std::string> & names);

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

/// @brief Pure interface providing the parameter service calls the collector needs.
/// @details Abstracted for polymorphic dependency injection, to test without full ros graph.
class ParameterServiceClient
{
public:
  virtual ~ParameterServiceClient() = default;

  using NamesCallback = std::function<void(std::optional<std::vector<std::string>>)>;

  /// @return Whether the node is currently reachable.
  /// Checked before starting, so an unreachable node does not occupy a concurrency slot waiting for a response.
  virtual bool is_ready(const std::string & node_name) = 0;

  /// @brief Get all parameter names from the node, calling callback when complete.
  /// @return Immediately. The callback is always called, with a std::nullopt on service call failure or gone node.
  virtual void list_parameters(const std::string & node_name, NamesCallback callback) = 0;

  /// @brief Drop any client state held for a node that has gone away.
  virtual void forget(const std::string & node_name) = 0;
};

/// @brief Observes the parameters of nodes, a bounded number at a time.
///
/// Each node costs a service round trip, and there may be as many of them as there are nodes in
/// the graph. This runs those requests without letting the number outstanding grow with the size
/// of the graph, and without blocking the caller.
///
/// Thread safe. Callbacks are never invoked while the internal lock is held, so the completion
/// handler is free to call back into the collector.
class ParameterCollector
{
public:
  /// @param node_name Node the parameters belong to
  /// @param parameters What was observed. Empty if the node reported no parameters.
  using CompleteCallback = std::function<void(const std::string & node_name, NodeParameters parameters)>;

  /// @brief Source of the current time, injected so tests can drive expiry directly.
  using NowFunc = std::function<std::chrono::nanoseconds()>;
  using Options = GraphMonitorConfiguration::ParameterObservation;

  ParameterCollector(
    std::shared_ptr<ParameterServiceClient> client,
    CompleteCallback on_complete,
    NowFunc now_fn,
    rclcpp::Logger logger,
    Options options = {});

  /// @brief Ask for a node's parameters. Queued if already at capacity.
  /// @details A node already queued or in flight is ignored, so repeated discovery of the same
  ///   node does not pile up duplicate work. A node whose parameter services are not up yet is
  ///   dropped rather than queued, so asking again later is how it eventually gets observed.
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
  /// A request to issue once the lock has been released. Collected under the lock and
  /// dispatched outside it, because a client is free to answer synchronously on the calling
  /// thread, and its response handler takes the same lock.
  struct Call
  {
    std::string node_name;
    uint64_t generation;
  };

  struct Observation
  {
    std::chrono::nanoseconds started;
    /// Distinguishes this observation from any earlier one of the same node, so a response
    /// arriving after its observation was abandoned can be told apart from a current one.
    uint64_t generation = 0;
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
