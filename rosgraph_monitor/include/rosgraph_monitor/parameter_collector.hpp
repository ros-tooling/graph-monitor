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
#include "rosgraph_monitor/mutex_protected.hpp"

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
  Observation(std::chrono::nanoseconds started, uint64_t generation)
  : started(started)
  , generation(generation)
  {}

  std::chrono::nanoseconds started;
  /// Distinguishes this observation from any earlier one,
  /// so a response arriving after its observation was abandoned can be distinguished.
  uint64_t generation = 0;
};

class ObservationQueue
{
public:
  ObservationQueue(std::chrono::milliseconds timeout, size_t max_concurrent);

  bool request(const std::string & name);
  void cancel(const std::string & name);
  std::vector<std::string> expire(std::chrono::nanoseconds now);

  bool can_start_new() const;
  std::string pop_next();
  uint64_t activate(const std::string & name, std::chrono::nanoseconds now);

  // Calls actually sent out (?)
  std::unordered_map<std::string, Observation> active_;

  // Calls waiting to be sent. A pair for fast lookup vs ordering.
  std::deque<std::string> pending_;

private:
  // To optimize lookups, duplicates ordered entries in pending_
  std::unordered_set<std::string> pending_lookup_;

  uint64_t next_generation_ = 1;

  const std::chrono::milliseconds timeout_;
  const size_t max_concurrent_;
};

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

/// @brief Observes the parameters of nodes.
///
/// Each node costs service round trips,
/// This runs requests with a set pool of threads, without blocking the caller.
///
/// Thread safe for reentrance, callbacks may call the collector.
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
  /// @note Puts as many pending calls as can fit from queue into calls.
  void start_next(ObservationQueue & queue, std::vector<Call> & calls);

  /// @brief Issue collected requests (from start_next)
  void dispatch(std::vector<Call> & calls);

  void on_names(const std::string & node_name, uint64_t generation, std::optional<std::vector<std::string>> names);

  const std::shared_ptr<ParameterServiceClient> client_;
  const CompleteCallback on_complete_;
  const NowFunc now_fn_;
  rclcpp::Logger logger_;

  MutexProtected<ObservationQueue> queue_;
};

}  // namespace rosgraph_monitor

#endif  // ROSGRAPH_MONITOR__PARAMETER_COLLECTOR_HPP_
