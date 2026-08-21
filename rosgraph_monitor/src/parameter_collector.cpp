// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "rosgraph_monitor/parameter_collector.hpp"

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rclcpp/logging.hpp"

namespace rosgraph_monitor
{

NodeParameters parameters_from_names(const std::vector<std::string> & names)
{
  NodeParameters result;
  result.descriptors.reserve(names.size());
  for (const auto & name : names) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.name = name;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET;
    result.descriptors.push_back(std::move(descriptor));
  }
  return result;
}

NodeParameters pair_parameters(
  const std::vector<rcl_interfaces::msg::ParameterDescriptor> & descriptors,
  const std::vector<rcl_interfaces::msg::ParameterValue> & values)
{
  NodeParameters result;
  const size_t paired = std::min(descriptors.size(), values.size());
  result.descriptors.assign(descriptors.begin(), descriptors.begin() + paired);
  result.values.assign(values.begin(), values.begin() + paired);
  return result;
}

ParameterCollector::ParameterCollector(
  std::shared_ptr<ParameterServiceClient> client,
  CompleteCallback on_complete,
  NowFunc now_fn,
  rclcpp::Logger logger,
  GraphMonitorConfiguration::ParameterObservation options)
: client_(std::move(client))
, on_complete_(std::move(on_complete))
, now_fn_(std::move(now_fn))
, logger_(logger)
, options_(options)
{}

void ParameterCollector::request(const std::string & node_name)
{
  std::vector<Call> calls;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_.count(node_name) || queued_.count(node_name)) {
      // Already being observed; discovering the node again is not new work.
      return;
    }
    pending_.push_back(node_name);
    queued_.insert(node_name);
    start_next(calls);
  }
  dispatch(calls);
}

void ParameterCollector::cancel(const std::string & node_name)
{
  std::vector<Call> calls;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queued_.erase(node_name)) {
      pending_.erase(std::remove(pending_.begin(), pending_.end(), node_name), pending_.end());
    }
    // Erasing the observation is what makes any response still in flight stale: its generation
    // will no longer match, so it is dropped rather than applied to a node that has gone.
    active_.erase(node_name);
    start_next(calls);
  }
  client_->forget(node_name);
  dispatch(calls);
}

void ParameterCollector::tick()
{
  const auto now = now_fn_();
  std::vector<std::string> expired;
  std::vector<Call> calls;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & [node_name, observation] : active_) {
      if (now - observation.started > options_.timeout) {
        expired.push_back(node_name);
      }
    }
    for (const auto & node_name : expired) {
      active_.erase(node_name);
    }
    start_next(calls);
  }
  dispatch(calls);

  for (const auto & node_name : expired) {
    RCLCPP_WARN(logger_, "Parameter observation of %s timed out, abandoning it", node_name.c_str());
    client_->forget(node_name);
  }
}

size_t ParameterCollector::active_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return active_.size();
}

size_t ParameterCollector::pending_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return pending_.size();
}

void ParameterCollector::start_next(std::vector<Call> & calls)
{
  while (active_.size() < options_.max_concurrent && !pending_.empty()) {
    const std::string node_name = pending_.front();
    pending_.pop_front();
    queued_.erase(node_name);
    begin(node_name, calls);
  }
}

void ParameterCollector::dispatch(std::vector<Call> & calls)
{
  for (auto & call : calls) {
    const std::string node_name = call.node_name;
    const uint64_t generation = call.generation;
    client_->list_parameters(
      node_name, [this, node_name, generation](auto names) { on_names(node_name, generation, std::move(names)); });
  }
  calls.clear();
}

void ParameterCollector::begin(const std::string & node_name, std::vector<Call> & calls)
{
  if (!client_->is_ready(node_name)) {
    // Not reachable yet. Dropped rather than held, so it does not occupy a slot; the monitor
    // asks again on the next graph change if the node is still there.
    RCLCPP_DEBUG(logger_, "Parameter services for %s are not available yet", node_name.c_str());
    return;
  }

  Observation observation;
  observation.started = now_fn_();
  observation.generation = next_generation_++;
  const uint64_t generation = observation.generation;
  active_.emplace(node_name, std::move(observation));
  calls.push_back(Call{node_name, generation});
}

bool ParameterCollector::still_current(const std::string & node_name, uint64_t generation) const
{
  const auto it = active_.find(node_name);
  return it != active_.end() && it->second.generation == generation;
}

void ParameterCollector::finish(const std::string & node_name)
{
  active_.erase(node_name);
}

void ParameterCollector::on_names(
  const std::string & node_name, uint64_t generation, std::optional<std::vector<std::string>> names)
{
  std::optional<NodeParameters> observed;
  std::vector<Call> calls;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!still_current(node_name, generation)) {
      // The observation was abandoned while this response was in flight.
      return;
    }
    if (names) {
      // A node with no parameters is a normal outcome, not a failure, and is reported as such.
      observed = parameters_from_names(*names);
    } else {
      RCLCPP_WARN(logger_, "Could not list parameters of %s", node_name.c_str());
    }
    finish(node_name);
    start_next(calls);
  }

  dispatch(calls);

  if (observed && on_complete_) {
    on_complete_(node_name, std::move(*observed));
  }
}

}  // namespace rosgraph_monitor
