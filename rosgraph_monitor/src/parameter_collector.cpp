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

namespace
{

rosgraph_monitor::NodeParameters parameters_from_names(const std::vector<std::string> & names)
{
  rosgraph_monitor::NodeParameters result;
  result.descriptors.reserve(names.size());
  for (const auto & name : names) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.name = name;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET;
    result.descriptors.push_back(std::move(descriptor));
  }
  return result;
}

}  // namespace

namespace rosgraph_monitor
{

ObservationQueue::ObservationQueue(std::chrono::milliseconds timeout, size_t max_concurrent)
: timeout_(timeout)
, max_concurrent_(max_concurrent)
{}

bool ObservationQueue::request(const std::string & name)
{
  if (active_.count(name) || pending_lookup_.count(name)) {
    return false;
  }
  pending_.push_back(name);
  pending_lookup_.insert(name);
  return true;
}

void ObservationQueue::cancel(const std::string & name)
{
  if (pending_lookup_.erase(name)) {
    pending_.erase(std::remove(pending_.begin(), pending_.end(), name), pending_.end());
  }
  // Erasing the observation makes any response still in flight stale:
  // its generation will no longer match, so it is dropped rather than applied to the same name next time.
  active_.erase(name);
}

std::vector<std::string> ObservationQueue::expire(std::chrono::nanoseconds now)
{
  std::vector<std::string> expired;
  for (const auto & [name, observation] : active_) {
    if (now - observation.started > timeout_) {
      expired.push_back(name);
    }
  }
  for (const auto & node_name : expired) {
    active_.erase(node_name);
  }
  return expired;
}

bool ObservationQueue::can_start_new() const
{
  return active_.size() < max_concurrent_ && !pending_.empty();
}

std::string ObservationQueue::pop_next()
{
  const std::string name = pending_.front();
  pending_.pop_front();
  pending_lookup_.erase(name);
  return name;
}

uint64_t ObservationQueue::activate(const std::string & name, std::chrono::nanoseconds now)
{
  const uint64_t generation = next_generation_++;
  Observation observation{now, generation};
  active_.emplace(name, std::move(observation));
  return generation;
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
, queue_(options.timeout, options.max_concurrent)
{}

void ParameterCollector::request(const std::string & node_name)
{
  std::vector<Call> calls;
  {
    auto queue = queue_.lock();
    if (queue->request(node_name)) {
      start_next(*queue, calls);
    }
  }
  dispatch(calls);
}

void ParameterCollector::cancel(const std::string & node_name)
{
  std::vector<Call> calls;
  {
    auto queue = queue_.lock();
    queue->cancel(node_name);
    start_next(*queue, calls);
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
    auto queue = queue_.lock();
    expired = queue->expire(now);
    start_next(*queue, calls);
  }
  dispatch(calls);

  for (const auto & node_name : expired) {
    RCLCPP_WARN(logger_, "Parameter observation of %s timed out, abandoning it", node_name.c_str());
    client_->forget(node_name);
  }
}

size_t ParameterCollector::active_count() const
{
  return queue_.lock()->active_.size();
}

size_t ParameterCollector::pending_count() const
{
  return queue_.lock()->pending_.size();
}

void ParameterCollector::start_next(ObservationQueue & queue, std::vector<Call> & calls)
{
  while (queue.can_start_new()) {
    const std::string node_name = queue.pop_next();
    if (!client_->is_ready(node_name)) {
      // Not reachable yet. Dropped rather than held, so it does not occupy a slot; the monitor
      // asks again on the next graph change if the node is still there.
      RCLCPP_DEBUG(logger_, "Parameter services for %s are not available yet", node_name.c_str());
      continue;
    }
    auto generation = queue.activate(node_name, now_fn_());
    calls.push_back({node_name, generation});
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

void ParameterCollector::on_names(
  const std::string & node_name, uint64_t generation, std::optional<std::vector<std::string>> names)
{
  std::optional<NodeParameters> observed;
  std::vector<Call> calls;
  {
    auto queue = queue_.lock();
    const auto it = queue->active_.find(node_name);
    bool still_current = it != queue->active_.end() && it->second.generation == generation;
    if (!still_current) {
      // The observation was abandoned while this response was in flight.
      return;
    }
    if (names) {
      // A node with no parameters is a normal outcome, not a failure, and is reported as such.
      observed = parameters_from_names(*names);
    } else {
      RCLCPP_WARN(logger_, "Could not list parameters of %s", node_name.c_str());
    }
    queue->active_.erase(node_name);
    start_next(*queue, calls);
  }

  dispatch(calls);

  if (observed && on_complete_) {
    on_complete_(node_name, std::move(*observed));
  }
}

}  // namespace rosgraph_monitor
