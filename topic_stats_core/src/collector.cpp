// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "topic_stats_core/collector.hpp"

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace topic_stats_core
{

Collector::Collector(const Options & options, std::shared_ptr<Clock> clock)
: options_(options)
, clock_(std::move(clock))
{}

StatKind Collector::period_stat_for(EndpointKind kind)
{
  return kind == EndpointKind::Publisher ? StatKind::PublishedPeriod : StatKind::ReceivedPeriod;
}

Collector::Measurement & Collector::measurement_for(EndpointState & state, StatKind stat)
{
  for (auto & measurement : state.measurements) {
    if (measurement.stat == stat) {
      return measurement;
    }
  }
  state.measurements.emplace_back(stat, state.window_size);
  return state.measurements.back();
}

Collector::EndpointState * Collector::resolve(EndpointId endpoint)
{
  EndpointState * state = endpoints_.find(endpoint);
  if (state == nullptr) {
    stale_id_records_.fetch_add(1, std::memory_order_relaxed);
  }
  return state;
}

NodeId Collector::register_node(const NodeDescriptor & descriptor)
{
  std::unique_lock<std::shared_mutex> lock(registry_mutex_);
  return nodes_.emplace(descriptor).first;
}

void Collector::unregister_node(NodeId node)
{
  std::unique_lock<std::shared_mutex> lock(registry_mutex_);
  if (nodes_.find(node) == nullptr) {
    return;
  }

  // Collect before erasing rather than erasing inside the traversal. RMW teardown does not
  // guarantee endpoints are destroyed before their node, so this path is load bearing rather than
  // just defensive.
  std::vector<EndpointId> orphaned;
  endpoints_.for_each([&](EndpointId id, EndpointState & state) {
    if (state.node == node) {
      orphaned.push_back(id);
    }
  });
  for (const auto & id : orphaned) {
    endpoints_.erase(id);
  }
  nodes_.erase(node);
}

EndpointId Collector::register_endpoint(NodeId node, const EndpointDescriptor & descriptor)
{
  std::unique_lock<std::shared_mutex> lock(registry_mutex_);
  if (nodes_.find(node) == nullptr) {
    return EndpointId{};
  }
  return endpoints_.emplace(node, descriptor, options_.window_size).first;
}

void Collector::unregister_endpoint(EndpointId endpoint)
{
  std::unique_lock<std::shared_mutex> lock(registry_mutex_);
  endpoints_.erase(endpoint);
}

void Collector::record_publish(EndpointId endpoint)
{
  const auto now = clock_->monotonic();
  std::shared_lock<std::shared_mutex> lock(registry_mutex_);
  EndpointState * state = resolve(endpoint);
  if (state == nullptr) {
    return;
  }
  if (state->descriptor.kind != EndpointKind::Publisher) {
    mismatched_records_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  std::lock_guard<std::mutex> state_lock(state->mutex);
  if (state->last_event.has_value()) {
    auto & measurement = measurement_for(*state, StatKind::PublishedPeriod);
    measurement.window.accumulate(now - *state->last_event);
    measurement.changed = true;
  }
  state->last_event = now;
}

void Collector::record_take(EndpointId endpoint, std::optional<SourceTimestamp> source_timestamp)
{
  const auto now = clock_->monotonic();
  std::shared_lock<std::shared_mutex> lock(registry_mutex_);
  EndpointState * state = resolve(endpoint);
  if (state == nullptr) {
    return;
  }
  if (state->descriptor.kind != EndpointKind::Subscription) {
    mismatched_records_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  // Only sampled when needed, because it is a second clock read on the hot path.
  std::optional<Duration> age;
  if (source_timestamp.has_value()) {
    if (*source_timestamp > 0) {
      const auto system_now = std::chrono::duration_cast<Duration>(clock_->system().time_since_epoch());
      age = system_now - Duration{*source_timestamp};
    } else {
      // Several rmw implementations report zero rather than nothing when no timestamp is
      // available. Treating that as an age would report the entire Unix epoch as latency.
      unusable_source_timestamps_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  std::lock_guard<std::mutex> state_lock(state->mutex);
  if (state->last_event.has_value()) {
    auto & measurement = measurement_for(*state, StatKind::ReceivedPeriod);
    measurement.window.accumulate(now - *state->last_event);
    measurement.changed = true;
  }
  state->last_event = now;

  if (age.has_value()) {
    // Unlike a period, age is absolute, so the first take already yields a measurement.
    auto & measurement = measurement_for(*state, StatKind::TakeAge);
    measurement.window.accumulate(*age);
    measurement.changed = true;
  }
}

void Collector::record_duration(EndpointId endpoint, StatKind stat, Duration value)
{
  std::shared_lock<std::shared_mutex> lock(registry_mutex_);
  EndpointState * state = resolve(endpoint);
  if (state == nullptr) {
    return;
  }

  std::lock_guard<std::mutex> state_lock(state->mutex);
  auto & measurement = measurement_for(*state, stat);
  measurement.window.accumulate(value);
  measurement.changed = true;
}

StatsReport Collector::snapshot(SnapshotMode mode)
{
  StatsReport report;
  report.timestamp = clock_->system();

  std::shared_lock<std::shared_mutex> lock(registry_mutex_);

  // Index into report.nodes, created only once a node turns out to have something to report.
  std::unordered_map<NodeId, size_t> report_index;

  endpoints_.for_each([&](EndpointId, EndpointState & state) {
    const NodeState * node = nodes_.find(state.node);
    if (node == nullptr) {
      // Should be unreachable: unregister_node drops its endpoints under the same lock.
      return;
    }

    std::vector<StatSample> samples;
    {
      std::lock_guard<std::mutex> state_lock(state.mutex);
      for (auto & measurement : state.measurements) {
        // An endpoint that exists but has never been measured must not report a zero, which would
        // be indistinguishable from a real measurement downstream.
        if (measurement.window.empty()) {
          continue;
        }
        const bool report_it = (mode == SnapshotMode::All) || measurement.changed;
        // Cleared regardless of mode, so that an All snapshot does not leave a stale flag behind
        // for the next OnlyChanged snapshot to report again.
        measurement.changed = false;
        if (!report_it) {
          continue;
        }

        StatSample sample;
        sample.stat = measurement.stat;
        sample.node_name = node->descriptor.fully_qualified_name;
        sample.topic_name = state.descriptor.topic_name;
        sample.window_count = static_cast<uint32_t>(measurement.window.count());
        sample.mean = measurement.window.mean();
        sample.min = measurement.window.min();
        sample.max = measurement.window.max();
        samples.push_back(std::move(sample));
      }
    }

    if (samples.empty()) {
      return;
    }

    auto [it, inserted] = report_index.emplace(state.node, report.nodes.size());
    if (inserted) {
      NodeReport node_report;
      node_report.node = state.node;
      node_report.node_name = node->descriptor.fully_qualified_name;
      report.nodes.push_back(std::move(node_report));
    }
    auto & node_report = report.nodes[it->second];
    node_report.samples.insert(
      node_report.samples.end(), std::make_move_iterator(samples.begin()), std::make_move_iterator(samples.end()));
  });

  return report;
}

CollectorDiagnostics Collector::diagnostics() const
{
  CollectorDiagnostics diagnostics;
  diagnostics.stale_id_records = stale_id_records_.load(std::memory_order_relaxed);
  diagnostics.mismatched_records = mismatched_records_.load(std::memory_order_relaxed);
  diagnostics.unusable_source_timestamps = unusable_source_timestamps_.load(std::memory_order_relaxed);

  std::shared_lock<std::shared_mutex> lock(registry_mutex_);
  diagnostics.live_nodes = nodes_.live_count();
  diagnostics.live_endpoints = endpoints_.live_count();
  return diagnostics;
}

}  // namespace topic_stats_core
