// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef TOPIC_STATS_TRACETOOLS__TRACE_COLLECTOR_HPP_
#define TOPIC_STATS_TRACETOOLS__TRACE_COLLECTOR_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "topic_stats_core/collector.hpp"
#include "topic_stats_core/handle_map.hpp"
#include "topic_stats_core/sink.hpp"
#include "topic_stats_core/timer.hpp"
#include "topic_stats_core/types.hpp"

namespace topic_stats_tracetools
{

/// Ingest adapter driven by the tracepoints that rcl and the RMW implementations already call.
///
/// The instrumentation points are upstream and enabled in distribution binaries, so unlike the RMW
/// wrapper this needs no patched `rmw_implementation`: the library is interposed with LD_PRELOAD
/// and defines the handful of `ros_trace_*` symbols it cares about, chaining the rest through to
/// the real tracetools so that a live LTTng session keeps working unchanged.
///
/// The join it performs:
///   - `rcl_node_init` names a node and gives its handle
///   - `rcl_publisher_init` / `rcl_subscription_init` give the topic and, crucially, the *rmw*
///     handle, which is what the message tracepoints carry
///   - `rmw_publish` / `rmw_take` carry the rmw handle and, for takes, the source timestamp
///
/// Keyed on rmw handles rather than rcl handles on purpose. The `rcl_publisher_t` passed to
/// `rcl_publisher_init` can be a stack temporary, observed for `/rosout`, so it is not a stable
/// identity; the rmw handles are heap allocated and live as long as the endpoint.
///
/// Thread safe. Tracepoints fire on whatever thread published or took a message.
class TraceCollector
{
public:
  struct Config
  {
    size_t window_size = 50;
    std::chrono::milliseconds report_period{1000};

    /// Consecutive quiet reports after which an endpoint is dropped. Zero disables it.
    ///
    /// This adapter needs this and the RMW one does not, because ros2_tracing has no destruction
    /// tracepoints at all: there is `rcl_publisher_init` and no `rcl_publisher_fini`, so nothing
    /// ever says an endpoint went away and the registry would grow for the life of the process.
    /// Eviction is safe because the descriptors are kept here, so an endpoint that goes quiet and
    /// later speaks again is simply registered afresh.
    uint32_t idle_eviction_reports = 0;
  };

  TraceCollector(
    const Config & config,
    std::shared_ptr<topic_stats_core::Collector> collector,
    std::unique_ptr<topic_stats_core::StatsSink> sink);
  virtual ~TraceCollector();

  TraceCollector(const TraceCollector &) = delete;
  TraceCollector & operator=(const TraceCollector &) = delete;

  /// Process-wide instance, built from the environment on first use. Constructed lazily because
  /// the first tracepoint can fire before main().
  static TraceCollector & instance();

  /// Starts the reporting timer. Separate from construction so that tests can drive reporting
  /// themselves.
  void start();

  void on_node_init(const void * node_handle, const char * node_name, const char * node_namespace);
  void on_publisher_init(const void * node_handle, const void * rmw_publisher_handle, const char * topic_name);
  void on_subscription_init(const void * node_handle, const void * rmw_subscription_handle, const char * topic_name);
  void on_publish(const void * rmw_publisher_handle);

  /// \param taken false when the middleware polled and found nothing, which is the overwhelming
  ///   majority of calls and must not be counted as a message.
  void on_take(const void * rmw_subscription_handle, int64_t source_timestamp, bool taken);

  /// Takes a snapshot, hands it to the sink, and evicts anything that has gone quiet.
  void report();

  const topic_stats_core::Collector & collector() const
  {
    return *collector_;
  }

  /// Endpoints brought back after having been evicted, which says the eviction threshold is too
  /// aggressive for the traffic on this process.
  uint64_t revived_endpoints() const
  {
    return revived_endpoints_.load(std::memory_order_relaxed);
  }

private:
  /// What a middleware handle maps to. The descriptor is kept so that an evicted endpoint can be
  /// registered again without another init tracepoint, which will never come.
  struct EndpointRecord
  {
    topic_stats_core::NodeId node;
    topic_stats_core::EndpointId endpoint;
    topic_stats_core::EndpointDescriptor descriptor;
  };

  void add_endpoint(
    const void * node_handle, const void * rmw_handle, topic_stats_core::EndpointKind kind, const char * topic_name);

  /// Resolves a handle to a live endpoint id, registering it again if it was evicted. Returns an
  /// invalid id for a handle that was never registered, which is normal: the middleware has
  /// publishers of its own, for discovery, that rcl never sees.
  topic_stats_core::EndpointId resolve(const void * rmw_handle);

  const Config config_;
  const std::shared_ptr<topic_stats_core::Collector> collector_;
  const std::unique_ptr<topic_stats_core::StatsSink> sink_;

  topic_stats_core::HandleMap<topic_stats_core::NodeId> nodes_;
  topic_stats_core::HandleMap<EndpointRecord> endpoints_;

  std::atomic<uint64_t> revived_endpoints_{0};
  std::optional<topic_stats_core::Timer> timer_;
};

}  // namespace topic_stats_tracetools

#endif  // TOPIC_STATS_TRACETOOLS__TRACE_COLLECTOR_HPP_
