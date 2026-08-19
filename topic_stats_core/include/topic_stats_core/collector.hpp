// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef TOPIC_STATS_CORE__COLLECTOR_HPP_
#define TOPIC_STATS_CORE__COLLECTOR_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>
#include <vector>

#include "topic_stats_core/clock.hpp"
#include "topic_stats_core/recorder.hpp"
#include "topic_stats_core/rolling_window.hpp"
#include "topic_stats_core/slot_table.hpp"
#include "topic_stats_core/types.hpp"

namespace topic_stats_core
{

/// Counters describing the collector's own health, so that a misbehaving ingest adapter is
/// visible rather than silently producing thin statistics.
struct CollectorDiagnostics
{
  /// Records against an id that was never valid, or has been unregistered. A nonzero value from a
  /// running system means an ingest adapter is losing its handle mapping.
  uint64_t stale_id_records = 0;
  /// Records of the wrong shape for the endpoint, e.g. a publish recorded against a subscription.
  /// Always an ingest adapter bug.
  uint64_t mismatched_records = 0;
  /// Takes whose source timestamp was present but unusable, so no take age was produced.
  uint64_t unusable_source_timestamps = 0;
  /// Endpoints dropped for going quiet, which only happens if an adapter asks for it.
  uint64_t evicted_endpoints = 0;
  size_t live_nodes = 0;
  size_t live_endpoints = 0;
};

/// Aggregates endpoint measurements into rolling-window statistics.
///
/// This is the whole of the statistics domain: it knows nothing about how events were observed and
/// nothing about where reports go. Ingest adapters drive it through Recorder, and whatever owns the
/// reporting cadence pulls snapshots out of it.
///
/// Thread safety: every method is safe to call concurrently from any thread. Registration and
/// snapshots take an exclusive/shared lock over the registry respectively, and each endpoint's
/// measurements are guarded individually so that recording on unrelated endpoints does not
/// contend. The hot path is therefore one uncontended shared lock plus one uncontended mutex.
class Collector : public Recorder
{
public:
  struct Options
  {
    /// Number of measurements each statistic averages over. Only timestamps are retained, never
    /// message contents.
    size_t window_size = 50;
  };

  Collector(const Options & options, std::shared_ptr<Clock> clock);
  ~Collector() override = default;

  Collector(const Collector &) = delete;
  Collector & operator=(const Collector &) = delete;

  // Recorder
  NodeId register_node(const NodeDescriptor & descriptor) override;
  void unregister_node(NodeId node) override;
  EndpointId register_endpoint(NodeId node, const EndpointDescriptor & descriptor) override;
  void unregister_endpoint(EndpointId endpoint) override;
  void record_publish(EndpointId endpoint) override;
  void record_take(EndpointId endpoint, std::optional<SourceTimestamp> source_timestamp) override;
  void record_duration(EndpointId endpoint, StatKind stat, Duration value) override;

  /// Collect current statistics, grouped by node.
  ///
  /// In OnlyChanged mode a statistic is reported only if it gained a measurement since the last
  /// snapshot, and nodes with nothing to report are omitted entirely. Reporting is a
  /// consume operation: two snapshots in a row will find nothing the second time.
  StatsReport snapshot(SnapshotMode mode = SnapshotMode::OnlyChanged);

  /// Unregisters endpoints that have produced no measurement for this many consecutive snapshots,
  /// returning what was dropped so an ingest adapter can prune its own handle mapping.
  ///
  /// Exists for ingest adapters that are never told an endpoint went away. The tracepoint adapter
  /// is one: ros2_tracing emits `rcl_publisher_init` and friends but has no matching fini
  /// tracepoint, so without this its registry would grow for the life of the process. The RMW
  /// wrapper sees real destruction and has no need to call it.
  ///
  /// Idleness is counted in snapshots rather than in time, so it follows the reporting cadence
  /// rather than needing a clock of its own. A threshold of zero evicts nothing.
  std::vector<EndpointId> evict_idle(uint32_t idle_snapshots);

  CollectorDiagnostics diagnostics() const;

private:
  /// One statistic's rolling window plus whether it has been measured since the last snapshot.
  struct Measurement
  {
    Measurement(StatKind stat_kind, size_t window_size)
    : stat(stat_kind)
    , window(window_size)
    {}

    StatKind stat;
    RollingWindow<Duration> window;
    bool changed = false;
  };

  struct EndpointState
  {
    EndpointState(NodeId owner, const EndpointDescriptor & desc, size_t window)
    : node(owner)
    , descriptor(desc)
    , window_size(window)
    {}

    NodeId node;
    EndpointDescriptor descriptor;
    size_t window_size;

    mutable std::mutex mutex;
    /// Monotonic time of the previous publish or take, absent until the first one. The first event
    /// on an endpoint yields no period, only a starting point.
    std::optional<MonoTime> last_event;
    /// Created on first use. At most three entries in practice, so a linear scan beats a map.
    std::vector<Measurement> measurements;
    /// Consecutive snapshots that found nothing new to report about this endpoint. Reset whenever
    /// it produces a sample.
    uint32_t idle_snapshots = 0;
  };

  struct NodeState
  {
    explicit NodeState(const NodeDescriptor & desc)
    : descriptor(desc)
    {}

    NodeDescriptor descriptor;
  };

  /// Resolves an endpoint id, counting a stale one in the diagnostics. Caller must hold at least a
  /// shared lock on registry_mutex_.
  EndpointState * resolve(EndpointId endpoint);

  /// Finds or creates the window for a statistic. Caller must hold the endpoint's mutex.
  static Measurement & measurement_for(EndpointState & state, StatKind stat);

  /// Which period statistic an endpoint reports, given what kind of endpoint it is.
  static StatKind period_stat_for(EndpointKind kind);

  const Options options_;
  const std::shared_ptr<Clock> clock_;

  mutable std::shared_mutex registry_mutex_;
  SlotTable<NodeState, NodeId> nodes_;
  SlotTable<EndpointState, EndpointId> endpoints_;

  std::atomic<uint64_t> stale_id_records_{0};
  std::atomic<uint64_t> mismatched_records_{0};
  std::atomic<uint64_t> unusable_source_timestamps_{0};
  std::atomic<uint64_t> evicted_endpoints_{0};
};

}  // namespace topic_stats_core

#endif  // TOPIC_STATS_CORE__COLLECTOR_HPP_
