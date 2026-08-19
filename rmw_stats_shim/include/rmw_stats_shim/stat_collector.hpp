// SPDX-FileCopyrightText: 2024 Bonsai Robotics, Inc.
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef RMW_STATS_SHIM__STAT_COLLECTOR_HPP_
#define RMW_STATS_SHIM__STAT_COLLECTOR_HPP_

#include <memory>
#include <optional>
#include <string>

#include "rcpputils/shared_library.hpp"
#include "rmw/rmw.h"
#include "rmw_stats_shim/rmw_publisher_sink.hpp"
#include "topic_stats_core/collector.hpp"
#include "topic_stats_core/handle_map.hpp"
#include "topic_stats_core/sink.hpp"
#include "topic_stats_core/timer.hpp"
#include "topic_stats_core/types.hpp"

namespace rmw_stats_shim
{

/// Which egress adapter a process reports through.
///
/// Selected per process by environment variable, so the same binary can be evaluated either way
/// without rebuilding. The statistics core and the ingest adapter are identical in both cases: only
/// the StatsSink implementation differs, which is the whole point of the split.
enum class EgressKind
{
  /// A /topic_statistics publisher per node, created against the RMW implementation. Needs no extra
  /// infrastructure, but publishes from inside the middleware's own call stack and adds a publisher
  /// per node to the graph being measured.
  RmwPublisher,
  /// A shared memory segment per process, drained by a separate collector. The instrumented process
  /// creates no ROS entity and cannot re-enter the middleware, at the cost of a collector to run.
  SharedMemory,
  /// Measure but do not report. Useful for isolating the cost of ingest.
  None,
};

/// Parses the egress selection. Returns false for an unrecognised name, leaving `kind` untouched.
bool egress_kind_from_string(const std::string & name, EgressKind & kind);

/// RMW ingest adapter, one per process.
///
/// Translates intercepted RMW API calls into topic_stats_core::Recorder calls, and owns the
/// reporting cadence and the egress adapter. It computes no statistics itself: everything below
/// the handle mapping lives in topic_stats_core, so that the tracepoint-based ingest can share it.
class StatCollector
{
public:
  StatCollector(StatCollector const &) = delete;
  void operator=(StatCollector const &) = delete;
  virtual ~StatCollector();

  /// @brief Accessor for singleton instance
  /// @return Static singleton instance of the class
  static StatCollector & instance();

  void setRmwImplementation(rcpputils::SharedLibrary * rmw_impl);
  void addNode(rmw_node_t * node);
  void removeNode(rmw_node_t * node);
  void addPublisher(rmw_publisher_t * publisher, const rmw_node_t * node);
  void removePublisher(rmw_publisher_t * publisher);
  void onPublish(const rmw_publisher_t * publisher);
  void addSubscription(rmw_subscription_t * subscription, const rmw_node_t * node);
  void removeSubscription(rmw_subscription_t * subscription);
  void onReceive(const rmw_subscription_t * subscription, rmw_message_info_t * message_info = nullptr);
  void publishStatistics();

private:
  StatCollector();

  /// What an endpoint handle maps to. The owning node is kept so that destroying a node can drop
  /// its endpoints from this map, which RMW teardown order does not otherwise guarantee.
  struct EndpointEntry
  {
    topic_stats_core::NodeId node;
    topic_stats_core::EndpointId endpoint;
  };

  /// Registers an endpoint of either kind. Returns quietly if the node is not tracked, which is
  /// the case for entities created before rmw_init reached the shim.
  void addEndpoint(
    const void * handle, const rmw_node_t * node, topic_stats_core::EndpointKind kind, const char * topic_name);
  void removeEndpoint(const void * handle);

  /// Builds the egress adapter selected by the environment. Falls back to reporting nothing if the
  /// selected one cannot be created, rather than preventing the process from starting.
  void createSink();

  std::string stats_pub_topic_name_;
  std::chrono::milliseconds pub_period_;

  std::shared_ptr<topic_stats_core::Collector> collector_;
  EgressKind egress_kind_ = EgressKind::RmwPublisher;
  std::unique_ptr<topic_stats_core::StatsSink> sink_;
  /// Non-owning view of sink_, non-null only when the publishing egress is selected. The RMW
  /// publisher sink needs per-node setup that the generic StatsSink interface does not expose,
  /// because creating a publisher needs an rmw_node_t that only this adapter has.
  RmwPublisherSink * rmw_sink_ = nullptr;

  topic_stats_core::HandleMap<topic_stats_core::NodeId> nodes_;
  topic_stats_core::HandleMap<EndpointEntry> endpoints_;

  std::optional<topic_stats_core::Timer> timer_;
};

}  // namespace rmw_stats_shim

#endif  // RMW_STATS_SHIM__STAT_COLLECTOR_HPP_
