// SPDX-FileCopyrightText: 2024 Bonsai Robotics, Inc.
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef RMW_STATS_SHIM__RMW_PUBLISHER_SINK_HPP_
#define RMW_STATS_SHIM__RMW_PUBLISHER_SINK_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "rcpputils/shared_library.hpp"
#include "rmw/rmw.h"
#include "rosgraph_monitor_msgs/msg/topic_statistics.hpp"
#include "topic_stats_core/sink.hpp"
#include "topic_stats_core/types.hpp"

namespace rmw_stats_shim
{

/// One TopicStatistics publisher, created directly against the RMW implementation.
///
/// Deliberately bypasses the wrapper by resolving symbols out of the implementation library, so
/// that statistics traffic is not itself measured.
class StatPublisher
{
public:
  StatPublisher(rcpputils::SharedLibrary * rmw_impl, rmw_node_t * node, const std::string & topic_name);
  virtual ~StatPublisher();

  StatPublisher(const StatPublisher &) = delete;
  StatPublisher & operator=(const StatPublisher &) = delete;

  void publish(rosgraph_monitor_msgs::msg::TopicStatistics & msg) const;

  rmw_node_t * node() const
  {
    return node_;
  }

private:
  rmw_node_t * node_;
  rmw_publisher_options_t pub_opts_;
  rmw_publisher_t * pub_;

  decltype(rmw_create_publisher) * create_publisher_;
  decltype(rmw_destroy_publisher) * destroy_publisher_;
  decltype(rmw_publish) * publish_;
};

/// Egress adapter: publishes each node's statistics on a topic owned by that node.
///
/// This is the arrangement the shim has always used, kept as-is while the statistics themselves
/// move into topic_stats_core. It is also the awkward one, because creating and destroying RMW
/// entities emits graph events that re-enter the shim, so publisher lifetime is managed with no
/// lock of this class held.
class RmwPublisherSink : public topic_stats_core::StatsSink
{
public:
  explicit RmwPublisherSink(std::string topic_name);
  ~RmwPublisherSink() override;

  /// Must be called before any node is added. The shim learns the implementation library at
  /// rmw_init, which always precedes node creation.
  void set_rmw_implementation(rcpputils::SharedLibrary * rmw_impl);

  /// Creates the node's statistics publisher. Does nothing if the implementation library is not
  /// known yet, or if the node already has one.
  void add_node(topic_stats_core::NodeId id, rmw_node_t * node);

  /// Detaches the node's publisher and hands it back rather than destroying it here, so that the
  /// caller can destroy it outside of any lock. Destruction emits a graph event which re-enters
  /// the shim.
  std::unique_ptr<StatPublisher> release_node(topic_stats_core::NodeId id);

  /// Destroys every publisher. Must run before the statistics core goes away.
  void clear();

  void publish(const topic_stats_core::StatsReport & report) override;

  const std::string & topic_name() const
  {
    return topic_name_;
  }

private:
  const std::string topic_name_;
  rcpputils::SharedLibrary * rmw_implementation_lib_ = nullptr;

  mutable std::mutex mutex_;
  std::unordered_map<topic_stats_core::NodeId, std::unique_ptr<StatPublisher>> publishers_;
};

}  // namespace rmw_stats_shim

#endif  // RMW_STATS_SHIM__RMW_PUBLISHER_SINK_HPP_
