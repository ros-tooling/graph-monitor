// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef TOPIC_STATS_CORE__RECORDER_HPP_
#define TOPIC_STATS_CORE__RECORDER_HPP_

#include <optional>

#include "topic_stats_core/types.hpp"

namespace topic_stats_core
{

/// Everything an ingest adapter is allowed to do.
///
/// This is the entire surface between "something observed a message" and "statistics exist". An
/// adapter's job is to translate its native events into these calls and to remember the mapping
/// from its own handles to the ids returned here. Adapters do not compute statistics, do not
/// sample time, and never touch a transport.
///
/// Deliberately no timestamp parameters: implementations sample their own clock. An adapter
/// replaying recorded events supplies a clock it controls instead of passing timestamps through.
///
/// Implementations must tolerate calls on any thread, and must tolerate stale ids, since an ingest
/// adapter can be told about an endpoint's destruction out of order with a message on it.
class Recorder
{
public:
  virtual ~Recorder() = default;

  virtual NodeId register_node(const NodeDescriptor & descriptor) = 0;

  /// Also drops every endpoint still registered against the node. Adapters are not required to
  /// unregister endpoints first, because RMW teardown order does not guarantee they can.
  virtual void unregister_node(NodeId node) = 0;

  /// Returns an invalid id if the node handle is stale, which the adapter may ignore.
  virtual EndpointId register_endpoint(NodeId node, const EndpointDescriptor & descriptor) = 0;

  virtual void unregister_endpoint(EndpointId endpoint) = 0;

  /// A message was published on this endpoint. Hot path.
  virtual void record_publish(EndpointId endpoint) = 0;

  /// A message was taken on this endpoint. Hot path.
  ///
  /// \param source_timestamp The publisher's system-clock timestamp for the message, if the
  ///   middleware reported one. Used for take age. Pass nullopt when unavailable, which is the
  ///   common case for a take that reports no message info.
  virtual void record_take(EndpointId endpoint, std::optional<SourceTimestamp> source_timestamp) = 0;

  /// An already-measured duration, for statistics the collector cannot derive from event timing.
  virtual void record_duration(EndpointId endpoint, StatKind stat, Duration value) = 0;
};

}  // namespace topic_stats_core

#endif  // TOPIC_STATS_CORE__RECORDER_HPP_
