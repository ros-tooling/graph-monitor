// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef TOPIC_STATS_CORE__TYPES_HPP_
#define TOPIC_STATS_CORE__TYPES_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace topic_stats_core
{

using Duration = std::chrono::nanoseconds;
using MonoClock = std::chrono::steady_clock;
using SysClock = std::chrono::system_clock;
using MonoTime = std::chrono::time_point<MonoClock>;
using SysTime = std::chrono::time_point<SysClock>;

/// Nanoseconds since the system clock epoch, matching what rmw_message_info_t and the
/// ros2:rmw_take tracepoint both report for a message's source timestamp.
using SourceTimestamp = int64_t;

/// A DDS/rmw GID, if the ingest adapter has one. All zeroes means "not known".
using Gid = std::array<uint8_t, 16>;

/// Handle into one of the collector's slot tables.
///
/// Templated on a tag so that a NodeId cannot be silently passed where an EndpointId is wanted.
/// Slots are reused after being freed, so the generation counter is what makes a stale handle
/// detectable rather than silently aliasing onto whatever took the slot next.
template <typename Tag>
struct Id
{
  static constexpr uint32_t kInvalidIndex = std::numeric_limits<uint32_t>::max();

  uint32_t index = kInvalidIndex;
  uint32_t generation = 0;

  bool valid() const
  {
    return index != kInvalidIndex;
  }
};

template <typename Tag>
inline bool operator==(const Id<Tag> & lhs, const Id<Tag> & rhs)
{
  return lhs.index == rhs.index && lhs.generation == rhs.generation;
}

template <typename Tag>
inline bool operator!=(const Id<Tag> & lhs, const Id<Tag> & rhs)
{
  return !(lhs == rhs);
}

struct NodeTag;
struct EndpointTag;

using NodeId = Id<NodeTag>;
using EndpointId = Id<EndpointTag>;

enum class EndpointKind
{
  Publisher,
  Subscription,
};

/// What is being measured about an endpoint.
///
/// Period is derived by the collector from the timing of recorded events. Everything else is a
/// duration handed to the collector already computed, or computed from a source timestamp.
enum class StatKind
{
  /// Time between successive publishes on a publisher.
  PublishedPeriod,
  /// Time between successive takes on a subscription.
  ReceivedPeriod,
  /// Publisher's source timestamp to the moment the subscription took the message. Sum of
  /// transport latency and executor latency, and only meaningful if publisher and subscriber
  /// clocks agree.
  TakeAge,
  /// Wall time spent inside a subscription or timer callback.
  CallbackDuration,
};

struct NodeDescriptor
{
  /// Fully qualified node name, namespace included, e.g. "/some_ns/some_node".
  std::string fully_qualified_name;
};

struct EndpointDescriptor
{
  EndpointKind kind = EndpointKind::Publisher;
  std::string topic_name;
  Gid gid{};
};

/// One statistic about one endpoint over the collector's rolling window.
struct StatSample
{
  StatKind stat = StatKind::PublishedPeriod;
  std::string node_name;
  std::string topic_name;
  /// Number of measurements the mean/min/max were computed over. Never zero in a report.
  uint32_t window_count = 0;
  Duration mean{0};
  Duration min{0};
  Duration max{0};
};

/// Samples belonging to a single node, so that sinks which publish per-node do not have to
/// regroup by name.
struct NodeReport
{
  NodeId node;
  std::string node_name;
  std::vector<StatSample> samples;
};

struct StatsReport
{
  SysTime timestamp{};
  std::vector<NodeReport> nodes;

  bool empty() const
  {
    return nodes.empty();
  }
};

/// Whether a snapshot reports every endpoint or only those with measurements since the last
/// snapshot. OnlyChanged is what a periodic publisher wants: an endpoint that has gone silent
/// should stop being reported so that downstream staleness detection can fire.
enum class SnapshotMode
{
  OnlyChanged,
  All,
};

}  // namespace topic_stats_core

namespace std
{

template <typename Tag>
struct hash<topic_stats_core::Id<Tag>>
{
  size_t operator()(const topic_stats_core::Id<Tag> & id) const noexcept
  {
    return (static_cast<size_t>(id.generation) << 32) ^ static_cast<size_t>(id.index);
  }
};

}  // namespace std

#endif  // TOPIC_STATS_CORE__TYPES_HPP_
