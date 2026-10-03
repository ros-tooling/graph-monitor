// SPDX-FileCopyrightText: 2024 Bonsai Robotics, Inc.
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef RMW_STATS_SHIM__STATS_MESSAGE_HPP_
#define RMW_STATS_SHIM__STATS_MESSAGE_HPP_

#include <cstdint>

#include "rosgraph_monitor_msgs/msg/topic_statistics.hpp"
#include "topic_stats_core/types.hpp"

namespace rmw_stats_shim
{

/// Maps a core statistic onto its TopicStatistic constant.
///
/// Returns false for statistics the message has no representation for, which is not an error: the
/// core can measure more than this message can carry, and an egress adapter is entitled to drop
/// what it cannot express.
bool to_statistic_type(topic_stats_core::StatKind kind, uint8_t & statistic_type);

/// Converts one node's worth of a snapshot into the message the graph monitor consumes.
///
/// Split out from publishing so that it can be tested without a middleware. The monitor keys each
/// sample on (node_name, topic_name), so grouping by node is a convenience rather than a
/// requirement of the contract.
rosgraph_monitor_msgs::msg::TopicStatistics to_message(
  const topic_stats_core::NodeReport & node_report, topic_stats_core::SysTime timestamp);

}  // namespace rmw_stats_shim

#endif  // RMW_STATS_SHIM__STATS_MESSAGE_HPP_
