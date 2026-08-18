// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef TOPIC_STATS_COLLECTOR__NODE_HPP_
#define TOPIC_STATS_COLLECTOR__NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_monitor_msgs/msg/topic_statistics.hpp"
#include "topic_stats_collector/segment_pool.hpp"

namespace topic_stats_collector
{

/// Publishes the statistics every instrumented process on this machine has written to shared
/// memory.
///
/// One of these per machine. It is an ordinary node with no special privileges, and nothing it does
/// affects the processes it is reading from: they write into their own segments whether or not this
/// is running.
class Node : public rclcpp::Node
{
public:
  explicit Node(const rclcpp::NodeOptions & options);

private:
  void poll();

  SegmentPool pool_;
  rclcpp::Publisher<rosgraph_monitor_msgs::msg::TopicStatistics>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  /// Accumulated since the last time they were reported, so that a persistent problem is visible
  /// without logging on every poll.
  uint64_t lapped_since_report_ = 0;
  uint64_t torn_since_report_ = 0;
  rclcpp::Time last_loss_report_;
};

}  // namespace topic_stats_collector

#endif  // TOPIC_STATS_COLLECTOR__NODE_HPP_
