// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "topic_stats_collector/node.hpp"

#include <unistd.h>

#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <string>

#include "topic_stats_ros/stats_message.hpp"
#include "topic_stats_shm/wire_format.hpp"

namespace topic_stats_collector
{

namespace
{

SegmentPool::Options pool_options_from(rclcpp::Node & node)
{
  SegmentPool::Options options;
  options.reclaim_dead_segments = node.declare_parameter<bool>("reclaim_dead_segments", true);
  options.skip_backlog_on_attach = node.declare_parameter<bool>("skip_backlog_on_attach", true);
  // If this collector is itself running under the statistics shim it will have written a segment of
  // its own, which there is no point in reporting on.
  options.ignore_identity = topic_stats_shm::current_process_identity();
  return options;
}

}  // namespace

Node::Node(const rclcpp::NodeOptions & options)
: rclcpp::Node("topic_stats_collector", options)
, pool_(pool_options_from(*this))
, last_loss_report_(now())
{
  const auto output_topic = declare_parameter<std::string>("output_topic", "/topic_statistics");
  const int64_t poll_period_ms = declare_parameter<int>("poll_period_ms", 200);

  pub_ = create_publisher<rosgraph_monitor_msgs::msg::TopicStatistics>(output_topic, rclcpp::QoS{10});
  timer_ = create_wall_timer(std::chrono::milliseconds(poll_period_ms), [this]() { poll(); });

  RCLCPP_INFO(
    get_logger(),
    "Collecting topic statistics from shared memory every %ldms, publishing on %s",
    poll_period_ms,
    output_topic.c_str());
}

void Node::poll()
{
  auto result = pool_.poll();

  for (const auto & report : result.reports) {
    for (const auto & node_report : report.nodes) {
      auto msg = topic_stats_ros::to_message(node_report, report.timestamp);
      if (msg.statistics.empty()) {
        continue;
      }
      pub_->publish(msg);
    }
  }

  if (result.attached > 0) {
    RCLCPP_INFO(get_logger(), "Attached to %zu new statistics writer(s)", result.attached);
  }
  if (result.detached > 0) {
    RCLCPP_INFO(
      get_logger(), "%zu statistics writer(s) exited, %zu segment(s) reclaimed", result.detached, result.reclaimed);
  }
  if (result.rejected > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      60000,
      "%zu shared memory segment(s) could not be read; check format versions",
      result.rejected);
  }
  if (result.unknown_statistics > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      60000,
      "Discarded %" PRIu64 " sample(s) measuring statistics this build does not recognise",
      result.unknown_statistics);
  }

  // Loss means this collector is not keeping up, which would otherwise show downstream as topics
  // that intermittently look stale. Worth saying out loud, but not every poll.
  lapped_since_report_ += result.lapped;
  torn_since_report_ += result.torn;
  const auto since_report = now() - last_loss_report_;
  if ((lapped_since_report_ > 0 || torn_since_report_ > 0) && since_report > rclcpp::Duration(10, 0)) {
    RCLCPP_WARN(
      get_logger(),
      "Lost %" PRIu64 " sample(s) to overwrite and %" PRIu64
      " to contention in the last %.0fs; poll faster or raise %s",
      lapped_since_report_,
      torn_since_report_,
      since_report.seconds(),
      "ROS_TOPIC_STATISTICS_SHM_CAPACITY");
    lapped_since_report_ = 0;
    torn_since_report_ = 0;
    last_loss_report_ = now();
  }
}

}  // namespace topic_stats_collector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(topic_stats_collector::Node)
