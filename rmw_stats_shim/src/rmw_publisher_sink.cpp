// SPDX-FileCopyrightText: 2024 Bonsai Robotics, Inc.
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "rmw_stats_shim/rmw_publisher_sink.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "rmw_stats_shim/stats_message.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"

#define REINTERP(SYMBOL, LIB) reinterpret_cast<decltype(SYMBOL) *>(LIB->get_symbol(#SYMBOL))

namespace rmw_stats_shim
{

StatPublisher::StatPublisher(rcpputils::SharedLibrary * rmw_impl, rmw_node_t * node, const std::string & topic_name)
: node_(node)
, create_publisher_(REINTERP(rmw_create_publisher, rmw_impl))
, destroy_publisher_(REINTERP(rmw_destroy_publisher, rmw_impl))
, publish_(REINTERP(rmw_publish, rmw_impl))
{
  pub_opts_ = rmw_get_default_publisher_options();
  pub_ = create_publisher_(
    node,
    rosidl_typesupport_cpp::get_message_type_support_handle<rosgraph_monitor_msgs::msg::TopicStatistics>(),
    topic_name.c_str(),
    &rmw_qos_profile_default,
    &pub_opts_);
}

StatPublisher::~StatPublisher()
{
  if (pub_ != nullptr) {
    const auto ret = destroy_publisher_(node_, pub_);
    (void)ret;
  }
}

void StatPublisher::publish(rosgraph_monitor_msgs::msg::TopicStatistics & msg) const
{
  if (pub_ == nullptr) {
    return;
  }
  const auto ret = publish_(pub_, &msg, nullptr);
  (void)ret;
}

RmwPublisherSink::RmwPublisherSink(std::string topic_name)
: topic_name_(std::move(topic_name))
{}

RmwPublisherSink::~RmwPublisherSink()
{
  clear();
}

void RmwPublisherSink::set_rmw_implementation(rcpputils::SharedLibrary * rmw_impl)
{
  std::lock_guard<std::mutex> lock(mutex_);
  rmw_implementation_lib_ = rmw_impl;
}

void RmwPublisherSink::add_node(topic_stats_core::NodeId id, rmw_node_t * node)
{
  rcpputils::SharedLibrary * rmw_impl = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (rmw_implementation_lib_ == nullptr || publishers_.count(id) > 0) {
      return;
    }
    rmw_impl = rmw_implementation_lib_;
  }

  // Constructed outside the lock: rmw_create_publisher emits a graph event, which comes back
  // through the shim, and a re-entrant publish() must not deadlock against this.
  auto publisher = std::make_unique<StatPublisher>(rmw_impl, node, topic_name_);

  std::lock_guard<std::mutex> lock(mutex_);
  publishers_.emplace(id, std::move(publisher));
}

std::unique_ptr<StatPublisher> RmwPublisherSink::release_node(topic_stats_core::NodeId id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = publishers_.find(id);
  if (it == publishers_.end()) {
    return nullptr;
  }
  auto publisher = std::move(it->second);
  publishers_.erase(it);
  return publisher;
}

void RmwPublisherSink::clear()
{
  // Same reasoning as add_node: destruction re-enters the shim, so it happens outside the lock.
  std::unordered_map<topic_stats_core::NodeId, std::unique_ptr<StatPublisher>> doomed;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    doomed.swap(publishers_);
  }
  doomed.clear();
}

void RmwPublisherSink::publish(const topic_stats_core::StatsReport & report)
{
  for (const auto & node_report : report.nodes) {
    auto msg = to_message(node_report, report.timestamp);
    if (msg.statistics.empty()) {
      continue;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = publishers_.find(node_report.node);
    if (it != publishers_.end()) {
      it->second->publish(msg);
    }
  }
}

}  // namespace rmw_stats_shim
