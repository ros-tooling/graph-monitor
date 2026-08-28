// SPDX-FileCopyrightText: 2024 Bonsai Robotics, Inc.
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "rmw_stats_shim/stat_collector.hpp"

#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "rcpputils/env.hpp"

static const char * TSTAT_WINDOW_SIZE_VAR = "ROS_TOPIC_STATISTICS_WINDOW_SIZE";
static const char * TSTAT_TOPIC_VAR = "ROS_TOPIC_STATISTICS_TOPIC_NAME";
static const char * TSTAT_PUB_PERIOD_VAR = "ROS_TOPIC_STATISTICS_PUBLISH_PERIOD";

namespace
{

std::string getEnv(const char * env_var, const char * default_val)
{
  std::string val = rcpputils::get_env_var(env_var);
  if (val.empty()) {
    return default_val;
  }
  return val;
}

std::string fully_qualified_node_name(const char * name, const char * namespace_)
{
  if ('/' == namespace_[strlen(namespace_) - 1]) {
    return std::string(namespace_) + std::string(name);
  } else {
    return std::string(namespace_) + "/" + std::string(name);
  }
}

}  // namespace

namespace rmw_stats_shim
{

StatCollector & StatCollector::instance()
{
  // Lazy constructs the singleton on first access
  static StatCollector instance_;
  return instance_;
}

StatCollector::StatCollector()
: stats_pub_topic_name_(getEnv(TSTAT_TOPIC_VAR, "/topic_statistics"))
, sink_(stats_pub_topic_name_)
{
  topic_stats_core::Collector::Options options;
  options.window_size = std::stoul(getEnv(TSTAT_WINDOW_SIZE_VAR, "50"));
  collector_ =
    std::make_shared<topic_stats_core::Collector>(options, std::make_shared<topic_stats_core::SystemClock>());

  const float pub_period_s = std::stof(getEnv(TSTAT_PUB_PERIOD_VAR, "1.0"));
  pub_period_ = std::chrono::milliseconds(static_cast<size_t>(pub_period_s * 1000));

  timer_.emplace(std::bind(&StatCollector::publishStatistics, this), pub_period_);
  timer_->start();
}

StatCollector::~StatCollector()
{
  // Stop the timer first so it cannot be mid-publish while the publishers below are destroyed.
  if (timer_) {
    timer_->stop();
  }
  // Then destroy the publishers, which calls rmw_destroy_publisher and emits a graph event that
  // comes back through this class. That re-entrant call must find consistent state, so the handle
  // maps and the statistics core are still intact at this point and are torn down afterwards.
  sink_.clear();
  endpoints_.clear();
  nodes_.clear();
}

void StatCollector::setRmwImplementation(rcpputils::SharedLibrary * rmw_impl)
{
  sink_.set_rmw_implementation(rmw_impl);
}

void StatCollector::addNode(rmw_node_t * node)
{
  const auto id = collector_->register_node({fully_qualified_node_name(node->name, node->namespace_)});
  // Creating the statistics publisher emits a graph event that re-enters this class. It happens
  // before the node handle is mapped, so a re-entrant call simply finds nothing and returns.
  sink_.add_node(id, node);
  nodes_.add(node, id);
}

void StatCollector::removeNode(rmw_node_t * node)
{
  const auto id = nodes_.remove(node);
  if (!id) {
    return;
  }

  // Detached and destroyed here rather than inside the sink, so that the graph event emitted by
  // rmw_destroy_publisher re-enters with no lock of ours held.
  auto publisher = sink_.release_node(*id);
  publisher.reset();

  // Drop the endpoint handles before the core forgets the node, otherwise messages still in flight
  // would resolve to ids the core has already invalidated.
  endpoints_.remove_if([&](const EndpointEntry & entry) { return entry.node == *id; });
  collector_->unregister_node(*id);
}

void StatCollector::addEndpoint(
  const void * handle, const rmw_node_t * node, topic_stats_core::EndpointKind kind, const char * topic_name)
{
  const auto node_id = nodes_.find(node);
  if (!node_id) {
    return;
  }
  topic_stats_core::EndpointDescriptor descriptor;
  descriptor.kind = kind;
  descriptor.topic_name = topic_name;
  const auto endpoint_id = collector_->register_endpoint(*node_id, descriptor);
  if (!endpoint_id.valid()) {
    return;
  }
  endpoints_.add(handle, EndpointEntry{*node_id, endpoint_id});
}

void StatCollector::removeEndpoint(const void * handle)
{
  const auto entry = endpoints_.remove(handle);
  if (entry) {
    collector_->unregister_endpoint(entry->endpoint);
  }
}

void StatCollector::addPublisher(rmw_publisher_t * publisher, const rmw_node_t * node)
{
  addEndpoint(publisher, node, topic_stats_core::EndpointKind::Publisher, publisher->topic_name);
}

void StatCollector::removePublisher(rmw_publisher_t * publisher)
{
  removeEndpoint(publisher);
}

void StatCollector::onPublish(const rmw_publisher_t * publisher)
{
  // Unmapped handles are expected rather than exceptional: this class's own statistics publishers
  // are never registered, so they are naturally excluded from measurement.
  const auto entry = endpoints_.find(publisher);
  if (!entry) {
    return;
  }
  collector_->record_publish(entry->endpoint);
}

void StatCollector::addSubscription(rmw_subscription_t * subscription, const rmw_node_t * node)
{
  if (std::string(subscription->topic_name) == stats_pub_topic_name_) {
    // Seems more noisy than it's worth to also report on /topic_statistics
    return;
  }
  addEndpoint(subscription, node, topic_stats_core::EndpointKind::Subscription, subscription->topic_name);
}

void StatCollector::removeSubscription(rmw_subscription_t * subscription)
{
  removeEndpoint(subscription);
}

void StatCollector::onReceive(const rmw_subscription_t * subscription, rmw_message_info_t * message_info)
{
  const auto entry = endpoints_.find(subscription);
  if (!entry) {
    return;
  }
  std::optional<topic_stats_core::SourceTimestamp> source_timestamp;
  if (message_info != nullptr) {
    source_timestamp = message_info->source_timestamp;
  }
  collector_->record_take(entry->endpoint, source_timestamp);
}

void StatCollector::publishStatistics()
{
  sink_.publish(collector_->snapshot());
}

}  // namespace rmw_stats_shim
