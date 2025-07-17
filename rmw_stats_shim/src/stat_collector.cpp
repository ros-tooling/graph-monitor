// Copyright 2024, Bonsai Robotics, Inc - All Rights Reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rmw_stats_shim/stat_collector.hpp"

#include <string>

#include "rcpputils/env.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"

static const char * TSTAT_WINDOW_SIZE_VAR = "ROS_TOPIC_STATISTICS_WINDOW_SIZE";
static const char * TSTAT_TOPIC_VAR = "ROS_TOPIC_STATISTICS_TOPIC_NAME";
static const char * TSTAT_PUB_PERIOD_VAR = "ROS_TOPIC_STATISTICS_PUBLISH_PERIOD";

namespace
{

template<typename T>
size_t micros(T time)
{
  return std::chrono::duration_cast<std::chrono::microseconds>(time.time_since_epoch()).count();
}

template<typename C, typename D>
size_t nanos(std::chrono::time_point<C, D> time)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
}

template<typename R, typename P>
size_t nanos(std::chrono::duration<R, P> duration)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

inline void toLower(std::string & data)
{
  std::transform(
    data.begin(), data.end(), data.begin(),
    [](unsigned char c) {return std::tolower(c);});
}

inline std::string getEnv(const char * env_var, const char * default_val)
{
  std::string val = rcpputils::get_env_var(env_var);
  if (val.empty()) {
    return default_val;
  }
  return val;
}

template<typename C, typename D>
builtin_interfaces::msg::Time timeMessage(std::chrono::time_point<C, D> time)
{
  rmw_time_t rt = rmw_time_from_nsec(nanos(time.time_since_epoch()));
  builtin_interfaces::msg::Time msg;
  msg.sec = rt.sec;
  msg.nanosec = rt.nsec;
  return msg;
}

template<typename C, typename D>
builtin_interfaces::msg::Duration durationMessage(std::chrono::duration<C, D> time)
{
  rmw_time_t rt = rmw_time_from_nsec(nanos(time));
  builtin_interfaces::msg::Duration msg;
  msg.sec = rt.sec;
  msg.nanosec = rt.nsec;
  return msg;
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

EndpointStatistics::EndpointStatistics(
  EndpointType stat_type,
  const char * topic_name,
  const rmw_node_t * node,
  size_t window_size)
: node_(node),
  type_(stat_type),
  topic_name_(topic_name),
  node_name_(fully_qualified_node_name(node->name, node->namespace_)),
  period_acc_(window_size),
  age_acc_(window_size)
{}

void EndpointStatistics::onMessage(MonoTime ts)
{
  if (!last_ts_) {
    last_ts_ = ts;
    return;
  }
  auto delta = ts - *last_ts_;
  last_ts_ = ts;
  period_acc_.accumulate(delta);
  new_data_ = true;
}

rosgraph_monitor_msgs::msg::TopicStatistic EndpointStatistics::periodMsg()
{
  rosgraph_monitor_msgs::msg::TopicStatistic msg;
  msg.statistic_type = static_cast<uint8_t>(type_ == EndpointType::Publisher ?
    rosgraph_monitor_msgs::msg::TopicStatistic::PUBLISHED_PERIOD :
    rosgraph_monitor_msgs::msg::TopicStatistic::RECEIVED_PERIOD);
  msg.node_name = node_name_;
  msg.topic_name = topic_name_;
  msg.window_count = period_acc_.dataCount();
  msg.mean = durationMessage(period_acc_.getRollingMean());
  return msg;
}

bool EndpointStatistics::checkNewData()
{
  return new_data_.exchange(false);
}

void EndpointStatistics::onAge(std::chrono::nanoseconds age)
{
  age_acc_.accumulate(age);
  new_age_data_ = true;
}

bool EndpointStatistics::checkNewAgeData()
{
  return new_age_data_.exchange(false);
}

rosgraph_monitor_msgs::msg::TopicStatistic EndpointStatistics::ageMsg()
{
  rosgraph_monitor_msgs::msg::TopicStatistic msg;
  msg.statistic_type = rosgraph_monitor_msgs::msg::TopicStatistic::TAKE_AGE;
  msg.node_name = node_name_;
  msg.topic_name = topic_name_;
  msg.window_count = age_acc_.dataCount();
  msg.mean = durationMessage(age_acc_.getRollingMean());
  return msg;
}

#define REINTERP(SYMBOL, LIB) reinterpret_cast<decltype(SYMBOL) *>(LIB->get_symbol(#SYMBOL))

StatPublisher::StatPublisher(
  rcpputils::SharedLibrary * rmw_impl,
  rmw_node_t * node,
  std::string & stats_topic_name)
: node_(node),
  create_publisher_(REINTERP(rmw_create_publisher, rmw_impl)),
  destroy_publisher_(REINTERP(rmw_destroy_publisher, rmw_impl)),
  publish_(REINTERP(rmw_publish, rmw_impl))
{
  pub_opts_ = rmw_get_default_publisher_options();
  pub_ = create_publisher_(
    node,
    rosidl_typesupport_cpp::get_message_type_support_handle<
      rosgraph_monitor_msgs::msg::TopicStatistics>(),
    stats_topic_name.c_str(),
    &rmw_qos_profile_default,
    &pub_opts_);
}

StatPublisher::~StatPublisher()
{
  auto ret = destroy_publisher_(node_, pub_);
  (void)ret;
}

void StatPublisher::publish(rosgraph_monitor_msgs::msg::TopicStatistics & msg) const
{
  auto ret = publish_(pub_, &msg, nullptr);
  (void)ret;
}


StatCollector & StatCollector::instance()
{
  // Lazy constructs the singleton on first access
  static StatCollector instance_;
  return instance_;
}

void StatCollector::setRmwImplementation(rcpputils::SharedLibrary * rmw_impl)
{
  rmw_implementation_lib_ = rmw_impl;
}

void StatCollector::addNode(rmw_node_t * node)
{
  auto [it, added] = nodes_.try_emplace(node, rmw_implementation_lib_, node, stats_pub_topic_name_);
  if (added) {
    stat_publishers_.insert(it->second.pub_);
  }
}

void StatCollector::removeNode(rmw_node_t * node)
{
  nodes_.erase(node);
}

void StatCollector::addPublisher(rmw_publisher_t * publisher, const rmw_node_t * node)
{
  publishers_.try_emplace(
    publisher,
    EndpointType::Publisher, publisher->topic_name, node, window_size_);
}

void StatCollector::removePublisher(rmw_publisher_t * publisher)
{
  publishers_.erase(publisher);
}

void StatCollector::onPublish(const rmw_publisher_t * publisher)
{
  auto now = MonoClock::now();
  // Not doing topic statistics on our generated statistic publishers
  if (stat_publishers_.count(publisher)) {
    return;
  }
  auto it = publishers_.find(publisher);
  if (it != publishers_.end()) {
    it->second.onMessage(now);
  }
}

void StatCollector::addSubscription(rmw_subscription_t * subscription, const rmw_node_t * node)
{
  if (std::string(subscription->topic_name) == stats_pub_topic_name_) {
    // Seems more noisy than it's worth to also report on /topic_statistics
    return;
  }
  subscriptions_.try_emplace(
    subscription,
    EndpointType::Subscription, subscription->topic_name, node, window_size_);
}

void StatCollector::removeSubscription(rmw_subscription_t * subscription)
{
  subscriptions_.erase(subscription);
}

void StatCollector::onReceive(
  const rmw_subscription_t * subscription,
  rmw_message_info_t * message_info)
{
  auto now = MonoClock::now();
  auto it = subscriptions_.find(subscription);
  if (it != subscriptions_.end()) {
    it->second.onMessage(now);
    if (message_info != nullptr) {
      // CycloneDDS stamps messages with system time, so measure latency that way
      auto sysnow = SysClock::now();
      auto age = std::chrono::nanoseconds(nanos(sysnow) - message_info->source_timestamp);
      it->second.onAge(age);
    }
  }
}

void StatCollector::publishStatistics()
{
  for (auto & [node_ptr, stat_pub] : nodes_) {
    rosgraph_monitor_msgs::msg::TopicStatistics stats_msg;
    // Publishing as system time, ROS time is just not going to be supported,
    // there are too many moving parts and it's not relevant to our use of this
    stats_msg.timestamp = timeMessage(SysClock::now());
    for (auto & [pub_ptr, endpoint_stats] : publishers_) {
      if (stat_pub.node_ == endpoint_stats.node_ && endpoint_stats.checkNewData()) {
        stats_msg.statistics.push_back(endpoint_stats.periodMsg());
      }
    }
    for (auto & [sub_ptr, endpoint_stats] : subscriptions_) {
      if (stat_pub.node_ == endpoint_stats.node_) {
        if (endpoint_stats.checkNewData()) {
          stats_msg.statistics.push_back(endpoint_stats.periodMsg());
        }
        if (endpoint_stats.checkNewAgeData()) {
          stats_msg.statistics.push_back(endpoint_stats.ageMsg());
        }
      }
    }
    if (stats_msg.statistics.size()) {
      stat_pub.publish(stats_msg);
    }
  }
}

StatCollector::StatCollector()
{
  std::string window_size_val = getEnv(TSTAT_WINDOW_SIZE_VAR, "50");
  window_size_ = std::stoi(window_size_val);

  stats_pub_topic_name_ = getEnv(TSTAT_TOPIC_VAR, "/topic_statistics");

  std::string pub_period_val = getEnv(TSTAT_PUB_PERIOD_VAR, "1.0");
  float pub_period_s = std::stof(pub_period_val);
  pub_period_ = std::chrono::milliseconds(static_cast<size_t>(pub_period_s * 1000));

  timer_.emplace(
    std::bind(&StatCollector::publishStatistics, this),
    pub_period_);
  timer_->start();
}

StatCollector::~StatCollector()
{
  // It's important to destroy all the StatPublishers before the other member maps.
  // ~StatPublisher calls rmw_destroy_publisher, which triggers a graph event publish,
  // leading to rmw_publish->StatCollector::onPublish,
  // which uses these members and can cause segfault if they're already destroyed
  nodes_.clear();
}

}  // namespace rmw_stats_shim
