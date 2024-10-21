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

#ifndef RMW_STATS_SHIM__STAT_COLLECTOR_HPP_
#define RMW_STATS_SHIM__STAT_COLLECTOR_HPP_

#include <atomic>
#include <chrono>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "rcpputils/shared_library.hpp"
#include "rosgraph_monitor_msgs/msg/topic_statistics.hpp"
#include "rmw/rmw.h"

#include "rmw_stats_shim/rolling_mean_accumulator.hpp"
#include "rmw_stats_shim/timer.hpp"

typedef std::chrono::steady_clock MonoClock;
typedef std::chrono::system_clock SysClock;

typedef std::chrono::time_point<MonoClock> MonoTime;
typedef std::chrono::time_point<SysClock> SysTime;

namespace rmw_stats_shim
{

enum class EndpointType
{
  Publisher,
  Subscription
};

/**
 * @brief Rolling window statistics for a single topic endpoint (Publisher, Subscription).
 * Default statistic is "Period", that is the time between samples.
 * But this also provides another set of APIs for "Age", which is only relevant for subscriptions and is one
 * measure of latency.
 */
class EndpointStatistics
{
public:
  EndpointStatistics(
    EndpointType stat_type,
    const char * topic_name,
    const rmw_node_t * node,
    size_t window_size);
  virtual ~EndpointStatistics() = default;

  void onMessage(MonoTime ts);
  /// @brief Checks and clears new data flag
  /// @return Whether any new messages have been added since last call
  bool checkNewData();
  rosgraph_monitor_msgs::msg::TopicStatistic periodMsg();

  void onAge(std::chrono::nanoseconds age);
  /// @brief Checks and clears new data flag
  /// @return Whether any new messages have been added since last call
  bool checkNewAgeData();
  rosgraph_monitor_msgs::msg::TopicStatistic ageMsg();

  const rmw_node_t * const node_;

protected:
  std::atomic<bool> new_age_data_ = false;
  std::atomic<bool> new_data_ = false;
  EndpointType type_;
  std::string topic_name_;
  std::string node_name_;
  RollingMeanAccumulator<std::chrono::nanoseconds> period_acc_;
  RollingMeanAccumulator<std::chrono::nanoseconds> age_acc_;
  std::optional<MonoTime> last_ts_;
};

/**
 * @brief Create and manage a TopicStatistics publisher for one Node.
 */
class StatPublisher
{
public:
  StatPublisher(
    rcpputils::SharedLibrary * rmw_impl,
    rmw_node_t * node,
    std::string & stats_topic_name);
  virtual ~StatPublisher();
  void publish(rosgraph_monitor_msgs::msg::TopicStatistics & msg) const;

  rmw_publisher_t * pub_;
  rmw_node_t * node_;

protected:
  rmw_publisher_options_t pub_opts_;

  decltype(rmw_create_publisher) * create_publisher_;
  decltype(rmw_destroy_publisher) * destroy_publisher_;
  decltype(rmw_publish) * publish_;
};

/**
 * @brief RMW Topic Statistics Collector singleton class, for an entire process.
 * Receives intercepted RMW-API calls,
 * calculates statistics about endpoints,
 * and publishes them periodically.
 */
class StatCollector
{
private:
  StatCollector();

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
  void onReceive(
    const rmw_subscription_t * subscription,
    rmw_message_info_t * message_info = nullptr);
  void publishStatistics();

private:
  std::string stats_pub_topic_name_;
  std::chrono::milliseconds pub_period_;
  size_t window_size_;

  rcpputils::SharedLibrary * rmw_implementation_lib_ = nullptr;
  std::unordered_set<const rmw_publisher_t *> stat_publishers_;
  std::unordered_map<const rmw_publisher_t *, EndpointStatistics> publishers_;
  std::unordered_map<const rmw_subscription_t *, EndpointStatistics> subscriptions_;
  std::optional<rmw_stats_shim::Timer> timer_;
  std::unordered_map<const rmw_node_t *, StatPublisher> nodes_;
};

}  // namespace rmw_stats_shim

#endif  // RMW_STATS_SHIM__STAT_COLLECTOR_HPP_
