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

#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string_view>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "gmock/gmock.h"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"

#include "rosgraph_monitor/monitor.hpp"

using testing::SizeIs;
using testing::Return;


class MockGraph : public rclcpp::node_interfaces::NodeGraphInterface
{
public:
  // Have to do wrapper function to implement optional parameter
  std::map<std::string, std::vector<std::string>>
  get_topic_names_and_types(
    bool no_demangle = false) const override
  {
    return get_topic_names_and_types_mock_(no_demangle);
  }
  MOCK_METHOD(
    (std::map<std::string, std::vector<std::string>>),
    get_topic_names_and_types_mock_, (bool no_demangle), (const));
  MOCK_METHOD(
    (std::map<std::string, std::vector<std::string>>),
    get_service_names_and_types, (), (const, override));
  MOCK_METHOD(
    (std::map<std::string, std::vector<std::string>>),
    get_service_names_and_types_by_node,
    (const std::string &, const std::string &),
    (const, override));
  MOCK_METHOD(
    (std::map<std::string, std::vector<std::string>>),
    get_client_names_and_types_by_node,
    (const std::string &, const std::string &),
    (const, override));
  MOCK_METHOD(
    (std::map<std::string, std::vector<std::string>>),
    get_publisher_names_and_types_by_node,
    (const std::string &, const std::string &, bool),
    (const, override));
  MOCK_METHOD(
    (std::map<std::string, std::vector<std::string>>),
    get_subscriber_names_and_types_by_node,
    (const std::string &, const std::string &, bool),
    (const, override));
  MOCK_METHOD((std::vector<std::string>), get_node_names, (), (const, override));
  MOCK_METHOD(
    (std::vector<std::tuple<std::string, std::string, std::string>>),
    get_node_names_with_enclaves, (), (const, override));
  MOCK_METHOD(
    (std::vector<std::pair<std::string, std::string>>),
    get_node_names_and_namespaces, (), (const, override));
  MOCK_METHOD(size_t, count_publishers, (const std::string &), (const, override));
  MOCK_METHOD(size_t, count_subscribers, (const std::string &), (const, override));
  MOCK_METHOD(const rcl_guard_condition_t *, get_graph_guard_condition, (), (const, override));
  #ifndef ROS2_HUMBLE
  MOCK_METHOD(size_t, count_clients, (const std::string &), (const, override));
  MOCK_METHOD(size_t, count_services, (const std::string &), (const, override));
  #endif

  void
  notify_graph_change() override
  {
    {
      std::lock_guard<std::mutex> lock(mu_);
      for (auto & event : events_) {
        event->set();
      }
    }
    cv_.notify_all();
  }

  void
  notify_shutdown() override
  {
    cv_.notify_all();
  }

  rclcpp::Event::SharedPtr
  get_graph_event() override
  {
    std::lock_guard<std::mutex> lock(mu_);
    auto event = std::make_shared<rclcpp::Event>();
    events_.push_back(event);
    return event;
  }

  void
  wait_for_graph_change(
    rclcpp::Event::SharedPtr event,
    std::chrono::nanoseconds timeout) override
  {
    auto pred = [&event]() {
        return event->check();
      };
    std::unique_lock<std::mutex> graph_lock(mu_);
    if (!pred()) {
      cv_.wait_for(graph_lock, timeout, pred);
    }
  }

  MOCK_METHOD(size_t, count_graph_users, (), (const, override));
  MOCK_METHOD(
    (std::vector<rclcpp::TopicEndpointInfo>),
    get_publishers_info_by_topic,
    (const std::string &, bool),
    (const, override));
  MOCK_METHOD(
    (std::vector<rclcpp::TopicEndpointInfo>),
    get_subscriptions_info_by_topic,
    (const std::string &, bool),
    (const, override));

protected:
  std::vector<rclcpp::Event::SharedPtr> events_;
  std::mutex mu_;
  std::condition_variable cv_;
};

static rclcpp::TopicEndpointInfo blank_info()
{
  auto cinfo = rmw_get_zero_initialized_topic_endpoint_info();
  // It's okay to use this local pointer only because the data is copied into std::strings
  // and the pointers do not live on past this function
  const char * tmp = "";
  cinfo.node_name = tmp;
  cinfo.node_namespace = tmp;
  cinfo.topic_type = tmp;
  return rclcpp::TopicEndpointInfo{cinfo};
}

struct Endpoint
{
  std::string topic_name;
  rclcpp::TopicEndpointInfo info;

  Endpoint(
    rclcpp::EndpointType endpoint_type,
    const std::string & topic_name,
    const std::string & topic_type,
    const std::string & node_name,
    const rclcpp::QoS & qos)
  : topic_name(topic_name)
    , info(blank_info())
  {
    static RosRmwGid next_gid{0};
    info.node_name() = node_name;
    info.node_namespace() = "/";
    info.topic_type() = topic_type;
    info.endpoint_type() = endpoint_type;
    info.endpoint_gid() = next_gid;
    info.qos_profile() = qos;
    // Note this'll break past 255 endpoints, but let's don't build tests that large
    next_gid[0]++;
  }
};


class GraphMonitorTest : public testing::Test
{
protected:
  GraphMonitorTest()
  : logger_(rclcpp::get_logger("test_graphmon"))
  {
    // logger_.set_level(rclcpp::Logger::Level::Debug);
    node_graph_ = std::make_shared<testing::StrictMock<MockGraph>>();

    // Set up default empty graph state, to be overridden
    EXPECT_CALL(*node_graph_, get_node_names)
    .WillRepeatedly(
      [this]() {
        return node_names_;
      });
    EXPECT_CALL(*node_graph_, get_topic_names_and_types_mock_)
    .WillRepeatedly(
      [this](bool) {
        std::map<std::string, std::vector<std::string>> out;
        for (const auto & [git, endpoint] : endpoints_) {
          out[endpoint.topic_name].push_back(endpoint.info.topic_type());
        }
        return out;
      });
    EXPECT_CALL(*node_graph_, count_publishers)
    .WillRepeatedly(
      [this](const std::string & topic_name) {
        size_t count = 0;
        for (const auto & [gid, endpoint] : endpoints_) {
          if (endpoint.info.endpoint_type() == rclcpp::EndpointType::Publisher &&
          endpoint.topic_name == topic_name)
          {
            count++;
          }
        }
        return count;
      });
    EXPECT_CALL(*node_graph_, count_subscribers)
    .WillRepeatedly(
      [this](const std::string & topic_name) {
        size_t count = 0;
        for (const auto & [gid, endpoint] : endpoints_) {
          if (endpoint.info.endpoint_type() == rclcpp::EndpointType::Subscription &&
          endpoint.topic_name == topic_name)
          {
            count++;
          }
        }
        return count;
      });
    EXPECT_CALL(*node_graph_, get_publishers_info_by_topic)
    .WillRepeatedly(
      [this](const std::string & topic_name, bool = false) {
        std::vector<rclcpp::TopicEndpointInfo> out;
        for (const auto & [gid, endpoint] : endpoints_) {
          if (endpoint.info.endpoint_type() == rclcpp::EndpointType::Publisher &&
          endpoint.topic_name == topic_name)
          {
            out.emplace_back(endpoint.info);
          }
        }
        return out;
      });
    EXPECT_CALL(*node_graph_, get_subscriptions_info_by_topic)
    .WillRepeatedly(
      [this](const std::string & topic_name, bool = false) {
        std::vector<rclcpp::TopicEndpointInfo> out;
        for (const auto & [gid, endpoint] : endpoints_) {
          if (endpoint.info.endpoint_type() == rclcpp::EndpointType::Subscription &&
          endpoint.topic_name == topic_name)
          {
            out.emplace_back(endpoint.info);
          }
        }
        return out;
      });

    auto logger = logger_.get_child("graphmon");
    graphmon_.emplace(node_graph_, [this]() {return now_;}, logger);
  }

  void trigger_and_wait()
  {
    node_graph_->notify_graph_change();
    ASSERT_TRUE(graphmon_->wait_for_update(std::chrono::milliseconds(10)));
  }

  void set_node_names(std::vector<std::string> node_names)
  {
    // Add the root namespace / onto the names, which should not be specified with it
    node_names_.clear();
    node_names_.reserve(node_names.size());
    for (const auto & name : node_names) {
      node_names_.push_back("/" + name);
    }
    trigger_and_wait();
  }

  Endpoint add_endpoint(Endpoint endpoint)
  {
    endpoints_.emplace(endpoint.info.endpoint_gid(), endpoint);
    return endpoint;
  }

  Endpoint add_pub(
    const std::string & topic_name,
    const std::string & topic_type,
    const std::optional<std::string> & node_name = std::nullopt,
    const std::optional<rclcpp::QoS> & qos = std::nullopt)
  {
    return add_endpoint(
      Endpoint(
        rclcpp::EndpointType::Publisher,
        topic_name,
        topic_type,
        node_name.value_or(default_node_name_),
        qos.value_or(default_qos_)));
  }

  Endpoint add_sub(
    const std::string & topic_name,
    const std::string & topic_type,
    const std::optional<std::string> & node_name = std::nullopt,
    const std::optional<rclcpp::QoS> & qos = std::nullopt)
  {
    return add_endpoint(
      Endpoint(
        rclcpp::EndpointType::Subscription,
        topic_name,
        topic_type,
        node_name.value_or(default_node_name_),
        qos.value_or(default_qos_)));
  }

  void remove_endpoint(const Endpoint & endpoint)
  {
    endpoints_.erase(endpoint.info.endpoint_gid());
  }

  rosgraph_monitor_msgs::msg::TopicStatistic make_stat(
    uint8_t statistic_type,
    std::chrono::milliseconds mean,
    const std::optional<std::string> & node_name = std::nullopt,
    const std::optional<std::string> & topic_name = std::nullopt,
    int32_t window_count = 1)
  {
    rosgraph_monitor_msgs::msg::TopicStatistic stat;
    stat.statistic_type = statistic_type;
    stat.node_name = node_name.value_or("/" + default_node_name_);
    stat.topic_name = topic_name.value_or(default_topic_name_);
    stat.window_count = window_count;
    stat.mean = rclcpp::Duration(mean);
    return stat;
  }

  static const auto OK = diagnostic_msgs::msg::DiagnosticStatus::OK;
  static const auto WARN = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  static const auto ERROR = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  static const auto STALE = diagnostic_msgs::msg::DiagnosticStatus::STALE;
  const std::string nodes_diagnostic = "rosgraph/nodes";
  const std::string continuity_diagnostic = "rosgraph/continuity";
  const std::string pub_freq_diagnostic = "rosgraph/publish_frequency";
  const std::string sub_freq_diagnostic = "rosgraph/receive_frequency";

  /// @brief Evaluate the current graph monitoring status, look for the given diagnostic name,
  ///  and assert it matches expectations
  /// @param diagnostic_name Full name of the diagnostic to look at, ignore others
  /// @param level Expected level that diagnostic should be at
  /// @param maybe_message_pattern Optional string used to make a regex match against the message
  ///   if nullopt, message is not checked
  void
  check_status(
    const std::string & diagnostic_name,
    const uint8_t level,
    std::optional<std::string> maybe_message_pattern = std::nullopt)
  {
    diagnostic_msgs::msg::DiagnosticArray msg;
    graphmon_->evaluate(msg.status);
    auto it =
      std::find_if(
      msg.status.begin(), msg.status.end(), [&diagnostic_name](const auto & status) {
        return status.name == diagnostic_name;
      });
    ASSERT_NE(it, msg.status.end()) << "Expected diagnostic " << diagnostic_name << " not present";

    EXPECT_EQ(it->level, level);
    if (maybe_message_pattern) {
      std::regex message_re{*maybe_message_pattern};
      EXPECT_TRUE(
        std::regex_search(
          it->message,
          message_re)) << "Message '" << it->message << "' does not match regex R'" <<
        *maybe_message_pattern << "'";
    }
  }

  rclcpp::Time now_{0, 0, RCL_ROS_TIME};
  rclcpp::Logger logger_;
  std::shared_ptr<MockGraph> node_graph_;
  std::optional<rosgraph_monitor::RosGraphMonitor> graphmon_;

  const std::string default_node_name_ = "testy0";
  const std::string default_topic_name_ = "/topic1";
  const rclcpp::QoS default_qos_{10};
  std::vector<std::string> node_names_;
  std::unordered_map<RosRmwGid, Endpoint> endpoints_;
};

#define CHECK_STATUS(message, ...) {SCOPED_TRACE(message); check_status(__VA_ARGS__);}

TEST_F(GraphMonitorTest, node_liveness)
{
  const auto & name = nodes_diagnostic;
  std::vector<std::string> both_nodes = {"testy1", "testy2"};
  std::vector<std::string> one_node = {"testy1"};
  std::vector<std::string> no_nodes;

  set_node_names(both_nodes);
  set_node_names(one_node);
  CHECK_STATUS("One node gone missing", name, ERROR);

  // Returned
  set_node_names(both_nodes);
  CHECK_STATUS("Missing node returned", name, OK);

  // Both down
  set_node_names(no_nodes);
  CHECK_STATUS("Both nodes missing", name, ERROR, "^2 required node");

  // One returned
  set_node_names(one_node);
  CHECK_STATUS("One missing node returned", name, ERROR, "^1 required node");
}

TEST_F(GraphMonitorTest, ignore_nodes)
{
  const auto & name = nodes_diagnostic;
  graphmon_->config().nodes.ignore_prefixes = {"/ignore"};

  set_node_names({"ignore", "not_ignore"});
  set_node_names({"not_ignore"});
  CHECK_STATUS("Okay if ignored node is down", name, OK);

  set_node_names({"ignore", "ignore234"});
  set_node_names({});
  CHECK_STATUS("not_ignore went down", name, ERROR);
}

TEST_F(GraphMonitorTest, warn_nodes)
{
  const auto & name = nodes_diagnostic;
  graphmon_->config().nodes.warn_only_prefixes = {"/not_important"};

  set_node_names({"important", "not_important", "not_important_2"});
  set_node_names({"important"});
  CHECK_STATUS("Warn-only node warns when missing", name, WARN);
}

TEST_F(GraphMonitorTest, endpoint_continuity)
{
  const auto & name = continuity_diagnostic;

  set_node_names({default_node_name_});
  // /topic1 has pub and sub
  auto pub1 = add_pub("/topic1", "type1");
  auto sub1 = add_sub("/topic1", "type1");
  // /topic2 has no subs
  add_pub("/topic2", "type2");
  // /topic3 has no pubs
  add_sub("/topic3", "type3");
  trigger_and_wait();

  CHECK_STATUS("Two disconnected", name, WARN);

  // Connect /topic2
  add_sub("/topic2", "type2");
  trigger_and_wait();
  CHECK_STATUS("One reconnected", name, WARN);

  // Connect /topic3
  add_pub("/topic3", "type3");
  trigger_and_wait();
  CHECK_STATUS("Second reconnected", name, OK);

  // Disconnect something that was connected
  remove_endpoint(pub1);
  trigger_and_wait();
  CHECK_STATUS("Became disconnected", name, WARN);

  // Remove the last endpoint on a topic, no longer a discontinuity
  remove_endpoint(sub1);
  trigger_and_wait();
  CHECK_STATUS("Topic no longer exists", name, OK);
}

TEST_F(GraphMonitorTest, endpoint_continuity_ignored_subbernode_pub)
{
  const auto & name = continuity_diagnostic;
  graphmon_->config().continuity.ignore_subscriber_nodes = {"/ignore_subber"};
  set_node_names({"ignore_subber", "regular"});
  add_pub("/topic", "type", "ignore_subber");
  trigger_and_wait();
  CHECK_STATUS("Ignored sub's pub still discontinuous", name, WARN);

  add_sub("/topic", "type", "regular");
  trigger_and_wait();
  CHECK_STATUS("Ignore subber's pub got matched up", name, OK);
}

TEST_F(GraphMonitorTest, endpoint_continuity_ignored_subbernode_sub)
{
  const auto & name = continuity_diagnostic;

  graphmon_->config().continuity.ignore_subscriber_nodes = {"/ignore_subber"};
  set_node_names({"ignore_subber", "regular"});
  add_sub("/topic", "type", "ignore_subber");
  trigger_and_wait();
  CHECK_STATUS("Ignore subber unmet subscription is fine", name, OK);

  add_pub("/topic", "type", "regular");
  trigger_and_wait();
  CHECK_STATUS("Regular's pub is discontinuous since subscriber's node ignored.", name, WARN);
}

TEST_F(GraphMonitorTest, endpoint_continuity_ignore_topic_types)
{
  const auto & name = continuity_diagnostic;

  std::string ignore_type1 = "debug_msgs::msg::Debug";
  std::string ignore_type2 = "visualiser_msgs::msg::Vizz";
  graphmon_->config().continuity.ignore_topic_types = {ignore_type1, ignore_type2};
  set_node_names({default_node_name_});
  add_pub("/topic1", ignore_type1);
  add_sub("/topic2", ignore_type2);
  trigger_and_wait();
  CHECK_STATUS("Ignored topic types not reported disconnected.", name, OK);
}

TEST_F(GraphMonitorTest, endpoint_continuity_ignore_topic_names)
{
  const auto & name = continuity_diagnostic;

  std::string ignore_topic1 = "/some_debug_topic";
  std::string ignore_topic2 = "/other_debug_topic";
  graphmon_->config().continuity.ignore_topic_names = {ignore_topic1, ignore_topic2};
  set_node_names({default_node_name_});
  add_pub(ignore_topic1, "type");
  add_sub(ignore_topic2, "type");
  trigger_and_wait();
  CHECK_STATUS("Ignored topic names not reported disconnected", name, OK);
}

TEST_F(GraphMonitorTest, topic_frequency_no_deadline_dont_care)
{
  set_node_names({default_node_name_});
  rclcpp::QoS cyclone_received_qos = rclcpp::QoS{10}.deadline(
    rclcpp::Duration::from_rmw_time(RMW_DURATION_INFINITE));
  add_pub("/topic1", "type1");
  add_sub("/topic1", "type1", default_node_name_, cyclone_received_qos);
  trigger_and_wait();
  now_ = rclcpp::Time(5, 0, RCL_ROS_TIME);
  CHECK_STATUS(
    "Publisher with no deadline reports no frequency diagnostic", pub_freq_diagnostic,
    OK);
  CHECK_STATUS(
    "Subscription to no deadline reports no frequency diagnostic", sub_freq_diagnostic,
    OK);
}

TEST_F(GraphMonitorTest, topic_frequency_happy)
{
  const std::string topic_type = "type1";
  set_node_names({default_node_name_});
  rclcpp::QoS qos = rclcpp::QoS{10}.deadline(std::chrono::milliseconds(10));
  add_pub(default_topic_name_, topic_type, default_node_name_, qos);
  add_sub(default_topic_name_, topic_type, default_node_name_, qos);
  trigger_and_wait();

  rosgraph_monitor_msgs::msg::TopicStatistics stats;
  stats.timestamp = rclcpp::Time(1, 0);
  stats.statistics.push_back(
    make_stat(
      rosgraph_monitor_msgs::msg::TopicStatistic::PUBLISHED_PERIOD, std::chrono::milliseconds(9)));
  stats.statistics.push_back(
    make_stat(
      rosgraph_monitor_msgs::msg::TopicStatistic::RECEIVED_PERIOD, std::chrono::milliseconds(11)));
  graphmon_->on_topic_statistics(stats);
  CHECK_STATUS("Publisher with good topic stats", pub_freq_diagnostic, OK);
  CHECK_STATUS("Subscription with good topic stats", sub_freq_diagnostic, OK);
}

TEST_F(GraphMonitorTest, topic_frequency_slow)
{
  const std::string topic_type = "type1";
  set_node_names({default_node_name_});
  rclcpp::QoS qos = rclcpp::QoS{10}.deadline(std::chrono::milliseconds(10));
  add_pub(default_topic_name_, topic_type, default_node_name_, qos);
  add_sub(default_topic_name_, topic_type, default_node_name_, qos);
  trigger_and_wait();

  rosgraph_monitor_msgs::msg::TopicStatistics stats;
  stats.timestamp = rclcpp::Time(1, 0);
  stats.statistics.push_back(
    make_stat(
      rosgraph_monitor_msgs::msg::TopicStatistic::PUBLISHED_PERIOD, std::chrono::milliseconds(12)));
  stats.statistics.push_back(
    make_stat(
      rosgraph_monitor_msgs::msg::TopicStatistic::RECEIVED_PERIOD, std::chrono::milliseconds(15)));
  graphmon_->on_topic_statistics(stats);
  CHECK_STATUS("Publisher sending too slow", pub_freq_diagnostic, WARN);
  CHECK_STATUS("Subscription receiving from too slow pub", sub_freq_diagnostic, WARN);
}

TEST_F(GraphMonitorTest, topic_frequency_fast)
{
  const std::string topic_type = "type1";
  set_node_names({default_node_name_});
  rclcpp::QoS qos = rclcpp::QoS{10}.deadline(std::chrono::milliseconds(10));
  add_pub(default_topic_name_, topic_type, default_node_name_, qos);
  add_sub(default_topic_name_, topic_type, default_node_name_, qos);
  trigger_and_wait();

  rosgraph_monitor_msgs::msg::TopicStatistics stats;
  stats.timestamp = rclcpp::Time(1, 0);
  stats.statistics.push_back(
    make_stat(
      rosgraph_monitor_msgs::msg::TopicStatistic::PUBLISHED_PERIOD, std::chrono::milliseconds(8)));
  stats.statistics.push_back(
    make_stat(
      rosgraph_monitor_msgs::msg::TopicStatistic::RECEIVED_PERIOD, std::chrono::milliseconds(8)));
  graphmon_->on_topic_statistics(stats);
  CHECK_STATUS("Publisher sending fast is fine", pub_freq_diagnostic, OK);
  CHECK_STATUS("Subscription receiving fast is fine", sub_freq_diagnostic, OK);
}

TEST_F(GraphMonitorTest, topic_frequency_tx_good_rx_bad)
{
  const std::string topic_type = "type1";
  set_node_names({default_node_name_});
  rclcpp::QoS qos = rclcpp::QoS{10}.deadline(std::chrono::milliseconds(10));
  add_pub(default_topic_name_, topic_type, default_node_name_, qos);
  add_sub(default_topic_name_, topic_type, default_node_name_, qos);
  trigger_and_wait();

  rosgraph_monitor_msgs::msg::TopicStatistics stats;
  stats.timestamp = rclcpp::Time(1, 0);
  stats.statistics.push_back(
    make_stat(
      rosgraph_monitor_msgs::msg::TopicStatistic::PUBLISHED_PERIOD, std::chrono::milliseconds(10)));
  stats.statistics.push_back(
    make_stat(
      rosgraph_monitor_msgs::msg::TopicStatistic::RECEIVED_PERIOD, std::chrono::milliseconds(20)));
  graphmon_->on_topic_statistics(stats);
  CHECK_STATUS("Publisher sending fine", pub_freq_diagnostic, OK);
  CHECK_STATUS("But the subscription is receiving too slowly", sub_freq_diagnostic, WARN);
}

TEST_F(GraphMonitorTest, topic_frequency_tx_bad_rx_good)
{
  const std::string topic_type = "type1";
  set_node_names({default_node_name_});
  rclcpp::QoS qos = rclcpp::QoS{10}.deadline(std::chrono::milliseconds(10));
  add_pub(default_topic_name_, topic_type, default_node_name_, qos);
  add_sub(default_topic_name_, topic_type, default_node_name_, qos);
  trigger_and_wait();

  rosgraph_monitor_msgs::msg::TopicStatistics stats;
  stats.timestamp = rclcpp::Time(1, 0);
  stats.statistics.push_back(
    make_stat(
      rosgraph_monitor_msgs::msg::TopicStatistic::PUBLISHED_PERIOD, std::chrono::milliseconds(7)));
  stats.statistics.push_back(
    make_stat(
      rosgraph_monitor_msgs::msg::TopicStatistic::RECEIVED_PERIOD, std::chrono::milliseconds(10)));
  graphmon_->on_topic_statistics(stats);

  CHECK_STATUS("Publisher sending faster than deadline is OK", pub_freq_diagnostic, OK);
  CHECK_STATUS(
    "Subscription receiving slower than sent, but still within deadline",
    sub_freq_diagnostic, OK);
}

TEST_F(GraphMonitorTest, topic_frequency_not_received)
{
  const std::string topic_type = "type1";
  now_ = rclcpp::Time(100, 0, RCL_ROS_TIME);
  set_node_names({default_node_name_});
  rclcpp::QoS qos = rclcpp::QoS{10}.deadline(std::chrono::milliseconds(10));
  add_pub(default_topic_name_, topic_type, default_node_name_, qos);
  add_sub(default_topic_name_, topic_type, default_node_name_, qos);
  trigger_and_wait();

  now_ = rclcpp::Time(101, 0, RCL_ROS_TIME);
  CHECK_STATUS("Publisher stats not stale yet", pub_freq_diagnostic, OK);
  CHECK_STATUS("Subscription stats not stale yet", sub_freq_diagnostic, OK);

  now_ = rclcpp::Time(104, 0, RCL_ROS_TIME);
  CHECK_STATUS("Publisher stats stale", pub_freq_diagnostic, ERROR);
  CHECK_STATUS("Subscription stats stale", sub_freq_diagnostic, ERROR);
}

TEST_F(GraphMonitorTest, topic_frequency_stale)
{
  const std::string topic_type = "type1";
  set_node_names({default_node_name_});
  rclcpp::QoS qos = rclcpp::QoS{10}.deadline(std::chrono::milliseconds(10));
  auto pub = add_pub(default_topic_name_, topic_type, default_node_name_, qos);
  add_sub(default_topic_name_, topic_type, default_node_name_, qos);
  trigger_and_wait();

  rosgraph_monitor_msgs::msg::TopicStatistics stats;
  stats.timestamp = rclcpp::Time(1, 0);
  stats.statistics.push_back(
    make_stat(
      rosgraph_monitor_msgs::msg::TopicStatistic::PUBLISHED_PERIOD, std::chrono::milliseconds(10)));
  stats.statistics.push_back(
    make_stat(
      rosgraph_monitor_msgs::msg::TopicStatistic::RECEIVED_PERIOD, std::chrono::milliseconds(10)));
  graphmon_->on_topic_statistics(stats);
  CHECK_STATUS("Publisher stats fine", pub_freq_diagnostic, OK);
  CHECK_STATUS("Subscription stats fine", sub_freq_diagnostic, OK);

  now_ = rclcpp::Time(5, 0, RCL_ROS_TIME);
  CHECK_STATUS("Publisher stats now stale", pub_freq_diagnostic, ERROR);
  CHECK_STATUS("Subscription stats now stale", sub_freq_diagnostic, ERROR);

  stats.timestamp = rclcpp::Time(4, 0);
  graphmon_->on_topic_statistics(stats);
  CHECK_STATUS("Previously stale publisher stats back", pub_freq_diagnostic, OK);
  CHECK_STATUS("Previously stale subscription stats back", sub_freq_diagnostic, OK);

  remove_endpoint(pub);
  auto new_pub = add_pub(default_topic_name_, topic_type, default_node_name_, qos);
  trigger_and_wait();
  now_ = rclcpp::Time(10, 0, RCL_ROS_TIME);
  stats.timestamp = rclcpp::Time(10, 0);
  graphmon_->on_topic_statistics(stats);
  CHECK_STATUS("Publisher removed and replaced, not stale", pub_freq_diagnostic, OK);
  CHECK_STATUS("Subscription removed and replaced, not stale", sub_freq_diagnostic, OK);
}

TEST_F(GraphMonitorTest, rosgraph_generation) {
  // Set up test nodes
  set_node_names({"node1", "node2", "node3"});
  trigger_and_wait();

  // Generate rosgraph message
  rosgraph_monitor_msgs::msg::Graph rosgraph_msg;
  graphmon_->fill_rosgraph_msg(rosgraph_msg);

  // Verify the message contains expected nodes
  EXPECT_EQ(rosgraph_msg.nodes.size(), 3);

  // Verify node names are present
  std::vector<std::string> node_names;
  for (const auto & node : rosgraph_msg.nodes) {
    node_names.push_back(node.name);
  }

  EXPECT_THAT(
    node_names,
    testing::UnorderedElementsAre("/node1", "/node2", "/node3"));

  // Verify timestamp is set (should be current time in test environment)
  EXPECT_EQ(rosgraph_msg.timestamp, now_);
}

TEST_F(GraphMonitorTest, rosgraph_ignores_ignored_nodes) {
  // Set up some nodes, including one that should be ignored
  graphmon_->config().nodes.ignore_prefixes = {"/dummy"};
  set_node_names({"node1", "node2", "dummy/ignored_node"});
  trigger_and_wait();

  // Generate rosgraph message
  rosgraph_monitor_msgs::msg::Graph rosgraph_msg;
  graphmon_->fill_rosgraph_msg(rosgraph_msg);

  // Verify the message contains only non-ignored nodes
  EXPECT_EQ(rosgraph_msg.nodes.size(), 2);

  // Verify node names are present (should not include ignored node)
  std::vector<std::string> node_names;
  for (const auto & node : rosgraph_msg.nodes) {
    node_names.push_back(node.name);
  }

  EXPECT_THAT(node_names, testing::UnorderedElementsAre("/node1", "/node2"));
}
