// SPDX-FileCopyrightText: 2024 Bonsai Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "gmock/gmock.h"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rosgraph_monitor/monitor.hpp"

using testing::Return;
using testing::SizeIs;

/// How long to wait for a state change that should happen.
constexpr auto kGenerousWait = std::chrono::seconds(5);
/// How long to wait before asserting that a state change did not happen.
constexpr auto kSettle = std::chrono::milliseconds(300);

/// Polls `predicate` until it holds or `timeout` expires.
template <typename Predicate>
bool wait_for_condition(Predicate predicate, std::chrono::milliseconds timeout = kGenerousWait)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return predicate();
}

/// Configuration whose parameter queries retry quickly and never time out on their own.
rosgraph_monitor::GraphMonitorConfiguration patient_config(size_t max_concurrent = 4)
{
  rosgraph_monitor::GraphMonitorConfiguration config;
  config.parameters.max_concurrent = max_concurrent;
  config.parameters.timeout = std::chrono::seconds(30);
  config.parameters.retry_delay = std::chrono::milliseconds(50);
  return config;
}

class MockGraph : public rclcpp::node_interfaces::NodeGraphInterface
{
public:
  // Have to do wrapper function to implement optional parameter
  std::map<std::string, std::vector<std::string>> get_topic_names_and_types(bool no_demangle = false) const override
  {
    return get_topic_names_and_types_mock_(no_demangle);
  }

  MOCK_METHOD(
    (std::map<std::string, std::vector<std::string>>), get_topic_names_and_types_mock_, (bool no_demangle), (const));
  MOCK_METHOD((std::map<std::string, std::vector<std::string>>), get_service_names_and_types, (), (const, override));
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
    get_node_names_with_enclaves,
    (),
    (const, override));
  MOCK_METHOD((std::vector<std::pair<std::string, std::string>>), get_node_names_and_namespaces, (), (const, override));
  MOCK_METHOD(size_t, count_publishers, (const std::string &), (const, override));
  MOCK_METHOD(size_t, count_subscribers, (const std::string &), (const, override));
  MOCK_METHOD(const rcl_guard_condition_t *, get_graph_guard_condition, (), (const, override));
#ifndef ROS2_HUMBLE
  MOCK_METHOD(size_t, count_clients, (const std::string &), (const, override));
  MOCK_METHOD(size_t, count_services, (const std::string &), (const, override));
#endif

#if !defined(ROS2_HUMBLE) && !defined(ROS2_JAZZY) && !defined(ROS2_KILTED)
  MOCK_METHOD(
    (std::vector<rclcpp::ServiceEndpointInfo>),
    get_clients_info_by_service,
    (const std::string &, bool),
    (const, override));
  MOCK_METHOD(
    (std::vector<rclcpp::ServiceEndpointInfo>),
    get_servers_info_by_service,
    (const std::string &, bool),
    (const, override));
#endif

  void notify_graph_change() override
  {
    {
      std::lock_guard<std::mutex> lock(mu_);
      for (auto & event : events_) {
        event->set();
      }
    }
    cv_.notify_all();
  }

  void notify_shutdown() override
  {
    cv_.notify_all();
  }

  rclcpp::Event::SharedPtr get_graph_event() override
  {
    std::lock_guard<std::mutex> lock(mu_);
    auto event = std::make_shared<rclcpp::Event>();
    events_.push_back(event);
    return event;
  }

  void wait_for_graph_change(rclcpp::Event::SharedPtr event, std::chrono::nanoseconds timeout) override
  {
    auto pred = [&event]() { return event->check(); };
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

struct MockedNode
{
  std::string name;
  std::vector<std::string> params;

  explicit MockedNode(const std::string & name, const std::vector<std::string> & params = {})
  : name(name)
  , params(params)
  {}
};

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

/// @return One descriptor per name, each carrying `type`.
static std::vector<rcl_interfaces::msg::ParameterDescriptor> descriptors_named(
  const std::vector<std::string> & names, uint8_t type)
{
  std::vector<rcl_interfaces::msg::ParameterDescriptor> descriptors;
  descriptors.reserve(names.size());
  for (const auto & name : names) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.name = name;
    descriptor.type = type;
    descriptors.push_back(std::move(descriptor));
  }
  return descriptors;
}

/// @return One integer value per name, each carrying that name's index in `names`.
static std::vector<rcl_interfaces::msg::ParameterValue> indexed_values(const std::vector<std::string> & names)
{
  std::vector<rcl_interfaces::msg::ParameterValue> values;
  values.reserve(names.size());
  for (size_t index = 0; index < names.size(); index++) {
    rcl_interfaces::msg::ParameterValue value;
    value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    value.integer_value = static_cast<int64_t>(index);
    values.push_back(std::move(value));
  }
  return values;
}

/// @return The integer value of each entry, in order.
static std::vector<int64_t> integer_values(const std::vector<rcl_interfaces::msg::ParameterValue> & values)
{
  std::vector<int64_t> integers;
  integers.reserve(values.size());
  for (const auto & value : values) {
    integers.push_back(value.integer_value);
  }
  return integers;
}

/// @return The names of each descriptor, in order.
static std::vector<std::string> descriptor_names(
  const std::vector<rcl_interfaces::msg::ParameterDescriptor> & descriptors)
{
  std::vector<std::string> names;
  names.reserve(descriptors.size());
  for (const auto & descriptor : descriptors) {
    names.push_back(descriptor.name);
  }
  return names;
}

/// @return Whether every descriptor carries `type`, false if there are none.
static bool all_typed(const std::vector<rcl_interfaces::msg::ParameterDescriptor> & descriptors, uint8_t type)
{
  return !descriptors.empty() && std::all_of(descriptors.begin(), descriptors.end(), [type](const auto & descriptor) {
    return descriptor.type == type;
  });
}

/// Answers parameter queries straight from the mocked graph, on the calling thread.
/// Names come from MockedNode, which is enough to see the monitor carry them into the graph message.
/// A node listed in `unreachable` has no parameter services up yet.
class MockParameterService : public rosgraph_monitor::ParameterServiceClient
{
public:
  explicit MockParameterService(std::function<std::vector<std::string>(const std::string &)> names_for)
  : names_for_(std::move(names_for))
  {}

  bool is_ready(const std::string & node_name) override
  {
    std::lock_guard<std::mutex> lock(mutex);
    return unreachable.count(node_name) == 0;
  }

  void list_parameters(const std::string & node_name, NamesCallback callback) override
  {
    callback(names_for_(node_name));
  }

  void describe_parameters(
    const std::string &, const std::vector<std::string> & names, DescriptorsCallback callback) override
  {
    callback(descriptors_named(names, described_type));
  }

  void get_parameters(const std::string &, const std::vector<std::string> & names, ValuesCallback callback) override
  {
    callback(indexed_values(names));
  }

  void forget(const std::string & node_name) override
  {
    std::lock_guard<std::mutex> lock(mutex);
    forgotten.insert(node_name);
  }

  void set_reachable(const std::string & node_name)
  {
    std::lock_guard<std::mutex> lock(mutex);
    unreachable.erase(node_name);
  }

  std::mutex mutex;
  std::set<std::string> unreachable;
  std::set<std::string> forgotten;
  /// The type every described parameter is reported as.
  uint8_t described_type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;

private:
  std::function<std::vector<std::string>(const std::string &)> names_for_;
};

/// Holds every parameter query until the test answers or fails it, counting attempts per node
/// and recording which nodes have been forgotten.
/// Every method is safe to call from the queue's worker thread and the test thread at once.
class BlockingParameterService : public rosgraph_monitor::ParameterServiceClient
{
public:
  bool is_ready(const std::string &) override
  {
    return true;
  }

  void list_parameters(const std::string & node_name, NamesCallback callback) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    starts_[node_name]++;
    pending_[node_name] = std::move(callback);
  }

  void describe_parameters(
    const std::string & node_name, const std::vector<std::string> & names, DescriptorsCallback callback) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    describe_starts_[node_name]++;
    described_names_[node_name] = names;
    pending_describes_[node_name] = std::move(callback);
  }

  void get_parameters(
    const std::string & node_name, const std::vector<std::string> & names, ValuesCallback callback) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    value_starts_[node_name]++;
    valued_names_[node_name] = names;
    pending_values_[node_name] = std::move(callback);
  }

  void forget(const std::string & node_name) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    forgotten_.insert(node_name);
  }

  /// Delivers `names` as the result of the outstanding query for `node_name`.
  void answer(const std::string & node_name, std::vector<std::string> names)
  {
    if (auto callback = take(node_name)) {
      callback(std::move(names));
    }
  }

  /// Reports the outstanding query for `node_name` as failed.
  void fail(const std::string & node_name)
  {
    if (auto callback = take(node_name)) {
      callback(std::nullopt);
    }
  }

  /// Delivers `descriptors` as the result of the outstanding describe for `node_name`.
  void answer_describe(const std::string & node_name, std::vector<rcl_interfaces::msg::ParameterDescriptor> descriptors)
  {
    if (auto callback = take_describe(node_name)) {
      callback(std::move(descriptors));
    }
  }

  /// Reports the outstanding describe for `node_name` as failed.
  void fail_describe(const std::string & node_name)
  {
    if (auto callback = take_describe(node_name)) {
      callback(std::nullopt);
    }
  }

  /// Delivers `values` as the result of the outstanding value query for `node_name`.
  void answer_values(const std::string & node_name, std::vector<rcl_interfaces::msg::ParameterValue> values)
  {
    if (auto callback = take_values(node_name)) {
      callback(std::move(values));
    }
  }

  /// Reports the outstanding value query for `node_name` as failed.
  void fail_values(const std::string & node_name)
  {
    if (auto callback = take_values(node_name)) {
      callback(std::nullopt);
    }
  }

  /// @return The nodes whose queries are outstanding.
  std::set<std::string> outstanding()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::set<std::string> node_names;
    for (const auto & [node_name, callback] : pending_) {
      node_names.insert(node_name);
    }
    return node_names;
  }

  /// @return How many distinct nodes have had a query started for them.
  size_t started_count()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return starts_.size();
  }

  /// @return How many queries have been started for `node_name`.
  size_t start_count(const std::string & node_name)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = starts_.find(node_name);
    return it == starts_.end() ? 0 : it->second;
  }

  /// @return How many describes have been started for `node_name`.
  size_t describe_start_count(const std::string & node_name)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = describe_starts_.find(node_name);
    return it == describe_starts_.end() ? 0 : it->second;
  }

  /// @return The names the most recent describe for `node_name` asked about.
  std::vector<std::string> described_names(const std::string & node_name)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = described_names_.find(node_name);
    return it == described_names_.end() ? std::vector<std::string>{} : it->second;
  }

  /// @return How many value queries have been started for `node_name`.
  size_t value_start_count(const std::string & node_name)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = value_starts_.find(node_name);
    return it == value_starts_.end() ? 0 : it->second;
  }

  /// @return The names the most recent value query for `node_name` asked about.
  std::vector<std::string> valued_names(const std::string & node_name)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = valued_names_.find(node_name);
    return it == valued_names_.end() ? std::vector<std::string>{} : it->second;
  }

  /// @return Whether `node_name` has been forgotten.
  bool forgotten(const std::string & node_name)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return forgotten_.count(node_name) != 0;
  }

private:
  /// @return The outstanding query's callback for `node_name`, removing it. Empty if there is none.
  NamesCallback take(const std::string & node_name)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = pending_.find(node_name);
    if (it == pending_.end()) {
      return NamesCallback();
    }
    NamesCallback callback = std::move(it->second);
    pending_.erase(it);
    return callback;
  }

  /// @return The outstanding describe's callback for `node_name`, removing it. Empty if there is none.
  DescriptorsCallback take_describe(const std::string & node_name)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = pending_describes_.find(node_name);
    if (it == pending_describes_.end()) {
      return DescriptorsCallback();
    }
    DescriptorsCallback callback = std::move(it->second);
    pending_describes_.erase(it);
    return callback;
  }

  /// @return The outstanding value query's callback for `node_name`, removing it. Empty if there is none.
  ValuesCallback take_values(const std::string & node_name)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = pending_values_.find(node_name);
    if (it == pending_values_.end()) {
      return ValuesCallback();
    }
    ValuesCallback callback = std::move(it->second);
    pending_values_.erase(it);
    return callback;
  }

  std::mutex mutex_;
  std::map<std::string, NamesCallback> pending_;
  std::map<std::string, size_t> starts_;
  std::map<std::string, DescriptorsCallback> pending_describes_;
  std::map<std::string, size_t> describe_starts_;
  std::map<std::string, std::vector<std::string>> described_names_;
  std::map<std::string, ValuesCallback> pending_values_;
  std::map<std::string, size_t> value_starts_;
  std::map<std::string, std::vector<std::string>> valued_names_;
  std::set<std::string> forgotten_;
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
    EXPECT_CALL(*node_graph_, get_node_names).WillRepeatedly([this]() {
      std::lock_guard<std::mutex> lock(mocks_mutex_);
      std::vector<std::string> node_names;
      for (const auto & node : mocked_nodes_) {
        node_names.push_back(node.name);
      }
      return node_names;
    });
    EXPECT_CALL(*node_graph_, get_topic_names_and_types_mock_).WillRepeatedly([this](bool) {
      std::map<std::string, std::vector<std::string>> out;
      for (const auto & [git, endpoint] : endpoints_) {
        out[endpoint.topic_name].push_back(endpoint.info.topic_type());
      }
      return out;
    });
    EXPECT_CALL(*node_graph_, count_publishers).WillRepeatedly([this](const std::string & topic_name) {
      size_t count = 0;
      for (const auto & [gid, endpoint] : endpoints_) {
        if (endpoint.info.endpoint_type() == rclcpp::EndpointType::Publisher && endpoint.topic_name == topic_name) {
          count++;
        }
      }
      return count;
    });
    EXPECT_CALL(*node_graph_, count_subscribers).WillRepeatedly([this](const std::string & topic_name) {
      size_t count = 0;
      for (const auto & [gid, endpoint] : endpoints_) {
        if (endpoint.info.endpoint_type() == rclcpp::EndpointType::Subscription && endpoint.topic_name == topic_name) {
          count++;
        }
      }
      return count;
    });
    EXPECT_CALL(*node_graph_, get_publishers_info_by_topic)
      .WillRepeatedly([this](const std::string & topic_name, bool = false) {
        std::lock_guard<std::mutex> lock(mocks_mutex_);
        std::vector<rclcpp::TopicEndpointInfo> out;
        for (const auto & [gid, endpoint] : endpoints_) {
          if (endpoint.info.endpoint_type() == rclcpp::EndpointType::Publisher && endpoint.topic_name == topic_name) {
            out.emplace_back(endpoint.info);
          }
        }
        return out;
      });
    EXPECT_CALL(*node_graph_, get_subscriptions_info_by_topic)
      .WillRepeatedly([this](const std::string & topic_name, bool = false) {
        std::lock_guard<std::mutex> lock(mocks_mutex_);
        std::vector<rclcpp::TopicEndpointInfo> out;
        for (const auto & [gid, endpoint] : endpoints_) {
          if (endpoint.info.endpoint_type() == rclcpp::EndpointType::Subscription && endpoint.topic_name == topic_name)
          {
            out.emplace_back(endpoint.info);
          }
        }
        return out;
      });

    parameter_service_ = std::make_shared<MockParameterService>([this](const std::string & node_name) {
      std::lock_guard<std::mutex> lock(mocks_mutex_);
      std::vector<std::string> names;
      for (const auto & node : mocked_nodes_) {
        if (node.name == node_name) {
          names = node.params;
        }
      }
      return names;
    });

    reset_monitor(rosgraph_monitor::GraphMonitorConfiguration{}, parameter_service_);
  }

  /// @brief Replace the monitor with one built from `config` and `parameter_client`.
  /// @details Queue options are read once at construction, so changing them means a new monitor.
  /// Any graph messages from the previous monitor are dropped.
  void reset_monitor(
    rosgraph_monitor::GraphMonitorConfiguration config,
    std::shared_ptr<rosgraph_monitor::ParameterServiceClient> parameter_client)
  {
    graphmon_.reset();
    drain_graphmon_msgs();
    graphmon_.emplace(
      node_graph_,
      [this]() { return now_; },
      logger_.get_child("graphmon"),
      std::move(parameter_client),
      config,
      [this](rosgraph_msgs::msg::Graph & msg) {
        std::lock_guard<std::mutex> lock(graphmon_msg_mutex_);
        queue_.push_back(msg);
        graphmon_msg_cv_.notify_one();
      });
  }

  ~GraphMonitorTest() override
  {
    // The monitor runs threads that read the mocked graph and push into queue_, and it joins
    // them when it is destroyed. It is declared before those members, so default destruction
    // order would tear them down while those threads were still using them.
    graphmon_.reset();
  }

  void trigger_and_wait()
  {
    node_graph_->notify_graph_change();
    ASSERT_TRUE(graphmon_->wait_for_update(std::chrono::milliseconds(10)));
  }

  /// @return The next graph message, or nullopt if none arrives within `timeout`.
  std::optional<rosgraph_msgs::msg::Graph> await_graphmon_msg(
    std::chrono::milliseconds timeout = std::chrono::milliseconds(100))
  {
    std::unique_lock<std::mutex> lock(graphmon_msg_mutex_);
    if (!graphmon_msg_cv_.wait_for(lock, timeout, [this]() { return !queue_.empty(); })) {
      return std::nullopt;
    }
    rosgraph_msgs::msg::Graph msg = queue_.front();
    queue_.pop_front();
    return msg;
  }

  /// Discards every graph message received so far.
  void drain_graphmon_msgs()
  {
    std::lock_guard<std::mutex> lock(graphmon_msg_mutex_);
    queue_.clear();
  }

  template <typename Predicate>
  rosgraph_msgs::msg::Graph await_graphmon_msg_until(
    Predicate condition,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(1000),
    const std::string & timeout_message = "Timed out waiting for condition")
  {
    auto start_time = std::chrono::steady_clock::now();
    rosgraph_msgs::msg::Graph last_msg;

    while (true) {
      auto msg = await_graphmon_msg();
      if (msg) {
        last_msg = *msg;
        if (condition(last_msg)) {
          return last_msg;
        }
      }

      auto elapsed = std::chrono::steady_clock::now() - start_time;
      if (elapsed > timeout) {
        ADD_FAILURE() << timeout_message;
        return last_msg;
      }
    }
  }

  void set_node_names(std::vector<std::string> node_names)
  {
    std::vector<MockedNode> nodes;
    nodes.reserve(node_names.size());
    for (const auto & name : node_names) {
      nodes.emplace_back("/" + name);
    }
    set_nodes(nodes);
  }

  void set_nodes(std::vector<MockedNode> nodes)
  {
    {
      // Released before triggering, since the monitor reads this while we wait on it.
      std::lock_guard<std::mutex> lock(mocks_mutex_);
      mocked_nodes_.clear();
      mocked_nodes_.reserve(nodes.size());
      for (const auto & node : nodes) {
        mocked_nodes_.push_back(node);
      }
    }
    trigger_and_wait();
  }

  Endpoint add_endpoint(Endpoint endpoint)
  {
    {
      std::lock_guard<std::mutex> lock(mocks_mutex_);
      endpoints_.emplace(endpoint.info.endpoint_gid(), endpoint);
    }
    return endpoint;
  }

  Endpoint add_pub(
    const std::string & topic_name,
    const std::string & topic_type,
    const std::optional<std::string> & node_name = std::nullopt,
    const std::optional<rclcpp::QoS> & qos = std::nullopt)
  {
    return add_endpoint(Endpoint(
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
    return add_endpoint(Endpoint(
      rclcpp::EndpointType::Subscription,
      topic_name,
      topic_type,
      node_name.value_or(default_node_name_),
      qos.value_or(default_qos_)));
  }

  void remove_endpoint(const Endpoint & endpoint)
  {
    {
      std::lock_guard<std::mutex> lock(mocks_mutex_);
      endpoints_.erase(endpoint.info.endpoint_gid());
    }
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
  void check_status(
    const std::string & diagnostic_name,
    const uint8_t level,
    std::optional<std::string> maybe_message_pattern = std::nullopt)
  {
    diagnostic_msgs::msg::DiagnosticArray msg;
    graphmon_->evaluate(msg.status);
    auto it = std::find_if(msg.status.begin(), msg.status.end(), [&diagnostic_name](const auto & status) {
      return status.name == diagnostic_name;
    });
    ASSERT_NE(it, msg.status.end()) << "Expected diagnostic " << diagnostic_name << " not present";

    EXPECT_EQ(it->level, level);
    if (maybe_message_pattern) {
      std::regex message_re{*maybe_message_pattern};
      EXPECT_TRUE(std::regex_search(it->message, message_re))
        << "Message '" << it->message << "' does not match regex R'" << *maybe_message_pattern << "'";
    }
  }

  rclcpp::Time now_{0, 0, RCL_ROS_TIME};
  rclcpp::Logger logger_;
  std::shared_ptr<MockGraph> node_graph_;
  std::shared_ptr<MockParameterService> parameter_service_;
  std::optional<rosgraph_monitor::RosGraphMonitor> graphmon_;

  // Graph message handling
  std::mutex graphmon_msg_mutex_;
  std::condition_variable graphmon_msg_cv_;
  std::deque<rosgraph_msgs::msg::Graph> queue_;

  const std::string default_node_name_ = "testy0";
  const std::string default_topic_name_ = "/topic1";
  const rclcpp::QoS default_qos_{10};

  /// Guards the mocked graph below. The test thread rewrites it while the monitor's watch
  /// thread and its parameter-query threads are reading it through the mock.
  mutable std::mutex mocks_mutex_;
  std::vector<MockedNode> mocked_nodes_;
  std::unordered_map<RosRmwGid, Endpoint> endpoints_;
};

#define CHECK_STATUS(message, ...) \
  {                                \
    SCOPED_TRACE(message);         \
    check_status(__VA_ARGS__);     \
  }

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
  rclcpp::QoS cyclone_received_qos = rclcpp::QoS{10}.deadline(rclcpp::Duration::from_rmw_time(RMW_DURATION_INFINITE));
  add_pub("/topic1", "type1");
  add_sub("/topic1", "type1", default_node_name_, cyclone_received_qos);
  trigger_and_wait();
  now_ = rclcpp::Time(5, 0, RCL_ROS_TIME);
  CHECK_STATUS("Publisher with no deadline reports no frequency diagnostic", pub_freq_diagnostic, OK);
  CHECK_STATUS("Subscription to no deadline reports no frequency diagnostic", sub_freq_diagnostic, OK);
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
    make_stat(rosgraph_monitor_msgs::msg::TopicStatistic::PUBLISHED_PERIOD, std::chrono::milliseconds(9)));
  stats.statistics.push_back(
    make_stat(rosgraph_monitor_msgs::msg::TopicStatistic::RECEIVED_PERIOD, std::chrono::milliseconds(11)));
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
    make_stat(rosgraph_monitor_msgs::msg::TopicStatistic::PUBLISHED_PERIOD, std::chrono::milliseconds(12)));
  stats.statistics.push_back(
    make_stat(rosgraph_monitor_msgs::msg::TopicStatistic::RECEIVED_PERIOD, std::chrono::milliseconds(15)));
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
    make_stat(rosgraph_monitor_msgs::msg::TopicStatistic::PUBLISHED_PERIOD, std::chrono::milliseconds(8)));
  stats.statistics.push_back(
    make_stat(rosgraph_monitor_msgs::msg::TopicStatistic::RECEIVED_PERIOD, std::chrono::milliseconds(8)));
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
    make_stat(rosgraph_monitor_msgs::msg::TopicStatistic::PUBLISHED_PERIOD, std::chrono::milliseconds(10)));
  stats.statistics.push_back(
    make_stat(rosgraph_monitor_msgs::msg::TopicStatistic::RECEIVED_PERIOD, std::chrono::milliseconds(20)));
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
    make_stat(rosgraph_monitor_msgs::msg::TopicStatistic::PUBLISHED_PERIOD, std::chrono::milliseconds(7)));
  stats.statistics.push_back(
    make_stat(rosgraph_monitor_msgs::msg::TopicStatistic::RECEIVED_PERIOD, std::chrono::milliseconds(10)));
  graphmon_->on_topic_statistics(stats);

  CHECK_STATUS("Publisher sending faster than deadline is OK", pub_freq_diagnostic, OK);
  CHECK_STATUS("Subscription receiving slower than sent, but still within deadline", sub_freq_diagnostic, OK);
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
    make_stat(rosgraph_monitor_msgs::msg::TopicStatistic::PUBLISHED_PERIOD, std::chrono::milliseconds(10)));
  stats.statistics.push_back(
    make_stat(rosgraph_monitor_msgs::msg::TopicStatistic::RECEIVED_PERIOD, std::chrono::milliseconds(10)));
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

TEST_F(GraphMonitorTest, rosgraph_generation)
{
  // Set up test nodes
  set_node_names({"node1", "node2", "node3"});

  // The monitor takes an initial look at the graph when it is constructed and reports that,
  // so skip past it rather than asserting on whichever message happens to be first.
  rosgraph_msgs::msg::Graph rosgraph_msg = await_graphmon_msg_until(
    [](const rosgraph_msgs::msg::Graph & msg) { return msg.nodes.size() == 3; },
    std::chrono::milliseconds(500),
    "Timed out waiting for the graph to report all three nodes");

  // Verify the message contains expected nodes
  EXPECT_EQ(rosgraph_msg.nodes.size(), 3);

  // Verify node names are present
  std::vector<std::string> node_names;
  for (const auto & node : rosgraph_msg.nodes) {
    node_names.push_back(node.name);
  }

  EXPECT_THAT(node_names, testing::UnorderedElementsAre("/node1", "/node2", "/node3"));
}

TEST_F(GraphMonitorTest, rosgraph_ignores_ignored_nodes)
{
  // Set up some nodes, including one that should be ignored
  graphmon_->config().nodes.ignore_prefixes = {"/dummy"};
  set_node_names({"node1", "node2", "dummy/ignored_node"});

  rosgraph_msgs::msg::Graph rosgraph_msg = await_graphmon_msg_until(
    [](const rosgraph_msgs::msg::Graph & msg) {
      return msg.nodes.size() == 2;  // We expect only 2 nodes after ignoring
    },
    std::chrono::milliseconds(500),
    "Timed out waiting for ignored nodes to be filtered");

  // Verify the message contains only non-ignored nodes
  EXPECT_EQ(rosgraph_msg.nodes.size(), 2);

  // Verify node names are present (should not include ignored node)
  std::vector<std::string> node_names;
  for (const auto & node : rosgraph_msg.nodes) {
    node_names.push_back(node.name);
  }

  EXPECT_THAT(node_names, testing::UnorderedElementsAre("/node1", "/node2"));
}

TEST_F(GraphMonitorTest, rosgraph_query_params_from_one_node)
{
  // Set up some nodes, including one that should be warn-only
  std::vector<MockedNode> mocked_nodes{
    MockedNode("/node1", {"param1", "param2"}),
  };

  set_nodes(mocked_nodes);
  node_graph_->notify_graph_change();

  // Wait for the graph message with parameters populated
  // The parameter query is async, so we need to wait for it to complete
  auto rosgraph_msg = await_graphmon_msg_until(
    [](const rosgraph_msgs::msg::Graph & msg) {
      return msg.nodes.size() == 1 && msg.nodes.front().parameters.size() == 2 &&
             all_typed(msg.nodes.front().parameters, rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) &&
             msg.nodes.front().parameter_values.size() == 2;
    },
    std::chrono::milliseconds(500),
    "Timed out waiting for parameters to be populated");

  // Verify the message contains the expected node and parameters
  EXPECT_EQ(rosgraph_msg.nodes.size(), 1);
  auto node = rosgraph_msg.nodes.front();
  EXPECT_EQ(node.name, "/node1");
  EXPECT_EQ(node.parameters.size(), 2);

  std::vector<std::string> param_names{};
  for (const auto & param : node.parameters) {
    param_names.push_back(param.name);
  }
  EXPECT_THAT(param_names, testing::UnorderedElementsAre("param1", "param2"));

  // Describing gives the types the mock reports, and reading gives one value per named parameter.
  for (const auto & descriptor : node.parameters) {
    EXPECT_EQ(descriptor.type, rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER);
  }
  ASSERT_EQ(node.parameter_values.size(), node.parameters.size());
  EXPECT_THAT(integer_values(node.parameter_values), testing::ElementsAre(0, 1))
    << "The values are not lined up with the parameters they belong to";
}

TEST_F(GraphMonitorTest, rosgraph_drops_parameter_query_for_a_departed_node)
{
  set_nodes({MockedNode("/leaving", {"param1"})});
  node_graph_->notify_graph_change();
  await_graphmon_msg_until(
    [](const rosgraph_msgs::msg::Graph & msg) {
      return msg.nodes.size() == 1 && !msg.nodes.front().parameters.empty();
    },
    std::chrono::milliseconds(500),
    "Timed out waiting for the node's parameters");

  // The node leaves; the monitor should stop reporting it rather than keep its parameters.
  set_nodes({});
  node_graph_->notify_graph_change();

  auto rosgraph_msg = await_graphmon_msg_until(
    [](const rosgraph_msgs::msg::Graph & msg) { return msg.nodes.empty(); },
    std::chrono::milliseconds(500),
    "Timed out waiting for the departed node to be dropped");
  EXPECT_TRUE(rosgraph_msg.nodes.empty());
}

TEST_F(GraphMonitorTest, rosgraph_reports_a_node_with_no_parameters)
{
  set_nodes({MockedNode("/bare", {})});
  node_graph_->notify_graph_change();

  auto rosgraph_msg = await_graphmon_msg_until(
    [](const rosgraph_msgs::msg::Graph & msg) { return msg.nodes.size() == 1; },
    std::chrono::milliseconds(500),
    "Timed out waiting for the node");

  ASSERT_EQ(rosgraph_msg.nodes.size(), 1u);
  EXPECT_TRUE(rosgraph_msg.nodes.front().parameters.empty());
  EXPECT_TRUE(rosgraph_msg.nodes.front().parameter_values.empty());
}

TEST_F(GraphMonitorTest, rosgraph_still_reports_a_node_whose_parameters_cannot_be_read)
{
  // An unreachable parameter service must not stop the node appearing in the graph at all.
  parameter_service_->unreachable.insert("/silent");
  set_nodes({MockedNode("/silent", {"param1"})});
  node_graph_->notify_graph_change();

  auto rosgraph_msg = await_graphmon_msg_until(
    [](const rosgraph_msgs::msg::Graph & msg) { return msg.nodes.size() == 1; },
    std::chrono::milliseconds(500),
    "Timed out waiting for the node");

  ASSERT_EQ(rosgraph_msg.nodes.size(), 1u);
  EXPECT_EQ(rosgraph_msg.nodes.front().name, "/silent");
  EXPECT_EQ(rosgraph_msg.nodes.front().parameters.size(), 0u);
}

TEST_F(GraphMonitorTest, rosgraph_observes_a_node_whose_parameter_services_come_up_late)
{
  // A node can be in the graph before its parameter services are discoverable. Nothing else
  // about the node changes when they appear, so the monitor has to keep asking.
  reset_monitor(patient_config(), parameter_service_);
  parameter_service_->unreachable.insert("/late");
  set_nodes({MockedNode("/late", {"param1"})});
  node_graph_->notify_graph_change();
  await_graphmon_msg_until(
    [](const rosgraph_msgs::msg::Graph & msg) { return msg.nodes.size() == 1; },
    std::chrono::milliseconds(500),
    "Timed out waiting for the node");

  parameter_service_->set_reachable("/late");
  node_graph_->notify_graph_change();

  auto rosgraph_msg = await_graphmon_msg_until(
    [](const rosgraph_msgs::msg::Graph & msg) {
      return msg.nodes.size() == 1 && !msg.nodes.front().parameters.empty();
    },
    std::chrono::milliseconds(500),
    "Timed out waiting for the parameters of a node that became reachable");

  ASSERT_EQ(rosgraph_msg.nodes.front().parameters.size(), 1u);
  EXPECT_EQ(rosgraph_msg.nodes.front().parameters.front().name, "param1");
}

TEST_F(GraphMonitorTest, parameter_queries_are_bounded)
{
  auto blocking = std::make_shared<BlockingParameterService>();
  reset_monitor(patient_config(2), blocking);

  set_node_names({"n1", "n2", "n3", "n4"});
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->started_count() == 2; }))
    << "Expected two queries to start, saw " << blocking->started_count();
  EXPECT_FALSE(wait_for_condition([&blocking]() { return blocking->started_count() > 2; }, kSettle))
    << "A third query started before a slot was free";

  const auto in_flight = blocking->outstanding();
  ASSERT_EQ(in_flight.size(), 2u);
  blocking->answer(*in_flight.begin(), {"param1"});

  EXPECT_TRUE(wait_for_condition([&blocking]() { return blocking->started_count() == 3; }))
    << "Answering one query did not free a slot for the next";
}

TEST_F(GraphMonitorTest, repeated_graph_updates_do_not_duplicate_queries)
{
  auto blocking = std::make_shared<BlockingParameterService>();
  reset_monitor(patient_config(), blocking);

  set_node_names({"repeat"});
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->start_count("/repeat") == 1; }));

  for (int update = 0; update < 5; update++) {
    trigger_and_wait();
  }

  EXPECT_FALSE(wait_for_condition([&blocking]() { return blocking->start_count("/repeat") > 1; }, kSettle))
    << "A query in flight was started again by a later graph update";
}

TEST_F(GraphMonitorTest, a_failed_query_is_retried_until_it_succeeds)
{
  auto blocking = std::make_shared<BlockingParameterService>();
  reset_monitor(patient_config(), blocking);

  set_node_names({"flaky"});
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->start_count("/flaky") == 1; }));

  blocking->fail("/flaky");
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->start_count("/flaky") == 2; }))
    << "A failed query was not retried";

  // Nothing touches the graph from here on but the answer itself.
  drain_graphmon_msgs();
  blocking->answer("/flaky", {"param1"});

  auto msg = await_graphmon_msg(kGenerousWait);
  ASSERT_TRUE(msg.has_value()) << "The retry's result did not publish the graph";
  ASSERT_EQ(msg->nodes.size(), 1u);
  ASSERT_EQ(msg->nodes.front().parameters.size(), 1u);
  EXPECT_EQ(msg->nodes.front().parameters.front().name, "param1");
}

TEST_F(GraphMonitorTest, a_departed_nodes_query_is_cancelled)
{
  auto blocking = std::make_shared<BlockingParameterService>();
  reset_monitor(patient_config(), blocking);

  set_node_names({"leaving"});
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->start_count("/leaving") == 1; }));

  set_node_names({});
  EXPECT_TRUE(wait_for_condition([&blocking]() { return blocking->forgotten("/leaving"); }))
    << "A departed node's client state was not dropped";

  drain_graphmon_msgs();
  blocking->answer("/leaving", {"param1"});
  EXPECT_FALSE(await_graphmon_msg(kSettle).has_value()) << "A cancelled query's response was recorded";
}

TEST_F(GraphMonitorTest, a_response_publishes_the_graph_by_itself)
{
  auto blocking = std::make_shared<BlockingParameterService>();
  reset_monitor(patient_config(), blocking);

  set_node_names({"quiet"});
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->start_count("/quiet") == 1; }));

  // No further graph event follows, so only the response can publish these parameters.
  drain_graphmon_msgs();
  blocking->answer("/quiet", {"param1", "param2"});

  auto msg = await_graphmon_msg(kGenerousWait);
  ASSERT_TRUE(msg.has_value()) << "A parameter observation did not publish the graph on its own";
  ASSERT_EQ(msg->nodes.size(), 1u);
  std::vector<std::string> param_names;
  for (const auto & descriptor : msg->nodes.front().parameters) {
    param_names.push_back(descriptor.name);
  }
  EXPECT_THAT(param_names, testing::UnorderedElementsAre("param1", "param2"));
}

TEST_F(GraphMonitorTest, destroying_the_monitor_with_an_unanswered_query_does_not_hang)
{
  auto blocking = std::make_shared<BlockingParameterService>();
  reset_monitor(patient_config(), blocking);

  set_node_names({"stuck"});
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->start_count("/stuck") == 1; }));

  auto destroyed = std::async(std::launch::async, [this]() { graphmon_.reset(); });
  ASSERT_EQ(destroyed.wait_for(kGenerousWait), std::future_status::ready) << "Destruction blocked on a query in flight";
  destroyed.get();
}

TEST_F(GraphMonitorTest, an_answer_after_destruction_is_safe)
{
  auto blocking = std::make_shared<BlockingParameterService>();
  reset_monitor(patient_config(), blocking);

  set_node_names({"stuck"});
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->start_count("/stuck") == 1; }));

  graphmon_.reset();
  // The done callback owns everything it touches.
  blocking->answer("/stuck", {"param1"});
}

TEST_F(GraphMonitorTest, names_receipt_requests_descriptors)
{
  auto blocking = std::make_shared<BlockingParameterService>();
  reset_monitor(patient_config(), blocking);

  set_node_names({"typed"});
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->start_count("/typed") == 1; }));

  blocking->answer("/typed", {"param1", "param2"});
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->describe_start_count("/typed") == 1; }))
    << "Recording parameter names did not start a descriptor query";
  EXPECT_THAT(blocking->described_names("/typed"), testing::ElementsAre("param1", "param2"));
}

TEST_F(GraphMonitorTest, descriptor_response_updates_types_and_publishes)
{
  auto blocking = std::make_shared<BlockingParameterService>();
  reset_monitor(patient_config(), blocking);

  set_node_names({"typed"});
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->start_count("/typed") == 1; }));
  blocking->answer("/typed", {"param1"});
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->describe_start_count("/typed") == 1; }));

  // No further graph event follows, so only the describe response can publish these types.
  blocking->answer_describe(
    "/typed", descriptors_named({"param1"}, rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE));

  auto msg = await_graphmon_msg_until(
    [](const rosgraph_msgs::msg::Graph & msg) {
      return msg.nodes.size() == 1 &&
             all_typed(msg.nodes.front().parameters, rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);
    },
    kGenerousWait,
    "Timed out waiting for the described types to be published");
  EXPECT_THAT(descriptor_names(msg.nodes.front().parameters), testing::ElementsAre("param1"));

  EXPECT_FALSE(await_graphmon_msg(kSettle).has_value()) << "The graph was published again after the types arrived";
}

TEST(ReconcileDescriptors, a_relist_preserves_known_types)
{
  using rcl_interfaces::msg::ParameterType;
  // A re-list of a fully observed node is not reachable through the monitor's public behavior:
  // names are queried once per successful observation, and a node that returns from missing
  // drops its observation. The reconcile is tested at its own interface instead.
  const auto recorded = descriptors_named({"param1", "param2"}, ParameterType::PARAMETER_STRING);

  const auto reconciled = rosgraph_monitor::reconcile_descriptors(recorded, {"param2", "param3"});

  ASSERT_EQ(reconciled.size(), 2u);
  EXPECT_EQ(reconciled[0].name, "param2");
  EXPECT_EQ(reconciled[0].type, ParameterType::PARAMETER_STRING) << "A re-listed name lost its known type";
  EXPECT_EQ(reconciled[1].name, "param3");
  EXPECT_EQ(reconciled[1].type, ParameterType::PARAMETER_NOT_SET);
}

TEST_F(GraphMonitorTest, a_failed_describe_leaves_names_visible_and_retries)
{
  auto blocking = std::make_shared<BlockingParameterService>();
  reset_monitor(patient_config(), blocking);

  set_node_names({"flaky_describe"});
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->start_count("/flaky_describe") == 1; }));
  blocking->answer("/flaky_describe", {"param1"});
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->describe_start_count("/flaky_describe") == 1; }));

  auto names_msg = await_graphmon_msg_until(
    [](const rosgraph_msgs::msg::Graph & msg) {
      return msg.nodes.size() == 1 &&
             all_typed(msg.nodes.front().parameters, rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET);
    },
    kGenerousWait,
    "Timed out waiting for the names of a node whose descriptors are outstanding");
  EXPECT_THAT(descriptor_names(names_msg.nodes.front().parameters), testing::ElementsAre("param1"));

  blocking->fail_describe("/flaky_describe");
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->describe_start_count("/flaky_describe") == 2; }))
    << "A failed describe was not retried";

  blocking->answer_describe(
    "/flaky_describe", descriptors_named({"param1"}, rcl_interfaces::msg::ParameterType::PARAMETER_STRING));
  auto typed_msg = await_graphmon_msg_until(
    [](const rosgraph_msgs::msg::Graph & msg) {
      return msg.nodes.size() == 1 &&
             all_typed(msg.nodes.front().parameters, rcl_interfaces::msg::ParameterType::PARAMETER_STRING);
    },
    kGenerousWait,
    "Timed out waiting for the types from the retried describe");
  EXPECT_THAT(descriptor_names(typed_msg.nodes.front().parameters), testing::ElementsAre("param1"));
}

TEST_F(GraphMonitorTest, departure_cancels_descriptor_queries)
{
  auto blocking = std::make_shared<BlockingParameterService>();
  reset_monitor(patient_config(), blocking);

  set_node_names({"leaving"});
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->start_count("/leaving") == 1; }));
  blocking->answer("/leaving", {"param1"});
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->describe_start_count("/leaving") == 1; }));

  set_node_names({});
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->forgotten("/leaving"); }))
    << "A departed node's client state was not dropped";

  drain_graphmon_msgs();
  blocking->answer_describe(
    "/leaving", descriptors_named({"param1"}, rcl_interfaces::msg::ParameterType::PARAMETER_STRING));
  EXPECT_FALSE(await_graphmon_msg(kSettle).has_value()) << "A cancelled describe's response was recorded";
}

TEST_F(GraphMonitorTest, no_descriptor_query_for_a_node_with_no_parameters)
{
  auto blocking = std::make_shared<BlockingParameterService>();
  reset_monitor(patient_config(), blocking);

  set_node_names({"bare"});
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->start_count("/bare") == 1; }));

  blocking->answer("/bare", {});
  EXPECT_FALSE(wait_for_condition([&blocking]() { return blocking->describe_start_count("/bare") > 0; }, kSettle))
    << "A node with no parameters had its descriptors queried";
}

TEST_F(GraphMonitorTest, names_receipt_requests_values_and_descriptors_together)
{
  auto blocking = std::make_shared<BlockingParameterService>();
  reset_monitor(patient_config(), blocking);

  set_node_names({"valued"});
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->start_count("/valued") == 1; }));

  // Neither query is answered, so both are outstanding at once.
  blocking->answer("/valued", {"param1", "param2"});
  ASSERT_TRUE(wait_for_condition([&blocking]() {
    return blocking->describe_start_count("/valued") == 1 && blocking->value_start_count("/valued") == 1;
  }))
    << "Recording parameter names did not start both a descriptor and a value query";
  EXPECT_THAT(blocking->valued_names("/valued"), testing::ElementsAre("param1", "param2"));
}

TEST_F(GraphMonitorTest, value_response_publishes_values_parallel_to_descriptors)
{
  auto blocking = std::make_shared<BlockingParameterService>();
  reset_monitor(patient_config(), blocking);

  set_node_names({"valued"});
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->start_count("/valued") == 1; }));
  blocking->answer("/valued", {"param1", "param2"});
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->value_start_count("/valued") == 1; }));

  // The describe is left outstanding, so the descriptors are still the names alone.
  blocking->answer_values("/valued", indexed_values(blocking->valued_names("/valued")));

  auto msg = await_graphmon_msg_until(
    [](const rosgraph_msgs::msg::Graph & msg) {
      return msg.nodes.size() == 1 && !msg.nodes.front().parameter_values.empty();
    },
    kGenerousWait,
    "Timed out waiting for the read values to be published");
  const auto & node = msg.nodes.front();
  EXPECT_THAT(descriptor_names(node.parameters), testing::ElementsAre("param1", "param2"));
  ASSERT_EQ(node.parameter_values.size(), node.parameters.size());
  EXPECT_THAT(integer_values(node.parameter_values), testing::ElementsAre(0, 1));
}

TEST_F(GraphMonitorTest, a_failed_value_query_is_retried)
{
  auto blocking = std::make_shared<BlockingParameterService>();
  reset_monitor(patient_config(), blocking);

  set_node_names({"flaky_values"});
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->start_count("/flaky_values") == 1; }));
  blocking->answer("/flaky_values", {"param1"});
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->value_start_count("/flaky_values") == 1; }));

  blocking->fail_values("/flaky_values");
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->value_start_count("/flaky_values") == 2; }))
    << "A failed value query was not retried";

  blocking->answer_values("/flaky_values", indexed_values({"param1"}));
  auto msg = await_graphmon_msg_until(
    [](const rosgraph_msgs::msg::Graph & msg) {
      return msg.nodes.size() == 1 && !msg.nodes.front().parameter_values.empty();
    },
    kGenerousWait,
    "Timed out waiting for the values from the retried query");
  EXPECT_THAT(integer_values(msg.nodes.front().parameter_values), testing::ElementsAre(0));
}

TEST_F(GraphMonitorTest, departure_cancels_value_queries)
{
  auto blocking = std::make_shared<BlockingParameterService>();
  reset_monitor(patient_config(), blocking);

  set_node_names({"leaving"});
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->start_count("/leaving") == 1; }));
  blocking->answer("/leaving", {"param1"});
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->value_start_count("/leaving") == 1; }));

  set_node_names({});
  ASSERT_TRUE(wait_for_condition([&blocking]() { return blocking->forgotten("/leaving"); }))
    << "A departed node's client state was not dropped";

  drain_graphmon_msgs();
  blocking->answer_values("/leaving", indexed_values({"param1"}));
  EXPECT_FALSE(await_graphmon_msg(kSettle).has_value()) << "A cancelled value query's response was recorded";
}

TEST(AlignValues, a_reordered_response_is_lined_up_by_name)
{
  using rcl_interfaces::msg::ParameterType;
  const auto descriptors = descriptors_named({"param1", "param2"}, ParameterType::PARAMETER_INTEGER);

  const auto aligned =
    rosgraph_monitor::align_values(descriptors, {{"param2", "param1"}, indexed_values({"param2", "param1"})});

  ASSERT_TRUE(aligned.has_value());
  EXPECT_THAT(integer_values(*aligned), testing::ElementsAre(1, 0));
}

TEST(AlignValues, values_for_a_stale_name_set_are_not_published)
{
  using rcl_interfaces::msg::ParameterType;
  // A name set that changes between the request and the response is not reachable through the
  // monitor's public behavior: names are queried once per successful observation, and departure,
  // the only thing that drops an observation, cancels the value query too.
  // The alignment is tested at its own interface instead.
  const auto descriptors = descriptors_named({"param1", "param2"}, ParameterType::PARAMETER_INTEGER);

  const auto aligned = rosgraph_monitor::align_values(descriptors, {{"param1"}, indexed_values({"param1"})});

  EXPECT_FALSE(aligned.has_value()) << "A recorded name with no value still produced a values array";
}
