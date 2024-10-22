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

#include "rosgraph_monitor/graph_analyzer.hpp"

namespace
{
bool startswith(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0) != std::string::npos;
}

}  // namespace

namespace rosgraph_monitor
{

GraphAnalyzer::GraphAnalyzer()
: logger_(rclcpp::get_logger("graph_analyzer"))
{}

GraphAnalyzer::~GraphAnalyzer()
{}

bool GraphAnalyzer::init(
  const std::string & /* base_path */,
  const std::string & /* breadcrumb */,
  const rclcpp::Node::SharedPtr /* node */)
{
  throw std::runtime_error("Intentionally unimplemented.");
}

bool GraphAnalyzer::init(
  const std::string & base_path,
  const rosgraph_monitor::Params::GraphAnalyzer & params)
{
  name_match_regex_text_ = "^" + graph_diagnostic_prefix_ + "(Node|PublishFrequency)::.+";
  name_match_regex_ = name_match_regex_text_;
  pub_freq_diagnostic_prefix_ = graph_diagnostic_prefix_ + "PublishFrequency::";
  node_diagnostic_prefix_ = graph_diagnostic_prefix_ + "Node::";

  path_ = base_path + "/" + name_;
  RCLCPP_INFO(logger_, "BASE BAPTH IS %s", path_.c_str());
  ignore_deadline_topics_ = std::unordered_set<std::string>(
    params.ignore_deadline_topics.begin(),
    params.ignore_deadline_topics.end());
  all_nodes_required_ = params.all_nodes_required;
  items_.clear();

  // For configured expected topics, put a missing status in up front
  for (const auto & topic : params.mandatory_frequency_topics) {
    std::string name = pub_freq_diagnostic_prefix_ + topic;
    items_[name] = std::make_shared<diagnostic_aggregator::StatusItem>(
      name, "Not observed yet", diagnostic_aggregator::Level_Error);
  }
  return true;
}


bool GraphAnalyzer::match(const std::string & status_name)
{
  bool is_match = std::regex_match(status_name.c_str(), name_match_regex_);
  return is_match;
}

bool GraphAnalyzer::analyze(const std::shared_ptr<diagnostic_aggregator::StatusItem> item)
{
  items_[item->getName()] = item;
  return true;
}

std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> GraphAnalyzer::report()
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  using diagnostic_msgs::msg::KeyValue;

  auto header_status = std::make_shared<DiagnosticStatus>();
  header_status->level = DiagnosticStatus::OK;
  header_status->name = path_;
  header_status->message = "OK";

  auto node_status = std::make_shared<DiagnosticStatus>();
  node_status->level = DiagnosticStatus::OK;
  node_status->name = path_ + "/Nodes";
  node_status->message = "OK";

  auto frequency_status = std::make_shared<DiagnosticStatus>();
  frequency_status->level = DiagnosticStatus::OK;
  frequency_status->name = path_ + "/PublishFrequency";
  frequency_status->message = "OK";

  for (const auto & [name, item] : items_) {
    const std::string item_yaml =
      diagnostic_msgs::msg::to_yaml(*item->toStatusMsg("Health/Graph", false));

    auto age = clock_->now() - item->getLastUpdateTime();
    bool stale = age.to_chrono<std::chrono::milliseconds>() > stale_timeout_;

    const int8_t level = item->getLevel();
    if (all_nodes_required_ && startswith(name, node_diagnostic_prefix_)) {
      if (stale) {
        node_status->level = DiagnosticStatus::ERROR;
        node_status->message = "Node status not reported";
      }
      node_status->level = std::max(static_cast<int8_t>(node_status->level), level);
      if (item->getLevel() != DiagnosticStatus::OK) {
        std::string node_name = name.substr(name.rfind("::") + 2);
        KeyValue kv;
        kv.key = node_name;
        kv.value = item->getMessage();
        node_status->values.push_back(kv);
      }
    } else if (startswith(name, pub_freq_diagnostic_prefix_)) {
      std::string topic_name = name.substr(name.rfind("::") + 2);
      if (ignore_deadline_topics_.count(topic_name) != 0) {
        continue;
      }
      if (stale) {
        frequency_status->level = DiagnosticStatus::ERROR;
        frequency_status->message = "Topic statistics not reported";
      }
      frequency_status->level = std::max(static_cast<int8_t>(frequency_status->level), level);
      if (item->getLevel() != DiagnosticStatus::OK) {
        KeyValue kv;
        kv.key = topic_name;
        kv.value = item->getMessage();
        frequency_status->values.push_back(kv);
      }
    }
  }
  if (node_status->level != DiagnosticStatus::OK) {
    node_status->message = "Required nodes missing";
  }
  if (frequency_status->level == DiagnosticStatus::WARN) {
    frequency_status->message = "Required publish frequencies not being met";
  } else if (frequency_status->level == DiagnosticStatus::ERROR) {
    frequency_status->message = "Required publishers with frequencies not publishing";
  }

  std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> report_statuses = {
    header_status, node_status, frequency_status
  };
  for (const auto & status : report_statuses) {
    header_status->level =
      std::max(static_cast<int8_t>(header_status->level), static_cast<int8_t>(status->level));
    if (status->level != DiagnosticStatus::OK) {
      KeyValue kv;
      kv.key = status->name;
      kv.value = status->message;
      header_status->values.push_back(kv);
    }
  }
  header_status->message = diagnostic_aggregator::valToMsg(header_status->level);
  return report_statuses;
}

std::string GraphAnalyzer::getPath() const
{
  return path_;
}

/// @brief Returns nice name for display purposes
std::string GraphAnalyzer::getName() const
{
  return name_;
}

}  // namespace rosgraph_monitor
