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

#ifndef ROSGRAPH_MONITOR__GRAPH_ANALYZER_HPP_
#define ROSGRAPH_MONITOR__GRAPH_ANALYZER_HPP_

#include <map>
#include <memory>
#include <regex>
#include <string>
#include <unordered_set>
#include <vector>

#include "diagnostic_aggregator/analyzer.hpp"
#include "rclcpp/logger.hpp"

#include "rosgraph_monitor/rosgraph_monitor_generated_parameters.hpp"

namespace rosgraph_monitor
{

class GraphAnalyzer : public diagnostic_aggregator::Analyzer
{
public:
  GraphAnalyzer();
  virtual ~GraphAnalyzer();

  /// @brief Implemented for virtual interface compliance, but will throw exception on call.
  /// The interface here is bad for current usage pattern because a Node cannot call
  /// shared_from_this() in its constructor, and may not even be a shared_ptr in the first place
  bool init(
    const std::string & base_path,
    const std::string & breadcrumb,
    const rclcpp::Node::SharedPtr node) override;

  /// @brief Actual initialize function, call me instead
  bool init(
    const std::string & base_path,
    const rosgraph_monitor::Params::GraphAnalyzer & params);

  /// @brief Returns true if analyzer will handle this item
  bool match(const std::string & status_name) override;

  /// @brief Returns true if item was analyzed and will be part of report
  bool analyze(const std::shared_ptr<diagnostic_aggregator::StatusItem> item) override;

  /// @brief Run full analysis, returns processed results from analyze() items
  std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> report() override;

  /// @brief Return full prefix of analyzer outputs
  std::string getPath() const override;

  /// @brief Returns nice name for display purposes
  std::string getName() const override;

protected:
  // Configuration
  const std::string name_{"RosGraph"};
  std::unordered_set<std::string> ignore_deadline_topics_;
  bool all_nodes_required_ = false;

  // Internal derived configuration
  std::string path_;
  std::string graph_diagnostic_prefix_;
  std::string pub_freq_diagnostic_prefix_;
  std::string node_diagnostic_prefix_;
  std::string name_match_regex_text_;
  std::regex name_match_regex_;
  std::chrono::milliseconds stale_timeout_{1000};

  // Internal state
  rclcpp::Logger logger_;
  std::map<std::string, std::shared_ptr<diagnostic_aggregator::StatusItem>> items_;
};

}  // namespace rosgraph_monitor

#endif  // ROSGRAPH_MONITOR__GRAPH_ANALYZER_HPP_
