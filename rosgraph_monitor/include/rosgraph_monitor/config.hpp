// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <chrono>
#include <string>
#include <unordered_set>
#include <vector>

namespace rosgraph_monitor
{

struct GraphMonitorConfiguration
{
  std::string diagnostic_namespace{"rosgraph"};

  struct NodeChecks
  {
    // Matching nodes will not be considered in any graph analysis
    std::vector<std::string> ignore_prefixes;
    // Downgrade ERROR to WARN for matching nodes when they are missing.
    std::vector<std::string> warn_only_prefixes;
  } nodes;

  struct ContinuityChecks
  {
    // If set, don't perform any continuity checks
    bool enable = true;
    // These nodes don't count for subscriptions when reporting discontinuity
    std::unordered_set<std::string> ignore_subscriber_nodes;
    // Any topics of these types will be ignored entirely for continuity checks
    std::unordered_set<std::string> ignore_topic_types;
    // Any topics with these names will be ignored entirely for continuity checks
    std::unordered_set<std::string> ignore_topic_names;
  } continuity;

  // Not dynamically changeable at runtime.
  struct ParameterObservation
  {
    // How many nodes may be observed at once
    size_t max_concurrent = 4;
    // How long one node's observation may take before it is abandoned
    std::chrono::milliseconds timeout{10000};
  } parameters;

  struct TopicStatisticsChecks
  {
    // What fraction of the promised deadline the topic statistics may err by
    // and still be considered compliant.
    // For example if 0.1, then a deadline of 10 milliseconds will be considered OK
    // if average measured interval is 9-11 milliseconds
    // This equates to: expectation of 100Hz will be considered OK from 90.9-111.1Hz
    float deadline_allowed_error = 0.1;
    // For topics whose frequency is tracked, if new statistics are not received within this
    // time frame then the statistic will be reported as stale with an ERROR.
    std::chrono::milliseconds stale_timeout{3000};
    // List of topics that must exist and have deadlines
    std::unordered_set<std::string> mandatory_topics;
    // List of topics that should not be considered for frequency checks
    // (e.g. topics that are known to be misconfigured and not meeting their deadlines
    std::unordered_set<std::string> ignore_topics;
  } topic_statistics;
};

}  // namespace rosgraph_monitor
