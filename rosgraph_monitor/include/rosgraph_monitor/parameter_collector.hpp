// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"

namespace rosgraph_monitor
{

/// @brief Pure interface providing parameter service calls to discover parameter information from nodes.
/// @details Abstracted for polymorphic dependency injection to test without full RMW.
class ParameterServiceClient
{
public:
  virtual ~ParameterServiceClient() = default;

  using NamesCallback = std::function<void(std::optional<std::vector<std::string>>)>;
  using DescriptorsCallback = std::function<void(std::optional<std::vector<rcl_interfaces::msg::ParameterDescriptor>>)>;
  using ValuesCallback = std::function<void(std::optional<std::vector<rcl_interfaces::msg::ParameterValue>>)>;

  /// @return Whether the node is currently reachable.
  /// Check before starting, so an unreachable node does not occupy a concurrency slot waiting for a response.
  virtual bool is_ready(const std::string & node_name) = 0;

  /// @brief Get all parameter names from the node, calling callback when complete.
  /// @return Immediately. The callback is always called eventually, with a std::nullopt on failure.
  virtual void list_parameters(const std::string & node_name, NamesCallback callback) = 0;

  /// @brief Get descriptors for the named parameters of the node, calling callback when complete.
  /// @return Immediately. The callback is always called eventually, with a std::nullopt on failure.
  virtual void describe_parameters(
    const std::string & node_name, const std::vector<std::string> & names, DescriptorsCallback callback) = 0;

  /// @brief Get the values of the named parameters of the node, calling callback when complete.
  /// @return Immediately. The callback is always called eventually, with a std::nullopt on failure.
  /// The values are positional against `names`.
  virtual void get_parameters(
    const std::string & node_name, const std::vector<std::string> & names, ValuesCallback callback) = 0;

  /// @brief Drop any client state held for a node that has gone away.
  virtual void forget(const std::string & node_name) = 0;
};

}  // namespace rosgraph_monitor
