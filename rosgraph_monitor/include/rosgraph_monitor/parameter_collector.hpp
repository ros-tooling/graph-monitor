
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <vector>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"

namespace rosgraph_monitor
{

struct NodeParameters
{
  std::vector<rcl_interfaces::msg::ParameterDescriptor> descriptors;
  std::vector<rcl_interfaces::msg::ParameterValue> values;

  bool empty() const
  {
    return descriptors.empty();
  }
};

}  // namespace rosgraph_monitor
