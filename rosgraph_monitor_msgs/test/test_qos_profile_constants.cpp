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

#include <gtest/gtest.h>
#include <rclcpp/qos.hpp>
#include "rosgraph_monitor_msgs/msg/qos_profile.hpp"

/**
 * Tests to ensure that the rosgraph_monitor_msgs::msg::QosProfile constants
 * stay in sync with the target ROS distribution.
 */
class QosProfileConstantsTest : public ::testing::Test
{
};

TEST_F(QosProfileConstantsTest, HistoryPolicyConstantsMatchRclcpp)
{
  using QosMsg = rosgraph_monitor_msgs::msg::QosProfile;

  EXPECT_EQ(QosMsg::HISTORY_SYSTEM_DEFAULT, static_cast<uint8_t>(rclcpp::HistoryPolicy::SystemDefault));
  EXPECT_EQ(QosMsg::HISTORY_KEEP_LAST, static_cast<uint8_t>(rclcpp::HistoryPolicy::KeepLast));
  EXPECT_EQ(QosMsg::HISTORY_KEEP_ALL, static_cast<uint8_t>(rclcpp::HistoryPolicy::KeepAll));
  EXPECT_EQ(QosMsg::HISTORY_UNKNOWN, static_cast<uint8_t>(rclcpp::HistoryPolicy::Unknown));
}

TEST_F(QosProfileConstantsTest, ReliabilityPolicyConstantsMatchRclcpp)
{
  using QosMsg = rosgraph_monitor_msgs::msg::QosProfile;

  EXPECT_EQ(QosMsg::RELIABILITY_SYSTEM_DEFAULT, static_cast<uint8_t>(rclcpp::ReliabilityPolicy::SystemDefault));
  EXPECT_EQ(QosMsg::RELIABILITY_RELIABLE, static_cast<uint8_t>(rclcpp::ReliabilityPolicy::Reliable));
  EXPECT_EQ(QosMsg::RELIABILITY_BEST_EFFORT, static_cast<uint8_t>(rclcpp::ReliabilityPolicy::BestEffort));
  EXPECT_EQ(QosMsg::RELIABILITY_UNKNOWN, static_cast<uint8_t>(rclcpp::ReliabilityPolicy::Unknown));
}

TEST_F(QosProfileConstantsTest, DurabilityPolicyConstantsMatchRclcpp)
{
  using QosMsg = rosgraph_monitor_msgs::msg::QosProfile;

  EXPECT_EQ(QosMsg::DURABILITY_SYSTEM_DEFAULT, static_cast<uint8_t>(rclcpp::DurabilityPolicy::SystemDefault));
  EXPECT_EQ(QosMsg::DURABILITY_TRANSIENT_LOCAL, static_cast<uint8_t>(rclcpp::DurabilityPolicy::TransientLocal));
  EXPECT_EQ(QosMsg::DURABILITY_VOLATILE, static_cast<uint8_t>(rclcpp::DurabilityPolicy::Volatile));
  EXPECT_EQ(QosMsg::DURABILITY_UNKNOWN, static_cast<uint8_t>(rclcpp::DurabilityPolicy::Unknown));
}

TEST_F(QosProfileConstantsTest, LivelinessPolicyConstantsMatchRclcpp)
{
  using QosMsg = rosgraph_monitor_msgs::msg::QosProfile;

  EXPECT_EQ(QosMsg::LIVELINESS_SYSTEM_DEFAULT, static_cast<uint8_t>(rclcpp::LivelinessPolicy::SystemDefault));
  EXPECT_EQ(QosMsg::LIVELINESS_AUTOMATIC, static_cast<uint8_t>(rclcpp::LivelinessPolicy::Automatic));
  EXPECT_EQ(QosMsg::LIVELINESS_MANUAL_BY_TOPIC, static_cast<uint8_t>(rclcpp::LivelinessPolicy::ManualByTopic));
  EXPECT_EQ(QosMsg::LIVELINESS_UNKNOWN, static_cast<uint8_t>(rclcpp::LivelinessPolicy::Unknown));
}
