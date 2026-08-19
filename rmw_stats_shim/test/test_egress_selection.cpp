// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <string>

#include "rmw_stats_shim/stat_collector.hpp"

using rmw_stats_shim::egress_kind_from_string;
using rmw_stats_shim::EgressKind;

TEST(EgressSelection, every_supported_egress_is_selectable_by_name)
{
  EgressKind kind = EgressKind::None;
  ASSERT_TRUE(egress_kind_from_string("rmw_publisher", kind));
  EXPECT_EQ(kind, EgressKind::RmwPublisher);
  ASSERT_TRUE(egress_kind_from_string("shared_memory", kind));
  EXPECT_EQ(kind, EgressKind::SharedMemory);
  ASSERT_TRUE(egress_kind_from_string("none", kind));
  EXPECT_EQ(kind, EgressKind::None);
}

TEST(EgressSelection, an_unrecognised_name_is_rejected_without_disturbing_the_caller)
{
  // The caller falls back to the default on false. It must not be left holding a half-parsed
  // value, and it must not silently end up reporting nothing, which looks exactly like a healthy
  // silent system.
  EgressKind kind = EgressKind::RmwPublisher;
  EXPECT_FALSE(egress_kind_from_string("shared-memory", kind));
  EXPECT_EQ(kind, EgressKind::RmwPublisher);

  EXPECT_FALSE(egress_kind_from_string("", kind));
  EXPECT_FALSE(egress_kind_from_string("RMW_PUBLISHER", kind)) << "matching is deliberately exact";
  EXPECT_EQ(kind, EgressKind::RmwPublisher);
}
