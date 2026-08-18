// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <unistd.h>

#include <string>

#include "topic_stats_shm/wire_format.hpp"

using topic_stats_core::StatKind;

namespace tss = topic_stats_shm;

// The layout is a contract between two independently built processes. If any of these change, the
// format version has to change with them, which is what the static_asserts in the header enforce at
// compile time; these pin the intended values so a change is deliberate.
TEST(WireFormat, layout_is_pinned)
{
  EXPECT_EQ(sizeof(tss::Sample), 256u);
  EXPECT_EQ(sizeof(tss::Header), 256u);
  EXPECT_EQ(tss::kFormatVersion, 2u);
  EXPECT_EQ(tss::segment_size(4), sizeof(tss::Header) + 4 * sizeof(tss::Sample));
}

TEST(WireFormat, stat_kinds_round_trip)
{
  for (const auto kind :
       {StatKind::PublishedPeriod, StatKind::ReceivedPeriod, StatKind::TakeAge, StatKind::CallbackDuration})
  {
    uint8_t wire = 255;
    ASSERT_TRUE(tss::to_wire_stat(kind, wire)) << "no wire value for a statistic the core can produce";
    StatKind decoded{};
    ASSERT_TRUE(tss::from_wire_stat(wire, decoded));
    EXPECT_EQ(decoded, kind);
  }
}

TEST(WireFormat, wire_values_are_fixed_not_enum_order)
{
  // Reordering StatKind must not silently reinterpret existing segments.
  uint8_t wire = 255;
  ASSERT_TRUE(tss::to_wire_stat(StatKind::PublishedPeriod, wire));
  EXPECT_EQ(wire, 0u);
  ASSERT_TRUE(tss::to_wire_stat(StatKind::ReceivedPeriod, wire));
  EXPECT_EQ(wire, 1u);
  ASSERT_TRUE(tss::to_wire_stat(StatKind::TakeAge, wire));
  EXPECT_EQ(wire, 2u);
}

TEST(WireFormat, unknown_wire_statistic_is_rejected_rather_than_guessed)
{
  // A newer writer measuring something this build has never heard of.
  StatKind decoded = StatKind::PublishedPeriod;
  EXPECT_FALSE(tss::from_wire_stat(200, decoded));
}

TEST(WireFormat, segment_names_carry_the_namespace_and_pid)
{
  const auto name = tss::segment_name_for(tss::ProcessIdentity{4026534050, 4321});
  EXPECT_EQ(name, "topic_stats.4026534050.4321");

  tss::ProcessIdentity identity;
  ASSERT_TRUE(tss::identity_from_segment_name(name, identity));
  EXPECT_EQ(identity.pid_namespace, 4026534050u);
  EXPECT_EQ(identity.pid, 4321);
}

TEST(WireFormat, identical_pids_in_different_namespaces_get_different_names)
{
  // The whole point. Every PID namespace numbers from 1, so two containers each running a few
  // nodes will collide in the low tens almost immediately. Colliding names would mean one
  // container's writer destroying another's segment.
  const auto first = tss::segment_name_for(tss::ProcessIdentity{4026534050, 42});
  const auto second = tss::segment_name_for(tss::ProcessIdentity{4026534099, 42});
  EXPECT_NE(first, second);
}

TEST(WireFormat, this_process_has_a_namespace_qualified_identity)
{
  const auto identity = tss::current_process_identity();
  EXPECT_EQ(identity.pid, static_cast<int64_t>(getpid()));
  // Zero would mean /proc/self/ns/pid could not be read, which costs the ability to tell
  // containers apart. Not fatal, but worth noticing on a platform where it happens.
  EXPECT_NE(identity.pid_namespace, 0u) << "could not determine this process's PID namespace";
}

TEST(WireFormat, foreign_segment_names_are_not_claimed)
{
  // /dev/shm is shared with the DDS implementation and anything else on the machine, and under
  // --ipc=host it is shared with every other container too.
  tss::ProcessIdentity identity;
  EXPECT_FALSE(tss::identity_from_segment_name("fastdds_0629026be36fe5b8", identity));
  EXPECT_FALSE(tss::identity_from_segment_name("topic_stats.", identity));
  EXPECT_FALSE(tss::identity_from_segment_name("topic_stats.4026534050", identity)) << "no pid part";
  EXPECT_FALSE(tss::identity_from_segment_name("topic_stats.abc.12", identity));
  EXPECT_FALSE(tss::identity_from_segment_name("topic_stats.12.abc", identity));
  EXPECT_FALSE(tss::identity_from_segment_name("topic_stats.12.", identity));
  EXPECT_FALSE(tss::identity_from_segment_name("not_ours.12.34", identity));
}

TEST(BoundedCopy, short_strings_are_copied_whole_and_terminated)
{
  char buffer[8];
  EXPECT_FALSE(tss::copy_bounded("abc", buffer, sizeof(buffer)));
  EXPECT_STREQ(buffer, "abc");
}

TEST(BoundedCopy, long_strings_are_truncated_terminated_and_reported)
{
  // Truncation is flagged rather than dropping the sample: a shortened name still identifies the
  // problem, a missing measurement does not.
  char buffer[4];
  EXPECT_TRUE(tss::copy_bounded("abcdefgh", buffer, sizeof(buffer)));
  EXPECT_STREQ(buffer, "abc");
}

TEST(BoundedCopy, exactly_fitting_strings_are_not_reported_as_truncated)
{
  char buffer[4];
  EXPECT_FALSE(tss::copy_bounded("abc", buffer, sizeof(buffer)));
  EXPECT_STREQ(buffer, "abc");
}

TEST(BoundedCopy, trailing_bytes_are_zeroed_so_no_stale_name_leaks)
{
  // Slots are reused, so a shorter name overwriting a longer one must not leave the old tail
  // visible past the terminator.
  char buffer[8];
  tss::copy_bounded("abcdefg", buffer, sizeof(buffer));
  tss::copy_bounded("xy", buffer, sizeof(buffer));
  EXPECT_STREQ(buffer, "xy");
  for (size_t i = 2; i < sizeof(buffer); ++i) {
    EXPECT_EQ(buffer[i], '\0') << "byte " << i << " still holds the previous name";
  }
}
