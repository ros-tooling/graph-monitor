// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <string>

#include "topic_stats_shm/segment.hpp"

namespace tss = topic_stats_shm;

namespace
{

/// Unique per test so that parallel runs do not collide on a name in a shared /dev/shm, which
/// under --ipc=host is shared with every other container on the host as well.
std::string unique_name()
{
  static std::atomic<int> counter{0};
  tss::ProcessIdentity identity = tss::current_process_identity();
  identity.pid = identity.pid * 1000 + counter.fetch_add(1);
  return tss::segment_name_for(identity);
}

}  // namespace

TEST(Segment, create_produces_a_valid_header)
{
  const auto name = unique_name();
  std::string error;
  auto segment = tss::Segment::create(name, 8, error);
  ASSERT_NE(segment, nullptr) << error;

  EXPECT_EQ(segment->header().capacity, 8u);
  EXPECT_EQ(segment->header().sample_size, sizeof(tss::Sample));
  EXPECT_EQ(segment->header().creator_pid, static_cast<int64_t>(getpid()));
  EXPECT_EQ(segment->header().creator_pid_namespace, tss::current_process_identity().pid_namespace);
  EXPECT_EQ(segment->size(), tss::segment_size(8));
  EXPECT_FALSE(segment->read_only());
  EXPECT_TRUE(segment->owns_name());
}

TEST(Segment, a_fresh_segment_starts_zeroed)
{
  // The counters are only valid at zero because ftruncate guarantees zero fill. If that ever stops
  // being true the ring protocol starts from garbage.
  const auto name = unique_name();
  std::string error;
  auto segment = tss::Segment::create(name, 4, error);
  ASSERT_NE(segment, nullptr) << error;
  EXPECT_EQ(segment->header().write_count.load(), 0u);
  EXPECT_EQ(segment->header().snapshot_count.load(), 0u);
  for (uint64_t i = 0; i < 4; ++i) {
    EXPECT_EQ(segment->sample(i).sequence.load(), 0u);
  }
}

TEST(Segment, zero_capacity_is_refused)
{
  std::string error;
  EXPECT_EQ(tss::Segment::create(unique_name(), 0, error), nullptr);
  EXPECT_FALSE(error.empty());
}

TEST(Segment, destroying_the_creator_unlinks_the_name)
{
  const auto name = unique_name();
  std::string error;
  {
    auto segment = tss::Segment::create(name, 4, error);
    ASSERT_NE(segment, nullptr) << error;
  }
  EXPECT_EQ(tss::Segment::open_readonly(name, error), nullptr);
}

TEST(Segment, creating_over_a_stale_segment_replaces_it)
{
  // A crashed process leaves its segment behind. Restarting must not inherit the old contents, or
  // worse, the old capacity. Truncating to zero before resizing is what guarantees that, since
  // resizing to the same size would leave the old bytes in place.
  const auto name = unique_name();
  std::string error;
  auto first = tss::Segment::create(name, 4, error);
  ASSERT_NE(first, nullptr) << error;
  first->header().write_count.store(99);
  first->release_ownership();
  first.reset();

  auto second = tss::Segment::create(name, 16, error);
  ASSERT_NE(second, nullptr) << error;
  EXPECT_EQ(second->header().capacity, 16u);
  EXPECT_EQ(second->header().write_count.load(), 0u);
}

TEST(Segment, a_second_writer_cannot_steal_a_live_writers_segment)
{
  // Under --ipc=host every container shares /dev/shm, so a name clash is a cross-container
  // accident rather than an impossibility. Replacing a running writer's segment would silently
  // destroy its statistics, so creation refuses instead.
  const auto name = unique_name();
  std::string error;
  auto owner = tss::Segment::create(name, 4, error);
  ASSERT_NE(owner, nullptr) << error;

  std::string second_error;
  auto thief = tss::Segment::create(name, 4, second_error);
  EXPECT_EQ(thief, nullptr);
  EXPECT_NE(second_error.find("running writer"), std::string::npos) << second_error;

  // The original is untouched.
  EXPECT_EQ(owner->header().capacity, 4u);
}

TEST(Segment, a_writer_holding_its_segment_reports_itself_alive)
{
  const auto name = unique_name();
  std::string error;
  auto writer = tss::Segment::create(name, 4, error);
  ASSERT_NE(writer, nullptr) << error;

  // Probed the way a collector in another container would, which is why this cannot be a pid
  // question: that pid means nothing in the collector's namespace.
  EXPECT_TRUE(tss::Segment::writer_alive(name));

  auto reader = tss::Segment::open_readonly(name, error);
  ASSERT_NE(reader, nullptr) << error;
  EXPECT_TRUE(reader->writer_alive());
}

TEST(Segment, a_reader_outlives_its_writer_and_notices)
{
  const auto name = unique_name();
  std::string error;
  auto writer = tss::Segment::create(name, 4, error);
  ASSERT_NE(writer, nullptr) << error;
  writer->release_ownership();  // keep the name around after the writer lets go

  auto reader = tss::Segment::open_readonly(name, error);
  ASSERT_NE(reader, nullptr) << error;
  ASSERT_TRUE(reader->writer_alive());

  // Standing in for the writing process exiting: the kernel drops the lock either way.
  writer.reset();

  EXPECT_FALSE(reader->writer_alive());
  EXPECT_FALSE(tss::Segment::writer_alive(name));

  std::string ignored;
  tss::Segment::unlink(name, ignored);
}

TEST(Segment, a_segment_that_does_not_exist_has_no_live_writer)
{
  EXPECT_FALSE(tss::Segment::writer_alive("topic_stats.1.999999999"));
}

TEST(Segment, readers_see_what_the_writer_wrote)
{
  const auto name = unique_name();
  std::string error;
  auto writer = tss::Segment::create(name, 4, error);
  ASSERT_NE(writer, nullptr) << error;
  writer->header().write_count.store(7);

  auto reader = tss::Segment::open_readonly(name, error);
  ASSERT_NE(reader, nullptr) << error;
  EXPECT_EQ(reader->header().write_count.load(), 7u);
  EXPECT_TRUE(reader->read_only());
  EXPECT_FALSE(reader->owns_name()) << "a reader must never unlink someone else's segment";
}

TEST(Segment, opening_a_missing_segment_reports_rather_than_crashes)
{
  std::string error;
  EXPECT_EQ(tss::Segment::open_readonly("topic_stats.999999999", error), nullptr);
  EXPECT_FALSE(error.empty());
}

TEST(Segment, a_segment_without_the_magic_is_refused)
{
  // /dev/shm is shared with the DDS implementation, so a name collision must not be dereferenced.
  const auto name = unique_name();
  std::string error;
  auto writer = tss::Segment::create(name, 4, error);
  ASSERT_NE(writer, nullptr) << error;
  writer->header().magic[0] = 'X';

  EXPECT_EQ(tss::Segment::open_readonly(name, error), nullptr);
  EXPECT_NE(error.find("magic"), std::string::npos) << error;
}

TEST(Segment, a_segment_from_a_different_format_version_is_refused)
{
  const auto name = unique_name();
  std::string error;
  auto writer = tss::Segment::create(name, 4, error);
  ASSERT_NE(writer, nullptr) << error;
  writer->header().format_version = tss::kFormatVersion + 1;

  EXPECT_EQ(tss::Segment::open_readonly(name, error), nullptr);
  EXPECT_NE(error.find("version"), std::string::npos) << error;
}

TEST(Segment, a_segment_whose_record_size_disagrees_is_refused)
{
  // Same version but a different record size: two builds that disagree about the layout without
  // anyone remembering to bump the version.
  const auto name = unique_name();
  std::string error;
  auto writer = tss::Segment::create(name, 4, error);
  ASSERT_NE(writer, nullptr) << error;
  writer->header().sample_size = sizeof(tss::Sample) + 8;

  EXPECT_EQ(tss::Segment::open_readonly(name, error), nullptr);
  EXPECT_NE(error.find("record size"), std::string::npos) << error;
}

TEST(Segment, a_segment_smaller_than_its_declared_capacity_is_refused)
{
  const auto name = unique_name();
  std::string error;
  auto writer = tss::Segment::create(name, 4, error);
  ASSERT_NE(writer, nullptr) << error;
  writer->header().capacity = 4096;

  EXPECT_EQ(tss::Segment::open_readonly(name, error), nullptr);
  EXPECT_FALSE(error.empty());
}

TEST(Segment, list_finds_our_segments_and_ignores_others)
{
  const auto name = unique_name();
  std::string error;
  auto writer = tss::Segment::create(name, 4, error);
  ASSERT_NE(writer, nullptr) << error;

  const auto names = tss::Segment::list(error);
  EXPECT_NE(std::find(names.begin(), names.end(), name), names.end());
  for (const auto & found : names) {
    tss::ProcessIdentity identity;
    EXPECT_TRUE(tss::identity_from_segment_name(found, identity)) << found << " is not one of ours";
  }
}
