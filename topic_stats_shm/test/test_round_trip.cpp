// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "topic_stats_shm/shm_reader.hpp"
#include "topic_stats_shm/shm_sink.hpp"

using topic_stats_core::Duration;
using topic_stats_core::NodeReport;
using topic_stats_core::StatKind;
using topic_stats_core::StatSample;
using topic_stats_core::StatsReport;
using topic_stats_core::SysTime;

namespace tss = topic_stats_shm;

namespace
{

Duration ms(int64_t count)
{
  return std::chrono::duration_cast<Duration>(std::chrono::milliseconds(count));
}

std::string unique_name()
{
  static std::atomic<int> counter{0};
  tss::ProcessIdentity identity = tss::current_process_identity();
  identity.pid = identity.pid * 1000 + counter.fetch_add(1);
  return tss::segment_name_for(identity);
}

StatSample make_sample(StatKind stat, const std::string & topic, Duration mean)
{
  StatSample sample;
  sample.stat = stat;
  sample.topic_name = topic;
  sample.window_count = 5;
  sample.mean = mean;
  sample.min = mean - ms(1);
  sample.max = mean + ms(1);
  return sample;
}

StatsReport make_report(SysTime timestamp, const std::string & node, std::vector<StatSample> samples)
{
  StatsReport report;
  report.timestamp = timestamp;
  NodeReport node_report;
  node_report.node_name = node;
  node_report.samples = std::move(samples);
  for (auto & sample : node_report.samples) {
    sample.node_name = node;
  }
  report.nodes.push_back(std::move(node_report));
  return report;
}

/// A writer and a reader attached to the same segment.
struct Pair
{
  explicit Pair(uint32_t capacity)
  {
    name = unique_name();
    tss::SharedMemorySink::Options options;
    options.capacity = capacity;
    options.segment_name = name;
    std::string error;
    sink = tss::SharedMemorySink::create(options, error);
    EXPECT_NE(sink, nullptr) << error;
    reader = tss::SegmentReader::open(name, error);
    EXPECT_NE(reader, nullptr) << error;
  }

  std::string name;
  std::unique_ptr<tss::SharedMemorySink> sink;
  std::unique_ptr<tss::SegmentReader> reader;
};

}  // namespace

TEST(RoundTrip, a_snapshot_survives_the_crossing_intact)
{
  Pair pair(64);
  pair.sink->publish(
    make_report(SysTime{} + ms(1500), "/ns/talker", {make_sample(StatKind::PublishedPeriod, "/chatter", ms(100))}));

  const auto result = pair.reader->drain();
  ASSERT_EQ(result.reports.size(), 1u);
  const auto & report = result.reports.front();
  EXPECT_EQ(report.timestamp, SysTime{} + ms(1500)) << "the report must carry when it was taken";
  ASSERT_EQ(report.nodes.size(), 1u);
  EXPECT_EQ(report.nodes.front().node_name, "/ns/talker");
  ASSERT_EQ(report.nodes.front().samples.size(), 1u);

  const auto & sample = report.nodes.front().samples.front();
  EXPECT_EQ(sample.stat, StatKind::PublishedPeriod);
  EXPECT_EQ(sample.topic_name, "/chatter");
  EXPECT_EQ(sample.node_name, "/ns/talker");
  EXPECT_EQ(sample.window_count, 5u);
  EXPECT_EQ(sample.mean, ms(100));
  EXPECT_EQ(sample.min, ms(99));
  EXPECT_EQ(sample.max, ms(101));
  EXPECT_EQ(result.lapped, 0u);
  EXPECT_EQ(result.torn, 0u);
}

TEST(RoundTrip, negative_take_age_crosses_unharmed)
{
  // Clock skew between hosts. The wire fields are signed for this reason.
  Pair pair(64);
  pair.sink->publish(make_report(SysTime{}, "/n", {make_sample(StatKind::TakeAge, "/t", ms(-250))}));

  const auto result = pair.reader->drain();
  ASSERT_EQ(result.reports.size(), 1u);
  EXPECT_EQ(result.reports.front().nodes.front().samples.front().mean, ms(-250));
}

TEST(RoundTrip, draining_twice_does_not_repeat_a_snapshot)
{
  Pair pair(64);
  pair.sink->publish(make_report(SysTime{}, "/n", {make_sample(StatKind::PublishedPeriod, "/t", ms(10))}));
  EXPECT_EQ(pair.reader->drain().reports.size(), 1u);
  EXPECT_TRUE(pair.reader->drain().reports.empty());
}

TEST(RoundTrip, snapshots_stay_separate_rather_than_merging)
{
  // A collector polling slower than the writer must still be able to tell the periods apart, which
  // is why each sample carries its snapshot id and time.
  Pair pair(64);
  pair.sink->publish(make_report(SysTime{} + ms(1000), "/n", {make_sample(StatKind::PublishedPeriod, "/t", ms(10))}));
  pair.sink->publish(make_report(SysTime{} + ms(2000), "/n", {make_sample(StatKind::PublishedPeriod, "/t", ms(20))}));

  const auto result = pair.reader->drain();
  ASSERT_EQ(result.reports.size(), 2u);
  EXPECT_EQ(result.reports[0].timestamp, SysTime{} + ms(1000));
  EXPECT_EQ(result.reports[1].timestamp, SysTime{} + ms(2000));
  EXPECT_EQ(result.reports[0].nodes.front().samples.front().mean, ms(10));
  EXPECT_EQ(result.reports[1].nodes.front().samples.front().mean, ms(20));
}

TEST(RoundTrip, several_nodes_in_one_snapshot_are_grouped_by_name)
{
  Pair pair(64);
  StatsReport report;
  report.timestamp = SysTime{} + ms(5);
  for (const auto * node : {"/a", "/b"}) {
    NodeReport node_report;
    node_report.node_name = node;
    auto sample = make_sample(StatKind::PublishedPeriod, "/t", ms(10));
    sample.node_name = node;
    node_report.samples.push_back(sample);
    report.nodes.push_back(std::move(node_report));
  }
  pair.sink->publish(report);

  const auto result = pair.reader->drain();
  ASSERT_EQ(result.reports.size(), 1u);
  EXPECT_EQ(result.reports.front().nodes.size(), 2u);
}

TEST(RoundTrip, an_empty_report_is_not_written_at_all)
{
  Pair pair(64);
  pair.sink->publish(StatsReport{});
  EXPECT_TRUE(pair.reader->drain().reports.empty());
}

TEST(RoundTrip, overlong_names_are_truncated_and_counted_rather_than_dropped)
{
  Pair pair(64);
  const std::string long_topic(tss::kMaxTopicNameSize + 40, 'x');
  pair.sink->publish(make_report(SysTime{}, "/n", {make_sample(StatKind::PublishedPeriod, long_topic, ms(10))}));

  EXPECT_EQ(pair.sink->truncated_names(), 1u);
  const auto result = pair.reader->drain();
  ASSERT_EQ(result.reports.size(), 1u);
  const auto & sample = result.reports.front().nodes.front().samples.front();
  EXPECT_EQ(sample.topic_name.size(), tss::kMaxTopicNameSize - 1);
  EXPECT_EQ(sample.topic_name, long_topic.substr(0, tss::kMaxTopicNameSize - 1));
}

TEST(RoundTrip, a_reader_that_falls_behind_reports_what_it_lost)
{
  // The ring is deliberately small here. Silently dropping would make a slow collector look like a
  // system with no traffic.
  Pair pair(4);
  for (int i = 0; i < 20; ++i) {
    pair.sink->publish(make_report(SysTime{} + ms(i), "/n", {make_sample(StatKind::PublishedPeriod, "/t", ms(i + 1))}));
  }

  const auto result = pair.reader->drain();
  EXPECT_EQ(result.lapped, 16u);
  // Only what still fits in the ring survives, and it is the most recent.
  ASSERT_EQ(result.reports.size(), 4u);
  EXPECT_EQ(result.reports.back().nodes.front().samples.front().mean, ms(20));
}

TEST(RoundTrip, skip_to_head_discards_the_backlog_without_calling_it_a_loss)
{
  // A collector attaching to a process that has been running for hours should not report hours of
  // dropped samples on its first poll.
  Pair pair(64);
  for (int i = 0; i < 10; ++i) {
    pair.sink->publish(make_report(SysTime{}, "/n", {make_sample(StatKind::PublishedPeriod, "/t", ms(1))}));
  }
  pair.reader->skip_to_head();

  const auto result = pair.reader->drain();
  EXPECT_TRUE(result.reports.empty());
  EXPECT_EQ(result.lapped, 0u);
}

TEST(RoundTrip, a_reader_resyncs_when_the_write_counter_goes_backwards)
{
  // A writer that died without unlinking, whose segment is then recreated in place by a new
  // process that happens to land on the same identity. An already-attached reader finds its
  // position ahead of the new writer's counter, and must resync rather than believe it missed
  // billions of samples and report the whole ring as lost.
  Pair pair(8);
  pair.sink->publish(make_report(SysTime{}, "/n", {make_sample(StatKind::PublishedPeriod, "/t", ms(1))}));
  ASSERT_EQ(pair.reader->drain().reports.size(), 1u);
  ASSERT_GT(pair.reader->read_position(), 0u);

  // Standing in for the recreate, which resets the counter under the reader's existing mapping.
  const int fd = shm_open(("/" + pair.name).c_str(), O_RDWR, 0);
  ASSERT_GE(fd, 0);
  void * mapping = mmap(nullptr, sizeof(tss::Header), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  ASSERT_NE(mapping, MAP_FAILED);
  static_cast<tss::Header *>(mapping)->write_count.store(0);
  munmap(mapping, sizeof(tss::Header));
  close(fd);

  const auto result = pair.reader->drain();
  EXPECT_EQ(result.lapped, 0u) << "a restarted writer was mistaken for an enormous backlog";
  EXPECT_EQ(pair.reader->read_position(), 0u) << "the reader did not resync to the new writer";
}

TEST(RoundTrip, the_reader_knows_whether_the_writer_is_still_alive)
{
  Pair pair(8);
  EXPECT_TRUE(pair.reader->writer_alive());
  EXPECT_EQ(pair.reader->writer_identity().pid, static_cast<int64_t>(getpid()));
  EXPECT_EQ(pair.reader->writer_identity().pid_namespace, tss::current_process_identity().pid_namespace);
}

TEST(RoundTrip, reading_while_the_writer_runs_never_yields_a_mixed_sample)
{
  // The reason slots carry a sequence number. Every sample that comes out must be internally
  // consistent even though the writer is lapping the reader continuously.
  Pair pair(16);
  std::atomic<bool> stop{false};
  std::atomic<uint64_t> written{0};

  std::thread writer([&]() {
    int64_t value = 1;
    while (!stop.load(std::memory_order_relaxed)) {
      // mean/min/max are related, so a torn sample is detectable by checking the relationship.
      pair.sink->publish(make_report(
        SysTime{}, "/some/long/node/name", {make_sample(StatKind::PublishedPeriod, "/some/topic", ms(value))}));
      written.fetch_add(1, std::memory_order_relaxed);
      value = (value % 1000) + 1;
    }
  });

  uint64_t seen = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
  while (std::chrono::steady_clock::now() < deadline) {
    const auto result = pair.reader->drain();
    for (const auto & report : result.reports) {
      for (const auto & node : report.nodes) {
        EXPECT_EQ(node.node_name, "/some/long/node/name");
        for (const auto & sample : node.samples) {
          EXPECT_EQ(sample.topic_name, "/some/topic");
          EXPECT_EQ(sample.min, sample.mean - ms(1)) << "sample was stitched from two writes";
          EXPECT_EQ(sample.max, sample.mean + ms(1)) << "sample was stitched from two writes";
          ++seen;
        }
      }
    }
  }
  stop = true;
  writer.join();

  EXPECT_GT(written.load(), 0u);
  EXPECT_GT(seen, 0u) << "the reader never managed to read anything";
}
