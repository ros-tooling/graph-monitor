// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <utility>

#include "topic_stats_collector/segment_pool.hpp"
#include "topic_stats_shm/segment.hpp"
#include "topic_stats_shm/shm_sink.hpp"

using topic_stats_collector::SegmentPool;
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

/// A writer pretending to be some other process, so that the pool discovers it exactly the way it
/// discovers a real one. Its identity is its segment name; whether the pool believes it is alive is
/// controlled separately through the injected liveness predicate.
class FakeWriter
{
public:
  explicit FakeWriter(const tss::ProcessIdentity & identity, uint32_t capacity = 64, bool leave_segment_behind = false)
  : name_(tss::segment_name_for(identity))
  {
    tss::SharedMemorySink::Options options;
    options.capacity = capacity;
    options.segment_name = name_;
    options.unlink_on_destroy = !leave_segment_behind;
    std::string error;
    sink_ = tss::SharedMemorySink::create(options, error);
    EXPECT_NE(sink_, nullptr) << error;
  }

  void publish(SysTime timestamp, const std::string & node, const std::string & topic, Duration mean)
  {
    StatsReport report;
    report.timestamp = timestamp;
    NodeReport node_report;
    node_report.node_name = node;
    StatSample sample;
    sample.stat = StatKind::PublishedPeriod;
    sample.node_name = node;
    sample.topic_name = topic;
    sample.window_count = 3;
    sample.mean = mean;
    sample.min = mean;
    sample.max = mean;
    node_report.samples.push_back(sample);
    report.nodes.push_back(std::move(node_report));
    sink_->publish(report);
  }

  /// Drops the segment without unlinking it, standing in for a process that died rather than
  /// exited: the name stays behind and the kernel releases the advisory lock.
  void abandon()
  {
    sink_.reset();
  }

  const std::string & name() const
  {
    return name_;
  }

private:
  std::string name_;
  std::unique_ptr<tss::SharedMemorySink> sink_;
};

/// Lets a test decide which writers the pool considers running, so that a process exiting between
/// two polls can be exercised without spawning and killing anything.
///
/// Note that the pool's real predicate probes an advisory lock rather than a pid; there is a
/// separate test below that exercises that path, because injecting liveness everywhere is exactly
/// what let a PID-namespace assumption survive unnoticed once already.
class FakeLiveness
{
public:
  void kill(const tss::ProcessIdentity & identity)
  {
    dead_.insert(tss::segment_name_for(identity));
  }

  std::function<bool(const std::string &)> predicate()
  {
    return [this](const std::string & name) { return dead_.count(name) == 0; };
  }

private:
  std::set<std::string> dead_;
};

std::atomic<int64_t> g_pid_counter{9000000};

/// A distinct pretend process, in this process's own PID namespace so the names look real.
tss::ProcessIdentity next_fake_identity()
{
  tss::ProcessIdentity identity = tss::current_process_identity();
  identity.pid = g_pid_counter.fetch_add(1);
  return identity;
}

SegmentPool::Options default_options(FakeLiveness & liveness)
{
  SegmentPool::Options options;
  // The test process owns real segments of its own in other suites; ignoring it keeps them out.
  options.ignore_identity = tss::current_process_identity();
  options.is_alive = liveness.predicate();
  return options;
}

size_t count_reports_for(const topic_stats_collector::PollResult & result, const std::string & node_name)
{
  size_t count = 0;
  for (const auto & report : result.reports) {
    for (const auto & node : report.nodes) {
      if (node.node_name == node_name) {
        count += node.samples.size();
      }
    }
  }
  return count;
}

}  // namespace

TEST(SegmentPool, discovers_a_new_writer_and_reports_it_attached)
{
  FakeLiveness liveness;
  auto options = default_options(liveness);
  options.skip_backlog_on_attach = false;
  SegmentPool pool(options);

  FakeWriter writer(next_fake_identity());
  writer.publish(SysTime{} + ms(100), "/talker", "/chatter", ms(50));

  const auto result = pool.poll();
  EXPECT_EQ(result.attached, 1u);
  EXPECT_EQ(pool.attached_count(), 1u);
  EXPECT_EQ(count_reports_for(result, "/talker"), 1u);
}

TEST(SegmentPool, an_established_writer_is_not_reattached_every_poll)
{
  FakeLiveness liveness;
  auto options = default_options(liveness);
  options.skip_backlog_on_attach = false;
  SegmentPool pool(options);

  FakeWriter writer(next_fake_identity());
  writer.publish(SysTime{}, "/talker", "/chatter", ms(50));
  ASSERT_EQ(pool.poll().attached, 1u);

  writer.publish(SysTime{}, "/talker", "/chatter", ms(60));
  const auto result = pool.poll();
  EXPECT_EQ(result.attached, 0u);
  EXPECT_EQ(count_reports_for(result, "/talker"), 1u);
}

TEST(SegmentPool, polling_twice_does_not_republish_the_same_snapshot)
{
  FakeLiveness liveness;
  auto options = default_options(liveness);
  options.skip_backlog_on_attach = false;
  SegmentPool pool(options);

  FakeWriter writer(next_fake_identity());
  writer.publish(SysTime{}, "/talker", "/chatter", ms(50));
  ASSERT_EQ(count_reports_for(pool.poll(), "/talker"), 1u);
  EXPECT_EQ(count_reports_for(pool.poll(), "/talker"), 0u);
}

TEST(SegmentPool, backlog_is_skipped_on_attach_by_default)
{
  // A process running for hours holds a ring full of history. Replaying it on attach would look
  // like a burst of very stale traffic.
  FakeLiveness liveness;
  SegmentPool pool(default_options(liveness));

  FakeWriter writer(next_fake_identity());
  for (int i = 0; i < 10; ++i) {
    writer.publish(SysTime{}, "/talker", "/chatter", ms(50));
  }

  const auto result = pool.poll();
  EXPECT_EQ(result.attached, 1u);
  EXPECT_EQ(count_reports_for(result, "/talker"), 0u);

  // What it writes after attaching does come through.
  writer.publish(SysTime{}, "/talker", "/chatter", ms(60));
  EXPECT_EQ(count_reports_for(pool.poll(), "/talker"), 1u);
}

TEST(SegmentPool, several_writers_are_collected_together)
{
  FakeLiveness liveness;
  auto options = default_options(liveness);
  options.skip_backlog_on_attach = false;
  SegmentPool pool(options);

  FakeWriter first(next_fake_identity());
  FakeWriter second(next_fake_identity());
  first.publish(SysTime{}, "/talker", "/chatter", ms(50));
  second.publish(SysTime{}, "/listener", "/chatter", ms(50));

  const auto result = pool.poll();
  EXPECT_EQ(result.attached, 2u);
  EXPECT_EQ(count_reports_for(result, "/talker"), 1u);
  EXPECT_EQ(count_reports_for(result, "/listener"), 1u);
}

TEST(SegmentPool, a_dead_writers_last_words_are_collected_before_it_is_dropped)
{
  // The process exits between one poll and the next. Whatever it managed to write must still be
  // published rather than thrown away with the segment.
  FakeLiveness liveness;
  auto options = default_options(liveness);
  options.skip_backlog_on_attach = false;
  options.reclaim_dead_segments = false;
  SegmentPool pool(options);

  const auto identity = next_fake_identity();
  FakeWriter writer(identity);
  writer.publish(SysTime{}, "/talker", "/chatter", ms(50));
  ASSERT_EQ(pool.poll().attached, 1u);

  writer.publish(SysTime{}, "/talker", "/chatter", ms(70));
  liveness.kill(identity);

  const auto result = pool.poll();
  EXPECT_EQ(result.detached, 1u);
  EXPECT_EQ(count_reports_for(result, "/talker"), 1u) << "the writer's final snapshot was discarded";
  EXPECT_EQ(pool.attached_count(), 0u);
}

TEST(SegmentPool, a_dead_writers_segment_is_reclaimed)
{
  // Without this a machine that restarts nodes slowly fills /dev/shm with dead segments, which is
  // shared with the DDS implementation's own.
  FakeLiveness liveness;
  SegmentPool pool(default_options(liveness));

  const auto identity = next_fake_identity();
  const auto name = tss::segment_name_for(identity);
  {
    FakeWriter writer(identity);
    writer.publish(SysTime{}, "/talker", "/chatter", ms(50));
    ASSERT_EQ(pool.poll().attached, 1u);
    liveness.kill(identity);

    const auto result = pool.poll();
    EXPECT_EQ(result.detached, 1u);
    EXPECT_EQ(result.reclaimed, 1u);
  }

  std::string error;
  const auto names = tss::Segment::list(error);
  EXPECT_EQ(std::find(names.begin(), names.end(), name), names.end()) << "segment outlived its writer";
}

TEST(SegmentPool, reclamation_can_be_turned_off)
{
  FakeLiveness liveness;
  auto options = default_options(liveness);
  options.reclaim_dead_segments = false;
  SegmentPool pool(options);

  const auto identity = next_fake_identity();
  FakeWriter writer(identity);
  ASSERT_EQ(pool.poll().attached, 1u);
  liveness.kill(identity);

  const auto result = pool.poll();
  EXPECT_EQ(result.detached, 1u);
  EXPECT_EQ(result.reclaimed, 0u);
}

TEST(SegmentPool, our_own_segment_is_ignored)
{
  // The collector may itself be running under the instrumentation. Reporting on itself through
  // itself is a loop worth avoiding.
  FakeLiveness liveness;
  auto options = default_options(liveness);
  options.ignore_identity = tss::current_process_identity();
  SegmentPool pool(options);

  FakeWriter self(tss::current_process_identity());
  self.publish(SysTime{}, "/collector", "/chatter", ms(50));

  const auto result = pool.poll();
  EXPECT_EQ(result.attached, 0u);
  EXPECT_EQ(count_reports_for(result, "/collector"), 0u);
}

TEST(SegmentPool, an_unreadable_segment_is_counted_and_not_retried_in_earnest)
{
  // A writer from a future format version. It must not stop the collector from serving everyone
  // else, and it must not be reopened noisily forever.
  FakeLiveness liveness;
  auto options = default_options(liveness);
  options.skip_backlog_on_attach = false;
  SegmentPool pool(options);

  const auto identity = next_fake_identity();
  const auto name = tss::segment_name_for(identity);
  std::string error;
  auto segment = tss::Segment::create(name, 8, error);
  ASSERT_NE(segment, nullptr) << error;
  segment->header().format_version = tss::kFormatVersion + 99;

  FakeWriter good(next_fake_identity());
  good.publish(SysTime{}, "/talker", "/chatter", ms(50));

  const auto first = pool.poll();
  EXPECT_EQ(first.rejected, 1u);
  EXPECT_EQ(count_reports_for(first, "/talker"), 1u) << "one bad segment blocked the others";

  const auto second = pool.poll();
  EXPECT_EQ(second.rejected, 1u);
  EXPECT_EQ(second.attached, 0u);
}

TEST(SegmentPool, a_writer_that_vanishes_between_polls_is_dropped_cleanly)
{
  FakeLiveness liveness;
  auto options = default_options(liveness);
  options.skip_backlog_on_attach = false;
  SegmentPool pool(options);

  {
    FakeWriter writer(next_fake_identity());
    writer.publish(SysTime{}, "/talker", "/chatter", ms(50));
    ASSERT_EQ(pool.poll().attached, 1u);
  }

  const auto result = pool.poll();
  EXPECT_EQ(result.detached, 1u);
  EXPECT_EQ(pool.attached_count(), 0u);
}

// Everything above injects liveness, which is convenient but is exactly how a PID-namespace
// assumption survived unnoticed: the pool asked `kill(pid, 0)`, which is meaningless for a writer
// in another container, and no test ever ran the real predicate. These use it.

TEST(SegmentPoolRealLiveness, a_writer_still_holding_its_segment_is_seen_as_alive)
{
  SegmentPool::Options options;
  options.ignore_identity = tss::current_process_identity();
  options.skip_backlog_on_attach = false;
  options.reclaim_dead_segments = true;
  // No is_alive override: this is the shipping predicate, which probes the writer's advisory lock.
  SegmentPool pool(options);

  const auto identity = next_fake_identity();
  FakeWriter writer(identity);
  writer.publish(SysTime{}, "/talker", "/chatter", ms(50));

  const auto result = pool.poll();
  EXPECT_EQ(result.attached, 1u);
  EXPECT_EQ(result.detached, 0u) << "a live writer was mistaken for a dead one";
  EXPECT_EQ(result.reclaimed, 0u) << "a live writer's segment was reclaimed out from under it";
  EXPECT_EQ(count_reports_for(result, "/talker"), 1u);
  EXPECT_EQ(pool.attached_count(), 1u);
}

TEST(SegmentPoolRealLiveness, a_writer_that_lets_go_is_seen_as_dead_and_reclaimed)
{
  SegmentPool::Options options;
  options.ignore_identity = tss::current_process_identity();
  options.skip_backlog_on_attach = false;
  options.reclaim_dead_segments = true;
  SegmentPool pool(options);

  const auto identity = next_fake_identity();
  const auto name = tss::segment_name_for(identity);

  // Left behind deliberately, because a process that dies runs no destructor. That is the case the
  // lock exists for: the segment is still there, and only the released lock says its writer is not.
  FakeWriter writer(identity, 64, /* leave_segment_behind */ true);
  writer.publish(SysTime{}, "/talker", "/chatter", ms(50));
  ASSERT_EQ(pool.poll().attached, 1u);
  writer.abandon();

  std::string error;
  auto names = tss::Segment::list(error);
  ASSERT_NE(std::find(names.begin(), names.end(), name), names.end())
    << "test setup is wrong: the segment should still exist, only its lock released";

  const auto result = pool.poll();
  EXPECT_EQ(result.detached, 1u);
  EXPECT_EQ(result.reclaimed, 1u) << "a dead writer's segment was not reclaimed";
  EXPECT_EQ(pool.attached_count(), 0u);

  names = tss::Segment::list(error);
  EXPECT_EQ(std::find(names.begin(), names.end(), name), names.end());
}

TEST(SegmentPoolRealLiveness, a_writer_in_another_pid_namespace_is_judged_correctly)
{
  // The container case. Under --ipc=host the collector sees segments written by processes in other
  // PID namespaces, whose pids mean nothing here and may well collide with a local process. The
  // lock probe does not care, so a live writer is still seen as live.
  SegmentPool::Options options;
  options.ignore_identity = tss::current_process_identity();
  options.skip_backlog_on_attach = false;
  SegmentPool pool(options);

  // A pid of 1 is what a containerised node's init process gets, and almost certainly matches a
  // running process in the collector's own namespace too.
  tss::ProcessIdentity foreign;
  foreign.pid_namespace = tss::current_process_identity().pid_namespace + 1;
  foreign.pid = 1;
  FakeWriter writer(foreign);
  writer.publish(SysTime{}, "/other/container/talker", "/chatter", ms(50));

  const auto result = pool.poll();
  EXPECT_EQ(result.attached, 1u);
  EXPECT_EQ(result.detached, 0u);
  EXPECT_EQ(count_reports_for(result, "/other/container/talker"), 1u);
}
