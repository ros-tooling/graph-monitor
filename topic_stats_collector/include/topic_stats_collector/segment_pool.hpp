// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef TOPIC_STATS_COLLECTOR__SEGMENT_POOL_HPP_
#define TOPIC_STATS_COLLECTOR__SEGMENT_POOL_HPP_

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "topic_stats_core/types.hpp"
#include "topic_stats_shm/shm_reader.hpp"

namespace topic_stats_collector
{

/// What one poll of every segment on the machine produced.
struct PollResult
{
  /// Every snapshot drained this poll, across every writer.
  std::vector<topic_stats_core::StatsReport> reports;

  /// Samples overwritten before this collector read them. Nonzero means the collector is not
  /// keeping up, or a writer's ring is too small for its reporting rate.
  uint64_t lapped = 0;
  uint64_t torn = 0;
  uint64_t unknown_statistics = 0;

  /// Writers that appeared this poll.
  size_t attached = 0;
  /// Writers whose process exited. Their final contents are drained before they are dropped.
  size_t detached = 0;
  /// Segments belonging to dead processes that were removed from the system.
  size_t reclaimed = 0;
  /// Segments present but unusable: wrong format version, unreadable, or created by another user.
  size_t rejected = 0;
};

/// Tracks every statistics segment on the machine and drains them.
///
/// Deliberately free of ROS: it is the part of the collector worth testing in isolation, and its
/// awkward cases are all about process lifetime rather than about messages.
class SegmentPool
{
public:
  struct Options
  {
    /// Remove segments left behind by processes that have exited. Without this a machine that
    /// restarts nodes frequently slowly fills its shared memory with dead segments.
    bool reclaim_dead_segments = true;

    /// A writer to ignore, normally the collector's own if it is itself instrumented. Reporting on
    /// the collector through the collector is a loop worth avoiding. Left zeroed to ignore nobody.
    topic_stats_shm::ProcessIdentity ignore_identity;

    /// Discard whatever a newly discovered writer has already accumulated instead of publishing it.
    /// A process that has been running for hours holds a ring full of history whose timestamps are
    /// long past; replaying it on attach would look like a burst of stale traffic.
    bool skip_backlog_on_attach = true;

    /// How to decide whether a writer is still running, given its segment name.
    ///
    /// Takes a name rather than a pid because pids are not comparable across PID namespaces, and
    /// under --ipc=host the writers are in other containers. The default asks whether the writer
    /// still holds its advisory lock on the segment, which the kernel drops on process death
    /// regardless of namespace. Injectable because process lifetime is the hard part of this class.
    std::function<bool(const std::string & segment_name)> is_alive;
  };

  explicit SegmentPool(const Options & options);

  /// Discovers new writers, drains everyone, and retires writers that have exited.
  PollResult poll();

  size_t attached_count() const
  {
    return readers_.size();
  }

  /// Names currently attached, for diagnostics and tests.
  std::vector<std::string> attached_names() const;

private:
  Options options_;
  /// Keyed by segment name so that discovery is a set difference. Ordered so that poll results are
  /// deterministic, which matters for the tests more than for production.
  std::map<std::string, std::unique_ptr<topic_stats_shm::SegmentReader>> readers_;
  /// Segments that failed to open, so that a permanently unreadable one is not retried noisily
  /// every poll.
  std::set<std::string> rejected_;
};

}  // namespace topic_stats_collector

#endif  // TOPIC_STATS_COLLECTOR__SEGMENT_POOL_HPP_
