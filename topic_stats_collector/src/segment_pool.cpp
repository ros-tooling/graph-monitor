// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "topic_stats_collector/segment_pool.hpp"

#include <algorithm>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "topic_stats_shm/segment.hpp"
#include "topic_stats_shm/wire_format.hpp"

namespace topic_stats_collector
{

namespace
{

void accumulate(PollResult & result, topic_stats_shm::DrainResult && drained)
{
  result.lapped += drained.lapped;
  result.torn += drained.torn;
  result.unknown_statistics += drained.unknown_statistics;
  for (auto & report : drained.reports) {
    result.reports.push_back(std::move(report));
  }
}

}  // namespace

SegmentPool::SegmentPool(const Options & options)
: options_(options)
{
  if (!options_.is_alive) {
    options_.is_alive = [](const std::string & name) { return topic_stats_shm::Segment::writer_alive(name); };
  }
}

std::vector<std::string> SegmentPool::attached_names() const
{
  std::vector<std::string> names;
  names.reserve(readers_.size());
  for (const auto & [name, reader] : readers_) {
    (void)reader;
    names.push_back(name);
  }
  return names;
}

PollResult SegmentPool::poll()
{
  PollResult result;

  std::string list_error;
  const auto names = topic_stats_shm::Segment::list(list_error);
  const std::set<std::string> present(names.begin(), names.end());

  // Anything we had a reader for that has since been removed from the filesystem. Its writer is
  // gone and there is nothing left to drain, because the mapping outlives the name but the writer
  // stopped writing when it exited.
  for (auto it = readers_.begin(); it != readers_.end();) {
    if (present.count(it->first) == 0) {
      accumulate(result, it->second->drain());
      it = readers_.erase(it);
      result.detached++;
    } else {
      ++it;
    }
  }

  for (const auto & name : names) {
    topic_stats_shm::ProcessIdentity identity;
    if (!topic_stats_shm::identity_from_segment_name(name, identity)) {
      continue;
    }
    if (identity == options_.ignore_identity) {
      continue;
    }

    // Asked by name rather than by pid, so that a segment whose header is unreadable can still be
    // judged, and so that the answer is meaningful when the writer lives in another container.
    const bool alive = options_.is_alive(name);
    auto existing = readers_.find(name);

    if (existing == readers_.end()) {
      if (rejected_.count(name) > 0) {
        // Already known bad. Counted so it stays visible, but not reopened in earnest every poll.
        result.rejected++;
        continue;
      }

      std::string error;
      auto reader = topic_stats_shm::SegmentReader::open(name, error);
      if (reader == nullptr) {
        // Wrong format version, another user's segment, or caught mid-creation.
        rejected_.insert(name);
        result.rejected++;
        continue;
      }
      if (options_.skip_backlog_on_attach && alive) {
        reader->skip_to_head();
      }
      result.attached++;
      existing = readers_.emplace(name, std::move(reader)).first;
    }

    // Drained before the liveness verdict is acted on, so that a process which exited since the
    // last poll still gets its final snapshot published rather than discarded with its segment.
    accumulate(result, existing->second->drain());

    if (!alive) {
      readers_.erase(existing);
      result.detached++;
      if (options_.reclaim_dead_segments) {
        std::string error;
        if (topic_stats_shm::Segment::unlink(name, error)) {
          result.reclaimed++;
        }
      }
    }
  }

  // A rejected segment whose writer has since exited is worth reclaiming too, and forgetting so
  // that a new writer taking the same identity gets a fresh attempt.
  for (auto it = rejected_.begin(); it != rejected_.end();) {
    if (present.count(*it) == 0) {
      it = rejected_.erase(it);
      continue;
    }
    if (!options_.is_alive(*it)) {
      if (options_.reclaim_dead_segments) {
        std::string error;
        if (topic_stats_shm::Segment::unlink(*it, error)) {
          result.reclaimed++;
        }
      }
      it = rejected_.erase(it);
    } else {
      ++it;
    }
  }

  return result;
}

}  // namespace topic_stats_collector
