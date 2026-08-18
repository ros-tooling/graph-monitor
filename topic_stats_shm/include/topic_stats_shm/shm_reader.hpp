// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef TOPIC_STATS_SHM__SHM_READER_HPP_
#define TOPIC_STATS_SHM__SHM_READER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "topic_stats_core/types.hpp"
#include "topic_stats_shm/segment.hpp"

namespace topic_stats_shm
{

/// What one drain of a segment produced.
struct DrainResult
{
  /// One report per snapshot the writer published, oldest first. Reconstructed from the samples,
  /// so a report here carries the timestamp of when it was taken, not when it was read.
  std::vector<topic_stats_core::StatsReport> reports;
  /// Samples the writer overwrote before this reader got to them. Nonzero means the collector is
  /// not keeping up with the writer, or the ring is too small.
  uint64_t lapped = 0;
  /// Samples carrying a statistic this build does not know, which implies a newer writer.
  uint64_t unknown_statistics = 0;
  /// Samples abandoned because the writer overwrote the slot during the copy. Distinct from
  /// `lapped` only in that it was detected mid-read.
  uint64_t torn = 0;
};

/// Reads one writer's segment.
///
/// Holds its own read position, so repeated drains return only what is new. Safe to use while the
/// writer is running: it detects both a slot being overwritten before it was read and a slot being
/// overwritten during the read.
class SegmentReader
{
public:
  static std::unique_ptr<SegmentReader> open(const std::string & name, std::string & error);

  /// Collects everything written since the previous call.
  DrainResult drain();

  /// Positions the reader at the writer's current head, discarding the backlog without counting it
  /// as lost. Used on first attach, where a partially filled ring is history rather than a gap.
  void skip_to_head();

  /// Identity of the writing process, for diagnosis. Not used for liveness: a pid from another PID
  /// namespace is meaningless to this process.
  ProcessIdentity writer_identity() const;

  /// Whether the writing process is still running, decided from its advisory lock on the segment
  /// rather than from its pid, so that the answer holds across containers. A segment whose writer
  /// has exited holds only history and can be unlinked once drained.
  bool writer_alive() const;

  const std::string & name() const;

  uint64_t read_position() const
  {
    return read_position_;
  }

private:
  explicit SegmentReader(std::unique_ptr<Segment> segment);

  std::unique_ptr<Segment> segment_;
  uint64_t read_position_ = 0;
};

}  // namespace topic_stats_shm

#endif  // TOPIC_STATS_SHM__SHM_READER_HPP_
