// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef TOPIC_STATS_SHM__SEGMENT_HPP_
#define TOPIC_STATS_SHM__SEGMENT_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "topic_stats_shm/wire_format.hpp"

namespace topic_stats_shm
{

/// A mapped POSIX shared memory segment.
///
/// Errors are returned rather than thrown. This code is loaded into every instrumented process, and
/// a process must never fail to start because its instrumentation could not get a segment.
class Segment
{
public:
  ~Segment();

  Segment(const Segment &) = delete;
  Segment & operator=(const Segment &) = delete;

  /// Creates and maps a new segment, replacing any existing one of the same name. Returns nullptr
  /// on failure with a reason in `error`.
  static std::unique_ptr<Segment> create(const std::string & name, uint32_t capacity, std::string & error);

  /// Maps an existing segment read only, validating its header. Returns nullptr if the segment is
  /// missing, too small, or written in a format this build does not understand.
  static std::unique_ptr<Segment> open_readonly(const std::string & name, std::string & error);

  /// Removes the segment's name so that no new reader can open it. Existing mappings survive until
  /// unmapped, which is what makes this safe to call while readers are attached.
  static bool unlink(const std::string & name, std::string & error);

  /// Lists the names of every statistics segment currently present.
  ///
  /// Linux specific: it reads the tmpfs that POSIX shared memory is exposed through, because there
  /// is no portable way to enumerate shm objects.
  static std::vector<std::string> list(std::string & error);

  /// Whether the process that created a segment is still running.
  ///
  /// Decided by whether the writer's advisory lock on the segment is still held, not by asking
  /// about its pid. The kernel drops the lock when the process dies however it dies, and unlike a
  /// pid the answer is meaningful across PID namespaces, which matters as soon as writers and
  /// readers live in different containers.
  ///
  /// Returns true for a segment that cannot be opened at all, because a segment we cannot inspect
  /// is one we must not reclaim.
  static bool writer_alive(const std::string & name);

  Header & header()
  {
    return *header_;
  }

  const Header & header() const
  {
    return *header_;
  }

  Sample & sample(uint64_t index)
  {
    return samples_[index % header_->capacity];
  }

  const Sample & sample(uint64_t index) const
  {
    return samples_[index % header_->capacity];
  }

  const std::string & name() const
  {
    return name_;
  }

  size_t size() const
  {
    return size_;
  }

  bool read_only() const
  {
    return read_only_;
  }

  /// Whether this object will unlink the segment when destroyed. True for the creator only.
  bool owns_name() const
  {
    return owns_name_;
  }

  void release_ownership()
  {
    owns_name_ = false;
  }

  /// Whether the writer of this segment is still running. Meaningful on a reader's mapping; a
  /// writer always sees its own lock and so always reports itself alive.
  bool writer_alive() const;

private:
  Segment(std::string name, int fd, void * mapping, size_t size, bool read_only, bool owns_name);

  std::string name_;
  /// Held open for the life of the segment rather than closed after mapping, because the writer's
  /// advisory lock lives on it and is what tells readers the writer is still there.
  int fd_ = -1;
  void * mapping_ = nullptr;
  size_t size_ = 0;
  bool read_only_ = true;
  bool owns_name_ = false;
  Header * header_ = nullptr;
  Sample * samples_ = nullptr;
};

}  // namespace topic_stats_shm

#endif  // TOPIC_STATS_SHM__SEGMENT_HPP_
