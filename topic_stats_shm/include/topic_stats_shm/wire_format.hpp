// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef TOPIC_STATS_SHM__WIRE_FORMAT_HPP_
#define TOPIC_STATS_SHM__WIRE_FORMAT_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

#include "topic_stats_core/types.hpp"

namespace topic_stats_shm
{

/// On-disk layout of a statistics segment.
///
/// Two processes built at different times must agree on this byte for byte, so everything here is
/// fixed size, explicitly padded, and version stamped. A reader that does not recognise the version
/// or the record size refuses the segment rather than guessing.
///
/// The design is a single-producer ring with per-slot sequence numbers. A snapshot is appended as a
/// run of samples; readers drain from wherever they left off. A ring rather than a single latched
/// snapshot because the core consumes measurements when it snapshots, so a reader that falls behind
/// would otherwise lose them permanently rather than merely late.

inline constexpr char kMagic[8] = {'T', 'O', 'P', 'I', 'C', 'S', 'T', 'A'};

/// Bump on any layout change. Readers reject anything they do not recognise.
///
/// Version 2 replaced bare pids with (pid namespace, pid) identities and moved liveness onto an
/// advisory lock, because a bare pid means nothing to a reader in a different container.
inline constexpr uint32_t kFormatVersion = 2;

/// Including the terminating NUL. Names longer than this are truncated and flagged rather than
/// dropped: a truncated name is still useful for diagnosis, a missing sample is not.
inline constexpr size_t kMaxNodeNameSize = 80;
inline constexpr size_t kMaxTopicNameSize = 120;

/// Wire values for topic_stats_core::StatKind.
///
/// Deliberately not the enum's own numeric values: reordering the enum would silently reinterpret
/// every existing segment. A reader that meets an unknown value skips the sample.
enum WireStat : uint8_t
{
  kWirePublishedPeriod = 0,
  kWireReceivedPeriod = 1,
  kWireTakeAge = 2,
  kWireCallbackDuration = 3,
};

enum WireFlags : uint8_t
{
  kFlagNone = 0,
  kFlagNodeNameTruncated = 1 << 0,
  kFlagTopicNameTruncated = 1 << 1,
};

/// One statistic. Fixed size so that ring slots are addressable by index.
struct Sample
{
  /// Zero while the slot is being written, otherwise the ring index of this sample plus one.
  /// Checked before and after reading the payload to detect a writer lapping a reader mid-copy.
  std::atomic<uint64_t> sequence;
  int64_t mean_ns;
  int64_t min_ns;
  int64_t max_ns;
  /// Which snapshot this sample belongs to. A reader that falls behind can still reconstruct each
  /// snapshot separately instead of merging several periods into one report.
  uint64_t snapshot_id;
  /// System clock of the snapshot, so each reconstructed report carries the time it was taken
  /// rather than the time it happened to be read.
  int64_t snapshot_ns;
  uint32_t window_count;
  uint8_t stat;
  uint8_t flags;
  uint8_t reserved[2];
  char node_name[kMaxNodeNameSize];
  char topic_name[kMaxTopicNameSize];
};

struct Header
{
  char magic[8];
  uint32_t format_version;
  /// sizeof(Sample), so a mismatched build is caught even if the version was not bumped.
  uint32_t sample_size;
  uint32_t capacity;
  uint32_t header_size;
  /// Whoever created the segment. Recorded for diagnosis only: liveness comes from the advisory
  /// lock the writer holds on the segment, because a pid from another PID namespace is either
  /// meaningless or, worse, matches an unrelated local process.
  int64_t creator_pid;
  /// Inode of the writer's PID namespace, which is unique host-wide. Together with the pid it
  /// identifies a process across containers that share /dev/shm through --ipc=host.
  uint64_t creator_pid_namespace;
  /// Total samples ever written. The ring index of the next write is this modulo capacity.
  std::atomic<uint64_t> write_count;
  std::atomic<uint64_t> snapshot_count;
  /// System clock of the most recent snapshot, for staleness reporting.
  std::atomic<int64_t> last_snapshot_ns;
  char reserved[192];
};

static_assert(sizeof(Sample) == 256, "Sample layout changed; bump kFormatVersion");
static_assert(sizeof(Header) == 256, "Header layout changed; bump kFormatVersion");
static_assert(alignof(Header) <= 8, "Header must not need over-aligned storage");
static_assert(
  std::atomic<uint64_t>::is_always_lock_free,
  "Segment atomics are shared between processes, so they must not be backed by a lock");
static_assert(
  std::atomic<int64_t>::is_always_lock_free,
  "Segment atomics are shared between processes, so they must not be backed by a lock");

/// Number of samples the ring holds by default.
///
/// 256 samples is 64KiB of payload. Kept small on purpose: /dev/shm is commonly a modest tmpfs
/// shared with the DDS implementation's own segments, and this is instrumentation, not the payload.
inline constexpr uint32_t kDefaultCapacity = 256;

inline size_t segment_size(uint32_t capacity)
{
  return sizeof(Header) + static_cast<size_t>(capacity) * sizeof(Sample);
}

/// Identifies a writing process in a way that survives container boundaries.
///
/// A bare pid does not: every PID namespace numbers from 1, so two containers running a handful of
/// nodes each will collide in the low tens almost immediately. The PID namespace's inode is
/// allocated host-wide and is unique per namespace, so the pair is unambiguous wherever /dev/shm is
/// shared.
struct ProcessIdentity
{
  uint64_t pid_namespace = 0;
  int64_t pid = 0;
};

inline bool operator==(const ProcessIdentity & lhs, const ProcessIdentity & rhs)
{
  return lhs.pid_namespace == rhs.pid_namespace && lhs.pid == rhs.pid;
}

/// This process's identity. The namespace is zero if it could not be determined, which only makes
/// names less unique, never wrong within a single namespace.
ProcessIdentity current_process_identity();

/// Segments are named "topic_stats.<pid namespace>.<pid>" so that a reader can enumerate every
/// producer on the machine, and can tell which process in which container each belongs to without
/// opening it.
inline constexpr char kSegmentPrefix[] = "topic_stats.";

std::string segment_name_for(const ProcessIdentity & identity);

/// Extracts the identity from a segment name. Returns false if the name is not one of ours.
bool identity_from_segment_name(const std::string & name, ProcessIdentity & identity);

bool to_wire_stat(topic_stats_core::StatKind kind, uint8_t & wire);
bool from_wire_stat(uint8_t wire, topic_stats_core::StatKind & kind);

/// Copies a string into a fixed buffer, always NUL terminating. Returns true if it was truncated.
bool copy_bounded(const std::string & source, char * destination, size_t destination_size);

}  // namespace topic_stats_shm

#endif  // TOPIC_STATS_SHM__WIRE_FORMAT_HPP_
