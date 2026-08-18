// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "topic_stats_shm/segment.hpp"

#include <dirent.h>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <memory>
#include <new>
#include <string>
#include <utility>
#include <vector>

namespace topic_stats_shm
{

namespace
{

/// Where Linux exposes POSIX shared memory objects. Only used for enumeration; everything else
/// goes through shm_open.
constexpr char kShmDirectory[] = "/dev/shm";

std::string errno_message(const char * what)
{
  return std::string(what) + ": " + std::strerror(errno);
}

std::string shm_path(const std::string & name)
{
  return "/" + name;
}

}  // namespace

Segment::Segment(std::string name, int fd, void * mapping, size_t size, bool read_only, bool owns_name)
: name_(std::move(name))
, fd_(fd)
, mapping_(mapping)
, size_(size)
, read_only_(read_only)
, owns_name_(owns_name)
, header_(static_cast<Header *>(mapping))
, samples_(reinterpret_cast<Sample *>(static_cast<char *>(mapping) + sizeof(Header)))
{}

Segment::~Segment()
{
  if (mapping_ != nullptr) {
    munmap(mapping_, size_);
  }
  if (owns_name_) {
    std::string ignored;
    unlink(name_, ignored);
  }
  // Closing releases the writer's advisory lock, which is how a reader learns the writer is gone.
  // The kernel does the same on process death, however abrupt.
  if (fd_ >= 0) {
    close(fd_);
  }
}

std::unique_ptr<Segment> Segment::create(const std::string & name, uint32_t capacity, std::string & error)
{
  if (capacity == 0) {
    error = "capacity must be greater than zero";
    return nullptr;
  }

  // Opened without O_EXCL so that a segment left behind by a previous process can be reclaimed,
  // but deliberately not unlinked first: unlinking before knowing whether someone is using it is
  // how one container would destroy another's segment.
  const int fd = shm_open(shm_path(name).c_str(), O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
  if (fd < 0) {
    error = errno_message("shm_open");
    return nullptr;
  }

  // Held for the life of the process. It both claims the name against any other writer and is what
  // readers probe to decide whether this process is still running.
  if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
    error = (errno == EWOULDBLOCK) ? "segment is already owned by a running writer" : errno_message("flock");
    close(fd);
    return nullptr;
  }

  const size_t size = segment_size(capacity);
  // Truncating to zero first guarantees the zero fill that the ring protocol's initial state
  // depends on. Resizing an existing segment to the same size would otherwise keep stale contents.
  if (ftruncate(fd, 0) != 0 || ftruncate(fd, static_cast<off_t>(size)) != 0) {
    error = errno_message("ftruncate");
    close(fd);
    shm_unlink(shm_path(name).c_str());
    return nullptr;
  }

  void * mapping = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (mapping == MAP_FAILED) {
    error = errno_message("mmap");
    close(fd);
    shm_unlink(shm_path(name).c_str());
    return nullptr;
  }

  const auto identity = current_process_identity();
  auto * header = static_cast<Header *>(mapping);
  std::memcpy(header->magic, kMagic, sizeof(kMagic));
  header->format_version = kFormatVersion;
  header->sample_size = sizeof(Sample);
  header->capacity = capacity;
  header->header_size = sizeof(Header);
  header->creator_pid = identity.pid;
  header->creator_pid_namespace = identity.pid_namespace;

  return std::unique_ptr<Segment>(new Segment(name, fd, mapping, size, false, true));
}

std::unique_ptr<Segment> Segment::open_readonly(const std::string & name, std::string & error)
{
  const int fd = shm_open(shm_path(name).c_str(), O_RDONLY, 0);
  if (fd < 0) {
    error = errno_message("shm_open");
    return nullptr;
  }

  struct stat info{};
  if (fstat(fd, &info) != 0) {
    error = errno_message("fstat");
    close(fd);
    return nullptr;
  }
  const size_t actual_size = static_cast<size_t>(info.st_size);
  if (actual_size < sizeof(Header)) {
    // Most likely caught mid-creation, between shm_open and ftruncate.
    error = "segment is smaller than its header";
    close(fd);
    return nullptr;
  }

  void * mapping = mmap(nullptr, actual_size, PROT_READ, MAP_SHARED, fd, 0);
  if (mapping == MAP_FAILED) {
    error = errno_message("mmap");
    close(fd);
    return nullptr;
  }

  auto * header = static_cast<Header *>(mapping);
  auto reject = [&](const std::string & reason) {
    error = reason;
    munmap(mapping, actual_size);
    close(fd);
    return nullptr;
  };

  if (std::memcmp(header->magic, kMagic, sizeof(kMagic)) != 0) {
    return reject("segment does not carry the statistics magic");
  }
  if (header->format_version != kFormatVersion) {
    return reject(
      "segment format version " + std::to_string(header->format_version) + " is not version " +
      std::to_string(kFormatVersion));
  }
  if (header->sample_size != sizeof(Sample)) {
    // Same version but a different record size means the two builds disagree about the layout,
    // which the version alone failed to catch.
    return reject("segment record size does not match this build");
  }
  if (header->header_size != sizeof(Header)) {
    return reject("segment header size does not match this build");
  }
  if (header->capacity == 0) {
    return reject("segment declares zero capacity");
  }
  if (actual_size < segment_size(header->capacity)) {
    return reject("segment is smaller than its declared capacity");
  }

  return std::unique_ptr<Segment>(new Segment(name, fd, mapping, actual_size, true, false));
}

bool Segment::unlink(const std::string & name, std::string & error)
{
  if (shm_unlink(shm_path(name).c_str()) != 0) {
    error = errno_message("shm_unlink");
    return false;
  }
  return true;
}

std::vector<std::string> Segment::list(std::string & error)
{
  std::vector<std::string> names;
  DIR * dir = opendir(kShmDirectory);
  if (dir == nullptr) {
    error = errno_message("opendir");
    return names;
  }
  while (const dirent * entry = readdir(dir)) {
    const std::string name = entry->d_name;
    ProcessIdentity identity;
    if (identity_from_segment_name(name, identity)) {
      names.push_back(name);
    }
  }
  closedir(dir);
  return names;
}

namespace
{

/// Tries to take the writer's lock. Success means nobody holds it, so the writer is gone.
bool lock_is_free(int fd)
{
  if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
    return false;
  }
  flock(fd, LOCK_UN);
  return true;
}

}  // namespace

bool Segment::writer_alive() const
{
  if (fd_ < 0) {
    return true;
  }
  return !lock_is_free(fd_);
}

bool Segment::writer_alive(const std::string & name)
{
  // Read only is enough: flock cares about the open file description, not the access mode.
  const int fd = shm_open(shm_path(name).c_str(), O_RDONLY, 0);
  if (fd < 0) {
    // Gone, or belongs to a user we cannot read. Either way we must not treat it as reclaimable
    // on the strength of a failed open.
    return errno != ENOENT;
  }
  const bool alive = !lock_is_free(fd);
  close(fd);
  return alive;
}

}  // namespace topic_stats_shm
