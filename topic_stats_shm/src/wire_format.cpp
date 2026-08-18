// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "topic_stats_shm/wire_format.hpp"

#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <string>

namespace topic_stats_shm
{

namespace
{

/// Parses the inode out of what /proc/self/ns/pid points at, which reads "pid:[4026534050]".
bool parse_namespace_inode(const std::string & link, uint64_t & inode)
{
  const size_t open_bracket = link.find('[');
  const size_t close_bracket = link.find(']', open_bracket + 1);
  if (open_bracket == std::string::npos || close_bracket == std::string::npos) {
    return false;
  }
  const std::string digits = link.substr(open_bracket + 1, close_bracket - open_bracket - 1);
  if (digits.empty() || digits.find_first_not_of("0123456789") != std::string::npos) {
    return false;
  }
  try {
    inode = std::stoull(digits);
  } catch (const std::exception &) {
    return false;
  }
  return true;
}

bool parse_unsigned(const std::string & digits, uint64_t & value)
{
  if (digits.empty() || digits.find_first_not_of("0123456789") != std::string::npos) {
    return false;
  }
  try {
    value = std::stoull(digits);
  } catch (const std::exception &) {
    return false;
  }
  return true;
}

}  // namespace

ProcessIdentity current_process_identity()
{
  ProcessIdentity identity;
  identity.pid = static_cast<int64_t>(getpid());

  char buffer[256];
  const ssize_t length = readlink("/proc/self/ns/pid", buffer, sizeof(buffer) - 1);
  if (length > 0) {
    buffer[length] = '\0';
    // A failure here leaves the namespace at zero, which is still correct within one namespace and
    // only loses the ability to tell containers apart.
    parse_namespace_inode(std::string(buffer), identity.pid_namespace);
  }
  return identity;
}

std::string segment_name_for(const ProcessIdentity & identity)
{
  return std::string(kSegmentPrefix) + std::to_string(identity.pid_namespace) + "." + std::to_string(identity.pid);
}

bool identity_from_segment_name(const std::string & name, ProcessIdentity & identity)
{
  const std::string prefix(kSegmentPrefix);
  if (name.size() <= prefix.size() || name.compare(0, prefix.size(), prefix) != 0) {
    return false;
  }
  const std::string remainder = name.substr(prefix.size());
  const size_t separator = remainder.find('.');
  if (separator == std::string::npos) {
    return false;
  }

  uint64_t name_space = 0;
  uint64_t pid = 0;
  if (!parse_unsigned(remainder.substr(0, separator), name_space)) {
    return false;
  }
  if (!parse_unsigned(remainder.substr(separator + 1), pid)) {
    return false;
  }
  identity.pid_namespace = name_space;
  identity.pid = static_cast<int64_t>(pid);
  return true;
}

bool to_wire_stat(topic_stats_core::StatKind kind, uint8_t & wire)
{
  switch (kind) {
    case topic_stats_core::StatKind::PublishedPeriod:
      wire = kWirePublishedPeriod;
      return true;
    case topic_stats_core::StatKind::ReceivedPeriod:
      wire = kWireReceivedPeriod;
      return true;
    case topic_stats_core::StatKind::TakeAge:
      wire = kWireTakeAge;
      return true;
    case topic_stats_core::StatKind::CallbackDuration:
      wire = kWireCallbackDuration;
      return true;
  }
  return false;
}

bool from_wire_stat(uint8_t wire, topic_stats_core::StatKind & kind)
{
  switch (wire) {
    case kWirePublishedPeriod:
      kind = topic_stats_core::StatKind::PublishedPeriod;
      return true;
    case kWireReceivedPeriod:
      kind = topic_stats_core::StatKind::ReceivedPeriod;
      return true;
    case kWireTakeAge:
      kind = topic_stats_core::StatKind::TakeAge;
      return true;
    case kWireCallbackDuration:
      kind = topic_stats_core::StatKind::CallbackDuration;
      return true;
    default:
      // A newer writer measuring something this reader has never heard of. Skipping one sample is
      // the right outcome; refusing the whole segment would not be.
      return false;
  }
}

bool copy_bounded(const std::string & source, char * destination, size_t destination_size)
{
  const size_t copied = std::min(source.size(), destination_size - 1);
  std::memcpy(destination, source.data(), copied);
  std::memset(destination + copied, 0, destination_size - copied);
  return copied < source.size();
}

}  // namespace topic_stats_shm
