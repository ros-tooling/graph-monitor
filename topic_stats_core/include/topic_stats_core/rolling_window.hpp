// SPDX-FileCopyrightText: 2020 PAL Robotics S.L.
// SPDX-FileCopyrightText: 2024 Bonsai Robotics, Inc.
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef TOPIC_STATS_CORE__ROLLING_WINDOW_HPP_
#define TOPIC_STATS_CORE__ROLLING_WINDOW_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace topic_stats_core
{

/// Fixed-size rolling window over the most recent N measurements.
///
/// Descended from rcppmath's RollingMeanAccumulator, extended to report min and max and to make
/// the current sample count observable.
///
/// The mean is maintained incrementally, but min and max are computed by scanning the buffer on
/// demand. That is deliberate: a rolling min/max needs monotonic deques to be O(1), and the scan
/// is only paid once per reporting period over a window of a few dozen entries, whereas the
/// insertion path runs on every message.
///
/// Not thread safe. The collector owns the synchronization.
template <typename T>
class RollingWindow
{
public:
  explicit RollingWindow(size_t window_size)
  : buffer_(window_size == 0 ? 1 : window_size)
  {}

  void accumulate(T value)
  {
    sum_ -= buffer_[next_insert_];
    sum_ += value;
    buffer_[next_insert_] = value;
    next_insert_ = (next_insert_ + 1) % buffer_.size();
    if (count_ < buffer_.size()) {
      ++count_;
    }
  }

  /// Number of measurements currently in the window, saturating at the window size.
  size_t count() const
  {
    return count_;
  }

  bool empty() const
  {
    return count_ == 0;
  }

  size_t window_size() const
  {
    return buffer_.size();
  }

  /// Mean of the window. Returns T{} when empty rather than dividing by zero, so that callers
  /// which forget to check cannot trip undefined behaviour.
  T mean() const
  {
    if (count_ == 0) {
      return T{};
    }
    return sum_ / static_cast<int64_t>(count_);
  }

  T min() const
  {
    if (count_ == 0) {
      return T{};
    }
    return *std::min_element(buffer_.begin(), buffer_.begin() + count_);
  }

  T max() const
  {
    if (count_ == 0) {
      return T{};
    }
    return *std::max_element(buffer_.begin(), buffer_.begin() + count_);
  }

  void reset()
  {
    std::fill(buffer_.begin(), buffer_.end(), T{});
    next_insert_ = 0;
    count_ = 0;
    sum_ = T{};
  }

private:
  // Entries [0, count_) are always the valid ones because the buffer fills front to back before
  // it ever wraps, which is what lets min()/max() scan a prefix rather than the whole buffer.
  std::vector<T> buffer_;
  size_t next_insert_{0};
  size_t count_{0};
  T sum_{};
};

}  // namespace topic_stats_core

#endif  // TOPIC_STATS_CORE__ROLLING_WINDOW_HPP_
