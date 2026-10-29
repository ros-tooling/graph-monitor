// SPDX-FileCopyrightText: 2020 PAL Robotics S.L.
// SPDX-FileCopyrightText: 2024 Bonsai Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
#ifndef RMW_STATS_SHIM__ROLLING_MEAN_ACCUMULATOR_HPP_
#define RMW_STATS_SHIM__ROLLING_MEAN_ACCUMULATOR_HPP_

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <vector>

/* Copied from rcppmath, but modified to give access to the current window size, minimum and maximum. */

/**
 * \brief Computes the mean, minimum and maximum of the last accumulated elements.
 *
 * This is a simplified version of boost's rolling mean accumulator,
 * written to avoid dragging in boost dependencies.
 *
 */
template <typename T>
class RollingMeanAccumulator
{
public:
  /**
   * Constructs the rolling mean accumulator with a specified window size.
   *
   * \param[in] rolling_window_size The unsigned integral length of the accumulator's window length.
   */
  explicit RollingMeanAccumulator(size_t rolling_window_size)
  : buffer_(rolling_window_size)
  {}

  /**
   * Collects the provided value in the accumulator's buffer.
   *
   * \param[in] val The value to accumulate.
   */
  void accumulate(T val)
  {
    // A value that ties or beats the tracked minimum/maximum takes its place.
    // Overwriting the slot that held it with anything else leaves the extreme unknown,
    // so the next query searches the window again.
    if (!min_stale_) {
      if (val <= buffer_[min_index_]) {
        min_index_ = next_insert_;
      } else if (min_index_ == next_insert_) {
        min_stale_ = true;
      }
    }
    if (!max_stale_) {
      if (val >= buffer_[max_index_]) {
        max_index_ = next_insert_;
      } else if (max_index_ == next_insert_) {
        max_stale_ = true;
      }
    }
    sum_ -= buffer_[next_insert_];
    sum_ += val;
    buffer_[next_insert_] = val;
    next_insert_++;
    buffer_filled_ |= next_insert_ >= buffer_.size();
    next_insert_ = next_insert_ % buffer_.size();
  }

  /**
   * Calculates the rolling mean accumulated insofar.
   *
   * \return Rolling mean of the accumulated values.
   */
  T getRollingMean() const
  {
    size_t valid_data_count = dataCount();
    assert(valid_data_count > 0);
    return sum_ / valid_data_count;
  }

  /**
   * Returns the smallest value in the current window.
   *
   * This is O(1) unless the tracked minimum has left the window since the last query,
   * in which case the window is searched once.
   *
   * \return Minimum of the accumulated values.
   */
  T getRollingMin() const
  {
    assert(dataCount() > 0);
    if (min_stale_) {
      min_index_ = std::min_element(buffer_.begin(), buffer_.begin() + dataCount()) - buffer_.begin();
      min_stale_ = false;
    }
    return buffer_[min_index_];
  }

  /**
   * Returns the largest value in the current window.
   *
   * This is O(1) unless the tracked maximum has left the window since the last query,
   * in which case the window is searched once.
   *
   * \return Maximum of the accumulated values.
   */
  T getRollingMax() const
  {
    assert(dataCount() > 0);
    if (max_stale_) {
      max_index_ = std::max_element(buffer_.begin(), buffer_.begin() + dataCount()) - buffer_.begin();
      max_stale_ = false;
    }
    return buffer_[max_index_];
  }

  size_t dataCount() const
  {
    return buffer_filled_ * buffer_.size() + !buffer_filled_ * next_insert_;
  }

private:
  std::vector<T> buffer_;
  size_t next_insert_{0};
  T sum_{0};
  bool buffer_filled_{false};
  // Indices of the window's minimum and maximum, valid while the matching stale flag is false.
  // The getters refresh them lazily, so they are mutable.
  mutable size_t min_index_{0};
  mutable size_t max_index_{0};
  mutable bool min_stale_{true};
  mutable bool max_stale_{true};
};

#endif  // RMW_STATS_SHIM__ROLLING_MEAN_ACCUMULATOR_HPP__
