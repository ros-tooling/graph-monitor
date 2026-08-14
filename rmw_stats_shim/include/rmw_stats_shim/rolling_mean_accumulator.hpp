// SPDX-FileCopyrightText: 2020 PAL Robotics S.L.
// SPDX-FileCopyrightText: 2024 Bonsai Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
#ifndef RMW_STATS_SHIM__ROLLING_MEAN_ACCUMULATOR_HPP_
#define RMW_STATS_SHIM__ROLLING_MEAN_ACCUMULATOR_HPP_

#include <cassert>
#include <cstddef>
#include <vector>

/* Copied from rcppmath, but modified to give access to the current window size. */

/**
 * \brief Computes the mean of the last accumulated elements.
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

  size_t dataCount() const
  {
    return buffer_filled_ * buffer_.size() + !buffer_filled_ * next_insert_;
  }

private:
  std::vector<T> buffer_;
  size_t next_insert_{0};
  T sum_{0};
  bool buffer_filled_{false};
};

#endif  // RMW_STATS_SHIM__ROLLING_MEAN_ACCUMULATOR_HPP__
