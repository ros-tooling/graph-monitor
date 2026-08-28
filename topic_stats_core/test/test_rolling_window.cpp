// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>

#include "topic_stats_core/rolling_window.hpp"
#include "topic_stats_core/types.hpp"

using topic_stats_core::Duration;
using topic_stats_core::RollingWindow;

namespace
{
Duration ms(int64_t count)
{
  return std::chrono::duration_cast<Duration>(std::chrono::milliseconds(count));
}
}  // namespace

TEST(RollingWindow, empty_window_reports_nothing)
{
  RollingWindow<Duration> window(4);
  EXPECT_TRUE(window.empty());
  EXPECT_EQ(window.count(), 0u);
  EXPECT_EQ(window.window_size(), 4u);
  // Must not divide by zero, and must not report a meaningful-looking value.
  EXPECT_EQ(window.mean(), Duration{0});
  EXPECT_EQ(window.min(), Duration{0});
  EXPECT_EQ(window.max(), Duration{0});
}

TEST(RollingWindow, single_measurement_is_its_own_mean_min_and_max)
{
  RollingWindow<Duration> window(4);
  window.accumulate(ms(10));
  EXPECT_FALSE(window.empty());
  EXPECT_EQ(window.count(), 1u);
  EXPECT_EQ(window.mean(), ms(10));
  EXPECT_EQ(window.min(), ms(10));
  EXPECT_EQ(window.max(), ms(10));
}

TEST(RollingWindow, partially_filled_window_only_averages_real_measurements)
{
  RollingWindow<Duration> window(10);
  window.accumulate(ms(10));
  window.accumulate(ms(20));
  EXPECT_EQ(window.count(), 2u);
  // Not 30/10. The eight untouched slots must not drag the mean toward zero.
  EXPECT_EQ(window.mean(), ms(15));
  EXPECT_EQ(window.min(), ms(10));
  EXPECT_EQ(window.max(), ms(20));
}

TEST(RollingWindow, count_saturates_at_window_size)
{
  RollingWindow<Duration> window(3);
  for (int i = 0; i < 100; ++i) {
    window.accumulate(ms(5));
  }
  EXPECT_EQ(window.count(), 3u);
  EXPECT_EQ(window.mean(), ms(5));
}

TEST(RollingWindow, wrapping_drops_the_oldest_measurement)
{
  RollingWindow<Duration> window(3);
  window.accumulate(ms(1));
  window.accumulate(ms(2));
  window.accumulate(ms(3));
  EXPECT_EQ(window.mean(), ms(2));

  // Pushes out the 1.
  window.accumulate(ms(10));
  EXPECT_EQ(window.count(), 3u);
  EXPECT_EQ(window.mean(), ms(5));
  EXPECT_EQ(window.min(), ms(2));
  EXPECT_EQ(window.max(), ms(10));
}

TEST(RollingWindow, min_and_max_follow_the_window_rather_than_all_history)
{
  RollingWindow<Duration> window(2);
  window.accumulate(ms(100));
  window.accumulate(ms(1));
  EXPECT_EQ(window.min(), ms(1));
  EXPECT_EQ(window.max(), ms(100));

  // Both extremes are now out of the window.
  window.accumulate(ms(50));
  window.accumulate(ms(60));
  EXPECT_EQ(window.min(), ms(50));
  EXPECT_EQ(window.max(), ms(60));
}

TEST(RollingWindow, handles_negative_measurements)
{
  // Take age goes negative when publisher and subscriber clocks disagree. That is information, not
  // an error, so it must survive to the report.
  RollingWindow<Duration> window(2);
  window.accumulate(ms(-10));
  window.accumulate(ms(30));
  EXPECT_EQ(window.mean(), ms(10));
  EXPECT_EQ(window.min(), ms(-10));
  EXPECT_EQ(window.max(), ms(30));
}

TEST(RollingWindow, zero_window_size_is_coerced_to_one)
{
  RollingWindow<Duration> window(0);
  EXPECT_EQ(window.window_size(), 1u);
  window.accumulate(ms(7));
  EXPECT_EQ(window.count(), 1u);
  EXPECT_EQ(window.mean(), ms(7));
}

TEST(RollingWindow, reset_clears_measurements_and_running_sum)
{
  RollingWindow<Duration> window(3);
  window.accumulate(ms(10));
  window.accumulate(ms(20));
  window.reset();
  EXPECT_EQ(window.count(), 0u);
  EXPECT_EQ(window.mean(), Duration{0});

  // The stale sum must not leak into the next measurement.
  window.accumulate(ms(4));
  EXPECT_EQ(window.mean(), ms(4));
}
