// SPDX-FileCopyrightText: 2026 Rahul Seervi
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <algorithm>
#include <deque>
#include <random>

#include "rmw_stats_shim/rolling_mean_accumulator.hpp"

TEST(RollingMeanAccumulator, MinMaxOfPartialWindow)
{
  RollingMeanAccumulator<int> acc(4);
  acc.accumulate(5);
  acc.accumulate(1);
  acc.accumulate(9);

  EXPECT_EQ(acc.dataCount(), 3u);
  EXPECT_EQ(acc.getRollingMean(), 5);
  EXPECT_EQ(acc.getRollingMin(), 1);
  EXPECT_EQ(acc.getRollingMax(), 9);
}

TEST(RollingMeanAccumulator, MinMaxDropValuesThatLeaveTheWindow)
{
  RollingMeanAccumulator<int> acc(3);
  for (int val : {5, 1, 9, 4, 6}) {
    acc.accumulate(val);
  }
  // Window is now {9, 4, 6}
  EXPECT_EQ(acc.dataCount(), 3u);
  EXPECT_EQ(acc.getRollingMin(), 4);
  EXPECT_EQ(acc.getRollingMax(), 9);

  acc.accumulate(2);
  // Window is now {4, 6, 2}
  EXPECT_EQ(acc.getRollingMin(), 2);
  EXPECT_EQ(acc.getRollingMax(), 6);
}

TEST(RollingMeanAccumulator, MinMaxTrackedAcrossQueries)
{
  RollingMeanAccumulator<int> acc(3);
  for (int val : {5, 1, 9}) {
    acc.accumulate(val);
  }
  EXPECT_EQ(acc.getRollingMin(), 1);
  EXPECT_EQ(acc.getRollingMax(), 9);

  // Ties and new extremes replace the tracked ones without leaving the window.
  acc.accumulate(1);
  // Window is now {1, 9, 1}
  EXPECT_EQ(acc.getRollingMin(), 1);
  EXPECT_EQ(acc.getRollingMax(), 9);

  // The older 1 leaves, but the newer one is still in the window.
  acc.accumulate(3);
  // Window is now {9, 1, 3}
  EXPECT_EQ(acc.getRollingMin(), 1);
  EXPECT_EQ(acc.getRollingMax(), 9);

  // The maximum leaves without being queried in between.
  acc.accumulate(4);
  acc.accumulate(2);
  // Window is now {3, 4, 2}
  EXPECT_EQ(acc.getRollingMin(), 2);
  EXPECT_EQ(acc.getRollingMax(), 4);
}

TEST(RollingMeanAccumulator, MinMaxOfMonotonicValues)
{
  RollingMeanAccumulator<int> acc(4);
  for (int val = 0; val < 20; ++val) {
    acc.accumulate(val);
    EXPECT_EQ(acc.getRollingMin(), std::max(0, val - 3));
    EXPECT_EQ(acc.getRollingMax(), val);
  }
  for (int val = 20; val > 0; --val) {
    acc.accumulate(val);
  }
  // Window is now {4, 3, 2, 1}
  EXPECT_EQ(acc.getRollingMin(), 1);
  EXPECT_EQ(acc.getRollingMax(), 4);
}

TEST(RollingMeanAccumulator, MinMaxMatchFullSearch)
{
  const size_t window = 7;
  RollingMeanAccumulator<int> acc(window);
  std::deque<int> recent;
  std::mt19937 gen(42);
  std::uniform_int_distribution<int> value(-5, 5);
  std::bernoulli_distribution query(0.3);

  for (int i = 0; i < 2000; ++i) {
    const int val = value(gen);
    acc.accumulate(val);
    recent.push_back(val);
    if (recent.size() > window) {
      recent.pop_front();
    }
    if (query(gen)) {
      ASSERT_EQ(acc.getRollingMin(), *std::min_element(recent.begin(), recent.end())) << "after " << i;
      ASSERT_EQ(acc.getRollingMax(), *std::max_element(recent.begin(), recent.end())) << "after " << i;
    }
  }
}
