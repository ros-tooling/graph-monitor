// SPDX-FileCopyrightText: 2026 Rahul Seervi
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

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
