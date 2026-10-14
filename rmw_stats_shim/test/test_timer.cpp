// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include "rmw_stats_shim/timer.hpp"

using rmw_stats_shim::Timer;
using std::chrono_literals::operator""ms;

TEST(Timer, StopWithoutStartIsNoop)
{
  Timer timer([] {}, 10ms);
  timer.stop();
  EXPECT_FALSE(timer.isRunning());
}

TEST(Timer, StopTwiceIsNoop)
{
  Timer timer([] {}, 10ms);
  timer.start();
  timer.stop();
  timer.stop();
  EXPECT_FALSE(timer.isRunning());
}

TEST(Timer, DestroyAfterStop)
{
  auto timer = std::make_unique<Timer>([] {}, 10ms);
  timer->start();
  timer->stop();
  EXPECT_NO_THROW(timer.reset());
}

TEST(Timer, RestartAfterStop)
{
  Timer timer([] {}, 10ms);
  timer.start();
  timer.stop();
  timer.start();
  EXPECT_TRUE(timer.isRunning());
  timer.stop();
}
