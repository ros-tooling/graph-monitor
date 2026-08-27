// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <thread>

#include "gmock/gmock.h"
#include "rosgraph_monitor/event.hpp"

using rosgraph_monitor::Event;

namespace
{

/// Upper bound on how long an already-satisfied wait may take to return.
constexpr auto kImmediate = std::chrono::milliseconds(100);

std::chrono::nanoseconds time_call(const std::function<void()> & call)
{
  const auto start = std::chrono::steady_clock::now();
  call();
  return std::chrono::steady_clock::now() - start;
}

}  // namespace

TEST(EventTest, SetBeforeWaitForReturnsImmediately)
{
  Event event;
  event.set();
  bool result = false;
  const auto elapsed = time_call([&]() { result = event.wait_for(std::chrono::seconds(10)); });
  EXPECT_TRUE(result);
  EXPECT_LT(elapsed, kImmediate);
}

TEST(EventTest, SetBeforeWaitUntilReturnsImmediately)
{
  Event event;
  event.set();
  bool result = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  const auto elapsed = time_call([&]() { result = event.wait_until(deadline); });
  EXPECT_TRUE(result);
  EXPECT_LT(elapsed, kImmediate);
}

TEST(EventTest, SetBeforeWaitReturnsImmediately)
{
  Event event;
  event.set();
  const auto elapsed = time_call([&]() { event.wait(); });
  EXPECT_LT(elapsed, kImmediate);
}

TEST(EventTest, SetDuringWaitForWakesPromptly)
{
  Event event;
  std::chrono::nanoseconds elapsed{0};
  bool result = false;
  std::thread waiter([&]() { elapsed = time_call([&]() { result = event.wait_for(std::chrono::seconds(10)); }); });
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  event.set();
  waiter.join();
  EXPECT_TRUE(result);
  EXPECT_LT(elapsed, std::chrono::seconds(2));
}

TEST(EventTest, SetDuringWaitUntilWakesPromptly)
{
  Event event;
  std::chrono::nanoseconds elapsed{0};
  bool result = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  std::thread waiter([&]() { elapsed = time_call([&]() { result = event.wait_until(deadline); }); });
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  event.set();
  waiter.join();
  EXPECT_TRUE(result);
  EXPECT_LT(elapsed, std::chrono::seconds(2));
}

TEST(EventTest, WaitForTimesOutWhenNeverSet)
{
  Event event;
  bool result = true;
  const auto elapsed = time_call([&]() { result = event.wait_for(std::chrono::milliseconds(100)); });
  EXPECT_FALSE(result);
  EXPECT_GE(elapsed, std::chrono::milliseconds(100));
}

TEST(EventTest, WaitUntilTimesOutWhenNeverSet)
{
  Event event;
  EXPECT_FALSE(event.wait_until(std::chrono::steady_clock::now() + std::chrono::milliseconds(100)));
}

TEST(EventTest, ClearAllowsReuse)
{
  Event event;
  event.set();
  EXPECT_TRUE(event.is_set());
  event.clear();
  EXPECT_FALSE(event.is_set());
  EXPECT_FALSE(event.wait_for(std::chrono::milliseconds(10)));
  event.set();
  EXPECT_TRUE(event.wait_for(std::chrono::seconds(10)));
}

TEST(EventTest, CheckAndClearReturnsAndClears)
{
  Event event;
  EXPECT_FALSE(event.check_and_clear());
  event.set();
  EXPECT_TRUE(event.check_and_clear());
  EXPECT_FALSE(event.is_set());
  EXPECT_FALSE(event.check_and_clear());
}

TEST(EventTest, UntimedWaitWakesOnSet)
{
  Event event;
  std::future<void> waiter = std::async(std::launch::async, [&event]() { event.wait(); });
  EXPECT_EQ(waiter.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);
  event.set();
  ASSERT_EQ(waiter.wait_for(std::chrono::seconds(5)), std::future_status::ready);
}

/// Probabilistic stress test for the lost-wakeup race between set() and a waiter entering the wait set.
/// A waiter that misses a notification stalls until its timeout expires, so a large worst-case elapsed time is the symptom.
TEST(EventTest, SetIsNotLostWhileWaiterEntersWaitSet)
{
  constexpr int kIterations = 4000;
  constexpr auto kTimeout = std::chrono::seconds(2);
  std::chrono::nanoseconds worst{0};
  /// Sink that keeps the delay loop below from being optimized away.
  std::atomic_int spin_sink{0};

  for (int iteration = 0; iteration < kIterations; ++iteration) {
    Event event;
    std::chrono::nanoseconds elapsed{0};
    bool result = false;
    std::thread waiter([&]() { elapsed = time_call([&]() { result = event.wait_for(kTimeout); }); });

    // Aim the set() at the window in which the waiter holds the lock and evaluates the predicate.
    std::this_thread::yield();
    for (int spin = 0; spin < 100; ++spin) {
      spin_sink.fetch_add(1, std::memory_order_relaxed);
    }
    event.set();
    waiter.join();

    ASSERT_TRUE(result) << "iteration " << iteration;
    ASSERT_LT(elapsed, std::chrono::milliseconds(500)) << "iteration " << iteration;
    worst = std::max(worst, elapsed);
  }

  EXPECT_LT(worst, std::chrono::milliseconds(500));
}
