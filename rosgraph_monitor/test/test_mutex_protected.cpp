// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <atomic>
#include <chrono>
#include <future>
#include <optional>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>

#include "gmock/gmock.h"
#include "rosgraph_monitor/mutex_protected.hpp"

using rosgraph_monitor::MutexProtected;

namespace
{

struct Value
{
  Value(int number_in, std::string text_in)
  : number(number_in)
  , text(std::move(text_in))
  {}

  int sum(int addend) const
  {
    return number + addend;
  }

  int number;
  std::string text;
};

using ProtectedValue = MutexProtected<Value>;

/// Holds a Value that only ever gets touched under the lock, including from const methods.
class Owner
{
public:
  explicit Owner(int number)
  : value_(number, "owner")
  {}

  int number() const
  {
    auto guard = value_.lock();
    return guard->sum(0);
  }

  void set_number(int number)
  {
    auto guard = value_.lock();
    guard->number = number;
  }

private:
  ProtectedValue value_;
};

}  // namespace

TEST(MutexProtectedTest, LockMutatesThroughArrowAndStar)
{
  ProtectedValue protected_value(1, "one");
  {
    auto guard = protected_value.lock();
    guard->number = 2;
    (*guard).text = "two";
  }
  auto guard = protected_value.lock();
  EXPECT_EQ(guard->number, 2);
  EXPECT_EQ((*guard).text, "two");
}

TEST(MutexProtectedTest, ConstLockReadsThroughArrowAndStar)
{
  const ProtectedValue protected_value(7, "seven");
  auto guard = protected_value.lock();
  static_assert(std::is_const_v<std::remove_reference_t<decltype(*guard)>>);
  EXPECT_EQ(guard->sum(3), 10);
  EXPECT_EQ((*guard).text, "seven");
}

TEST(MutexProtectedTest, ConstMethodOfOwnerCanLock)
{
  Owner owner(3);
  const Owner & const_owner = owner;
  EXPECT_EQ(const_owner.number(), 3);
  owner.set_number(4);
  EXPECT_EQ(const_owner.number(), 4);
}

TEST(MutexProtectedTest, GuardExcludesSecondLocker)
{
  ProtectedValue protected_value(0, "zero");
  std::optional<ProtectedValue::Guard> guard(protected_value.lock());

  std::atomic_bool started{false};
  std::atomic_bool acquired{false};
  std::future<int> contender = std::async(std::launch::async, [&]() {
    started.store(true);
    auto second = protected_value.lock();
    acquired.store(true);
    return second->number;
  });

  while (!started.load()) {
    std::this_thread::yield();
  }
  EXPECT_EQ(contender.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);
  EXPECT_FALSE(acquired.load());

  (*guard)->number = 5;
  guard.reset();

  ASSERT_EQ(contender.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  EXPECT_TRUE(acquired.load());
  EXPECT_EQ(contender.get(), 5);
}

TEST(MutexProtectedTest, ConstGuardExcludesSecondLocker)
{
  const ProtectedValue protected_value(9, "nine");
  std::optional<ProtectedValue::ConstGuard> guard(protected_value.lock());

  std::atomic_bool started{false};
  std::future<int> contender = std::async(std::launch::async, [&]() {
    started.store(true);
    auto second = protected_value.lock();
    return second->number;
  });

  while (!started.load()) {
    std::this_thread::yield();
  }
  EXPECT_EQ(contender.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);

  guard.reset();

  ASSERT_EQ(contender.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  EXPECT_EQ(contender.get(), 9);
}

TEST(MutexProtectedTest, VariadicConstruction)
{
  ProtectedValue protected_value(42, "forty-two");
  auto guard = protected_value.lock();
  EXPECT_EQ(guard->number, 42);
  EXPECT_EQ(guard->text, "forty-two");
}

TEST(MutexProtectedTest, CopyConstruction)
{
  const Value seed(11, "eleven");
  ProtectedValue protected_value(seed);
  auto guard = protected_value.lock();
  EXPECT_EQ(guard->number, 11);
  EXPECT_EQ(guard->text, "eleven");
}
