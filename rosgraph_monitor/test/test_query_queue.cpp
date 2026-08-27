// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "gmock/gmock.h"
#include "rosgraph_monitor/query_queue.hpp"

using rosgraph_monitor::QueryQueue;
using testing::ElementsAre;
using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

using Queue = QueryQueue<std::string>;

/// How long to wait for a state change that should happen.
constexpr auto kGenerousWait = 5s;
/// How long to wait before asserting that a state change did not happen.
constexpr auto kSettle = 300ms;

/// Polls `predicate` until it holds or `timeout` expires.
template <typename Predicate>
bool wait_for_condition(Predicate predicate, std::chrono::milliseconds timeout = kGenerousWait)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(1ms);
  }
  return predicate();
}

/// Options whose attempts never time out on their own, so the test decides every outcome.
Queue::Options patient_options(size_t max_concurrent = 4)
{
  Queue::Options options;
  options.max_concurrent = max_concurrent;
  options.timeout = 30s;
  options.retry_delay = 30ms;
  return options;
}

/// Options whose attempts time out quickly.
Queue::Options impatient_options(size_t max_concurrent = 4)
{
  Queue::Options options;
  options.max_concurrent = max_concurrent;
  options.timeout = 40ms;
  options.retry_delay = 20ms;
  return options;
}

/// Records every start, and hands the test each attempt's done callback to invoke when it chooses.
class FakeCalls
{
public:
  Queue::StartCall start_call()
  {
    return [this](const std::string & key, Queue::Done done) {
      std::lock_guard<std::mutex> lock(mutex_);
      started_.emplace_back(key, std::move(done));
    };
  }

  size_t total_starts() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return started_.size();
  }

  size_t starts_for(const std::string & key) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<size_t>(
      std::count_if(started_.begin(), started_.end(), [&key](const auto & start) { return start.first == key; }));
  }

  std::vector<std::string> started_keys() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> keys;
    for (const auto & start : started_) {
      keys.push_back(start.first);
    }
    return keys;
  }

  /// @return The done callback of the `nth` start for `key`, counting from zero.
  Queue::Done done_for(const std::string & key, size_t nth = 0) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t seen = 0;
    for (const auto & start : started_) {
      if (start.first == key && seen++ == nth) {
        return start.second;
      }
    }
    throw std::out_of_range("no start recorded for " + key);
  }

private:
  mutable std::mutex mutex_;
  std::vector<std::pair<std::string, Queue::Done>> started_;
};

/// Records every delivered result.
class ResultLog
{
public:
  Queue::OnResult on_result()
  {
    return [this](const std::string & key, std::string result) {
      std::lock_guard<std::mutex> lock(mutex_);
      entries_.emplace_back(key, std::move(result));
    };
  }

  size_t size() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return entries_.size();
  }

  std::vector<std::pair<std::string, std::string>> entries() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return entries_;
  }

private:
  mutable std::mutex mutex_;
  std::vector<std::pair<std::string, std::string>> entries_;
};

TEST(QueryQueueTest, DeliversSuccessfulResult)
{
  ResultLog results;
  FakeCalls calls;
  Queue queue(calls.start_call(), results.on_result(), patient_options());

  queue.request("node_a");
  ASSERT_TRUE(wait_for_condition([&calls] { return calls.starts_for("node_a") == 1; }));

  calls.done_for("node_a")("params_a");
  ASSERT_TRUE(wait_for_condition([&results] { return results.size() == 1; }));

  EXPECT_THAT(results.entries(), ElementsAre(std::make_pair(std::string("node_a"), std::string("params_a"))));
  EXPECT_TRUE(wait_for_condition([&queue] { return queue.in_flight_count() == 0 && queue.waiting_count() == 0; }));
}

TEST(QueryQueueTest, DedupesWhileInFlight)
{
  ResultLog results;
  FakeCalls calls;
  Queue queue(calls.start_call(), results.on_result(), patient_options());

  queue.request("node_a");
  ASSERT_TRUE(wait_for_condition([&calls] { return calls.starts_for("node_a") == 1; }));

  queue.request("node_a");
  queue.request("node_a");
  std::this_thread::sleep_for(kSettle);

  EXPECT_EQ(calls.starts_for("node_a"), 1u);
  EXPECT_EQ(queue.in_flight_count(), 1u);
  EXPECT_EQ(queue.waiting_count(), 0u);
}

TEST(QueryQueueTest, DedupesWhileWaiting)
{
  ResultLog results;
  FakeCalls calls;
  Queue queue(calls.start_call(), results.on_result(), patient_options(1));

  queue.request("node_a");
  ASSERT_TRUE(wait_for_condition([&calls] { return calls.starts_for("node_a") == 1; }));

  queue.request("node_b");
  queue.request("node_b");
  queue.request("node_b");
  ASSERT_TRUE(wait_for_condition([&queue] { return queue.waiting_count() == 1; }));

  calls.done_for("node_a")("params_a");
  ASSERT_TRUE(wait_for_condition([&calls] { return calls.starts_for("node_b") == 1; }));
  std::this_thread::sleep_for(kSettle);

  EXPECT_EQ(calls.starts_for("node_b"), 1u);
  EXPECT_EQ(calls.total_starts(), 2u);
}

TEST(QueryQueueTest, BoundsConcurrentStarts)
{
  ResultLog results;
  FakeCalls calls;
  Queue queue(calls.start_call(), results.on_result(), patient_options(2));

  queue.request("node_a");
  queue.request("node_b");
  queue.request("node_c");
  ASSERT_TRUE(wait_for_condition([&calls] { return calls.total_starts() == 2; }));
  std::this_thread::sleep_for(kSettle);

  EXPECT_EQ(calls.total_starts(), 2u);
  EXPECT_EQ(queue.in_flight_count(), 2u);
  EXPECT_EQ(queue.waiting_count(), 1u);

  calls.done_for("node_a")("params_a");
  ASSERT_TRUE(wait_for_condition([&calls] { return calls.total_starts() == 3; }));

  EXPECT_THAT(calls.started_keys(), ElementsAre("node_a", "node_b", "node_c"));
  EXPECT_EQ(queue.in_flight_count(), 2u);
  EXPECT_EQ(queue.waiting_count(), 0u);
}

TEST(QueryQueueTest, CancelOfQueuedKeyNeverStarts)
{
  ResultLog results;
  FakeCalls calls;
  Queue queue(calls.start_call(), results.on_result(), patient_options(1));

  queue.request("node_a");
  ASSERT_TRUE(wait_for_condition([&calls] { return calls.starts_for("node_a") == 1; }));

  queue.request("node_b");
  ASSERT_TRUE(wait_for_condition([&queue] { return queue.waiting_count() == 1; }));

  queue.cancel("node_b");
  EXPECT_EQ(queue.waiting_count(), 0u);

  calls.done_for("node_a")("params_a");
  ASSERT_TRUE(wait_for_condition([&results] { return results.size() == 1; }));
  std::this_thread::sleep_for(kSettle);

  EXPECT_EQ(calls.starts_for("node_b"), 0u);
  EXPECT_EQ(calls.total_starts(), 1u);
}

TEST(QueryQueueTest, CancelOfInFlightKeyDropsResponse)
{
  ResultLog results;
  FakeCalls calls;
  Queue queue(calls.start_call(), results.on_result(), patient_options());

  queue.request("node_a");
  ASSERT_TRUE(wait_for_condition([&calls] { return calls.starts_for("node_a") == 1; }));

  queue.cancel("node_a");
  EXPECT_EQ(queue.in_flight_count(), 0u);

  calls.done_for("node_a")("params_a");
  std::this_thread::sleep_for(kSettle);

  EXPECT_EQ(results.size(), 0u);
  EXPECT_EQ(calls.starts_for("node_a"), 1u);
  EXPECT_EQ(queue.waiting_count(), 0u);
}

TEST(QueryQueueTest, RetriesAfterFailure)
{
  ResultLog results;
  FakeCalls calls;
  const auto options = patient_options(1);
  Queue queue(calls.start_call(), results.on_result(), options);

  queue.request("node_a");
  ASSERT_TRUE(wait_for_condition([&calls] { return calls.starts_for("node_a") == 1; }));

  const auto failed_at = std::chrono::steady_clock::now();
  calls.done_for("node_a", 0)(std::nullopt);
  ASSERT_TRUE(wait_for_condition([&calls] { return calls.starts_for("node_a") == 2; }));

  EXPECT_GE(std::chrono::steady_clock::now() - failed_at, options.retry_delay);
  EXPECT_EQ(results.size(), 0u);

  calls.done_for("node_a", 1)("params_a");
  ASSERT_TRUE(wait_for_condition([&results] { return results.size() == 1; }));
  EXPECT_THAT(results.entries(), ElementsAre(std::make_pair(std::string("node_a"), std::string("params_a"))));
}

TEST(QueryQueueTest, RetriesAfterTimeoutAndDropsLateResponse)
{
  ResultLog results;
  FakeCalls calls;
  Queue queue(calls.start_call(), results.on_result(), impatient_options(1));

  queue.request("node_a");
  ASSERT_TRUE(wait_for_condition([&calls] { return calls.starts_for("node_a") == 1; }));
  ASSERT_TRUE(wait_for_condition([&calls] { return calls.starts_for("node_a") == 2; }));

  calls.done_for("node_a", 0)("params_a");
  std::this_thread::sleep_for(kSettle);

  EXPECT_EQ(results.size(), 0u);
}

TEST(QueryQueueTest, DestructionWithUnansweredAttemptReturnsPromptly)
{
  ResultLog results;
  FakeCalls calls;
  auto queue = std::make_unique<Queue>(calls.start_call(), results.on_result(), patient_options());

  queue->request("node_a");
  ASSERT_TRUE(wait_for_condition([&calls] { return calls.starts_for("node_a") == 1; }));

  auto destroyed = std::async(std::launch::async, [&queue] { queue.reset(); });
  ASSERT_EQ(destroyed.wait_for(kGenerousWait), std::future_status::ready);
}

TEST(QueryQueueTest, DoneAfterDestructionIsSafe)
{
  ResultLog results;
  FakeCalls calls;
  {
    Queue queue(calls.start_call(), results.on_result(), patient_options());
    queue.request("node_a");
    ASSERT_TRUE(wait_for_condition([&calls] { return calls.starts_for("node_a") == 1; }));
  }

  calls.done_for("node_a")("params_a");
  std::this_thread::sleep_for(kSettle);

  EXPECT_EQ(results.size(), 0u);
}

TEST(QueryQueueTest, OnResultNotInvokedAfterDestruction)
{
  std::atomic<int> delivered{0};
  FakeCalls calls;
  {
    Queue queue(
      calls.start_call(), [&delivered](const std::string &, std::string) { ++delivered; }, patient_options(2));
    queue.request("node_a");
    queue.request("node_b");
    ASSERT_TRUE(wait_for_condition([&calls] { return calls.total_starts() == 2; }));

    calls.done_for("node_a")("params_a");
    ASSERT_TRUE(wait_for_condition([&delivered] { return delivered.load() == 1; }));
  }

  calls.done_for("node_b")("params_b");
  std::this_thread::sleep_for(kSettle);

  EXPECT_EQ(delivered.load(), 1);
}

TEST(QueryQueueTest, SynchronousSuccessDoesNotDeadlock)
{
  const auto delivered = std::make_shared<std::atomic<int>>(0);
  auto finished = std::async(std::launch::async, [delivered] {
    Queue queue(
      [](const std::string & key, Queue::Done done) { done("params_" + key); },
      [delivered](const std::string &, std::string) { ++*delivered; },
      patient_options());
    queue.request("node_a");
    wait_for_condition([delivered] { return delivered->load() == 1; });
  });

  ASSERT_EQ(finished.wait_for(kGenerousWait), std::future_status::ready);
  EXPECT_EQ(delivered->load(), 1);
}

TEST(QueryQueueTest, SynchronousFailureDoesNotDeadlock)
{
  const auto starts = std::make_shared<std::atomic<int>>(0);
  auto finished = std::async(std::launch::async, [starts] {
    Queue queue(
      [starts](const std::string &, Queue::Done done) {
        ++*starts;
        done(std::nullopt);
      },
      [](const std::string &, std::string) {},
      impatient_options());
    queue.request("node_a");
    wait_for_condition([starts] { return starts->load() >= 3; });
  });

  ASSERT_EQ(finished.wait_for(kGenerousWait), std::future_status::ready);
  EXPECT_GE(starts->load(), 3);
}
