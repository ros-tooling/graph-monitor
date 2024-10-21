// Copyright 2024, Bonsai Robotics, Inc - All Rights Reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rosgraph_monitor/event.hpp"

namespace rosgraph_monitor
{

bool Event::is_set() const
{
  return state_.load();
}

void Event::set()
{
  state_.exchange(true);
  cv_.notify_all();
}

void Event::clear()
{
  state_.exchange(false);
}

bool Event::check_and_clear()
{
  return state_.exchange(false);
}

bool Event::wait_for(const std::chrono::milliseconds & timeout)
{
  if (is_set()) {
    return true;
  }
  std::unique_lock<std::mutex> lock(mutex_);
  return cv_.wait_for(
    lock, timeout, [this]() {
      return state_.load();
    });
}


}  // namespace rosgraph_monitor
