// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <string>
#include <utility>
#include <vector>

#include "topic_stats_core/slot_table.hpp"
#include "topic_stats_core/types.hpp"

using topic_stats_core::EndpointId;
using topic_stats_core::SlotTable;

namespace
{

struct Thing
{
  explicit Thing(std::string n)
  : name(std::move(n))
  {}

  std::string name;
};

using Table = SlotTable<Thing, EndpointId>;

}  // namespace

TEST(SlotTable, emplace_returns_a_usable_handle_and_pointer)
{
  Table table;
  auto [id, thing] = table.emplace("a");
  ASSERT_NE(thing, nullptr);
  EXPECT_TRUE(id.valid());
  EXPECT_EQ(thing->name, "a");
  EXPECT_EQ(table.find(id), thing);
  EXPECT_EQ(table.live_count(), 1u);
}

TEST(SlotTable, default_constructed_handle_resolves_to_nothing)
{
  Table table;
  table.emplace("a");
  EXPECT_EQ(table.find(EndpointId{}), nullptr);
}

TEST(SlotTable, out_of_range_handle_resolves_to_nothing)
{
  Table table;
  EXPECT_EQ(table.find(EndpointId{7, 0}), nullptr);
}

TEST(SlotTable, erase_invalidates_the_handle)
{
  Table table;
  auto [id, thing] = table.emplace("a");
  (void)thing;
  EXPECT_TRUE(table.erase(id));
  EXPECT_EQ(table.find(id), nullptr);
  EXPECT_EQ(table.live_count(), 0u);
  // Erasing twice is not an error, it just reports that nothing was there.
  EXPECT_FALSE(table.erase(id));
}

TEST(SlotTable, recycled_slot_does_not_answer_to_the_old_handle)
{
  // The whole reason generations exist. Without them, an ingest adapter holding a handle to a
  // destroyed subscription would silently record onto whatever endpoint reused the slot.
  Table table;
  auto [old_id, old_thing] = table.emplace("old");
  (void)old_thing;
  ASSERT_TRUE(table.erase(old_id));

  auto [new_id, new_thing] = table.emplace("new");
  ASSERT_NE(new_thing, nullptr);
  EXPECT_EQ(new_id.index, old_id.index) << "expected the freed slot to be reused";
  EXPECT_NE(new_id.generation, old_id.generation);

  EXPECT_EQ(table.find(old_id), nullptr);
  EXPECT_EQ(table.find(new_id), new_thing);
  EXPECT_EQ(new_thing->name, "new");
}

TEST(SlotTable, indices_are_recycled_rather_than_growing_without_bound)
{
  Table table;
  for (int i = 0; i < 1000; ++i) {
    auto [id, thing] = table.emplace("churn");
    (void)thing;
    ASSERT_TRUE(table.erase(id));
  }
  EXPECT_EQ(table.live_count(), 0u);
  EXPECT_EQ(table.capacity(), 1u) << "churning one endpoint should not allocate 1000 slots";
}

TEST(SlotTable, element_addresses_survive_later_insertions)
{
  // The collector resolves an endpoint under a shared lock and then works with the pointer, so
  // insertion by another thread must not move existing elements.
  Table table;
  auto [first_id, first] = table.emplace("first");
  (void)first_id;
  for (int i = 0; i < 500; ++i) {
    table.emplace("filler");
  }
  EXPECT_EQ(table.find(first_id), first);
  EXPECT_EQ(first->name, "first");
}

TEST(SlotTable, for_each_visits_only_live_slots)
{
  Table table;
  auto [a, thing_a] = table.emplace("a");
  auto [b, thing_b] = table.emplace("b");
  auto [c, thing_c] = table.emplace("c");
  (void)thing_a;
  (void)thing_c;
  (void)b;
  ASSERT_TRUE(table.erase(b));
  (void)thing_b;

  std::vector<std::string> seen;
  std::vector<EndpointId> ids;
  table.for_each([&](EndpointId id, Thing & thing) {
    seen.push_back(thing.name);
    ids.push_back(id);
  });

  EXPECT_EQ(seen, std::vector<std::string>({"a", "c"}));
  ASSERT_EQ(ids.size(), 2u);
  // Handles yielded by iteration must be usable.
  EXPECT_EQ(table.find(ids[0]), table.find(a));
  EXPECT_EQ(table.find(ids[1]), table.find(c));
}
