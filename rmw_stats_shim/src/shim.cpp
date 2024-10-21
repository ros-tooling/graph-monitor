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

#include <cstdio>

#include "rmw_stats_shim/shim.h"

#include "rcpputils/shared_library.hpp"
#include "rmw/rmw.h"

#include "rmw_stats_shim/stat_collector.hpp"

using rmw_stats_shim::StatCollector;

extern "C"
{
rmw_ret_t
wrap_rmw_init(
  void * rmw_library,
  const rmw_init_options_t *,
  rmw_context_t *)
{
  auto plib = reinterpret_cast<rcpputils::SharedLibrary *>(rmw_library);
  StatCollector::instance().setRmwImplementation(plib);
  return RMW_RET_OK;
}

rmw_node_t *
wrap_rmw_create_node(
  void * impl,
  rmw_context_t * context,
  const char * name,
  const char * namespace_)
{
  auto ret = reinterpret_cast<decltype(rmw_create_node) *>(impl)(context, name, namespace_);
  if (ret != nullptr) {
    StatCollector::instance().addNode(ret);
  }
  return ret;
}

rmw_ret_t
wrap_rmw_destroy_node(
  void * impl,
  rmw_node_t * node)
{
  if (node) {
    StatCollector::instance().removeNode(node);
  }
  return reinterpret_cast<decltype(rmw_destroy_node) *>(impl)(node);
}

rmw_publisher_t *
wrap_rmw_create_publisher(
  void * impl,
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name,
  const rmw_qos_profile_t * qos_profile,
  const rmw_publisher_options_t * publisher_options)
{
  auto ret = reinterpret_cast<decltype(rmw_create_publisher) *>(impl)(
    node, type_support, topic_name, qos_profile, publisher_options);
  if (ret != nullptr) {
    StatCollector::instance().addPublisher(ret, node);
  }
  return ret;
}

rmw_ret_t
wrap_rmw_destroy_publisher(
  void * impl,
  rmw_node_t * node,
  rmw_publisher_t * publisher)
{
  if (publisher) {
    StatCollector::instance().removePublisher(publisher);
  }
  return reinterpret_cast<decltype(rmw_destroy_publisher) *>(impl)(node, publisher);
}

rmw_ret_t
wrap_rmw_publish(
  void * impl,
  const rmw_publisher_t * publisher,
  const void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  auto ret = reinterpret_cast<decltype(rmw_publish) *>(impl)(publisher, ros_message, allocation);
  if (ret == RMW_RET_OK) {
    StatCollector::instance().onPublish(publisher);
  }
  return ret;
}

rmw_ret_t
wrap_rmw_publish_loaned_message(
  void * impl,
  const rmw_publisher_t * publisher,
  void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  auto ret = reinterpret_cast<decltype(rmw_publish_loaned_message) *>(impl)(
    publisher, ros_message, allocation);
  if (ret == RMW_RET_OK) {
    StatCollector::instance().onPublish(publisher);
  }
  return ret;
}

rmw_ret_t
wrap_rmw_publish_serialized_message(
  void * impl,
  const rmw_publisher_t * publisher,
  const rmw_serialized_message_t * serialized_message,
  rmw_publisher_allocation_t * allocation)
{
  auto ret = reinterpret_cast<decltype(rmw_publish_serialized_message) *>(impl)(
    publisher, serialized_message, allocation);
  if (ret == RMW_RET_OK) {
    StatCollector::instance().onPublish(publisher);
  }
  return ret;
}

rmw_subscription_t *
wrap_rmw_create_subscription(
  void * impl,
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name,
  const rmw_qos_profile_t * qos_policies,
  const rmw_subscription_options_t * subscription_options)
{
  auto ret = reinterpret_cast<decltype(rmw_create_subscription) *>(impl)(
    node, type_support, topic_name, qos_policies, subscription_options);
  if (ret != nullptr) {
    StatCollector::instance().addSubscription(ret, node);
  }
  return ret;
}

rmw_ret_t
wrap_rmw_destroy_subscription(
  void * impl,
  rmw_node_t * node,
  rmw_subscription_t * subscription)
{
  if (subscription) {
    StatCollector::instance().removeSubscription(subscription);
  }
  return reinterpret_cast<decltype(rmw_destroy_subscription) *>(impl)(node, subscription);
}

rmw_ret_t
wrap_rmw_take(
  void * impl,
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  auto ret = reinterpret_cast<decltype(rmw_take) *>(impl)(
    subscription, ros_message, taken, allocation);
  if (ret == RMW_RET_OK && *taken) {
    StatCollector::instance().onReceive(subscription);
  }
  return ret;
}

rmw_ret_t
wrap_rmw_take_with_info(
  void * impl,
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  auto ret = reinterpret_cast<decltype(rmw_take_with_info) *>(impl)(
    subscription, ros_message, taken, message_info, allocation);
  if (ret == RMW_RET_OK && *taken) {
    StatCollector::instance().onReceive(subscription, message_info);
  }
  return ret;
}

rmw_ret_t
wrap_rmw_take_sequence(
  void * impl,
  const rmw_subscription_t * subscription,
  size_t count,
  rmw_message_sequence_t * message_sequence,
  rmw_message_info_sequence_t * message_info_sequence,
  size_t * taken,
  rmw_subscription_allocation_t * allocation)
{
  auto ret = reinterpret_cast<decltype(rmw_take_sequence) *>(impl)(
    subscription, count, message_sequence, message_info_sequence, taken, allocation);
  if (ret == RMW_RET_OK && *taken > 0) {
    for (size_t i = 0; i < message_info_sequence->size; i++) {
      StatCollector::instance().onReceive(subscription, message_info_sequence->data + i);
    }
  }
  return ret;
}

rmw_ret_t
wrap_rmw_take_serialized_message(
  void * impl,
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  auto ret = reinterpret_cast<decltype(rmw_take_serialized_message) *>(impl)(
    subscription, serialized_message, taken, allocation);
  if (ret == RMW_RET_OK && *taken) {
    StatCollector::instance().onReceive(subscription);
  }
  return ret;
}

rmw_ret_t
wrap_rmw_take_serialized_message_with_info(
  void * impl,
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  auto ret = reinterpret_cast<decltype(rmw_take_serialized_message_with_info) *>(impl)(
    subscription, serialized_message, taken, message_info, allocation);
  if (ret == RMW_RET_OK && *taken) {
    StatCollector::instance().onReceive(subscription, message_info);
  }
  return ret;
}

rmw_ret_t
wrap_rmw_take_loaned_message(
  void * impl,
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  auto ret = reinterpret_cast<decltype(rmw_take_loaned_message) *>(impl)(
    subscription, loaned_message, taken, allocation);
  if (ret == RMW_RET_OK && *taken) {
    StatCollector::instance().onReceive(subscription);
  }
  return ret;
}

rmw_ret_t
wrap_rmw_take_loaned_message_with_info(
  void * impl,
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  auto ret = reinterpret_cast<decltype(rmw_take_loaned_message_with_info) *>(impl)(
    subscription, loaned_message, taken, message_info, allocation);
  if (ret == RMW_RET_OK && *taken) {
    StatCollector::instance().onReceive(subscription, message_info);
  }
  return ret;
}
}  // extern "C"
