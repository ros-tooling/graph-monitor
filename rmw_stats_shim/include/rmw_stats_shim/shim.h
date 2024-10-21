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

#ifndef RMW_STATS_SHIM__SHIM_H_
#define RMW_STATS_SHIM__SHIM_H_

#include "rmw/rmw.h"

/*
Declares function wrappers for RMW API functions.
This allows us to instrument RMW implementation (DDS etc) usage by any ROS node.

To register a new wrapper, simply declare it here, with the pattern "wrap_{RMW_FUNCTION_NAME}".
Each wrapper function takes a void pointer to the underlying implementation function as its first argument.
To use the implementation in the wrapper, use `reinterpret_cast<decltype(RMW_FUNCTION_NAME) *>(impl)(args);`
*/

#ifdef __cplusplus
extern "C"
{
#endif

/// @brief Special function that performs any necessary initialization for the wrapper
/// @param rmw_library Pointer to the loaded rcpputils::SharedLibrary of the RMW implementation
/// @param options unused
/// @param context unused
/// @return RMW_RET_OK if no errors occurred
rmw_ret_t
wrap_rmw_init(
  void * rmw_library,
  const rmw_init_options_t *,
  rmw_context_t *);

/// Node API

rmw_node_t *
wrap_rmw_create_node(
  void * impl,
  rmw_context_t * context,
  const char * name,
  const char * namespace_);

rmw_ret_t
wrap_rmw_destroy_node(
  void * impl,
  rmw_node_t * node);


/// Publisher API

rmw_publisher_t *
wrap_rmw_create_publisher(
  void * impl,
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name,
  const rmw_qos_profile_t * qos_profile,
  const rmw_publisher_options_t * publisher_options);

rmw_ret_t
wrap_rmw_destroy_publisher(
  void * impl,
  rmw_node_t * node,
  rmw_publisher_t * publisher);

rmw_ret_t
wrap_rmw_publish(
  void * impl,
  const rmw_publisher_t * publisher,
  const void * ros_message,
  rmw_publisher_allocation_t * allocation);

rmw_ret_t
wrap_rmw_publish_loaned_message(
  void * impl,
  const rmw_publisher_t * publisher,
  void * ros_message,
  rmw_publisher_allocation_t * allocation);

rmw_ret_t
wrap_rmw_publish_serialized_message(
  void * impl,
  const rmw_publisher_t * publisher,
  const rmw_serialized_message_t * serialized_message,
  rmw_publisher_allocation_t * allocation);


/// Subscription API

rmw_subscription_t *
wrap_rmw_create_subscription(
  void * impl,
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name,
  const rmw_qos_profile_t * qos_policies,
  const rmw_subscription_options_t * subscription_options);

rmw_ret_t
wrap_rmw_destroy_subscription(
  void * impl,
  rmw_node_t * node,
  rmw_subscription_t * subscription);

rmw_ret_t
wrap_rmw_take(
  void * impl,
  const rmw_subscription_t * subscription, void * ros_message,
  bool * taken, rmw_subscription_allocation_t * allocation);

rmw_ret_t
wrap_rmw_take_with_info(
  void * impl,
  const rmw_subscription_t * subscription, void * ros_message,
  bool * taken, rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation);

rmw_ret_t
wrap_rmw_take_sequence(
  void * impl,
  const rmw_subscription_t * subscription,
  size_t count,
  rmw_message_sequence_t * message_sequence,
  rmw_message_info_sequence_t * message_info_sequence,
  size_t * taken,
  rmw_subscription_allocation_t * allocation);

rmw_ret_t
wrap_rmw_take_serialized_message(
  void * impl,
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation);

rmw_ret_t
wrap_rmw_take_serialized_message_with_info(
  void * impl,
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation);

rmw_ret_t
wrap_rmw_take_loaned_message(
  void * impl,
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation);

rmw_ret_t
wrap_rmw_take_loaned_message_with_info(
  void * impl,
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // RMW_STATS_SHIM__SHIM_H_
