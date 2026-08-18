# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(topic_stats_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=switch)
  add_link_options("-Wl,--no-undefined")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosgraph_monitor_msgs REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(topic_stats_core REQUIRED)

# Deliberately does not depend on rclcpp. The RMW ingest adapter links this from inside every
# instrumented process, and pulling the client library in there would be a significant imposition.
add_library(${PROJECT_NAME} SHARED
  src/stats_message.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  rosidl_runtime_cpp::rosidl_runtime_cpp
  topic_stats_core::topic_stats_core
  ${rosgraph_monitor_msgs_TARGETS}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_stats_message test/test_stats_message.cpp)
  target_link_libraries(test_stats_message ${PROJECT_NAME})
endif()

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rosgraph_monitor_msgs rosidl_runtime_cpp topic_stats_core)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
